#include <thread>
#include <atomic>
#include <iostream>
#include <memory>
#include <vector>
#include <chrono>
#include <string_view>
#include <filesystem>
#include <print>
#include <mutex>
#include <stop_token>

#include "utility.hpp"
#include "geometry.hpp"
#include "icp_on_svd.hpp"
#include "read_edges.hpp"
#include "carrot_pursuit.hpp"
// #include "global_map.hpp"
#include "ac25_test/ros_world.hpp"
#include "ac25_test/debug_node.hpp"

namespace test {
	using namespace std::string_view_literals;

	using Eigen::Matrix2Xd;
	using Eigen::Vector2d;

	using namespace ac_semi_2025::integer_type;
	using namespace ac_semi_2025::geometry;
	using namespace ac_semi_2025::icp_on_svd;
	using namespace ac_semi_2025::read_edges;
	// using namespace ac_semi_2025::global_map;
	using namespace ac_semi_2025::carrot_pursuit;
	using namespace ac25_test::ros_world;
	using namespace ac25_test::debug_node;

	// ロボの定数と状態
	struct RobotConstant final {
		std::vector<Line2d> global_edges;
		Polyline2d route;
		CarrotPursuit carrot;
		i64 number_of_iteration;
	};
	struct RobotState final {
		Pose2d pose;
		Pose2d icped_pose;
		i64 closest_milestone_index;
	};

	// ロボの更新式
	inline auto robot_update(const RobotConstant& cons, RobotState& state, const Matrix2Xd& laserscan, const double /*dt, auto& debug_node_sp*/) noexcept(false) -> Pose2d {
		// read state /////////////////////////////////////////////////////////////////////////////
		const auto pose = state.pose;

		// ICP on SVD /////////////////////////////////////////////////////////////////////////////
		// const auto visible_edges = cons.map.make_visible_lines(pose);
		const auto g2l = pose.to_isometry().inverse();
		auto edges = cons.global_edges;
		for(auto& edge : edges) {
			edge = Line2d{g2l * edge.p1, g2l * edge.p2};
		}
		const auto l2new_l = icp_p2l(laserscan, edges, cons.number_of_iteration/*, debug_node_sp*/).inverse();
		const auto new_pose = Pose2d::from_isometry((l2new_l * g2l).inverse());

		// calc control input /////////////////////////////////////////////////////////////////////
		const auto speed = cons.carrot.update(cons.route.vertices, new_pose, state.closest_milestone_index);
		if(!speed.has_value()) {
			throw std::runtime_error{"Panic: cannot calc speed by carrot pursuit."};
		}

		// update state ///////////////////////////////////////////////////////////////////////////
		// const auto state_pose = new_pose + *speed * dt;
		const auto state_pose = new_pose;
		if(state_pose.not_nan()) {
			state.pose = state_pose;
			state.icped_pose = state_pose;
		}

		return *speed;
	}

	struct MyClock final {
		std::chrono::time_point<std::chrono::steady_clock> last;

		static auto make() noexcept -> MyClock {
			return MyClock{std::chrono::steady_clock::now()};
		}

		auto watch() const noexcept -> std::chrono::duration<double> {
			const auto now = std::chrono::steady_clock::now();
			return now - this->last;
		}

		auto lap() noexcept -> std::chrono::duration<double> {
			const auto last = this->last;
			this->last = std::chrono::steady_clock::now();
			return this->last - last;
		}
	};

	void main(int argc, char ** argv) {
		if(const auto cwd = std::filesystem::current_path().string();
			!cwd.ends_with("ws/")
			&& !cwd.ends_with("ws")
		) {
			std::println(std::cerr, "cwd may not be a workspace directory");
		}

		// 停止トークン源の作成
		std::stop_source ssource{};

		// 終了用のキー受付
		std::jthread thread1{[&ssource] {
			char dummy;
			std::cin >> dummy;
			ssource.request_stop();
		}};

		// 外界の初期化
		rclcpp::init(argc, argv);
		struct Shutdown final {
			~Shutdown() {
				rclcpp::shutdown();
			}
		} shutdown{};
		auto ros_world_sp = std::make_shared<RosWorld> (
			ssource.get_token()
			, -std::numbers::pi / 2
			, std::numbers::pi / 2
		);
		auto debug_node_sp = std::make_shared<DebugNode> (
			ssource.get_token()
		);
		std::jthread thread2{[ros_world_sp, debug_node_sp, stoken = ssource.get_token()] {
			rclcpp::executors::MultiThreadedExecutor exec{};
			exec.add_node(ros_world_sp);
			exec.add_node(debug_node_sp);
			std::println("ros node start.");
			while(!stoken.stop_requested()) {
				exec.spin_some();
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			
			std::println("ros node end. shutdown start...");
			// std::this_thread::sleep_for(std::chrono::milliseconds(10000));
			// std::println("ros node end. shutdown wait end.");
		}};

		std::jthread thread3{[ros_world_sp, debug_node_sp, stoken = ssource.get_token()] {
			/// グローバルな図形情報を読み出し
			std::vector<Line2d> global_edges = [] {
				if(const auto res = read_edges("data/field.dat"sv)) {
					std::vector<Line2d> ret{};
					for(const auto& polygon : res.value()) {
						const auto lines = polygon.to_edges();
						ret.insert(ret.end(), lines.begin(), lines.end());
					}
					ret.shrink_to_fit();

					return ret;
				}
				else {
					throw std::runtime_error{res.error()};
				}
			}();

			// /// @todo グローバルマップを生成
			// const auto map = GlobalMap<Line2d>::from_shapes(shapes);

			/// ルート情報を読み出し
			Polyline2d route = [] {
				if(const auto res = read_route("data/test_route.dat"sv)) {
					return res.value();
				}
				else {
					throw std::runtime_error{res.error()};
				}
			}();

			/// ロボットの初期化
			RobotConstant rb_cons {
				.global_edges = std::move(global_edges)
				, .route = std::move(route)
				, .carrot = CarrotPursuit {
					.distance_threshold = 5
					, .speed_determination_destination = 0
				}
				, .number_of_iteration = 50
			};
			RobotState rb_state {
				.pose = Pose2d{Vector2d{0.5, 0.5}, 0.0}
				, .icped_pose = Pose2d{Vector2d::Zero(), 0.0}
				, .closest_milestone_index = 0
			};

			Pose2d control_input{Vector2d::Zero(), 0.0};

			auto sim_clock = MyClock::make();
			auto robo_clock = MyClock::make();
			// メインループ
			while(!stoken.stop_requested()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
				// calc world /////////////////////////////////////////////////////////////////////////
				const auto laserscan = [ros_world_sp, &control_input, &sim_clock] {
					return ros_world_sp->update(control_input, sim_clock.lap().count());
				}();

				// calc robot /////////////////////////////////////////////////////////////////////////
				if(laserscan && laserscan->cols() != 0) {
					control_input = robot_update(rb_cons, rb_state, *laserscan, robo_clock.lap().count()/*, debug_node_sp*/);
				}

				// snapshot ///////////////////////////////////////////////////////////////////////////
				std::println("{}", control_input.to_str());
				debug_node_sp->broadcast_pose(rb_state.icped_pose, "map"sv, "usdusd_laser"sv);
				if(laserscan) debug_node_sp->publish_laserscan(*laserscan, "usdusd_laser"sv);
				debug_node_sp->publish_polyline(rb_cons.global_edges, "map"sv, "lines"sv);
				// sim_state.snap(logger);
				// rb_state.snap(logger);

				// std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}

			std::println("main loop end");
		}};
	}
}

int main(int argc, char ** argv) {
	test::main(argc, argv);
}