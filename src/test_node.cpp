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
	using namespace std::chrono_literals;

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
	struct Robot final {
		struct Constant final {
			std::vector<Line2d> global_edges;
			Polyline2d route;
			i64 number_of_iteration;
		};
		const Constant cons;
		
		CarrotPursuit carrot;
		Pose2d pose;
		std::mutex mtx{};

		// icpでposeの更新
		auto icp_update(const Matrix2Xd& laserscan/*, auto& debug_node_sp*/) {
			const auto pose = [this] {
				std::unique_lock lck{this->mtx};
				const auto ret = this->pose;
				return ret;
			}();

			// const auto visible_edges = cons.map.make_visible_lines(pose);
			const auto g2l = pose.to_isometry().inverse();
			auto edges = this->cons.global_edges;
			for(auto& edge : edges) {
				edge = Line2d{g2l * edge.p1, g2l * edge.p2};
			}
			const auto l2new_l = icp_p2l(laserscan, edges, this->cons.number_of_iteration/*, debug_node_sp*/).inverse();
			const auto new_pose = Pose2d::from_isometry((l2new_l * g2l).inverse());
			if(new_pose.not_nan()) {
				{
					std::unique_lock lck{this->mtx};
					this->pose = new_pose;
				}
			}
		}

		// 制御入力の計算
		auto control_input(const double dt) noexcept(false) -> Pose2d {
			const auto pose = [this] {
				std::unique_lock lck{this->mtx};
				const auto ret = this->pose;
				return ret;
			}();
			
			const auto control_speed = carrot.update(this->cons.route.vertices, pose, dt);
			if(!control_speed.has_value()) {
				throw std::runtime_error{"Panic: cannot calc speed by carrot pursuit."};
			}

			return *control_speed;
		}
	};

	// 雑タイマー
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
		// cwdの確認
		if(const auto cwd = std::filesystem::current_path().string();
			!cwd.ends_with("ws/")
			&& !cwd.ends_with("ws")
		) {
			std::println(std::cerr, "cwd may not be a workspace directory");
		}

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
				std::this_thread::sleep_for(1ms);
			}
			
			std::println("ros node end. shutdown start...");
		}};

		// icp_updateのほう
		std::jthread thread3{[ros_world_sp, debug_node_sp, stoken = ssource.get_token(), route = std::move(route), global_edges = std::move(global_edges)] {
			/// ロボットの初期化
			Robot robot {
				.cons = Robot::Constant {
					.global_edges = std::move(global_edges)
					, .route = std::move(route)
					, .number_of_iteration = 50
				}
				, .carrot = CarrotPursuit::make (
					FeedForwardedPid::Constant {
						.kp = 3.0
						, .ki = 0.0
						, .kd = 0.0
						, .i_max = 0.0
						, .feed_forward = 1.3
					}
					, FeedForwardedPid::Constant {
						.kp = 4.0
						, .ki = 0.0
						, .kd = 0.0
						, .i_max = 0.0
						, .feed_forward = 0.0
					}
					, FeedForwardedPid::Constant {
						.kp = 1.0
						, .ki = 0.0
						, .kd = 0.1
						, .i_max = 0.0
						, .feed_forward = 0.0
					}
				)
				, .pose = Pose2d{Vector2d{0.5, 0.5}, 0.0}
			};
			
			Pose2d control_input{Vector2d::Zero(), 0.0};
			auto sim_clock = MyClock::make();
			auto robo_clock = MyClock::make();
			std::println("icp_update loop start");

			while(!stoken.stop_requested()) {
				const auto laserscan = [ros_world_sp, &control_input, &sim_clock] {
					return ros_world_sp->update(control_input, sim_clock.lap().count());
				}();

				if(laserscan && laserscan->cols() != 0) {
					robot.icp_update(*laserscan/*, debug_node_sp*/);
				}
				if(const auto res = robot.carrot.update(route.vertices, robot.pose, robo_clock.lap().count())) {
					control_input = *res;
				}
				else {
					std::println("Warn: cannot calc speed by carrot pursuit.");
					control_input = Pose2d{Vector2d::Zero(), 0.0};
				}
				
				// robot.pose = robot.pose + control_input * sim_clock.watch().count();

				std::println(std::cerr, "{}", control_input.to_str());
				debug_node_sp->broadcast_pose(robot.pose, "map"sv, "usdusd_laser"sv);
				if(laserscan) debug_node_sp->publish_laserscan(*laserscan, "usdusd_laser"sv);
				debug_node_sp->publish_polyline(robot.cons.global_edges, "map"sv, "lines"sv);
			}

			std::println("icp_update loop end");
		}};
	}
}

int main(int argc, char ** argv) {
	test::main(argc, argv);
}