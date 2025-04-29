#pragma once

#include <cmath>
#include <utility>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <stop_token>
#include <syncstream>

#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <ac25_test/msg/pose2d.hpp>

#include "utility.hpp"
#include "geometry.hpp"

namespace ac25_test::ros_world::impl {
	using namespace std::chrono_literals;

	using Eigen::Matrix2Xd;
	using Eigen::Vector2d;

	using ac_semi_2025::geometry::Pose2d;
	using namespace ac_semi_2025::integer_type;

	struct RosWorld final : rclcpp::Node {
		std::unique_ptr<Matrix2Xd> laserscan{};
		std::condition_variable condvar{};
		std::mutex mtx{};
		std::stop_token stoken;
		double th_min;
		double th_max;
		tf2_ros::TransformBroadcaster tf2_broadcaster;
		rclcpp::Publisher<ac25_test::msg::Pose2d>::SharedPtr robot_speed_pub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laserscan_pub;
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;

		RosWorld(std::stop_token&& stoken, const double th_min, const double th_max):
			rclcpp::Node{"ros_world"}
			, stoken{std::move(stoken)}
			, th_min{th_min}
			, th_max{th_max}
			, tf2_broadcaster{*this}
			, robot_speed_pub{this->create_publisher<ac25_test::msg::Pose2d>("robot_speed", 10)}
			, laserscan_pub{this->create_publisher<sensor_msgs::msg::PointCloud2>("laserscan", 10)}
			, lidar_sub{this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) -> void {
				this->laserscan_callback(msg);
			})}
		{}

		virtual ~RosWorld() override = default;

		auto update(const Pose2d& robot_speed, const double) noexcept -> std::unique_ptr<Matrix2Xd> {
			ac25_test::msg::Pose2d msg{};
			msg.x = robot_speed.xy(0);
			msg.y = robot_speed.xy(1);
			msg.th = robot_speed.th;
			{
				std::unique_lock lck{this->mtx};
				this->robot_speed_pub->publish(msg);
			}
			// {
			// 	std::osyncstream osycerr{std::cerr};
			// 	std::println(osycerr, "ros_world update.");
			// }
			std::unique_lock lck{this->mtx};
			while(!this->stoken.stop_requested() && !this->laserscan) {
					this->condvar.wait_for (
					lck
					, 1ms
				);

				// {
				// 	std::osyncstream osycerr{std::cerr};
				// 	std::println(osycerr, "in ros_world::update loop");
				// }
			}
			// {
			// 	std::osyncstream osycerr{std::cerr};
			// 	std::println(osycerr, "ros_world update2.");
			// }
			auto ret = std::move(this->laserscan);
			this->laserscan.reset();
			return ret;
		}

		void broadcast_pose(const Pose2d& pose) noexcept {
			geometry_msgs::msg::TransformStamped msg{};
			msg.header.frame_id = "map";
			msg.header.stamp = this->now();
			msg.child_frame_id = "upsidedown_upsidedown_laser";
			msg.transform.translation.x = pose.xy(0);
			msg.transform.translation.y = pose.xy(1);
			msg.transform.translation.z = 0.0;
			// 以下、ROS2の未整理で辛い型変換の部分。
			// ここを調べていくと、ROS2から逃れたくなるだろう
			tf2::Quaternion q{};
			q.setRPY(0.0, 0.0, pose.th);
			msg.transform.rotation.x = q.x();  // なんでtf2::Quaternionからrotationへの変換が無いんでしょうね
			msg.transform.rotation.y = q.y();
			msg.transform.rotation.z = q.z();
			msg.transform.rotation.w = q.w();
			{
				std::unique_lock lck{this->mtx};
				this->tf2_broadcaster.sendTransform(std::move(msg));
				std::osyncstream osycerr{std::cerr};
				std::println(osycerr, "broadcast.");
			}
		}

		void publish_laserscan(const Eigen::Matrix2Xd& points) {
			sensor_msgs::msg::PointCloud2 cloud{};
			cloud.header.frame_id = "upsidedown_upsidedown_laser";
			cloud.header.stamp = this->now();
			cloud.height = 1;
			cloud.width = points.cols();
			cloud.is_dense = true;
			cloud.is_bigendian = false;
		
			sensor_msgs::PointCloud2Modifier modifier(cloud);
			modifier.setPointCloud2FieldsByString(1, "xyz");
			modifier.resize(points.cols());
		
			sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
			sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
			sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
		
			for (int i = 0; i < points.cols(); ++i, ++iter_x, ++iter_y, ++iter_z) {
				*iter_x = static_cast<float>(points(0, i));
				*iter_y = static_cast<float>(points(1, i));
				*iter_z = 0.0f;
			}
		
			{
				std::unique_lock lck{this->mtx};
				this->laserscan_pub->publish(std::move(cloud));
			}
		}

		void laserscan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) noexcept {
			const auto& ranges = msg->ranges;
			const i64 n = ranges.size();
			const double angle_min = msg->angle_min;
			const double angle_increment = msg->angle_increment;
			Matrix2Xd laserscan{2, n};
			i64 net_points = 0;
			for(i64 i = 0; i < n; ++i) {
				const double th = angle_min + angle_increment * i;
				
				if(th < this->th_min
					|| this->th_max < th
					|| ranges[i] <= msg->range_min
					|| msg->range_max <= ranges[i]
				) continue;

				laserscan.col(net_points) = ranges[i] * Vector2d{std::cos(-th), std::sin(-th)};
				net_points++;
			}
			laserscan = laserscan.leftCols(net_points);
			// {
			// 	std::osyncstream osycerr{std::cerr};
			// 	std::println(osycerr, "ros_world call.");
			// }
			{
				std::unique_lock lck{this->mtx};
				this->laserscan = std::make_unique<Matrix2Xd>(std::move(laserscan));
				this->condvar.notify_all();
			}
			// {
			// 	std::osyncstream osycerr{std::cerr};
			// 	std::println(osycerr, "ros_world call2.");
			// }
		}
	};
}

namespace ac25_test::ros_world {
	using impl::RosWorld;
}