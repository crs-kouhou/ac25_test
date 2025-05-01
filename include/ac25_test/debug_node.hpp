#pragma once

#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stop_token>
#include <syncstream>
#include <utility>

#include <eigen3/Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <ac25_test/msg/pose2d.hpp>

#include "geometry.hpp"
#include "utility.hpp"

namespace ac25_test::debug_node::impl {
	using namespace std::chrono_literals;

	using Eigen::Vector2d;

	using ac_semi_2025::geometry::Pose2d;
	using ac_semi_2025::geometry::Line2d;
	using namespace ac_semi_2025::integer_type;

	struct DebugNode final : rclcpp::Node {
		std::condition_variable condvar{};
		std::mutex mtx{};
		std::stop_token stoken;
		tf2_ros::TransformBroadcaster tf2_broadcaster;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr laserscan_pub;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

		DebugNode(std::stop_token&& stoken)
			: rclcpp::Node{"debug_node"}
			, stoken{std::move(stoken)}
			, tf2_broadcaster{*this}
			, laserscan_pub{this->create_publisher<sensor_msgs::msg::PointCloud2>("laserscan", 10)}
			, marker_pub{this->create_publisher<visualization_msgs::msg::Marker>("lines", 10)}
		{}

		virtual ~DebugNode() override = default;

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
		
			for(int i = 0; i < points.cols(); ++i, ++iter_x, ++iter_y, ++iter_z) {
				*iter_x = static_cast<float>(points(0, i));
				*iter_y = static_cast<float>(points(1, i));
				*iter_z = 0.0f;
			}
		
			{
				std::unique_lock lck{this->mtx};
				this->laserscan_pub->publish(std::move(cloud));
			}
		}

		void publish_polyline(const std::vector<Line2d>& polyline) {
			visualization_msgs::msg::Marker marker;
			marker.header.frame_id = "map";  // 例: 座標フレーム
			marker.header.stamp = rclcpp::Clock().now();
			marker.ns = "lines";
			marker.id = 0;
			marker.type = visualization_msgs::msg::Marker::LINE_LIST;
			marker.action = visualization_msgs::msg::Marker::ADD;
			marker.scale.x = 0.05;  // 線の太さ

			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
			marker.color.a = 1.0;

			constexpr auto to_point_msg = [](const Vector2d& v) -> geometry_msgs::msg::Point {
				geometry_msgs::msg::Point msg{};
				msg.x = v(0);
				msg.y = v(1);
				msg.z = 0;
				return msg;
			};

			for(const auto& seg : polyline) {
				marker.points.push_back(to_point_msg(seg.p1));
				marker.points.push_back(to_point_msg(seg.p2));
			}

			{
				std::unique_lock lck{this->mtx};
				this->marker_pub->publish(marker);
			}
		}
	};
}

namespace ac25_test::debug_node {
	using impl::DebugNode;
}