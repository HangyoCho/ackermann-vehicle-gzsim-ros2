#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

class GtPosePublisher : public rclcpp::Node {
public:
    GtPosePublisher()
    : Node("gt_pose_publisher"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // path_pub_ = this->create_publisher<nav_msgs::msg::Path>("gt_path", 10);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gt_pose", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&GtPosePublisher::timerCallback, this)
        );

        gt_path_.header.frame_id = "odom";
        RCLCPP_INFO(this->get_logger(), "GT Pose Publisher Node Started.");
    }

private:
    void timerCallback() {
        geometry_msgs::msg::TransformStamped tf;
        double LidarHeight = 0.2; // Adjust this if your LiDAR is mounted at a height
        try {
            tf = tf_buffer_.lookupTransform(
                "odom", "saye/base_link",  // Change to your robot's base frame if needed
                rclcpp::Time(0),           // <-- Use latest available transform
                rclcpp::Duration::from_seconds(0.01)
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = this->get_clock()->now();
        pose_stamped.header.frame_id = "odom";
        pose_stamped.pose.position.x = tf.transform.translation.x;
        pose_stamped.pose.position.y = tf.transform.translation.y;
        pose_stamped.pose.position.z = tf.transform.translation.z + LidarHeight; // Adjust for LiDAR height if needed
        pose_stamped.pose.orientation = tf.transform.rotation;

        // gt_path_.header.stamp = pose_stamped.header.stamp;
        // gt_path_.poses.push_back(pose_stamped);

        pose_pub_->publish(pose_stamped);
        // path_pub_->publish(gt_path_);
    }

    nav_msgs::msg::Path gt_path_;
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GtPosePublisher>());
    rclcpp::shutdown();
    return 0;
}