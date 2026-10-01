#include "wheel_odometry/differential_drive_odometry.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <optional>

class WheelOdometryNode : public rclcpp::Node
{
public:
    // Constructor that initializes the ROS node and sets up the subscription to joint states
    WheelOdometryNode()
        : Node("wheel_odometry_node"),
            odometry_(0.0325, 0.30)
    {
        // Subscribe to the /joint_states topic to receive wheel position updates
        joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            10,
            std::bind(&WheelOdometryNode::jointStateCallback, this, std::placeholders::_1)
        );

        // Create a publisher for the /odom topic to publish the calculated odometry information
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom",
            10
        );

        // Create a TransformBroadcaster to publish the transform between the odom and base_link frames
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Log that the Wheel Odometry Node has started
    RCLCPP_INFO(this->get_logger(), "Wheel Odometry Node started");
    }

private:
    // Callback function that processes incoming joint state messages
    void jointStateCallback(
        const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Initialize wheel positions to zero
        double left_wheel_position = 0.0;
        double right_wheel_position = 0.0;

        // Flags to check if the left and right wheel positions have been found in the message
        bool left_wheel_found = false;
        bool right_wheel_found = false;

        // Iterate through the joint names in the message to find the left and right wheel positions
        for (std::size_t i = 0; i < msg->name.size(); ++i)
        {
            if (msg->name[i] == "left_wheel_joint")
            {
                left_wheel_position = msg->position[i];
                left_wheel_found = true;
            }
            else if (msg->name[i] == "right_wheel_joint")
            {
                right_wheel_position = msg->position[i];
                right_wheel_found = true;
            }
        }
        // If either wheel position is not found, log a warning and return early
        if (!left_wheel_found || !right_wheel_found)
        {
            RCLCPP_WARN(get_logger(), "Wheel positions not found in JointState message");
            return;
        }

        // Get the current time from the message header
        const rclcpp::Time current_time = msg->header.stamp;

        if (!last_joint_state_time_.has_value())
        {
            last_joint_state_time_ = current_time;

            odometry_.update(
                left_wheel_position,
                right_wheel_position,
                0.0);

            return;
        }

        // Calculate the time difference (dt) since the last joint state message
        const double dt = (current_time - last_joint_state_time_.value()).seconds();
        last_joint_state_time_ = current_time;

        // Update the odometry calculations with the new wheel positions and time difference
        odometry_.update(
            left_wheel_position,
            right_wheel_position,
            dt);
        
        // Create an Odometry message to publish the updated odometry information
        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // Set the position and orientation in the Odometry message based on the calculated odometry
        odom_msg.pose.pose.position.x = odometry_.x();
        odom_msg.pose.pose.position.y = odometry_.y();
        odom_msg.pose.pose.position.z = 0.0;
        
        // Calculate the yaw angle from the odometry and convert it to a quaternion for the orientation
        const double yaw = odometry_.yaw();
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
        odom_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
        
        // Set the orientation in the Odometry message using the yaw angle from the odometry calculations
        odom_msg.twist.twist.linear.x = odometry_.linearVelocity();
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        odom_msg.twist.twist.angular.z = odometry_.angularVelocity();
        
        // Create a TransformStamped message to publish the transform between the odom and base_link frames
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = odometry_.x();
        transform.transform.translation.y = odometry_.y();
        transform.transform.translation.z = 0.0;

        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = std::sin(yaw / 2.0);
        transform.transform.rotation.w = std::cos(yaw / 2.0);

        tf_broadcaster_->sendTransform(transform);

        odom_publisher_->publish(odom_msg);
        
        // Log the updated odometry information for debugging purposes
        RCLCPP_INFO(get_logger(), "Odometry: x=%.3f, y=%.3f, yaw=%.3f",
            odometry_.x(),
            odometry_.y(),
            odometry_.yaw());
    }
    
    // Instance of the DifferentialDriveOdometry class to handle odometry calculations
    DifferentialDriveOdometry odometry_;

    // Subscription to the /joint_states topic to receive wheel position updates
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_subscription_;

    // Publisher for the /odom topic to publish the calculated odometry information
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
    odom_publisher_;

    // TransformBroadcaster to publish the transform between the odom and base_link frames
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Optional variable to store the time of the last received joint state message
    std::optional<rclcpp::Time> last_joint_state_time_;
};

// Main function that initializes the ROS node and starts spinning to process callbacks
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WheelOdometryNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
