#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"

class TwistStampedNode : public rclcpp::Node
{
public:
    TwistStampedNode() : Node("twist_stamped_node")
    {
        // Suscriptor al topic /cmd_vel
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&TwistStampedNode::cmdVelCallback, this, std::placeholders::_1));

        // Publicador al topic /ackermann_steering_controller/reference
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/ackermann_steering_controller/reference",
            10);

        RCLCPP_INFO(this->get_logger(), "Node started");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Crear un mensaje TwistStamped
        geometry_msgs::msg::TwistStamped twist_stamped_msg;

        // Llenar la cabecera
        twist_stamped_msg.header.stamp = this->get_clock()->now(); // Sello de tiempo actual
        twist_stamped_msg.header.frame_id = "base_link";

        // Copiar los datos de Twist
        twist_stamped_msg.twist = *msg;

        // Publicar el mensaje
        publisher_->publish(twist_stamped_msg);

        // RCLCPP_INFO(this->get_logger(), "Published TwistStamped message");
        RCLCPP_DEBUG(this->get_logger(), "Velocity: %f, Angular: %f", msg->linear.x, msg->angular.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwistStampedNode>());
    rclcpp::shutdown();
    return 0;
}
