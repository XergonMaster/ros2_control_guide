#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <optional>

using std::placeholders::_1;

class OdometryPublisher : public rclcpp::Node
{
public:
    OdometryPublisher()
        : Node("odometry_publisher")
    {
        // Suscriptor para recibir la pose del robot
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/robot1/pose", 10, std::bind(&OdometryPublisher::poseCallback, this, _1));

        // Publicador para enviar el mensaje de odometría
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/odom", 10);

        // Inicializar el TransformBroadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Configurar un temporizador para la publicación periódica de la odometría
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&OdometryPublisher::publishOdometry, this));
        RCLCPP_INFO(this->get_logger(), "Odometry publisher node has been started. v1.0");
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;

        // Convertir la posición de milímetros a metros
        current_pose_->position.x /= 1000.0;
        current_pose_->position.y /= 1000.0;
        current_pose_->position.z /= 1000.0;
    }

    void publishOdometry()
    {
        if (!current_pose_)
        {
            return;
        }

        auto odom_msg = nav_msgs::msg::Odometry();

        // Poblar el mensaje de odometría
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        odom_msg.pose.pose = *current_pose_;

        // Para simplificar, establecer la velocidad en cero (asume actualizaciones estáticas de la pose)
        odom_msg.twist.twist.linear.x = 0.0;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = 0.0;

        odometry_publisher_->publish(odom_msg);

        // Publicar la transformación entre odom y base_link
        publishTransform();
    }

    void publishTransform()
    {
        if (!current_pose_)
        {
            RCLCPP_INFO(this->get_logger(), "tf refused");

            return;
        }

        geometry_msgs::msg::TransformStamped transform_stamped;

        // Poblar los datos de la transformación
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_footprint";

        transform_stamped.transform.translation.x = current_pose_->position.x;
        transform_stamped.transform.translation.y = current_pose_->position.y;
        transform_stamped.transform.translation.z = current_pose_->position.z;

        tf2::Quaternion q(
            current_pose_->orientation.x,
            current_pose_->orientation.y,
            current_pose_->orientation.z,
            current_pose_->orientation.w);

        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        // Publicar la transformación
        tf_broadcaster_->sendTransform(transform_stamped);
    }

    // Declaración de los miembros privados de la clase
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::optional<geometry_msgs::msg::Pose> current_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryPublisher>());
    rclcpp::shutdown();
    return 0;
}
