#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class PurpleSuitController : public rclcpp::Node
{
public:
    PurpleSuitController()
        : Node("purple_suit_controller"),
          max_time_to_reach_goal_(declare_parameter<double>("max_time_to_reach_goal", 10.0)) // Tiempo máximo para intentar llegar al objetivo (segundos)
    {
        // Declaración del parámetro de umbral de distancia para llegar al objetivo
        goal_reached_threshold_ = this->declare_parameter<double>("goal_reached_threshold", 0.2);

        // Suscripciones
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "/robot1/pose", 10, std::bind(&PurpleSuitController::poseCallback, this, _1));

        goal_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&PurpleSuitController::goalPoseCallback, this, _1));

        // Publicador de comandos de velocidad
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // Temporizador para el bucle de control
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&PurpleSuitController::controlLoop, this));
    }

private:
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        current_pose_ = *msg;
    }

    void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_pose_ = msg->pose;
        RCLCPP_INFO(this->get_logger(), "Nuevo objetivo recibido: (%.2f, %.2f)", goal_pose_->position.x, goal_pose_->position.y);
        goal_start_time_ = this->now();  // Registra el tiempo en que se establece el nuevo objetivo
        goal_reach_timer_active_ = true; // Activa el temporizador para verificar si se alcanza el objetivo
    }

    void controlLoop()
    {
        // Verificar si las poses actuales y objetivo están disponibles
        if (!current_pose_ || !goal_pose_)
        {
            return;
        }

        double robot_x = current_pose_->position.x / 1000;
        double robot_y = current_pose_->position.y / 1000;
        double goal_x = goal_pose_->position.x;
        double goal_y = goal_pose_->position.y;

        double dx = goal_x - robot_x;
        double dy = goal_y - robot_y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Verificar si el robot ha alcanzado el objetivo
        if (distance < goal_reached_threshold_)
        {
            RCLCPP_INFO(this->get_logger(), "Objetivo alcanzado.");
            stopRobot();
            goal_reach_timer_active_ = false; // Desactiva el temporizador ya que el objetivo se alcanzó
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Distancia al objetivo: %.2f", distance);

        // Verificar si el tiempo para alcanzar el objetivo ha excedido el límite
        if (goal_reach_timer_active_ && (this->now() - goal_start_time_).seconds() > max_time_to_reach_goal_)
        {
            RCLCPP_WARN(this->get_logger(), "No fue posible llegar al objetivo en el tiempo permitido. Esperando un nuevo punto.");
            stopRobot();
            goal_reach_timer_active_ = false; // Desactiva el temporizador después de la advertencia
            return;
        }

        // Control de navegación hacia el objetivo
        double target_angle = std::atan2(dy, dx);
        double current_angle = getYawFromQuaternion(current_pose_->orientation);
        double angle_error = normalizeAngle(target_angle - current_angle);

        // Ajustar la velocidad lineal en función de la distancia, pero disminuye si el ángulo de error es significativo
        double linear_velocity = std::min(0.5 * distance, 1.0) * std::exp(-std::abs(angle_error));

        // Ajustar la velocidad angular con una constante de proporcionalidad
        double angular_velocity = std::clamp(1.5 * angle_error, -1.0, 1.0); // Limitar velocidad angular entre -1.0 y 1.0 rad/s

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;

        cmd_vel_publisher_->publish(cmd_vel);
    }

    void stopRobot()
    {
        // Función para detener el robot
        geometry_msgs::msg::Twist stop_msg;
        RCLCPP_INFO(this->get_logger(), "Deteniendo el robot.");
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_vel_publisher_->publish(stop_msg);
    }

    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &q)
    {
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    double normalizeAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    // Variables privadas
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::optional<geometry_msgs::msg::Pose> current_pose_;
    std::optional<geometry_msgs::msg::Pose> goal_pose_;

    double goal_reached_threshold_;        // Umbral para considerar que se llegó al objetivo
    rclcpp::Time goal_start_time_;         // Tiempo en que se estableció el objetivo
    double max_time_to_reach_goal_;        // Tiempo máximo permitido para alcanzar el objetivo en segundos
    bool goal_reach_timer_active_ = false; // Bandera para activar/desactivar el temporizador de objetivo
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurpleSuitController>());
    rclcpp::shutdown();
    return 0;
}
