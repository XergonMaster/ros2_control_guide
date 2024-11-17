#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "yb_board/Rosmaster.hpp"

class ServoNode : public rclcpp::Node
{
public:
    ServoNode() : Node("servo_node")
    {
        // Inicializamos el objeto Rosmaster con los parámetros correctos
        board_ = std::make_shared<Rosmaster>(1, "/dev/myserial", true);

        // Suscripción al topic 'servo_command' para recibir el ángulo del servo
        servo_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "servo_command", 10, std::bind(&ServoNode::servo_callback, this, std::placeholders::_1));
        board_->create_receive_threading();
    }

private:
    // Callback que se ejecuta cuando se recibe un mensaje en 'servo_command'
    void servo_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        float angle = msg->data;
        RCLCPP_INFO(this->get_logger(), "Setting servo angle to: %.2f degrees", angle);
        // limitamos el angulo
        if (angle < 0)
            angle = 0;
        if (angle > 180)
            angle = 180;
        // Llama a la función set_pwm_servo de la clase Rosmaster con el ángulo recibido
        board_->set_pwm_servo(1, angle);
        board_->set_pwm_servo(2, 180 - angle);
    }

    std::shared_ptr<Rosmaster> board_;                                       // Objeto Rosmaster para controlar el servo
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr servo_subscriber_; // Suscriptor al topic 'servo_command'
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoNode>());
    rclcpp::shutdown();
    return 0;
}