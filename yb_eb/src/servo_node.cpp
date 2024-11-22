#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "yb_eb/eb_serial.hpp"

using namespace yb_eb;

class ServoNode : public rclcpp::Node
{
public:
    ServoNode() : Node("servo_node")
    {
        // Inicializamos el objeto EBSerial con los parámetros correctos
        board_ = std::make_shared<EBSerial>(1, "/dev/myserial", true);

        // Suscripción al topic 'servo_command' para recibir el ángulo del servo
        servo_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "servo_command", 10, std::bind(&ServoNode::servo_callback, this, std::placeholders::_1));

        // Suscripción al topic 'select_servo' para seleccionar qué servo controlar
        select_servo_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "select_servo", 10, std::bind(&ServoNode::select_servo_callback, this, std::placeholders::_1));

        board_->create_receive_threading();
        selected_servo_ = 1; // ID del servo por defecto
    }

private:
    // Callback que se ejecuta cuando se recibe un mensaje en 'servo_command'
    void servo_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        float angle = msg->data;
        RCLCPP_INFO(this->get_logger(), "Setting servo %d angle to: %.2f degrees", selected_servo_, angle);

        // Limitar el ángulo al rango permitido [0, 180]
        if (angle < 0)
            angle = 0;
        if (angle > 180)
            angle = 180;

        // Llama a la función set_pwm_servo de la clase EBSerial con el ángulo y el ID del servo seleccionado
        board_->set_pwm_servo(selected_servo_, angle);
    }

    // Callback que se ejecuta cuando se recibe un mensaje en 'select_servo'
    void select_servo_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int servo_id = msg->data;

        // Validar que el ID del servo esté en el rango permitido
        if (servo_id >= 1 && servo_id <= 4)
        {
            selected_servo_ = servo_id;
            RCLCPP_INFO(this->get_logger(), "Servo seleccionado: %d", selected_servo_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "ID de servo inválido: %d. Selección no cambiada.", servo_id);
        }
    }

    std::shared_ptr<EBSerial> board_;                                               // Objeto EBSerial para controlar los servos
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr servo_subscriber_;        // Suscriptor al topic 'servo_command'
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr select_servo_subscriber_; // Suscriptor al topic 'select_servo'
    int selected_servo_;                                                            // Variable para almacenar el ID del servo seleccionado
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoNode>());
    rclcpp::shutdown();
    return 0;
}
