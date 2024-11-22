#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "yb_eb/eb_serial.hpp"

using namespace yb_eb;

class BeepNode : public rclcpp::Node
{
public:
    BeepNode() : Node("beep_node")
    {
        // Inicializamos el objeto EBSerial con los parámetros correctos
        board_ = std::make_shared<EBSerial>(1, "/dev/ttyUSB0", true);

        // Suscripción al topic 'beep_command' para recibir la duración del beep
        beep_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
            "beep_command", 10, std::bind(&BeepNode::beep_callback, this, std::placeholders::_1));
        float v = board_->get_version();
        float batery = board_->get_battery_voltage();
        board_->create_receive_threading();
        RCLCPP_INFO(this->get_logger(), "Get versión: %.2f ", v);
        RCLCPP_INFO(this->get_logger(), "Get batery: %.2f ", batery);
        batery = board_->get_battery_voltage();
        RCLCPP_INFO(this->get_logger(), "Get batery: %.2f ", batery);
    }

private:
    // Callback que se ejecuta cuando se recibe un mensaje en 'beep_command'
    void beep_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int on_time = msg->data;
        RCLCPP_INFO(this->get_logger(), "Setting beep on time to: %d ms", on_time);

        // Llama a la función set_beep de la clase EBSerial con el tiempo recibido
        board_->set_beep(on_time);
    }

    std::shared_ptr<EBSerial> board_;                                       // Objeto EBSerial para controlar el beep
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr beep_subscriber_; // Suscriptor al topic 'beep_command'
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BeepNode>());
    rclcpp::shutdown();
    return 0;
}
