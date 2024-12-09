import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import termios
import tty


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        # Crear un publicador en el tópico
        self.publisher_ = self.create_publisher(TwistStamped, '/ackermann_steering_controller/reference', 10)
        self.timer = self.create_timer(0.1, self.publish_message)
        self.current_speed = 0.0
        self.current_steering = 0.0

    def get_key(self):
        # Captura la entrada de teclado
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_message(self):
        # Crear y publicar un mensaje TwistStamped
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.current_speed
        msg.twist.angular.z = self.current_steering
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: speed={self.current_speed}, steering={self.current_steering}")

    def run(self):
        print("Control del vehículo con el teclado:")
        print("w: acelerar, s: frenar")
        print("a: girar izquierda, d: girar derecha")
        print("x: detener")
        print("q: salir")

        while True:
            key = self.get_key()

            if key == 'w':  # Acelerar
                self.current_speed += 0.1
            elif key == 's':  # Frenar
                self.current_speed -= 0.1
            elif key == 'a':  # Girar izquierda
                self.current_steering += 0.1
            elif key == 'd':  # Girar derecha
                self.current_steering -= 0.1
            elif key == 'x':  # Detener
                self.current_speed = 0.0
                self.current_steering = 0.0
            elif key == 'q':  # Salir
                print("Saliendo...")
                break

            # Limitar velocidad y dirección
            self.current_speed = max(-2.0, min(self.current_speed, 2.0))  # Velocidad entre -2.0 y 2.0
            self.current_steering = max(-1.0, min(self.current_steering, 1.0))  # Dirección entre -1.0 y 1.0


def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()

    try:
        keyboard_publisher.run()
    except KeyboardInterrupt:
        print("Interrumpido por el usuario")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        keyboard_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
