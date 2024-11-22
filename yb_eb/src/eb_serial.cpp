#include "yb_eb/eb_serial.hpp"

namespace yb_eb
{
    EBSerial::EBSerial(uint8_t car_type, const std::string &com, bool debug)
        : serial(io, com), debug(debug), __CAR_TYPE(car_type)
    {
        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        if (serial.is_open())
        {
            std::cout << "EBSerial Serial Opened! Baudrate=115200" << std::endl;
        }
        else
        {
            std::cerr << "Serial Open Failed!" << std::endl;
        }

        // Inicializa otros parámetros necesarios
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    EBSerial::~EBSerial()
    {
        stop_thread = true;
        if (receive_thread && receive_thread->joinable())
        {
            receive_thread->join();
        }
        serial.close();
        std::cout << "Serial closed!" << std::endl;
    }

    void EBSerial::__parse_data(uint8_t ext_type, const std::vector<uint8_t> &ext_data)
    {
        if (ext_type == FUNC_REPORT_SPEED)
        {
            __vx = static_cast<int16_t>((ext_data[0] | (ext_data[1] << 8))) / 1000.0;
            __vy = static_cast<int16_t>((ext_data[2] | (ext_data[3] << 8))) / 1000.0;
            __vz = static_cast<int16_t>((ext_data[4] | (ext_data[5] << 8))) / 1000.0;
            __battery_voltage = ext_data[6];
        }
        else if (ext_type == FUNC_REPORT_IMU_RAW)
        {
            float gyro_ratio = 1 / 3754.9;
            __gx = static_cast<int16_t>((ext_data[0] | (ext_data[1] << 8))) * gyro_ratio;
            __gy = static_cast<int16_t>((ext_data[2] | (ext_data[3] << 8))) * -gyro_ratio;
            __gz = static_cast<int16_t>((ext_data[4] | (ext_data[5] << 8))) * -gyro_ratio;

            float accel_ratio = 1 / 1671.84;
            __ax = static_cast<int16_t>((ext_data[6] | (ext_data[7] << 8))) * accel_ratio;
            __ay = static_cast<int16_t>((ext_data[8] | (ext_data[9] << 8))) * accel_ratio;
            __az = static_cast<int16_t>((ext_data[10] | (ext_data[11] << 8))) * accel_ratio;

            int mag_ratio = 1;
            __mx = static_cast<int16_t>((ext_data[12] | (ext_data[13] << 8))) * mag_ratio;
            __my = static_cast<int16_t>((ext_data[14] | (ext_data[15] << 8))) * mag_ratio;
            __mz = static_cast<int16_t>((ext_data[16] | (ext_data[17] << 8))) * mag_ratio;
        }
        else if (ext_type == FUNC_REPORT_IMU_ATT)
        {
            __roll = static_cast<int16_t>((ext_data[0] | (ext_data[1] << 8))) / 10000.0;
            __pitch = static_cast<int16_t>((ext_data[2] | (ext_data[3] << 8))) / 10000.0;
            __yaw = static_cast<int16_t>((ext_data[4] | (ext_data[5] << 8))) / 10000.0;
        }
        else if (ext_type == FUNC_REPORT_ENCODER)
        {
            __encoder_m1 = (ext_data[0] | (ext_data[1] << 8) | (ext_data[2] << 16) | (ext_data[3] << 24));
            __encoder_m2 = (ext_data[4] | (ext_data[5] << 8) | (ext_data[6] << 16) | (ext_data[7] << 24));
            __encoder_m3 = (ext_data[8] | (ext_data[9] << 8) | (ext_data[10] << 16) | (ext_data[11] << 24));
            __encoder_m4 = (ext_data[12] | (ext_data[13] << 8) | (ext_data[14] << 16) | (ext_data[15] << 24));
        }
        else if (ext_type == FUNC_UART_SERVO)
        {
            __read_id = ext_data[0];
            __read_val = static_cast<int16_t>((ext_data[1] | (ext_data[2] << 8)));
            if (debug)
            {
                std::cout << "FUNC_UART_SERVO: " << static_cast<int>(__read_id) << " " << __read_val << std::endl;
            }
        }
        else if (ext_type == FUNC_ARM_CTRL)
        {
            for (int i = 0; i < 6; ++i)
            {
                __read_arm[i] = static_cast<int16_t>((ext_data[i * 2] | (ext_data[i * 2 + 1] << 8)));
            }
            __read_arm_ok = 1;
            if (debug)
            {
                std::cout << "FUNC_ARM_CTRL: ";
                for (int val : __read_arm)
                {
                    std::cout << val << " ";
                }
                std::cout << std::endl;
            }
        }
        else if (ext_type == FUNC_VERSION)
        {
            __version_H = ext_data[0];
            __version_L = ext_data[1];
            if (debug)
            {
                std::cout << "FUNC_VERSION: " << static_cast<int>(__version_H) << " " << static_cast<int>(__version_L) << std::endl;
            }
        }
        else if (ext_type == FUNC_SET_MOTOR_PID || ext_type == FUNC_SET_YAW_PID)
        {
            __pid_index = ext_data[0];
            __kp1 = static_cast<int16_t>((ext_data[1] | (ext_data[2] << 8)));
            __ki1 = static_cast<int16_t>((ext_data[3] | (ext_data[4] << 8)));
            __kd1 = static_cast<int16_t>((ext_data[5] | (ext_data[6] << 8)));
            if (debug)
            {
                std::cout << ((ext_type == FUNC_SET_MOTOR_PID) ? "FUNC_SET_MOTOR_PID: " : "FUNC_SET_YAW_PID: ")
                          << static_cast<int>(__pid_index) << " [" << __kp1 << ", " << __ki1 << ", " << __kd1 << "]" << std::endl;
            }
        }
        else if (ext_type == FUNC_ARM_OFFSET)
        {
            __arm_offset_id = ext_data[0];
            __arm_offset_state = ext_data[1];
            if (debug)
            {
                std::cout << "FUNC_ARM_OFFSET: " << static_cast<int>(__arm_offset_id) << " " << static_cast<int>(__arm_offset_state) << std::endl;
            }
        }
        else if (ext_type == FUNC_AKM_DEF_ANGLE)
        {
            uint8_t id = ext_data[0];
            __akm_def_angle = ext_data[1];
            __akm_readed_angle = true;
            if (debug)
            {
                std::cout << "FUNC_AKM_DEF_ANGLE: " << static_cast<int>(id) << " " << static_cast<int>(__akm_def_angle) << std::endl;
            }
        }
    }

    uint8_t EBSerial::read_byte_from_serial()
    {
        uint8_t byte;
        boost::asio::read(serial, boost::asio::buffer(&byte, 1));
        // imprime el byte recibido
        return byte;
    }

    void EBSerial::__receive_data()
    {
        while (!stop_thread)
        {
            try
            {
                if (!serial.is_open())
                {
                    break; // Sal del bucle si el puerto serial está cerrado
                }
                uint8_t head1 = read_byte_from_serial();
                if (head1 == __HEAD)
                {
                    uint8_t head2 = read_byte_from_serial();
                    uint8_t check_sum = 0;
                    uint8_t rx_check_num = 0;

                    if (head2 == (__DEVICE_ID - 1))
                    {
                        uint8_t ext_len = read_byte_from_serial();
                        uint8_t ext_type = read_byte_from_serial();
                        std::vector<uint8_t> ext_data;
                        check_sum = ext_len + ext_type;
                        int data_len = ext_len - 2;

                        for (int i = 0; i < data_len; ++i)
                        {
                            uint8_t value = read_byte_from_serial();
                            ext_data.push_back(value);

                            if (i == data_len - 1)
                            {
                                rx_check_num = value;
                            }
                            else
                            {
                                check_sum += value;
                                // }
                            }

                            if (check_sum % 256 == rx_check_num)
                            {
                                __parse_data(ext_type, ext_data);
                            }
                            else if (debug)
                            {
                                std::cerr << "check sum error: " << static_cast<int>(ext_len) << " "
                                          << static_cast<int>(ext_type) << " [";
                                for (const auto &byte : ext_data)
                                {
                                    std::cerr << static_cast<int>(byte) << " ";
                                }
                                std::cerr << "]" << std::endl;
                            }
                        }
                    }
                }
            }
            catch (const std::exception &e)
            {
                std::cerr << "Error in __receive_data: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "Unknown error in __receive_data" << std::endl;
            }
        }
    }

    void EBSerial::__request_data(uint8_t function, uint8_t param)
    {
        if (debug)
        {
            std::cout << "Request Data Command: ";
        }
        uint8_t function_cast = static_cast<uint8_t>(function & 0xff);
        uint8_t param_cast = static_cast<uint8_t>(param & 0xff);

        std::vector<uint8_t> cmd = {__HEAD, __DEVICE_ID, 0x05, FUNC_REQUEST_DATA, function_cast, param_cast};
        uint8_t checksum = std::accumulate(cmd.begin(), cmd.end(), __COMPLEMENT) & 0xff;
        cmd.push_back(checksum);

        if (debug)
        {
            for (auto byte : cmd)
            {
                std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
            }
            std::cout << std::dec << std::endl; // Restablece el modo decimal
        }

        boost::asio::write(serial, boost::asio::buffer(cmd));
        // std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    void EBSerial::create_receive_threading()
    {
        stop_thread = false;
        try
        {
            receive_thread = std::make_shared<std::thread>(&EBSerial::__receive_data, this);
            if (debug)
            {
                std::cout << "Receive thread created" << std::endl;
            }
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error creating receive thread: " << e.what() << std::endl;
        }
    }

    void EBSerial::stop_receiving()
    {
        stop_thread = true;
    }

    void EBSerial::set_pwm_servo(uint8_t servo_id, float angle)
    {
        if (servo_id < 1 || servo_id > 4)
        {
            if (debug)
            {
                std::cerr << "Invalid servo ID" << std::endl;
            }
            return;
        }

        angle = std::clamp(angle, 0.0f, 180.0f);

        uint8_t angle_val[2];
        int int_angle = static_cast<int>(angle);
        angle_val[0] = static_cast<uint8_t>(int_angle & 0xFF);
        angle_val[1] = static_cast<uint8_t>((int_angle >> 8) & 0xFF);

        std::vector<uint8_t> cmd = {__HEAD, __DEVICE_ID, 0x06, FUNC_PWM_SERVO, servo_id, angle_val[0], angle_val[1]};
        uint8_t checksum = std::accumulate(cmd.begin(), cmd.end(), __COMPLEMENT) & 0xFF;
        cmd.push_back(checksum);

        boost::asio::write(serial, boost::asio::buffer(cmd));

        if (debug)
        {
            std::cout << "PWM servo command sent: [";
            for (const auto &byte : cmd)
            {
                std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
            }
            std::cout << "]" << std::dec << std::endl;
        }
    }

    void EBSerial::set_steering_angle(float angle)
    {
        set_pwm_servo(1, angle);        // Asume que el servo 1 controla la dirección
        steering_angle_command = angle; // Guarda el comando en la variable interna
    }

    void EBSerial::set_motor_speed(float speed)
    {
        set_pwm_servo(3, speed);     // Asume que el servo 3 controla la velocidad del motor
        motor_speed_command = speed; // Guarda el comando en la variable interna
    }

    std::tuple<float, float, float> EBSerial::get_motion_data()
    {
        return {__vx, __vy, __vz};
    }

    float EBSerial::get_battery_voltage()
    {
        return __battery_voltage / 10.0f;
    }
    void EBSerial::set_beep(int on_time)
    {
        if (on_time < 0)
        {
            std::cerr << "Beep input error!" << std::endl;
            return;
        }

        // Convertir el tiempo de encendido a bytes
        uint8_t value[2];
        value[0] = static_cast<uint8_t>(on_time & 0xFF);
        value[1] = static_cast<uint8_t>((on_time >> 8) & 0xFF);

        // Construir el comando
        std::vector<uint8_t> cmd = {__HEAD, __DEVICE_ID, 0x05, FUNC_BEEP, value[0], value[1]};
        uint8_t checksum = std::accumulate(cmd.begin(), cmd.end(), __COMPLEMENT) & 0xFF;
        cmd.push_back(checksum);

        // Enviar el comando por el puerto serial
        boost::asio::write(serial, boost::asio::buffer(cmd));

        if (debug)
        {
            std::cout << "Beep command sent with time: " << on_time << " ms" << std::endl;
        }
    }

    // std::tuple<int32_t, int32_t, int32_t, int32_t> EBSerial::get_motor_encoder()
    // {
    //     return {__encoder_m1, __encoder_m2, __encoder_m3, __encoder_m4};
    // }
    float EBSerial::get_version()
    {
        // Solo realiza la solicitud si la versión no ha sido ya obtenida
        if (__version_H == 0)
        {
            // Solicita la versión usando la función FUNC_VERSION (defínela de acuerdo a tu implementación)
            __request_data(FUNC_VERSION);

            // Intenta obtener la versión durante un tiempo limitado
            for (int i = 0; i < 20; ++i)
            {
                if (__version_H != 0) // Verifica si ya se ha recibido la versión
                {
                    __version = __version_H * 1.0 + __version_L / 10.0;

                    // Si está activado el modo de depuración, muestra el valor de la versión y el intento actual
                    if (debug)
                    {
                        std::cout << "get_version: V" << __version << ", intento: " << i << std::endl;
                    }

                    return __version; // Retorna la versión si se ha recibido exitosamente
                }

                // Espera un breve periodo antes de volver a verificar
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        else
        {
            return __version; // Retorna la versión almacenada si ya se había obtenido previamente
        }

        // Retorna -1 si no se pudo obtener la versión en el tiempo límite
        return -1.0f;
    }
}
// namespace yb_eb