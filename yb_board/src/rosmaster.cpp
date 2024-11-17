#include "yb_board/Rosmaster.hpp"

#include <chrono>
#include <thread>

Rosmaster::Rosmaster(uint8_t car_type, const std::string &com, bool debug)
    : serial(io, com), debug(debug), __CAR_TYPE(car_type)
{
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

    if (debug)
    {
        std::cout << "cmd_delay=0.002s" << std::endl;
    }

    if (serial.is_open())
    {
        std::cout << "Rosmaster Serial Opened! Baudrate=115200" << std::endl;
    }
    else
    {
        std::cerr << "Serial Open Failed!" << std::endl;
    }

    set_uart_servo_torque(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

Rosmaster::~Rosmaster()
{
    stop_thread = true; // Indica al hilo que debe detenerse
    if (receive_thread.joinable())
    {
        receive_thread.join(); // Espera a que el hilo termine
    }
    serial.close();
    std::cout << "serial Close!" << std::endl;
}

void Rosmaster::__parse_data(uint8_t ext_type, const std::vector<uint8_t> &ext_data)
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

uint8_t Rosmaster::read_byte_from_serial()
{
    uint8_t byte;
    boost::asio::read(serial, boost::asio::buffer(&byte, 1));
    // imprime el byte recibido
    return byte;
}

void Rosmaster::__receive_data()
{
    while (!stop_thread)
    {
        try
        {
            if (!serial.is_open())
                break; // Sal del bucle si el puerto serial está cerrado

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
                        }
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

void Rosmaster::__request_data(uint8_t function, uint8_t param)
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

// int Rosmaster::__arm_convert_value(uint8_t s_id, float s_angle)
// {
//     // Implement conversion logic here
//     return -1;
// }

// float Rosmaster::__arm_convert_angle(uint8_t s_id, int s_value)
// {
//     // Implement conversion logic here
//     return -1;
// }

int Rosmaster::__limit_motor_value(int value)
{
    // Implement motor value limiting logic here
    return value;
}

void Rosmaster::create_receive_threading()
{
    stop_thread = false; // Asegúrate de que el `flag` esté en `false` al iniciar el hilo
    try
    {
        receive_thread = std::thread(&Rosmaster::__receive_data, this);
        if (debug)
        {
            std::cout << "----------------create receive threading--------------" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "---create_receive_threading error!--- " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "---create_receive_threading unknown error!---" << std::endl;
    }
}

// void Rosmaster::set_auto_report_state(bool enable, bool forever)
// {
//     // Implement auto report state setting logic here
// }

void Rosmaster::set_beep(int on_time)
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

void Rosmaster::set_pwm_servo(uint8_t servo_id, float angle)
{
    if (debug)
    {
        auto start_time = std::chrono::high_resolution_clock::now();
    }

    // Validar el rango de servo_id
    if (servo_id < 1 || servo_id > 4)
    {
        if (debug)
        {
            std::cerr << "set_pwm_servo input invalid" << std::endl;
        }
        return;
    }

    // Ajustar el rango de `angle`
    if (angle > 180)
    {
        angle = 180;
    }
    else if (angle < 0)
    {
        angle = 0;
    }

    // Convertir el ángulo a bytes
    uint8_t angle_val[2];
    int int_angle = static_cast<int>(angle); // Suponiendo una precisión de 0.1 grados
    angle_val[0] = static_cast<uint8_t>(int_angle & 0xFF);
    angle_val[1] = static_cast<uint8_t>((int_angle >> 8) & 0xFF);

    // Construir el comando
    std::vector<uint8_t> cmd = {__HEAD, __DEVICE_ID, 0x06, FUNC_PWM_SERVO, servo_id, angle_val[0], angle_val[1]};
    uint8_t checksum = std::accumulate(cmd.begin(), cmd.end(), __COMPLEMENT) & 0xFF;
    cmd.push_back(checksum);

    // Enviar el comando por el puerto serial
    boost::asio::write(serial, boost::asio::buffer(cmd));

    if (debug)
    {
        std::cout << "PWM servo command: [";
        for (const auto &byte : cmd)
        {
            std::cout << "0x" << std::hex << static_cast<int>(byte) << " ";
        }
        std::cout << "]" << std::dec << std::endl;
        auto end_time = std::chrono::high_resolution_clock::now();

        // Calcula la duración en microsegundos y muestra el resultado
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        std::cout << "Tiempo de ejecución de set_pwm_servo: " << duration << " microsegundos" << std::endl;
    }
}

void Rosmaster::set_pwm_servo_all(float angle_s1, float angle_s2, float angle_s3, float angle_s4)
{
    // Implement PWM servo all setting logic here
    Rosmaster::set_pwm_servo(1, angle_s1);
    Rosmaster::set_pwm_servo(2, angle_s2);
    Rosmaster::set_pwm_servo(3, angle_s3);
    Rosmaster::set_pwm_servo(4, angle_s4);
}

// void Rosmaster::set_colorful_lamps(uint8_t led_id, uint8_t red, uint8_t green, uint8_t blue)
// {
//     // Implement colorful lamps setting logic here
// }

// void Rosmaster::set_colorful_effect(uint8_t effect, uint8_t speed, uint8_t parm)
// {
//     // Implement colorful effect setting logic here
// }

// void Rosmaster::set_motor(int speed_1, int speed_2, int speed_3, int speed_4)
// {
//     // Implement motor setting logic here
// }

// void Rosmaster::set_car_run(uint8_t state, int speed, bool adjust)
// {
//     // Implement car run setting logic here
// }

// void Rosmaster::set_car_motion(float v_x, float v_y, float v_z)
// {
//     // Implement car motion setting logic here
// }

// void Rosmaster::set_pid_param(float kp, float ki, float kd, bool forever)
// {
//     // Implement PID parameter setting logic here
// }

// void Rosmaster::set_yaw_pid_param(float kp, float ki, float kd, bool forever)
// {
//     // Implement yaw PID parameter setting logic here
// }

// void Rosmaster::set_car_type(uint8_t car_type)
// {
//     // Implement car type setting logic here
// }

// void Rosmaster::set_uart_servo(uint8_t servo_id, int pulse_value, int run_time)
// {
//     // Implement UART servo setting logic here
// }

// void Rosmaster::set_uart_servo_angle(uint8_t s_id, float s_angle, int run_time)
// {
//     // Implement UART servo angle setting logic here
// }

// void Rosmaster::set_uart_servo_id(uint8_t servo_id)
// {
//     // Implement UART servo ID setting logic here
// }

void Rosmaster::set_uart_servo_torque(bool enable)
{
    // Lógica para enviar el comando de torque al servo
    if (debug)
    {
        std::cout << "Setting UART servo torque to " << (enable ? "enabled" : "disabled") << std::endl;
    }
}

// void Rosmaster::set_uart_servo_ctrl_enable(bool enable)
// {
//     // Implement UART servo control enable setting logic here
// }

// void Rosmaster::set_uart_servo_angle_array(std::vector<float> angle_s, int run_time)
// {
//     // Implement UART servo angle array setting logic here
// }

// void Rosmaster::set_uart_servo_offset(uint8_t servo_id)
// {
//     // Implement UART servo offset setting logic here
// }

// void Rosmaster::set_akm_default_angle(float angle, bool forever)
// {
//     // Implement AKM default angle setting logic here
// }

// void Rosmaster::set_akm_steering_angle(float angle)
// {
//     // Implement AKM steering angle setting logic here
// }

void Rosmaster::reset_flash_value()
{
    // Implement flash value reset logic here
}

void Rosmaster::clear_auto_report_data()
{
    // Implement auto report data clearing logic here
}

float Rosmaster::get_akm_default_angle()
{
    // Implement AKM default angle getting logic here
    return __akm_def_angle;
}

// std::pair<int, int> Rosmaster::get_uart_servo_value(uint8_t servo_id)
// {
//     // Implement UART servo value getting logic here
//     return {0, 0};
// }

// float Rosmaster::get_uart_servo_angle(uint8_t s_id)
// {
//     // Implement UART servo angle getting logic here
//     return 0.0f;
// }

std::vector<float> Rosmaster::get_uart_servo_angle_array()
{
    // Implement UART servo angle array getting logic here
    return std::vector<float>(6, 0.0f);
}

std::tuple<float, float, float> Rosmaster::get_accelerometer_data()
{
    // Implement accelerometer data getting logic here
    return {0.0f, 0.0f, 0.0f};
}

std::tuple<float, float, float> Rosmaster::get_gyroscope_data()
{
    // Implement gyroscope data getting logic here
    return {0.0f, 0.0f, 0.0f};
}

std::tuple<float, float, float> Rosmaster::get_magnetometer_data()
{
    // Implement magnetometer data getting logic here
    return {0.0f, 0.0f, 0.0f};
}

// std::tuple<float, float, float> Rosmaster::get_imu_attitude_data(bool ToAngle)
// {
//     // Implement IMU attitude data getting logic here
//     return {0.0f, 0.0f, 0.0f};
// }

std::tuple<float, float, float> Rosmaster::get_motion_data()
{
    // Implement motion data getting logic here
    return {__vx, __vy, __vz};
}

float Rosmaster::get_battery_voltage()
{
    // Implement battery voltage getting logic here
    return __battery_voltage / 10.0f;
}

std::tuple<int32_t, int32_t, int32_t, int32_t> Rosmaster::get_motor_encoder()
{
    // Implement motor encoder data getting logic here
    return {__encoder_m1, __encoder_m2, __encoder_m3, __encoder_m4};
}

std::vector<float> Rosmaster::get_motion_pid()
{
    // Implement motion PID getting logic here
    return {-1.0f, -1.0f, -1.0f};
}

std::vector<float> Rosmaster::get_yaw_pid()
{
    // Implement yaw PID getting logic here
    return {-1.0f, -1.0f, -1.0f};
}

float Rosmaster::get_version()
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