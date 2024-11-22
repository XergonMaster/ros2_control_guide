#ifndef EBSERIAL_HPP
#define EBSERIAL_HPP

#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <numeric>
#include <memory>
#include <tuple>

namespace yb_eb
{
    class EBSerial
    {
    public:
        EBSerial(uint8_t car_type = 1, const std::string &com = "/dev/myserial", bool debug = false);
        ~EBSerial();

        void create_receive_threading();
        void stop_receiving();
        void set_pwm_servo(uint8_t servo_id, float angle);
        void set_pwm_servo_all(float angle_s1, float angle_s2, float angle_s3, float angle_s4);
        void set_beep(int on_time);

        // Métodos de lectura para los estados del hardware
        std::tuple<float, float, float> get_motion_data();
        float get_battery_voltage();
        std::tuple<int32_t, int32_t, int32_t, int32_t> get_motor_encoder();

        // Métodos de comandos para mover los actuadores
        void set_steering_angle(float angle);
        void set_motor_speed(float speed);

        // Otros métodos que pueden ser necesarios
        void reset_flash_value();
        void clear_auto_report_data();
        float get_version();

    private:
        void __receive_data();
        void __parse_data(uint8_t ext_type, const std::vector<uint8_t> &ext_data);
        uint8_t read_byte_from_serial();
        void __request_data(uint8_t function, uint8_t param = 0);
        int __limit_motor_value(int value);

        std::shared_ptr<std::thread> receive_thread;
        bool stop_thread = false;

        boost::asio::io_service io;
        boost::asio::serial_port serial;
        bool debug;

        uint8_t __HEAD = 0xFF;
        uint8_t __DEVICE_ID = 0xFC;
        uint8_t __COMPLEMENT = 257 - __DEVICE_ID;
        uint8_t __CAR_TYPE;
        uint8_t __CAR_ADJUST = 0x80;

        // Define constants for function codes
        const uint8_t FUNC_AUTO_REPORT = 0x01;
        const uint8_t FUNC_BEEP = 0x02;
        const uint8_t FUNC_PWM_SERVO = 0x03;
        const uint8_t FUNC_PWM_SERVO_ALL = 0x04;
        const uint8_t FUNC_RGB = 0x05;
        const uint8_t FUNC_RGB_EFFECT = 0x06;
        const uint8_t FUNC_REPORT_SPEED = 0x0A;
        const uint8_t FUNC_REPORT_IMU_RAW = 0x0B;
        const uint8_t FUNC_REPORT_IMU_ATT = 0x0C;
        const uint8_t FUNC_REPORT_ENCODER = 0x0D;
        const uint8_t FUNC_MOTOR = 0x10;
        const uint8_t FUNC_CAR_RUN = 0x11;
        const uint8_t FUNC_MOTION = 0x12;
        const uint8_t FUNC_SET_MOTOR_PID = 0x13;
        const uint8_t FUNC_SET_YAW_PID = 0x14;
        const uint8_t FUNC_SET_CAR_TYPE = 0x15;
        const uint8_t FUNC_UART_SERVO = 0x20;
        const uint8_t FUNC_UART_SERVO_ID = 0x21;
        const uint8_t FUNC_UART_SERVO_TORQUE = 0x22;
        const uint8_t FUNC_ARM_CTRL = 0x23;
        const uint8_t FUNC_ARM_OFFSET = 0x24;
        const uint8_t FUNC_AKM_DEF_ANGLE = 0x30;
        const uint8_t FUNC_AKM_STEER_ANGLE = 0x31;
        const uint8_t FUNC_REQUEST_DATA = 0x50;
        const uint8_t FUNC_VERSION = 0x51;
        const uint8_t FUNC_RESET_FLASH = 0xA0;

        // Define constants for car types
        const uint8_t CARTYPE_X3 = 0x01;
        const uint8_t CARTYPE_X3_PLUS = 0x02;
        const uint8_t CARTYPE_X1 = 0x04;
        const uint8_t CARTYPE_R2 = 0x05;

        // Variables de comandos para la interfaz de hardware
        float steering_angle_command = 0.0;
        float motor_speed_command = 0.0;

        // Define variables for sensor data
        float __ax, __ay, __az;
        float __gx, __gy, __gz;
        float __mx, __my, __mz;
        float __vx, __vy, __vz;
        float __yaw, __roll, __pitch;
        int32_t __encoder_m1, __encoder_m2, __encoder_m3, __encoder_m4;
        uint8_t __read_id;
        int16_t __read_val;
        uint8_t __read_arm_ok;
        int16_t __read_arm[6];
        uint8_t __version_H, __version_L;
        float __version;
        uint8_t __pid_index;
        int16_t __kp1, __ki1, __kd1;
        uint8_t __arm_offset_state, __arm_offset_id;
        bool __arm_ctrl_enable;
        uint8_t __battery_voltage;
        uint8_t __akm_def_angle;
        bool __akm_readed_angle;
        uint8_t __AKM_SERVO_ID = 0x01;
    };
}
#endif // ROSMASTER_HPP
