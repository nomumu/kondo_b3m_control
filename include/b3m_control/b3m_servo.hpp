/**
 * @file b3m_servo.hpp
 * @brief B3Mサーボクラスの定義
 * @author nomumu
 * @date 2020.12.29
 */
#ifndef B3M_SERVO_HPP_
#define B3M_SERVO_HPP_

#include <stdint.h>
#include "b3m_control/b3m_design.hpp"

#define ADDR_SERVO_CURRENT_ALL  (ADDR_SERVO_CURRENT_POSITION)
#define SIZE_SERVO_CURRENT_ALL  (0x12)

/**
 * B3Mサーボクラス
 */
class B3mServo
{
public:
    B3mServo(uint8_t id, EN_OPTIONS control_type);
    ~B3mServo();

    uint8_t get_id(void);
    EN_OPTIONS get_run_state(void);
    EN_OPTIONS get_control_type(void);

    void set_run_state(EN_OPTIONS state);
    void set_control_type(EN_OPTIONS control_type);

    void set_sys_error(EN_SYS_ERRORS error);
    void set_motor_error(EN_MOTOR_ERRORS error);
    void set_uart_error(EN_UART_ERRORS error);
    void set_cmd_error(EN_COMMAND_ERRORS error);

    void set_current_pos( int16_t value );
    void set_current_vel( int16_t value );
    void set_current_trq( int16_t value );
    void parse_current_all( uint8_t* data, uint8_t length );
    void set_desired( int16_t value );

    int16_t get_current_pos( void );
    int16_t get_current_vel( void );
    int16_t get_current_trq( void );
    int16_t get_desired( void );

private:
    uint8_t             id_;
    EN_OPTIONS          run_state_;
    EN_OPTIONS          control_type_;
    int16_t             current_pos_;
    int16_t             current_vel_;
    int16_t             current_trq_;
    int16_t             desired_;

    EN_SYS_ERRORS       sys_error_;
    EN_MOTOR_ERRORS     motor_error_;
    EN_UART_ERRORS      uart_error_;
    EN_COMMAND_ERRORS   cmd_error_;

};

#endif  // B3M_SERVO_HPP_