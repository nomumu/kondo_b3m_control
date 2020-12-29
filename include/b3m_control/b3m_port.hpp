/**
 * @file b3m_port.hpp
 * @brief B3Mポートクラスの定義
 * @author nomumu
 * @date 2020.12.29
 */
#ifndef B3M_PORT_HPP_
#define B3M_PORT_HPP_

#include <string>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include "b3m_design.hpp"
#include "b3m_servo.hpp"

/**
 * B3M通信ポートクラス
 */
class B3mPort
{
public:
    B3mPort(std::string dev_name, uint32_t baudrate);
    ~B3mPort();
    bool is_init(void);

    void load(B3mServo &servo);
    void save(B3mServo &servo);
    void reset(B3mServo &servo);
    void clean(void);

    void readCurrent(B3mServo &servo);
    void writeDesired(B3mServo &servo);
    void writeRunMode(B3mServo &servo);

private:
    bool initialize(void);
    speed_t get_baudtype(uint32_t baudrate);
    void create_multi_packet();
    void create_single_packet();
    uint8_t calc_sum(uint8_t* data, uint8_t length);
    bool read_port(uint8_t* data, uint8_t length, uint8_t timeout_msec=30);
    bool write_port(uint8_t* data, uint8_t length);
    void wait(uint16_t msec);

    int             fd_;
    std::string     dev_name_;
    uint32_t        baudrate_;
    bool            init_result_;
};

#endif  // B3M_PORT_HPP_
