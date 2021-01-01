/**
 * @file b3m_joint.hpp
 * @brief B3Mジョイントクラスの定義
 * @author nomumu
 * @date 2020.12.29
 */
#ifndef B3M_JOINT_HPP_
#define B3M_JOINT_HPP_

/**
 * @file b3m_joint.hpp
 * @brief B3Mジョイントクラスの定義
 * @author nomumu
 * @date 2020.12.29
 */
#include <cstdint>
#include <cstdio>
#include <string>
#include "b3m_control/b3m_servo.hpp"

/**
 * B3Mジョイントタイプ列挙型
 */
typedef enum {
    enB3mJointType_Position,
    enB3MJointType_Velocity,
    enB3MJointType_Effort
} enB3mJointType;

/**
 * B3Mジョイントクラス
 */
class B3mJoint : public B3mServo
{
public:
    B3mJoint( uint8_t id, enB3mJointType type, std::string name, bool reverse );
    ~B3mJoint();
    static EN_OPTIONS conv_type( enB3mJointType type );

    void set_pos(double value);
    void set_vel(double value);
    void set_eff(double value);

    enB3mJointType get_type(void);
    std::string get_name(void);
    double get_pos(void);
    double get_vel(void);
    double get_eff(void);
    double get_cmd(void);
    double* get_pos_addr(void);
    double* get_vel_addr(void);
    double* get_eff_addr(void);
    double* get_cmd_addr(void);

private:
    enB3mJointType  type_;
    std::string     name_;
    bool            reverse_;
    double          pos_;   // [rad]
    double          vel_;   // [rad/sec]
    double          eff_;   // [Nm]
    double          cmd_;   // HardwareInterface command data
};

#endif  // B3M_JOINT_HPP_
