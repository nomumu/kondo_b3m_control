/**
 * @file b3m_hardware.hpp
 * @brief B3Mハードウェアインタフェースクラスの定義
 * @author nomumu
 * @date 2020.12.29
 */
#ifndef B3M_HARDWARE_HPP_
#define B3M_HARDWARE_HPP_

#include <stdint.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include "b3m_control/b3m_design.hpp"
#include "b3m_control/b3m_joint.hpp"
#include "b3m_control/b3m_port.hpp"

/**
 * B3Mハードウェアインタフェースクラス
 */
class B3mHardware : public hardware_interface::RobotHW
{
public:
    B3mHardware( std::string dev_name, uint32_t baudrate, ros::NodeHandle handle );
    ~B3mHardware();
    bool is_init(void);
    bool regist_joint( uint8_t id, enB3mJointType type, std::string name, bool reverse );
    void regist_interface(void);
    void update(void);
    void torque_on(void);
    void torque_free(void);
    void set_torque(bool torque);
    void reset(void);

private:
    B3mPort                                     port_;      // B3M通信ポート情報
    std::vector<B3mJoint>                       joint_;     // ジョイント情報

    ros::Time                                   updt_time_; // アップデート時刻
    ros::Duration                               updt_d_;    // アップデート間隔

    controller_manager::ControllerManager       manager_;   // コントローラマネージャ
    hardware_interface::JointStateInterface     stat_if_;   // ステータスインタフェース
    hardware_interface::PositionJointInterface  pos_if_;    // 位置指令インタフェース
    hardware_interface::VelocityJointInterface  vel_if_;    // 速度指令インタフェース
    hardware_interface::EffortJointInterface    eff_if_;    // トルク指令インタフェース
};

#endif  // B3M_HARDWARE_HPP_