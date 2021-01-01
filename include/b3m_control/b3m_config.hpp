/**
 * @file b3m_config.hpp
 * @brief B3M設定クラスの定義
 * @author nomumu
 * @date 2020.12.29
 */
#ifndef B3M_CONFIG_HPP_
#define B3M_CONFIG_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <string>


/* 定数・マクロ定義 */
#define     KEY_B3M_CONFIG      ("b3m_config")
#define     KEY_PORTNAME        ("/port")
#define     KEY_BAUDRATE        ("/baudrate")
#define     KEY_JOINTS          ("/joints")
#define     KEY_PARAM_ID        ("/id")
#define     KEY_PARAM_TYPE      ("/type")
#define     KEY_PARAM_REVERSE   ("/reverse")

#define     TYPE_POSITION       ("Position")
#define     TYPE_VELOCITY       ("Velocity")
#define     TYPE_EFFORT         ("Effort")

/**
 * B3Mジョイントタイプ列挙型
 */
typedef enum {
    enB3mJointConfigType_Position   = 0,
    enB3mJointConfigType_Velocity   = 1,
    enB3mJointConfigType_Effort     = 2
} enB3mJointConfigType;

/**
 * B3Mジョイント設定構造体
 */
typedef struct {
    std::string             name_;
    uint8_t                 id_;
    enB3mJointConfigType    type_;
    bool                    reverse_;
} stB3mJointConfig;

/**
 * B3M設定クラス
 */
class B3mConfig
{
public:
    B3mConfig( ros::NodeHandle handle );
    ~B3mConfig();
    bool load( void );
    std::string get_portname( void ){ return port_; }
    uint32_t get_baudrate( void ){ return baudrate_; }
    std::vector<stB3mJointConfig> get_joint_config( void ){ return joints_; }

private:
    ros::NodeHandle nh_;
    std::string port_;
    uint32_t baudrate_;
    std::vector<stB3mJointConfig> joints_;

    bool load_joints( void );
    std::string load_portname( void );
    uint32_t load_baudrate( void );
    bool load_param( void );
};


#endif  // B3M_CONFIG_HPP_