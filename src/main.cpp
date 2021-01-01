/**
 * @file main.cpp
 * @brief メイン処理
 * @author nomumu
 * @date 2020.12.29
 */

#include <signal.h>
#include <stdint.h>
#include "b3m_control/b3m_hardware.hpp"
#include "b3m_control/b3m_config.hpp"


/* 定数・マクロ定義 */
#define UPDATE_RATE_HZ  (250)       // メインループの周期
#define B3M_BAUDRATE    (1500000)   // B3M通信ポートの速度

/**
 * シグナルハンドラ
 * @brief シグナル受付処理する
 * @param sig：シグナル
 * @note 何らかシグナルを受け取ったら終了する
 */
void SigintHandler( int sig )
{
    ros::shutdown();
}

/**
 * メイン処理
 * @brief メインループ
 * @param argc,argv：引値
 * @return 実行結果
 */
int main( int argc, char* argv[] )
{
    // ノードの初期化
    ros::init( argc, argv, ros::this_node::getName(), ros::init_options::NoSigintHandler );
    ros::NodeHandle nh;
    signal(SIGINT, SigintHandler);

    // 設定読み込み処理
    B3mConfig config(nh);
    if( !config.load() ){
        ROS_ERROR("Config load failed.");
        return -1;
    }

    // B3Mハードウェアインタフェースの初期化
    B3mHardware b3m(config.get_portname(),config.get_baudrate(),nh);
    if( !b3m.is_init() ){
        ROS_ERROR("Port init failed.");
        return -1;
    }

    // ジョイント登録
    std::vector<stB3mJointConfig> joints = config.get_joint_config();
    for(std::vector<stB3mJointConfig>::iterator itr=joints.begin() ; itr!=joints.end() ; ++itr){
        uint8_t         reg_id;
        enB3mJointType  reg_type;
        std::string     reg_name;
        bool            reg_reverse;
        reg_id = itr->id_;
        switch( itr->type_ ){
        case enB3mJointConfigType_Velocity:
            reg_type = enB3MJointType_Velocity;
            break;
        case enB3mJointConfigType_Effort:
            reg_type = enB3MJointType_Effort;
            break;
        case enB3mJointConfigType_Position:
        default:
            reg_type = enB3mJointType_Position;
            break;
        }
        reg_name = itr->name_;
        reg_reverse= itr->reverse_;
        b3m.regist_joint(reg_id,reg_type,reg_name,reg_reverse); // 登録
    }
    b3m.regist_interface();
    b3m.reset();

    // トルクON
    b3m.torque_on();

    // spin開始
    ros::AsyncSpinner spinner(4);
    spinner.start();
 
    // メインループ処理
    ros::Rate r(UPDATE_RATE_HZ);
    while(ros::ok()){
        b3m.update();   // 更新処理
        r.sleep();
    }

    // 終了処理
    spinner.stop();
    ros::Duration(1).sleep();
    b3m.torque_free();

    return 0;
}