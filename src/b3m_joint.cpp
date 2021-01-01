/**
 * @file b3m_joint.cpp
 * @brief B3Mジョイントクラスの実装
 * @author nomumu
 * @date 2020.12.29
 */

#include <cstdio>
#include <cstring>
#include "b3m_control/b3m_joint.hpp"

/**
 * B3Mジョイント コンストラクタ
 * @brief B3Mジョイントクラスを初期化する
 * @param id：サーボID
 * @param type：ジョイントタイプ
 * @param name：ジョイント名
 * @param reverse：回転方向の反転設定
 */
B3mJoint::B3mJoint( uint8_t id, enB3mJointType type, std::string name, bool reverse )
 : B3mServo( id, conv_type(type) )
{
    type_   = type;
    if( name.length() == 0 ){
        name = std::string("joint") + std::to_string(id);
    }
    name_   = name;
    reverse_= reverse;
    pos_    = 0.0;
    vel_    = 0.0;
    eff_    = 0.0;
    cmd_    = 0.0;
}

/**
 * B3Mジョイント デストラクタ
 * @brief B3Mジョイントクラスを解放する
 * @param なし
 */
B3mJoint::~B3mJoint()
{
}

/**
 * B3Mジョイント ジョイント種別変換処理
 * @brief ジョイント種別をサーボクラスの定義へ変換する
 * @param type：ジョイント種別
 * @return サーボ制御タイプ
 */
EN_OPTIONS B3mJoint::conv_type( enB3mJointType type )
{
    EN_OPTIONS result;

    switch( type ){
    case enB3mJointType_Position:
        result = enOptions_ControlPosition;
        break;
    case enB3MJointType_Velocity:
        result = enOptions_ControlVelocity;
        break;
    case enB3MJointType_Effort:
        result = enOptions_ControlTorque;
        break;
    default:
        throw "Illegal joint type\n";
        break;
    }
    return result;
}

/**
 * B3Mジョイント 位置設定処理
 * @brief ジョイント位置を格納する
 * @param value：位置値[rad]
 */
void B3mJoint::set_pos(double value)
{
    pos_ = reverse_?-value:value;
}

/**
 * B3Mジョイント 速度設定処理
 * @brief ジョイント速度を格納する
 * @param value：速度値[rad/sec]
 */
void B3mJoint::set_vel(double value)
{
    vel_ = reverse_?-value:value;
}

/**
 * B3Mジョイント トルク設定処理
 * @brief ジョイントトルクを格納する
 * @param value：トルク値[Nm]
 */
void B3mJoint::set_eff(double value)
{
    eff_ = reverse_?-value:value;
}

/**
 * B3Mジョイント ジョイントタイプ取得処理
 * @brief ジョイントタイプを返却する
 * @param なし
 * @return ジョイントタイプ
 */
enB3mJointType B3mJoint::get_type(void)
{
    return type_;
}

/**
 * B3Mジョイント ジョイント名取得処理
 * @brief ジョイント名を返却する
 * @param なし
 * @return ジョイント名
 */
std::string B3mJoint::get_name(void)
{
    return name_;
}

/**
 * B3Mジョイント 位置取得処理
 * @brief ジョイント位置を返却する
 * @param なし
 * @return 位置値[rad]
 */
double B3mJoint::get_pos(void)
{
    return pos_;
}

/**
 * B3Mジョイント 速度取得処理
 * @brief ジョイント速度を返却する
 * @param なし
 * @return 速度値[rad/sec]
 */
double B3mJoint::get_vel(void)
{
    return vel_;
}

/**
 * B3Mジョイント トルク取得処理
 * @brief ジョイントトルクを返却する
 * @param なし
 * @return トルク値[Nm]
 */
double B3mJoint::get_eff(void)
{
    return eff_;
}

/**
 * B3Mジョイント HWIfコマンド値取得処理
 * @brief ジョイントHWIfコマンド値を返却する
 * @param なし
 * @return コマンド値
 */
double B3mJoint::get_cmd(void)
{
    return reverse_?-cmd_:cmd_;
}

/**
 * B3Mジョイント 位置データアドレス取得処理
 * @brief ジョイント位置格納アドレスを返却する
 * @param なし
 * @return 位置値格納アドレス
 */
double* B3mJoint::get_pos_addr(void)
{
    return &pos_;
}

/**
 * B3Mジョイント 速度データアドレス取得処理
 * @brief ジョイント速度格納アドレスを返却する
 * @param なし
 * @return 速度値格納アドレス
 */
double* B3mJoint::get_vel_addr(void)
{
    return &vel_;
}

/**
 * B3Mジョイント トルクデータアドレス取得処理
 * @brief ジョイントトルク格納アドレスを返却する
 * @param なし
 * @return トルク値格納アドレス
 */
double* B3mJoint::get_eff_addr(void)
{
    return &eff_;
}

/**
 * B3Mジョイント HWIfコマンドデータアドレス取得処理
 * @brief ジョイントHWIfコマンド格納アドレスを返却する
 * @param なし
 * @return コマンド値格納アドレス
 */
double* B3mJoint::get_cmd_addr(void)
{
    return &cmd_;
}
