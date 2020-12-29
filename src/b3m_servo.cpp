/**
 * @file b3m_servo.cpp
 * @brief B3Mサーボクラスの実装
 * @author nomumu
 * @date 2020.12.29
 */

#include <stdio.h>
#include "b3m_control/b3m_servo.hpp"

/**
 * B3Mサーボ コンストラクタ
 * @brief B3Mサーボクラスを初期化する
 * @param id：サーボID
 * @param control_type：サーボ制御種別
 */
B3mServo::B3mServo(uint8_t id, EN_OPTIONS control_type)
{
    id_             = id;
    run_state_      = enOptions_RunFree;

    switch(control_type){
    case enOptions_ControlPosition: // 位置指令
    case enOptions_ControlVelocity: // 速度指令
    case enOptions_ControlTorque:   // トルク指令
        control_type_   = control_type;
        break;
    default:
        control_type_   = enOptions_ControlPosition;
        break;
    }
}

/**
 * B3Mサーボ デストラクタ
 * @brief B3Mサーボクラスを解放する
 * @param なし
 */
B3mServo::~B3mServo()
{
}

/**
 * B3Mサーボ ID取得処理
 * @brief ID設定を返却する
 * @param なし
 * @return サーボID
 */
uint8_t B3mServo::get_id(void)
{
    return id_;
}

/**
 * B3Mサーボ 動作状態取得処理
 * @brief 動作状態を返却する
 * @param なし
 * @return 動作状態
 */
EN_OPTIONS B3mServo::get_run_state(void)
{
    return run_state_;
}

/**
 * B3Mサーボ 制御タイプ取得処理
 * @brief 制御タイプをを返却する
 * @param なし
 * @return 制御タイプ
 */
EN_OPTIONS B3mServo::get_control_type(void)
{
    return control_type_;
}

/* Private */
/**
 * B3Mサーボ RUN状態設定
 * @brief B3MサーボのRUN状態を設定する
 * @param state：RUN状態
 */
void B3mServo::set_run_state(EN_OPTIONS state)
{
    run_state_ = state;
    return;
}

/**
 * B3Mサーボ サーボ制御種別設定
 * @brief B3Mサーボの制御種別を設定する
 * @param control_type：制御種別
 */
void B3mServo::set_control_type(EN_OPTIONS control_type)
{
    control_type_ = control_type;
    return;
}

/**
 * B3Mサーボ システムエラー設定
 * @brief B3Mサーボのシステムエラーを設定する
 * @param error：エラー
 */
void B3mServo::set_sys_error(EN_SYS_ERRORS error)
{
    sys_error_ = error;
    return;
}

/**
 * B3Mサーボ モータエラー設定
 * @brief B3Mサーボのモータエラーを設定する
 * @param error：エラー
 */
void B3mServo::set_motor_error(EN_MOTOR_ERRORS error)
{
    motor_error_ = error;
    return;
}

/**
 * B3Mサーボ UARTエラー設定
 * @brief B3MサーボのUARTエラーを設定する
 * @param error：エラー
 */
void B3mServo::set_uart_error(EN_UART_ERRORS error)
{
    uart_error_ = error;
    return;
}

/**
 * B3Mサーボ コマンドエラー設定
 * @brief B3Mサーボのコマンドエラーを設定する
 * @param error：エラー
 */
void B3mServo::set_cmd_error(EN_COMMAND_ERRORS error)
{
    cmd_error_ = error;
    return;
}

/**
 * B3Mサーボ 現在位置設定処理
 * @brief B3Mサーボから読みだした現在位置を格納する
 * @param value：位置値[100倍deg]
 */
void B3mServo::set_current_pos( int16_t value )
{
    current_pos_ = value;
    return;
}

/**
 * B3Mサーボ 現在速度設定処理
 * @brief B3Mサーボから読みだした現在速度を格納する
 * @param value：速度値[100倍deg/sec]
 */
void B3mServo::set_current_vel( int16_t value )
{
    current_vel_ = value;
    return;
}

/**
 * B3Mサーボ 現在トルク設定処理
 * @brief B3Mサーボから読みだした現在トルクを格納する
 * @param value：トルク値[1000倍Nm]
 */
void B3mServo::set_current_trq( int16_t value )
{
    current_trq_ = value;
    return;
}

/**
 * B3Mサーボ 現在値パース処理
 * @brief B3Mサーボから読みだした現在値のバイト列を解析し格納する
 * @param data：解析する受信データ先頭アドレス
 * @param length：データ長
 * @note 現在トルク値は取得できないので指示値を現在値として扱う
 */
void B3mServo::parse_current_all( uint8_t* data, uint8_t length )
{
    int16_t value;
    uint8_t index;

    if( length==SIZE_SERVO_CURRENT_ALL ){
        index = 0;
        value = static_cast<int16_t>( (uint16_t)(data[index+1])<<8 | data[index]&0xFF );
        set_current_pos(value);
        index = (ADDR_SERVO_CURRENT_VEROCITY - ADDR_SERVO_CURRENT_ALL);
        value = static_cast<int16_t>( (uint16_t)(data[index+1])<<8 | data[index]&0xFF );
        set_current_vel(value);
        index = (ADDR_SERVO_DESIRED_TORQUE - ADDR_SERVO_CURRENT_ALL);
        value = static_cast<int16_t>( (uint16_t)(data[index+1])<<8 | data[index]&0xFF );
        set_current_trq(value);
    }
    return;
}

/**
 * B3Mサーボ 目標値格納処理
 * @brief B3Mサーボへ書き込む制御目標値を格納する
 * @param value：目標値
 * @note 目標値は単位変換してから入力すること
 */
void B3mServo::set_desired( int16_t value )
{
    int32_t max, min;

    switch( control_type_ ){
    case enOptions_ControlPosition: // 位置指令の場合
        max = MAX_POSITION;
        min = MIN_POSITION;
        break;
    case enOptions_ControlVelocity: // 速度指令の場合
        max = MAX_VELOCITY;
        min = MIN_VELOCITY;
        break;
    case enOptions_ControlTorque:   // トルク指令の場合
        max = MAX_TORQUE;
        min = MIN_TORQUE;
        break;
    default:                        // その他の場合
        max = MAX_UNDEFINED;
        min = MIN_UNDEFINED;
        break;
    }
    if( value > max ){
        value = static_cast<int16_t>(max);
    }
    if( value < min ){
        value = static_cast<int16_t>(min);
    }
    desired_ = value;
    return;
}

/**
 * B3Mサーボ 現在位置取得処理
 * @brief 格納されている現在位置を返却する
 * @param なし
 * @return 位置値[100倍deg]
 */
int16_t B3mServo::get_current_pos( void )
{
    return current_pos_;
}

/**
 * B3Mサーボ 現在速度取得処理
 * @brief 格納されている現在速度を返却する
 * @param なし
 * @return 位置値[100倍deg/sec]
 */
int16_t B3mServo::get_current_vel( void )
{
    return current_vel_;
}

/**
 * B3Mサーボ 現在トルク取得処理
 * @brief 格納されている現在トルクを返却する
 * @param なし
 * @return 位置値[1000倍Nm]
 */
int16_t B3mServo::get_current_trq( void )
{
    return current_trq_;
}

/**
 * B3Mサーボ 目標値取得処理
 * @brief 格納されている目標値を返却する
 * @param なし
 * @return 目標値
 */
int16_t B3mServo::get_desired( void )
{
    return desired_;
}
