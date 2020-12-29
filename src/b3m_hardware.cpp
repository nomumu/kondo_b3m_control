/**
 * @file b3m_hardware.cpp
 * @brief B3Mハードウェアインタフェースクラスの実装
 * @author nomumu
 * @date 2020.12.29
 */

#include "b3m_control/b3m_hardware.hpp"

#define DEG2RAD(deg)    ((deg)*(M_PI/180.0))
#define RAD2DEG(rad)    ((rad)*(180.0/M_PI))

/**
 * B3Mハードウェア コンストラクタ
 * @brief B3Mハードウェアクラスを初期化する
 * @param id：サーボID
 * @param control_type：サーボ制御種別
 */
B3mHardware::B3mHardware( std::string dev_name, uint32_t baudrate, ros::NodeHandle handle )
 : port_(dev_name,baudrate), manager_(this,handle)
{
    updt_time_  = ros::Time::now();
    updt_d_     = ros::Duration(0);
}

/**
 * B3Mハードウェア デストラクタ
 * @brief B3Mハードウェアクラスを解放する
 * @param なし
 */
B3mHardware::~B3mHardware()
{
}

/**
 * B3Mハードウェア 初期化状態確認処理
 * @brief B3Mハードウェアの初期化状態を返却する
 * @param なし
 * @return 初期化状態
 */
bool B3mHardware::is_init(void)
{
    return port_.is_init(); // 通信ポート以外で失敗しないのでそのまま返却
}

/**
 * B3Mハードウェア ジョイント情報登録処理
 * @brief 制御するジョイントを追加する
 * @param id：サーボID
 * @param type：ジョイント種別
 * @param name：ジョイント名称
 * @return 登録OK(true)
 * @return 登録NG(false)
 */
bool B3mHardware::regist_joint( uint8_t id, enB3mJointType type, std::string name )
{
    bool result = true;

    for(std::vector<B3mJoint>::iterator itr=joint_.begin() ; itr!=joint_.end() ; ++itr){
        if( itr->get_id() == id ){
            result = false;         // IDの重複は認めない
        }
    }
    if( result ){
        joint_.push_back( B3mJoint( id, type, name ) );
        std::vector<B3mJoint>::iterator joint_itr = (joint_.end()-1); // 今登録したデータ

        joint_itr->set_pos( 0.0 );
        joint_itr->set_vel( 0.0 );
        joint_itr->set_eff( 0.0 );
    }
    return result;
}

/**
 * B3Mハードウェア インタフェース登録処理
 * @brief インタフェース登録を実行し、ROSコントロール制御を開始する
 * @param なし
 * @note ジョイント登録が終わったら呼び出します
 */
void B3mHardware::regist_interface(void)
{
    for(std::vector<B3mJoint>::iterator itr=joint_.begin() ; itr!=joint_.end() ; ++itr){
        hardware_interface::JointStateHandle stat_handle(   itr->get_name(),
                                                            itr->get_pos_addr(),
                                                            itr->get_vel_addr(),
                                                            itr->get_eff_addr() );
        stat_if_.registerHandle( stat_handle );
        hardware_interface::JointHandle joint_handle( stat_if_.getHandle(itr->get_name()), itr->get_cmd_addr() );
        switch( itr->get_type() ){
        case enB3mJointType_Position:
            pos_if_.registerHandle( joint_handle );
            break;
        case enB3MJointType_Velocity:
            vel_if_.registerHandle( joint_handle );
            break;
        case enB3MJointType_Effort:
            eff_if_.registerHandle( joint_handle );
            break;
        }
    }
    registerInterface( &stat_if_ );
    registerInterface( &pos_if_ );
    registerInterface( &vel_if_ );
    registerInterface( &eff_if_ );
}

/**
 * B3Mハードウェア コントロール更新処理
 * @brief サーボの読み書きを行いROSコントロール制御を更新する
 * @param なし
 */
void B3mHardware::update( void )
{
    // Read Current
    for(std::vector<B3mJoint>::iterator ritr=joint_.begin() ; ritr!=joint_.end() ; ++ritr){
        port_.readCurrent(*ritr);
        ritr->set_pos( DEG2RAD(ritr->get_current_pos() / 100.0) );
        ritr->set_vel( DEG2RAD(ritr->get_current_vel() / 100.0) );
        ritr->set_eff( ritr->get_current_trq() / 1000.0 );
    }

    // Update
    ros::Time now = ros::Time::now();
    updt_d_ = (now - updt_time_);
    manager_.update(now,updt_d_);
    updt_time_ = now;

    // Write desired
    for(std::vector<B3mJoint>::iterator witr=joint_.begin() ; witr!=joint_.end() ; ++witr){
        int16_t desired;
        switch( witr->get_type() ){
        case enB3mJointType_Position:
        case enB3MJointType_Velocity:
            desired = static_cast<int16_t>(RAD2DEG(witr->get_cmd()) * 100.0); // rad->deg
            break;
        case enB3MJointType_Effort:
            desired = static_cast<int16_t>(witr->get_cmd() * 1000.0);// Nm->mNm
            break;
        }
        witr->set_desired( desired );
        port_.writeDesired(*witr);
    }
    return;
}

/**
 * B3Mハードウェア トルクON処理
 * @brief 登録されている全てのジョイントをトルクON状態にする
 * @param なし
 */
void B3mHardware::torque_on(void)
{
    set_torque(true);
}

/**
 * B3Mハードウェア トルクFREE処理
 * @brief 登録されている全てのジョイントのトルクをFREE状態にする
 * @param なし
 */
void B3mHardware::torque_free(void)
{
    set_torque(false);
}

/**
 * B3Mハードウェア ジョイントトルク設定処理
 * @brief 登録されている全てのジョイントのトルクを設定する
 * @param torque：設定状態
 */
void B3mHardware::set_torque(bool torque)
{
    EN_OPTIONS value = torque?enOptions_RunNormal:enOptions_RunFree;
    for(std::vector<B3mJoint>::iterator itr=joint_.begin() ; itr!=joint_.end() ; ++itr){
        itr->set_run_state( value );
        port_.writeRunMode(*itr);
    }
}

/**
 * B3Mハードウェア リセット処理
 * @brief 登録されている全てのジョイントをリセットする
 * @param なし
 */
void B3mHardware::reset(void)
{
    for(std::vector<B3mJoint>::iterator itr=joint_.begin() ; itr!=joint_.end() ; ++itr){
        port_.reset(*itr);
    }
}