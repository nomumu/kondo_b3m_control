/**
 * @file b3m_config.cpp
 * @brief B3M設定クラスの実装
 * @author nomumu
 * @date 2020.12.29
 */

#include "b3m_control/b3m_config.hpp"

/**
 * B3M設定 コンストラクタ
 * @brief B3M設定クラスを初期化する
 * @param handle：ノードハンドル
 */
B3mConfig::B3mConfig( ros::NodeHandle handle )
  : nh_(handle)
{
    joints_.clear();
}

/**
 * B3M設定 デストラクタ
 * @brief B3M設定クラスを解放する
 * @param なし
 */
B3mConfig::~B3mConfig()
{
    /* Nothing todo... */
}

/**
 * B3M設定 ロード処理
 * @brief B3M設定値を読み込む
 * @param なし
 * @return 読み込み成功(true)
 * @return 読み込み失敗(false)
 */
bool B3mConfig::load( void )
{
    std::string port_name;
    uint32_t baudrate;
    bool result;

    result = true;

    /* 通信ポートの読み込み */
    std::string name = load_portname();
    if( name.length() > 0 ){
        port_ = name;
    }else{
        return false;
    }

    /* 通信速度の読み込み */
    baudrate = load_baudrate();
    if( baudrate > 0 ){
        baudrate_ = baudrate;
    }else{
        result = false;
    }

    /* ジョイント設定の読み込み */
    if( load_joints() ){
        load_param();
    }else{
        result = false;
    }

    return result;
}

/**
 * B3M設定 通信ポート名ロード処理
 * @brief B3M設定通信ポート名を読み込む
 * @param なし
 * @return 通信ポート名(成功)
 * @return 空文字(失敗)
 */
std::string B3mConfig::load_portname( void )
{
    std::string key_port = KEY_B3M_CONFIG;
    std::string result;

    key_port += KEY_PORTNAME;
    if( !nh_.getParam( key_port, result ) ){
        ROS_ERROR("Undefined key %s", key_port.c_str());
        result = "";
    }
    return result;
}

/**
 * B3M設定 通信速度ロード処理
 * @brief B3M設定通信速度を読み込む
 * @param なし
 * @return !=0(成功)
 * @return =0(失敗)
 */
uint32_t B3mConfig::load_baudrate( void )
{
    std::string key_baudrate = KEY_B3M_CONFIG;
    key_baudrate += KEY_BAUDRATE;
    int result;
    
    if( !nh_.getParam( key_baudrate, result ) ){
        ROS_ERROR("Undefined key %s", key_baudrate.c_str());
        result = 0;
    }
    return static_cast<uint32_t>(result);
}

/**
 * B3M設定 ジョイント定義ロード処理
 * @brief B3M設定ジョイント定義を読み込む
 * @param なし
 * @return 読み込み結果
 */
bool B3mConfig::load_joints( void )
{
    std::string key_joints = KEY_B3M_CONFIG;
    bool result = false;
    XmlRpc::XmlRpcValue load_joints;

    key_joints += KEY_JOINTS;
    if( !nh_.getParam( key_joints, load_joints ) ){
        ROS_ERROR("Undefined key %s", key_joints.c_str());
    }else{
        // jointsキーを見つけた
        if( load_joints.getType() != XmlRpc::XmlRpcValue::TypeArray ){
            ROS_ERROR("XmlRpc get type error! line%d", __LINE__);
        }else{
            // jointsリストを作成する
            for( uint32_t ii=0; ii < load_joints.size(); ++ii ){
                if( load_joints[ii].getType() != XmlRpc::XmlRpcValue::TypeString ){
                    ROS_ERROR("XmlRpc get type[%d] error! line%d",ii,__LINE__);
                }else{
                    XmlRpc::XmlRpcValue &joint_work = load_joints[ii];
                    stB3mJointConfig work;
                    work.name_ = (std::string)joint_work;
                    joints_.push_back( work );
                }
            }
            result = true;
        }
    }
    return result;
}

/**
 * B3M設定 ジョイントパラメータロード処理
 * @brief B3M設定ジョイントパラメータを読み込む
 * @param なし
 * @return 読み込み結果
 */
bool B3mConfig::load_param( void )
{
    bool result = true;
    
    for(std::vector<stB3mJointConfig>::iterator itr=joints_.begin() ; itr!=joints_.end() ; ++itr){
        std::string key_joint       = (std::string(KEY_B3M_CONFIG) + "/" + itr->name_);
        std::string key_param_id    = (key_joint + KEY_PARAM_ID);
        std::string key_param_type  = (key_joint + KEY_PARAM_TYPE);
        int load_id = 0;
        std::string load_type = "";
        bool load_result = true;
        if( !nh_.getParam( key_param_id, load_id ) ){
            ROS_ERROR("Undefined key %s", key_param_id.c_str());
            load_result = false;
        }
        if( !nh_.getParam( key_param_type, load_type ) ){
            ROS_ERROR("Undefined key %s", key_param_type.c_str());
            load_result = false;
        }
        if( load_result ){
            itr->id_ = static_cast<uint8_t>(load_id);
            if(load_type == "Position"){
                itr->type_ = enB3mJointConfigType_Position;
            }else if(load_type == "Velocity"){
                itr->type_ = enB3mJointConfigType_Velocity;
            }else if(load_type == "Effort"){
                itr->type_ = enB3mJointConfigType_Effort;
            }else{
                ROS_ERROR("Undefined type %s", load_type.c_str());
                result = false;
                break;
            }
        }else{
            result = false;
            break;
        }
    }
    return result;
}
