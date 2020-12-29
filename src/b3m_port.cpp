/**
 * @file b3m_port.cpp
 * @brief B3Mポートクラスの実装
 * @author nomumu
 * @date 2020.12.29
 */

#include <errno.h>
#include "b3m_control/b3m_design.hpp"
#include "b3m_control/b3m_port.hpp"
#include "b3m_control/b3m_servo.hpp"

/**
 * B3M通信ポート コンストラクタ
 * @brief B3M通信ポートクラスを初期化する
 * @param dev_name：通信ポートのデバイスファイル名
 * @param baudrate：通信ポートのボーレート
 */
B3mPort::B3mPort(std::string dev_name, uint32_t baudrate)
    : dev_name_(dev_name), baudrate_(baudrate)
{
    init_result_ = initialize();
    if( !init_result_ ){
        // 初期化に失敗した場合
        printf("init NG\r\n");
    }
    return;
}

/**
 * B3M通信ポート デストラクタ
 * @brief B3M通信ポートクラスを解放する
 * @param なし
 */
B3mPort::~B3mPort()
{
    if( init_result_ ){
        // ポートクローズする
        close(fd_);
        init_result_ = false;
    }
    return;
}

/**
 * B3M通信ポート 初期化状態確認処理
 * @brief B3M通信ポートの初期化状態を返却する
 * @param なし
 * @return ポート初期化状態
 */
bool B3mPort::is_init(void)
{
    return init_result_;
}

/**
 * B3M通信ポート Load要求処理
 * @brief B3MへLoad操作を要求する
 * @param servo：サーボデータ
 */
void B3mPort::load(B3mServo &servo)
{
    uint8_t data[64];
    uint8_t res[64];
    uint8_t length=0;

    data[length++] = 0x05;
    data[length++] = enCmdTypes_Load;
    data[length++] = enProtocolOptions_ClearStat;

    data[length++] = servo.get_id();

    data[length++] = calc_sum( data, length );

    if( write_port(&data[0], length) ){
        read_port(res,5,100);
    }
    return;
}

/**
 * B3M通信ポート Save要求処理
 * @brief B3MへSave操作を要求する
 * @param servo：サーボデータ
 */
void B3mPort::save(B3mServo &servo)
{
    uint8_t data[64];
    uint8_t res[64];
    uint8_t length=0;

    data[length++] = 0x05;
    data[length++] = enCmdTypes_Save;
    data[length++] = enProtocolOptions_ClearStat;

    data[length++] = servo.get_id();

    data[length++] = calc_sum( data, length );

    if( write_port(&data[0], length) ){
        read_port(res,5,100);
    }
    return;
}

/**
 * B3M通信ポート Reset要求処理
 * @brief B3MへReset操作を要求する
 * @param servo：サーボデータ
 */
void B3mPort::reset(B3mServo &servo)
{
    uint8_t data[64];
    uint8_t length=0;

    data[length++] = 0x06;
    data[length++] = enCmdTypes_Reset;
    data[length++] = enProtocolOptions_ClearStat;

    data[length++] = servo.get_id();
    data[length++] = 0; // 即時

    data[length++] = calc_sum( data, length );

    write(fd_, &data[0], length);
    wait(100);          // 100msec待ち
    return;
}

/**
 * B3M通信ポート 受信バッファClean処理
 * @brief B3M通信ポートの受信バッファを掃除する
 * @param なし
 */
void B3mPort::clean(void)
{
    int cleanSize;
    uint8_t data[8];

    ioctl(fd_, FIONREAD, &cleanSize);
    for( uint32_t ii=0 ; ii< cleanSize ; ++ii ){
        (void)read(fd_,data,1);
    }
    return;
}

/**
 * B3M通信ポート 現在値読み込み処理
 * @brief B3Mへ現在値を要求する
 * @param servo：サーボデータ
 * @note ControlTypeで読みだす値を変える
 */
void B3mPort::readCurrent(B3mServo &servo)
{
    uint8_t data[64];
    uint8_t length=0;
    uint8_t sum=0;
    uint8_t read_length=0;

    data[length++] = 0x07;
    data[length++] = enCmdTypes_Read;
    data[length++] = enProtocolOptions_ClearStat;
    data[length++] = servo.get_id();
    data[length++] = ADDR_SERVO_CURRENT_ALL;
    data[length++] = SIZE_SERVO_CURRENT_ALL;
    data[length++] = calc_sum( data, length );
    if( write_port(&data[0], length) ){
        uint8_t buffer[64];
        read_length = (4+1+SIZE_SERVO_CURRENT_ALL); // 制御データ+ID+データ
        if( read_port(buffer,read_length,100) ){
            // 読んだ値を解析し格納する
            servo.parse_current_all(&buffer[4],SIZE_SERVO_CURRENT_ALL);
        }
    }

    return;
}

/**
 * B3M通信ポート 指示値書き込み処理
 * @brief B3Mへ指示値を設定する
 * @param servo：サーボデータ
 * @note ControlTypeで書き込み先を変える
 */
void B3mPort::writeDesired(B3mServo &servo)
{
    uint8_t data[64];
    uint8_t length=0;
    uint8_t sum=0;
    int16_t value = servo.get_desired();
    uint8_t address;

    switch(servo.get_control_type()){
    case enOptions_ControlVelocity:     // 速度指令
        address = ADDR_SERVO_DESIRED_VEROCITY;
        break;
    case enOptions_ControlTorque:       // トルク指令
        address = ADDR_SERVO_DESIRED_TORQUE;
        break;
    case enOptions_ControlPosition:     // 位置指令
    default:                            // それ以外の場合も位置
        address = ADDR_SERVO_DESIRED_POSITION;
        break;
    }

    data[length++] = 0x09;
    data[length++] = enCmdTypes_Write;
    data[length++] = enProtocolOptions_ClearStat;
    data[length++] = servo.get_id();
    data[length++] = static_cast<uint8_t>(value&0xFF);
    data[length++] = static_cast<uint8_t>(value>>8&0xFF);
    data[length++] = address;
    data[length++] = 1;
    data[length++] = calc_sum( data, length );
    if( write_port(&data[0], length) ){
        uint8_t buffer[64];
        read_port(buffer,5,100);
    }

    return;
}

void B3mPort::writeRunMode(B3mServo &servo)
{
    uint8_t data[64];
    uint8_t length=0;
    uint8_t sum=0;

    data[length++] = 0x08;
    data[length++] = enCmdTypes_Write;
    data[length++] = enProtocolOptions_ClearStat;

    data[length++] = servo.get_id();
    data[length++] = static_cast<uint8_t>(servo.get_control_type())
                    | static_cast<uint8_t>(servo.get_run_state());
    data[length++] = ADDR_SERVO_SERVO_MODE;
    data[length++] = 1;
    data[length++] = calc_sum( data, length );
    if( write_port(&data[0], length) ){
        uint8_t buffer[64];
        read_port(buffer,5,100);
    }

    return;
}


/* Private */
/**
 * B3M通信ポート 初期化処理
 * @brief B3M通信ポートを初期化する
 * @param なし
 */
bool B3mPort::initialize(void)
{
    bool result = false;
    fd_ = open(dev_name_.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    if( fd_ < 0 ){
        // デバイスの open() に失敗したら
        printf("Open NG %d(%s)\r\n",fd_,dev_name_.c_str());
    }else{
        speed_t baud = get_baudtype(baudrate_);
        if( baud != B0 ){
            struct termios tio;
            memset(&tio,0,sizeof(tio));
            tio.c_cflag     = baud | CS8 | CLOCAL | CREAD | CSTOPB;
            tio.c_iflag     = IGNPAR;
            tio.c_oflag     = 0;
            tio.c_lflag     = 0;
            tio.c_cc[VTIME] = 0;
            tio.c_cc[VMIN]  = 0;
            // デバイスに設定を行う
            tcflush(fd_, TCIFLUSH);
            tcsetattr(fd_,TCSANOW,&tio);
            clean();
            result = true;
        }else{
            // ボーレート選択エラー
            printf("Baudrate NG\r\n");
        }
    }
    return result;
}

/**
 * B3M通信ポート ボーレートタイプ取得処理
 * @brief B3M通信ポートへ設定するボーレート値をtermiosの定義に変換する
 * @param baudrate：ボーレート値
 * @return ボーレートタイプ（termiosの定義）
 */
speed_t B3mPort::get_baudtype(uint32_t baudrate)
{
    speed_t result = B0;

    switch(baudrate){
    case 115200:
        result = B115200;
        break;
    case 1000000:
        result = B1000000;
        break;
    case 1500000:
        result = B1500000;
        break;
    case 2000000:
        result = B2000000;
        break;
    case 3000000:
        result = B3000000;
        break;
    default:
        break;
    }
    return result;
}

/**
 * B3M通信ポート チェックサム計算処理
 * @brief チェックサムを計算する
 * @param data：計算するデータの先頭アドレス
 * @param length：計算するデータ長
 * @return チェックサム計算結果
 */
uint8_t B3mPort::calc_sum(uint8_t* data, uint8_t length)
{
    uint8_t result = 0;

    for( uint8_t ii=0 ; ii<length ; ++ii ){
        result += data[ii];
    }
    return result;
}

/**
 * B3M通信ポート ポート受信処理
 * @brief B3M通信ポートからデータを受信する
 * @param data：受信データ格納先アドレス
 * @param length：受信長さ
 * @param timeout_msec：タイムアウト[msec]
 */
bool B3mPort::read_port(uint8_t* data, uint8_t length, uint8_t timeout_msec)
{
    fd_set set_value;
    struct timeval tmout;
    int res;
    int len;
    int rcvSize;
    bool result=false;

    FD_ZERO(&set_value);
    FD_SET(fd_, &set_value);

    tmout.tv_sec = 0;
    tmout.tv_usec = timeout_msec*1000;

    if( !init_result_ ){
        printf("port invalid\r\n");
        return 0;
    }
    res = select(fd_ + 1, &set_value, NULL, NULL, &tmout);
    if( res > 0 ){
        ioctl(fd_, FIONREAD, &rcvSize);
        if( rcvSize < length ){
            usleep(2000);// FTDI latency=1が前提
            ioctl(fd_, FIONREAD, &rcvSize);
        }
        len=read(fd_,data,rcvSize);
        if( rcvSize==length ){
            if( data[length-1] != calc_sum(data,length-1) ){
                printf("chk sum error!\r\n");
            }else{
                result=true;
            }
        }
    }else{
        if( res==0 ){
            printf("read timeout!\r\n");
        }else{
            if( errno!=EINTR ){ // シグナル以外は何か変
                printf("read error [%s]\r\n",strerror(errno));
            }
        }
        clean();
    }
    return result;
}

/**
 * B3M通信ポート ポート送信処理
 * @brief B3M通信ポートへデータを送信する
 * @param data：送信データ戦闘アドレス
 * @param length：受信長さ
 * @return 成否
 */
bool B3mPort::write_port(uint8_t* data, uint8_t length)
{
    ssize_t res;
    res = write(fd_, data, length);
    if( (res < 0) && (errno != EINTR) ){
        printf("write error [%s]\r\n",strerror(errno));
    }
    return (res > 0);
}

/**
 * B3M通信ポート ウエイト処理
 * @brief 指定時間を待つ
 * @param msec：待ち時間[msec]
 */
void B3mPort::wait(uint16_t msec)
{
    struct timeval tmout;

    tmout.tv_sec = (msec/1000);
    tmout.tv_usec = ((msec%1000)*1000);
    (void)select(0, NULL, NULL, NULL, &tmout);
    
    return;
}