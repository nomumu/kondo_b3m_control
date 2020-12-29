/**
 * @file b3m_design.hpp
 * @brief B3Mポート設計値の定義
 * @author nomumu
 * @date 2020.12.29
 * @note プロトコルに関しての権利は近藤科学が保有しています。このファイルはプロトコル仕様を参考に作成されています。
 */
#ifndef B3M_DESIGN_HPP_
#define B3M_DESIGN_HPP_

/* MemoryAddress */
#define ADDR_SYSTEM_ID                              (0x00)
#define ADDR_SYSTEM_BAUDRATE                        (0x01)
#define ADDR_SYSTEM_POSITION_MIN                    (0x05)
#define ADDR_SYSTEM_POSITION_MAX                    (0x07)
#define ADDR_SYSTEM_POSITION_CENTER                 (0x09)
#define ADDR_SYSTEM_MCU_TEMP_LIMIT                  (0x0B)
#define ADDR_SYSTEM_MCU_TEMP_LIMIT_PR               (0x0D)
#define ADDR_SYSTEM_MOTOR_TEMP_LIMIT                (0x0E)
#define ADDR_SYSTEM_MOTOR_TEMP_LIMIT_PR             (0x10)
#define ADDR_SYSTEM_CURRENT_LIMIT                   (0x11)
#define ADDR_SYSTEM_CURRENT_LIMIT_PR                (0x13)
#define ADDR_SYSTEM_LOCKDETECT_TIME                 (0x14)
#define ADDR_SYSTEM_LOCKDETECT_OUTRATE              (0x15)
#define ADDR_SYSTEM_LOCKDETECT_TIME_PR              (0x16)
#define ADDR_SYSTEM_INPUT_VOLTAGE_MIN               (0x17)
#define ADDR_SYSTEM_INPUT_VOLTAGE_MAX               (0x19)
#define ADDR_SYSTEM_TORQUE_LIMIT                    (0x1B)
#define ADDR_SYSTEM_DEADBAND_WIDTH                  (0x1C)
#define ADDR_SYSTEM_MOTOR_CW_RATIO                  (0x22)
#define ADDR_SYSTEM_MOTOR_CCW_RATIO                 (0x23)
#define ADDR_SERVO_SERVO_OPTION                     (0x27)
#define ADDR_SERVO_SERVO_MODE                       (0x28)
#define ADDR_SERVO_TORQUE_ON                        (0x28)
#define ADDR_SERVO_RUN_MODE                         (0x29)
#define ADDR_SERVO_DESIRED_POSITION                 (0x2A)
#define ADDR_SERVO_CURRENT_POSITION                 (0x2C)
#define ADDR_SERVO_PREVIOUS_POSITION                (0x2E)
#define ADDR_SERVO_DESIRED_VEROCITY                 (0x30)
#define ADDR_SERVO_CURRENT_VEROCITY                 (0x32)
#define ADDR_SERVO_PREVIOUS_VEROCITY                (0x34)
#define ADDR_SERVO_DESIRED_TIME                     (0x36)
#define ADDR_SERVO_RUNNING_TIME                     (0x38)
#define ADDR_SERVO_WORKING_TIME                     (0x3A)
#define ADDR_SERVO_DESIRED_TORQUE                   (0x3C)
#define ADDR_SERVO_SYSTEM_CLOCK                     (0x3E)
#define ADDR_SERVO_SAMPLING_TIME                    (0x42)
#define ADDR_SERVO_MCU_TEMP                         (0x44)
#define ADDR_SERVO_MOTOR_TEMP                       (0x46)
#define ADDR_SERVO_CURRENT                          (0x48)
#define ADDR_SERVO_INPUT_VOLTAGE                    (0x4A)
#define ADDR_SERVO_PWM_DUTY                         (0x4C)
#define ADDR_SERVO_PWM_FREQUENCY                    (0x4E)
#define ADDR_SERVO_ENCODER_VALUE                    (0x50)
#define ADDR_SERVO_ENCODER_COUNT                    (0x52)
#define ADDR_SERVO_HALLIC_STATE                     (0x56)
#define ADDR_CONTROL_CONTROL_LOW                    (0x5C)
#define ADDR_CONTROL_GAIN_PRESETNO                  (0x5C)
#define ADDR_CONTROL_TYPE                           (0x5D)
#define ADDR_CONTROL_KP0                            (0x5E)
#define ADDR_CONTROL_KD0                            (0x62)
#define ADDR_CONTROL_KI0                            (0x66)
#define ADDR_CONTROL_STATIC_FRICTION0               (0x6A)
#define ADDR_CONTROL_DYNAMIC_FRICTION0              (0x6C)
#define ADDR_CONTROL_KP1                     　     (0x6E)
#define ADDR_CONTROL_KD1                     　     (0x72)
#define ADDR_CONTROL_KI1                     　     (0x76)
#define ADDR_CONTROL_STATIC_FRICTION1               (0x7A)
#define ADDR_CONTROL_DYNAMIC_FRICTION1              (0x7C)
#define ADDR_CONTROL_KP2                            (0x7E)
#define ADDR_CONTROL_KD2                            (0x82)
#define ADDR_CONTROL_KI2                            (0x86)
#define ADDR_CONTROL_STATIC_FRICTION2               (0x8A)
#define ADDR_CONTROL_DYNAMIC_FRICTION2              (0x8C)
#define ADDR_STATUS_BASE_ADDR                       (0x9D)
#define ADDR_STATUS_SYSTEM                          (0x9E)
#define ADDR_STATUS_SYSTEM                          (0x9E)
#define ADDR_STATUS_MOTOR                           (0x9F)
#define ADDR_STATUS_UART                            (0xA0)
#define ADDR_STATUS_COMMAND                         (0xA1)
#define ADDR_CONFIG_MODEL_NUMBER                    (0xA2)
#define ADDR_CONFIG_MODEL_NUMBER_VOLTAGE_CLASS      (0xA2)
#define ADDR_CONFIG_MODEL_NUMBER_VERSION            (0xA3)
#define ADDR_CONFIG_MODEL_NUMBER_TORQUE             (0xA4)
#define ADDR_CONFIG_MODEL_NUMBER_CASE               (0xA5)
#define ADDR_CONFIG_MODEL_TYPE                      (0xA6)
#define ADDR_CONFIG_MODEL_TYPE_MOTOR                (0xA8)
#define ADDR_CONFIG_MODEL_TYPE_DEVICE               (0xA9)
#define ADDR_CONFIG_FW_VERSION                      (0xAA)
#define ADDR_CONFIG_FW_BUID                         (0xAA)
#define ADDR_CONFIG_FW_REVISION                     (0xAB)
#define ADDR_CONFIG_FW_MINOR                        (0xAC)
#define ADDR_CONFIG_FW_MAJOR                        (0xAD)
#define ADDR_CONFIG_ENC_OFFSET_CENTER               (0xAE)
#define ADDR_CONFIG_ENC_OFFSET                      (0xB0)

/* MemorySzie */
#define SIZE_SYSTEM_ID                              (1)
#define SIZE_SYSTEM_BAUDRATE                        (4)
#define SIZE_SYSTEM_POSITION_MIN                    (2)
#define SIZE_SYSTEM_POSITION_MAX                    (2)
#define SIZE_SYSTEM_POSITION_CENTER                 (2)
#define SIZE_SYSTEM_MCU_TEMP_LIMIT                  (2)
#define SIZE_SYSTEM_MCU_TEMP_LIMIT_PR               (1)
#define SIZE_SYSTEM_MOTOR_TEMP_LIMIT                (2)
#define SIZE_SYSTEM_MOTOR_TEMP_LIMIT_PR             (1)
#define SIZE_SYSTEM_CURRENT_LIMIT                   (2)
#define SIZE_SYSTEM_CURRENT_LIMIT_PR                (1)
#define SIZE_SYSTEM_LOCKDETECT_TIME                 (1)
#define SIZE_SYSTEM_LOCKDETECT_OUTRATE              (1)
#define SIZE_SYSTEM_LOCKDETECT_TIME_PR              (1)
#define SIZE_SYSTEM_INPUT_VOLTAGE_MIN               (2)
#define SIZE_SYSTEM_INPUT_VOLTAGE_MAX               (2)
#define SIZE_SYSTEM_TORQUE_LIMIT                    (1)
#define SIZE_SYSTEM_DEADBAND_WIDTH                  (2)
#define SIZE_SYSTEM_MOTOR_CW_RATIO                  (1)
#define SIZE_SYSTEM_MOTOR_CCW_RATIO                 (1)
#define SIZE_SERVO_SERVO_OPTION                     (1)
#define SIZE_SERVO_SERVO_MODE                       (2)
#define SIZE_SERVO_TORQUE_ON                        (1)
#define SIZE_SERVO_RUN_MODE                         (1)
#define SIZE_SERVO_DESIRED_POSITION                 (2)
#define SIZE_SERVO_CURRENT_POSITION                 (2)
#define SIZE_SERVO_PREVIOUS_POSITION                (2)
#define SIZE_SERVO_DESIRED_VEROCITY                 (2)
#define SIZE_SERVO_CURRENT_VEROCITY                 (2)
#define SIZE_SERVO_PREVIOUS_VEROCITY                (2)
#define SIZE_SERVO_DESIRED_TIME                     (2)
#define SIZE_SERVO_RUNNING_TIME                     (2)
#define SIZE_SERVO_WORKING_TIME                     (2)
#define SIZE_SERVO_DESIRED_TORQUE                   (2)
#define SIZE_SERVO_SYSTEM_CLOCK                     (4)
#define SIZE_SERVO_SAMPLING_TIME                    (2)
#define SIZE_SERVO_MCU_TEMP                         (2)
#define SIZE_SERVO_MOTOR_TEMP                       (2)
#define SIZE_SERVO_CURRENT                          (2)
#define SIZE_SERVO_INPUT_VOLTAGE                    (2)
#define SIZE_SERVO_PWM_DUTY                         (2)
#define SIZE_SERVO_PWM_FREQUENCY                    (2)
#define SIZE_SERVO_ENCODER_VALUE                    (2)
#define SIZE_SERVO_ENCODER_COUNT                    (4)
#define SIZE_SERVO_HALLIC_STATE                     (1)
#define SIZE_CONTROL_CONTROL_LOW                    (2)
#define SIZE_CONTROL_GAIN_PRESETNO                  (1)
#define SIZE_CONTROL_TYPE                           (1)
#define SIZE_CONTROL_KP0                            (4)
#define SIZE_CONTROL_KD0                            (4)
#define SIZE_CONTROL_KI0                            (4)
#define SIZE_CONTROL_STATIC_FRICTION0               (2)
#define SIZE_CONTROL_DYNAMIC_FRICTION0              (2)
#define SIZE_CONTROL_KP1                     　     (4)
#define SIZE_CONTROL_KD1                     　     (4)
#define SIZE_CONTROL_KI1                     　     (4)
#define SIZE_CONTROL_STATIC_FRICTION1               (2)
#define SIZE_CONTROL_DYNAMIC_FRICTION1              (2)
#define SIZE_CONTROL_KP2                            (4)
#define SIZE_CONTROL_KD2                            (4)
#define SIZE_CONTROL_KI2                            (4)
#define SIZE_CONTROL_STATIC_FRICTION2               (2)
#define SIZE_CONTROL_DYNAMIC_FRICTION2              (2)
#define SIZE_STATUS_BASE_ADDR                       (1)
#define SIZE_STATUS_SYSTEM                          (4)
#define SIZE_STATUS_MOTOR                           (1)
#define SIZE_STATUS_UART                            (1)
#define SIZE_STATUS_COMMAND                         (1)
#define SIZE_CONFIG_MODEL_NUMBER                    (4)
#define SIZE_CONFIG_MODEL_NUMBER_VOLTAGE_CLASS      (1)
#define SIZE_CONFIG_MODEL_NUMBER_VERSION            (1)
#define SIZE_CONFIG_MODEL_NUMBER_TORQUE             (1)
#define SIZE_CONFIG_MODEL_NUMBER_CASE               (1)
#define SIZE_CONFIG_MODEL_TYPE                      (4)
#define SIZE_CONFIG_MODEL_TYPE_MOTOR                (1)
#define SIZE_CONFIG_MODEL_TYPE_DEVICE               (1)
#define SIZE_CONFIG_FW_VERSION                      (4)
#define SIZE_CONFIG_FW_BUID                         (1)
#define SIZE_CONFIG_FW_REVISION                     (1)
#define SIZE_CONFIG_FW_MINOR                        (1)
#define SIZE_CONFIG_FW_MAJOR                        (1)
#define SIZE_CONFIG_ENC_OFFSET_CENTER               (2)
#define SIZE_CONFIG_ENC_OFFSET                      (2)

/* 制限値定義 */
#define MAX_POSITION                                (32000)
#define MIN_POSITION                                (-32000)
#define MAX_VELOCITY                                (32767)
#define MIN_VELOCITY                                (-32768)
#define MAX_TORQUE                                  (32767)
#define MIN_TORQUE                                  (-32768)
#define MAX_UNDEFINED                               (32767)
#define MIN_UNDEFINED                               (-32768)

/**
 * 動作モード列挙型
 */
typedef enum {
    enOptions_RunNormal         = 0x00,
    enOptions_RunFree           = 0x02,
    enOptions_RunHold           = 0x03,
    enOptions_ControlPosition   = 0x00,
    enOptions_ControlVelocity   = 0x04,
    enOptions_ControlTorque     = 0x08,
    enOptions_ControlFForward   = 0x0C,
    enOptions_ServoNormal       = 0x00,
    enOptions_ServoClone        = 0x40,
    enOptions_ServoReverse      = 0x80
} EN_OPTIONS;

/**
 * システムエラー列挙型
 */
typedef enum {
    enSysErrors_Watchdog        = 0x01,
    enSysErrors_FlashAccess     = 0x02,
    enSysErrors_MemAllocation   = 0x04,
    enSysErrors_InputVoltage    = 0x08,
    enSysErrors_MCUTemperature  = 0x10,
    enSysErrors_ADConversion    = 0x20,
    enSysErrors_I2C             = 0x40,
    enSysErrors_SPI             = 0x80
} EN_SYS_ERRORS;

/**
 * モーターステータス異常列挙型
 */
typedef enum {
    enMotorErrors_Temperature   = 0x01,
    enMotorErrors_LockDetect    = 0x02,
    enMotorErrors_CurrentLimit  = 0x04,
    enMotorErrors_HallIC        = 0x08
} EN_MOTOR_ERRORS;

/**
 * UART受信エラー列挙型
 */
typedef enum {
    enUartErrors_Framing        = 0x01,
    enUartErrors_Parity         = 0x02,
    enUartErrors_Break          = 0x04,
    enUartErrors_Overrun        = 0x08
} EN_UART_ERRORS;

/**
 * コマンドエラーなど列挙型
 */
typedef enum {
    enCmdErrors_CheckSum        = 0x01,
    enCmdErrors_Length          = 0x02,
    enCmdErrors_Size            = 0x04,
    enCmdErrors_Address         = 0x08,
    enCmdErrors_WrongCmd        = 0x10
} EN_COMMAND_ERRORS;

/**
 * 軌道生成タイプ列挙型
 */
typedef enum {
    enTrajErrors_Normal         = 0x00,
    enTrajErrors_Even           = 0x01,
    enTrajErrors_ThirdPoly      = 0x03,
    enTrajErrors_FourthPoly     = 0x04,
    enTrajErrors_FifthPoly      = 0x05
} EN_TRAJECTORY_TYPES;

/**
 * コマンド列挙型
 */
typedef enum {
    enCmdTypes_Load             = 0x01,
    enCmdTypes_Save             = 0x02,
    enCmdTypes_Read             = 0x03,
    enCmdTypes_Write            = 0x04,
    enCmdTypes_Reset            = 0x05,
    enCmdTypes_Position         = 0x06
} EN_COMMAND_TYPES;

/**
 * プロトコルオプション列挙型
 */
typedef enum {
    enProtocolOptions_ErrStat   = 0x00,
    enProtocolOptions_SysStat   = 0x01,
    enProtocolOptions_MotorStat = 0x02,
    enProtocolOptions_UartStat  = 0x03,
    enProtocolOptions_CmdStat   = 0x04,
    enProtocolOptions_ClearStat = 0x80
} EN_PROTOCOL_OPTIONS;



#endif  // B3M_DESIGN_HPP_