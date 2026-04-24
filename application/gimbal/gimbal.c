#include "gimbal.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bmi088.h"

static attitude_t *gimbal_IMU_data; // 云台IMU数据
static DJIMotorInstance *yaw_motor, *pitch_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

static BMI088Instance *bmi088; // 云台IMU

float pid_gimbal_set[2];//pid双轴目标值

float PitchGravity_param = 9.0;
float PitchGravity_val;

float resistance_feedforword;

float static_resistance = 1300.0f;
float viscous_k_val = 0.0f;
float slope_val = 0.02f;
/**
 * @brief 自定义 Pitch 轴重力与偏心补偿
 * @note 适用于“水平平衡但倾斜会倒”的情况
 */
float PitchGravityCompensation() {
    // 获取当前 Pitch 角度（确保使用真实的反馈角度）
    float pitch_deg = gimbal_IMU_data->Pitch; 
    float pitch_rad = pitch_deg * DEGREE_2_RAD;

    // --- 实验参数区 (需要通过下面的实验测定) ---
    // k_cos 对应水平位置的平衡电流
    const float k_cos = 0.0f;  
    // k_sin 对应 45 度或 90 度位置的平衡电流
    const float k_sin = -670.0f; 
    // ---------------------------------------

    // 核心物理公式：
    // 理论上，重力矩是这两个三角函数的线性组合
    float gravity_comp = (k_cos * cosf(pitch_rad) + k_sin * sinf(pitch_rad)) * PitchGravity_param;

    return gravity_comp;
}

/**
 * @brief 改进型阻力前馈计算 (混合实际速度与位置误差)
 * * @param actual_vel 实际角速度 (来自陀螺仪，单位需要与系统统一)
 * @param pos_error  位置误差 (目标角度 - 实际角度)
 * @param static_ff  静态摩擦力补偿项 (克服起始摩擦和解决静差)
 * @param viscous_k  粘滞摩擦系数 (解决高速滞后)
 * @param vel_slope  速度过渡斜率 (平滑换向)
 * @return float     计算后的前馈力矩/电流输出
 */
float calculate_improved_friction_feedforward(float actual_vel, float pos_error, float static_ff, float viscous_k, float vel_slope) {
    float ff_out = 0.0f;
    
    // 阈值设定 (需要根据你的陀螺仪噪声和编码器精度实际调整)
    const float vel_noise_threshold = 0.5f; // 陀螺仪噪声死区 (比如 0.5度/秒)
    const float pos_deadzone = 0.05f;       // 允许的极小稳态误差绝对死区 (比如 0.05度)

    // ==========================================
    // 状态 1: 运动状态 (依赖陀螺仪实际速度)
    // ==========================================
    if (fabs(actual_vel) > vel_noise_threshold) {
        
        // 使用平滑斜坡计算方向系数 (-1.0 到 1.0)
        float direction = actual_vel / vel_slope;
        if (direction > 1.0f) direction = 1.0f;
        if (direction < -1.0f) direction = -1.0f;

        // 动摩擦补偿 = 库仑摩擦(方向) + 粘滞摩擦(速度成正比)
        ff_out = (direction * static_ff) + (viscous_k * actual_vel);
    } 
    // ==========================================
    // 状态 2: 停滞/微调状态 (依赖位置误差解决静差)
    // ==========================================
    else {
        // 此时速度极小，可能卡在了目标位置附近，靠位置误差决定发力方向
        if (pos_error > pos_deadzone) {
            // 目标在正方向，系统却没动，说明静摩擦力在阻碍往正方向走
            ff_out = static_ff; 
        } 
        else if (pos_error < -pos_deadzone) {
            // 目标在负方向
            ff_out = -static_ff;
        } 
        else {
            // 真正到达了绝对目标点以内，彻底关闭补偿，防止原地高频震荡
            ff_out = 0.0f; 
        }
    }

    return ff_out;
}
void GimbalInit()
{   
    gimbal_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 2.15, // 2.15
                .Ki = 5.0, //  5
                .Kd = 0.08,//  0.08
                .DeadBand = 0.1,
                .CoefA = 0.4,
                .CoefB = 0.3,
                .Derivative_LPF_RC = 0.00808,// 1 / 2 * pai * fc(一阶低通滤波计算RC) 0.00508位置环D微分项的带宽是20Hz
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit = 100,
                .MaxOut = 500,
            },
            .speed_PID = {
                .Kp = 1000,  // 1000
                .Ki = 3000, // 3000
                .Kd = 0,
                .CoefA = 0.7,
                .CoefB = 0.3,
                .Derivative_LPF_RC = 0.00508,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate,
                .IntegralLimit = 3500,
                .MaxOut = 20000,
                .DeadBand = 0.1,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[2],
            .current_feedforward_ptr = &resistance_feedforword,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            // .outer_loop_type = OPEN_LOOP,
            // .close_loop_type = OPEN_LOOP,            
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020};
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 6,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 2.5, // 2.5
                .Ki = 0,
                .Kd = 0,
                .CoefA = 0.5,
                .CoefB = 0.3,
                .Derivative_LPF_RC = 0.00808,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
                .IntegralLimit = 100,
                .MaxOut = 500,
                .DeadBand = 0.1,
            },
            .speed_PID = {
                .Kp = 5000,  // 5000
                .Ki = 6000, // 6000
                .Kd = 0,   // 0
                .CoefA = 0.5,
                .CoefB = 0.3,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_DerivativeFilter,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
                .Derivative_LPF_RC = 0.00808,
                .DeadBand = 0.1,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->Pitch,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimbal_IMU_data->Gyro[1]),
            .current_feedforward_ptr = &PitchGravity_val,
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = SPEED_LOOP | ANGLE_LOOP,         
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = GM6020,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DJIMotorInit(&pitch_config);
    
    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    PitchGravity_val = PitchGravityCompensation();
    float error_pos = gimbal_cmd_recv.yaw - gimbal_IMU_data->YawTotalAngle; 
    resistance_feedforword = calculate_improved_friction_feedforward(gimbal_IMU_data->Gyro[2],error_pos,static_resistance,viscous_k_val,slope_val);

    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref
    switch (gimbal_cmd_recv.gimbal_mode)
    {
    // 停止
    case GIMBAL_ZERO_FORCE:
        DJIMotorStop(yaw_motor);
        DJIMotorStop(pitch_motor);
        break;
    // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
    case GIMBAL_GYRO_MODE: // 后续只保留此模式
        DJIMotorEnable(pitch_motor);
        DJIMotorEnable(yaw_motor);
        /* 调试单轴的时候使用 */
        // DJIMotorStop(yaw_motor);
        // DJIMotorStop(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        // DJIMotorSetRef(yaw_motor, pid_gimbal_set[0]); 
        // DJIMotorSetRef(pitch_motor, test_open_loop_current);
        DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);        
        break;
    // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
    case GIMBAL_FREE_MODE: // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
        DJIMotorEnable(yaw_motor);
        DJIMotorEnable(pitch_motor);
        DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
        DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
        // DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
        // DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
        break;
    default:
        break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}