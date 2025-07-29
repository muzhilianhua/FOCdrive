#include <Arduino.h>
#include <SimpleFOC.h>
#include <Preferences.h>  // 该库用于在ESP32中进行键值对数据的存储和读取，实现数据的持久化
#include "FUTABA_SBUS.h"
#include "ServoControl.h"
#include "ICM42688.h"
#include "MahonyFilter.h"
#include "OllieFOCdrive.h"
#include "filter.h"
#include "touchscreen.h"

// commander communication instance
Commander command = Commander(Serial);


//设置工作模式
#define SensorSwitch 2          // 1:SPI 2:IIC AS5600
#define Communication_object 0  // 0:2轮平衡||4轮平衡移动 1:simpleFOC Studio上位机 2：控制双电机 3:采样力矩数据
#define TorqueCompensation 0    // 1：力矩补偿  0：不补偿力矩 不能修改
#define SwitchUser 4            // 0:查看编码器位置与方向 1:采样电机1力矩补偿数据   2：采样电机2力矩补偿数据 3:力矩 4：速度 5:角度模式
#define CurrentUser 0           // 1:启动电流环
#define M2CurrentUser 0         // 1:电机2启动电流环

#define AdjusParameter 0        // 0:平衡速度航向横滚调参   1:顶球
#define SwitchingPattern 0      // 0：两轮 1：4轮  切换模式
#define MasterSlaveSelection 1  // 0:从机   1:主机

//机体
#define Thigh 0.035f        // 大腿长度m
#define Shank 0.072f        // 小腿长度m
float TargetLegLength = 0;  //目标腿长   用于串口通讯调整腿长
float LegLength = 0.06f;    //腿长
float BarycenterX = 0;      //质心X
float BodyPitching = 0;     //俯仰
float BodyRoll = 0;         //横滚
float MovementSpeed = 0;    //移动速度
float BodyTurn = 0;         //转向
float SlideStep = 0;        //滑步
float BodyX = 0;            //X位置 控制器输出
int RobotTumble = 0;        //机器摔倒

/**********************************************************************************************/
// 新增串口控制相关变量  尝试加一下串口控制
  float serialCH1 = 1500;  // 默认中位值
  float serialCH2 = 1500;
  float serialCH3 = 1500;
  float serialCH4 = 1500;
  bool newSerialData = false;  // 新数据标志

  bool serialControlActive = true;
  unsigned long lastSerialTime = 0;  // 最后一次串口命令时间
  const int SERIAL_TIMEOUT = 300;  // 串口超时时间(ms)

  bool remoteControlActive = false;   // 遥控器控制激活标志

  //定义滤波器结构体全局变量
  biquadFilter_t SerialFilterLPF[4]; // 4个通道的滤波器
  float serialLegLength_f = 0.06f;   // 滤波后的腿长
  float serialBodyRoll_f = 0;        // 滤波后的横滚
  float serialMovementSpeed_f = 0;   // 滤波后的移动速度
  float serialBodyTurn_f = 0;        // 滤波后的转向

  // 在全局变量区域添加
  struct SerialCommandCache {
      float legLength = 0.06f;
      float bodyRoll = 0;
      float movementSpeed = 0;
      float bodyTurn = 0;
      bool updated = false;
  };

  SerialCommandCache serialRawCache;  // 原始值缓存
  SerialCommandCache serialFiltered;  // 滤波后值
  unsigned long lastSerialUpdate = 0; // 最后更新时间

  const unsigned long SERIAL_TIMEOUT_MS = 300; // 300ms超时时间
  unsigned long lastSerialCommandTime = 0;     // 最后收到串口指令的时间

  //串口调试print
  unsigned long lastPrintTime = 0;
  const unsigned long PRINT_INTERVAL = 1000; // 1秒打印间隔

/*************************************************************************************/



body_t body;

float Rax[2];
float Lax[2];
float bodyH = 0.06f;
float bodyRoll = 0;


//滤波
float LegLength_f = 0.06f;     //腿长
float BarycenterX_f = 0;       //质心X
float BodyPitching_f = 0;      //俯仰
float BodyRoll_f = 0;          //横滚
float MovementSpeed_f = 0;     //移动速度
float BodyTurn_f = 0;          //转向
float SlideStep_f = 0;         //滑步
float BodyX_f = 0;             //X位置
biquadFilter_t FilterLPF[15];  //二阶低通滤波器
float TouchY_Pid_outputF = 0;
float TouchX_Pid_outputF = 0;

float cutoffFreq = 200;
float enableDFilter = 1;      
float LpfOut[6];  //

void CutoffFreq(char *cmd) {
  command.scalar(&cutoffFreq, cmd);
}

void EnableDFilter(char *cmd) {
  command.scalar(&enableDFilter, cmd);
}

const int LED_Pin = 35;  //LED IO
int LED_HL = 1;
int LED_count = 0;
int LED_dt = 100;
const int analogInPin = 17;       //电池电压IO
int sensorValue = 0;              // value read from the pot
biquadFilter_t VoltageFilterLPF;  //二阶低通滤波器
uint16_t VoltageADC = 0;          //电压ADC数据
float VoltageADCf = 0;            //电压ADC数据
float Voltage = 0;                //电压

//IMU
//创建 MahonyFilter 对象，设置比例增益和积分增益
MahonyFilter mahonyFilter(0.4f, 0.001f);
// 加速度计量程（这里设置为 ±8g）
const float accelRange = 8.0;  // 单位：g
// 陀螺仪量程（这里假设为 ±2000°/s）
const float gyroRange = 2000.0;  // 单位：°/
attitude_t attitude;
float roll_ok;   //当前横滚角
float pitch_ok;  //当前俯仰角

zeroBias_t zeroBias;  //零点偏移
unsigned long timestamp_prev = 0;
float IMUtime_dt = 0;

/*低通滤波参数*/
float RATE_HZ_last = 1000.0f;        //采样频率
float LPF_CUTOFF_FREQ_last = 50.0f;  //截止频率

float RATE_HZ = 1000.0f;         //采样频率
float LPF_CUTOFF_FREQ = 50.0f;   //截止频率
biquadFilter_t ImuFilterLPF[6];  //二阶低通滤波器


void ImuRATE_HZ(char *cmd) {
  command.scalar(&RATE_HZ, cmd);
}
void ImuLPF_CUTOFF_FREQ(char *cmd) {
  command.scalar(&LPF_CUTOFF_FREQ, cmd);
}

void Target_Leg_Length(char *cmd) {
  command.scalar(&TargetLegLength, cmd);
}



//互补滤波
float angleGyroX, angleGyroY, angleGyroZ,
  angleAccX, angleAccY;
float angleX, angleY, angleZ;
float accCoef = 0.02f;
float gyroCoef = 0.98f;



// 声明一个Preferences对象，用于后续对闪存进行数据的读写操作
Preferences preferences;
// 定义一个浮点型数组，用于存储欧拉角数据
float zeroBiasFlash[9];
// 定义一个字符串指针数组，存储横滚 俯仰角对应的键名，用户可以直接修改这些键名
// 键名用于在闪存中唯一标识数据
const char *zeroBiasKeys[9] = {
  "roll",
  "pitch",
  "gyroX",
  "gyroY",
  "gyroZ",
  "servoAngle1",
  "servoAngle2",
  "servoAngle3",
  "servoAngle4"
};


//校准欧拉角
const int CALL_COUNT = 100;  // 设定函数调用次数
int callCounter = 0;         // 调用计数器



//舵机
const int CUSTOM_SERVO_1_PIN = 11;
const int CUSTOM_SERVO_2_PIN = 12;
const int CUSTOM_SERVO_3_PIN = 21;
const int CUSTOM_SERVO_4_PIN = 14;

void zeroBias_servo1(char *cmd) {
  command.scalar(&zeroBias.servo1, cmd);
}
void zeroBias_servo2(char *cmd) {
  command.scalar(&zeroBias.servo2, cmd);
}
void zeroBias_servo3(char *cmd) {
  command.scalar(&zeroBias.servo3, cmd);
}
void zeroBias_servo4(char *cmd) {
  command.scalar(&zeroBias.servo4, cmd);
}



// 创建 ServoControl 对象，传入自定义的引脚
ServoControl servoControl(CUSTOM_SERVO_1_PIN, CUSTOM_SERVO_2_PIN, CUSTOM_SERVO_3_PIN, CUSTOM_SERVO_4_PIN);

//遥控器
FUTABA_SBUS sBus;
float sbuschx[8] = { 0 }; //遥控器通道数据
int sbus_dt_ms = 0;   //sbus采样间隔
int sbus_swa = 0;
int sbus_swb = 0;
int sbus_swc = 0;
int sbus_swd = 0;

#define SBUS_chMax 1792
#define SBUS_chMin 192


#define Serial1_START1 12
#define Serial1_START2 34
#define Serial1_END1 0


// 创建 PID 控制器实例
float Select = 0;             //选择要打印的数据
float CalibrationSelect = 0;  //保存校准数据0：校准结束  1：校准陀螺仪 2：校准欧拉角 3：校准舵机

float PidParameterTuning = 0;  //0：禁止调参 1：使能调参

PIDController AnglePid(11, 200, 0.2, 0, 0.1);          //4 22 0.08   (Kp, Ki, Kd ,ramp ,limit)
PIDController SpeedPid(0.1, 0.1, 0, 0, 50);     //
PIDController YawPid(11, 33, 0, 0, 0);          //
PIDController RollPid(0.06, 1.5, 0.003, 0, 2);  //

float control_torque_compensation = 0;  //控制力矩补偿

float PidDt = 0.01;

// 创建MyPIDController实例，设置初始参数
// avatii无负重
// MyPIDController Angle_Pid(8, 222, 0.08 , 0.1, 0, PidDt, 0, 0);  //p i d iLimit outputLimit dt EnableDFilter cutoffFreq
MyPIDController Angle_Pid(0, 0, 0, 0, 0, PidDt, 0, 0);  //p i d iLimit outputLimit dt EnableDFilter cutoffFreq
MyPIDController Speed_Pid(0, 0, 0, 0, 0, PidDt, 0, 0);
MyPIDController Yaw_Pid(0, 0, 0, 0, 0, PidDt, 0, 0);
MyPIDController Roll_Pid(0, 0, 0, 0, 0, PidDt, 0, 0);
MyPIDController Pitching_Pid(0, 0, 0, 0, 0, PidDt, 0, 0);

void ControlTorqueCompensation(char *cmd) {
  command.scalar(&control_torque_compensation, cmd);
}


void Pid_Parameter_Tuning(char *cmd) {
  command.scalar(&PidParameterTuning, cmd);
}

void TwoKp(char *cmd) {
  command.scalar(&mahonyFilter.twoKp, cmd);
}
void TwoKi(char *cmd) {
  command.scalar(&mahonyFilter.twoKi, cmd);
}

void KeyScalar(char *cmd) {
  command.scalar(&Select, cmd);
}
void KeyCalibration(char *cmd) {
  command.scalar(&CalibrationSelect, cmd);
}

#if AdjusParameter == 0
void CbAnglePid(char *cmd) {
  command.pid(&AnglePid, cmd);
}
void CbSpeedPid(char *cmd) {
  command.pid(&SpeedPid, cmd);
}
void CbYawPid(char *cmd) {
  command.pid(&YawPid, cmd);
}

void CbRollPid(char *cmd) {
  command.pid(&RollPid, cmd);
}

#elif AdjusParameter == 1

void CbTouchXPid(char *cmd) {
  command.pid(&TouchXPid, cmd);
}

void CbTouchYPid(char *cmd) {
  command.pid(&TouchYPid, cmd);
}
#endif


float Motor1_voltage_compensation = 0;
float Motor2_voltage_compensation = 0;
double Motor1_place_last = 0;
float Motor1_Velocity = 0;
float Motor1_Velocity_f = 0;
LowPassFilter Motor1_Velocity_filter = LowPassFilter(0.01);  // Tf = 10ms

double Motor2_place_last = 0;
float Motor2_Velocity = 0;
float Motor2_Velocity_f = 0;
LowPassFilter Motor2_Velocity_filter = LowPassFilter(0.01);  // Tf = 10ms

Serial_t serial1;
Serial_t serial2;

float Motor1_Target = 0;
float Motor2_Target = 0;

float time_dt = 0;
unsigned long now_us = 0;
unsigned long now_us1 = 0;
unsigned long now_us2 = 0;
// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(7);  //电机极对数
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 7, 6, 16);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(40, 39, 38, 37);

#if SensorSwitch == 1
// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// config           - SPI config
//  cs              - SPI chip select pin
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5147_SPI, 19);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5147_SPI, 23);
// these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board (stm32f107rc)
SPIClass *hspi = NULL;

#elif SensorSwitch == 2
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

#endif

// 1. 定义电流传感器参数
#define SENSOR_MV_PER_AMP 90.0f  // ACS712-05B 的灵敏度为 185mV/A

#if CurrentUser == 1
// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
InlineCurrentSense current_sense1 = InlineCurrentSense(SENSOR_MV_PER_AMP, 18, 17);
#endif

#if M2CurrentUser == 1
// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
InlineCurrentSense current_sense2 = InlineCurrentSense(SENSOR_MV_PER_AMP, 35, 36);
#endif


void doMotion1(char *cmd) {
  command.motion(&motor1, cmd);
}
void doMotor1(char *cmd) {
  command.motor(&motor1, cmd);
}

void doMotion2(char *cmd) {
  command.motion(&motor2, cmd);
}
void doMotor2(char *cmd) {
  command.motor(&motor2, cmd);
}

void RXsbus();
int RightInverseKinematics(float x, float y, float p, float *ax);
int LeftInverseKinematics(float x, float y, float p, float *ax);
void print_data(void);
void ImuUpdate(void);
void FlashSave(int sw);
void FlashInit(void);
void PIDcontroller_posture(float dt);
void RemoteControlFiltering(void);
void ReadVoltage(void);
void PidParameter(void);
void Robot_Tumble(void);
void body_data_init(void);
void motor_init(void);

void command_init(void);

void command_init(void) { //串口命令定义
  // subscribe motor to the commander
  command.add('T', doMotion1, "motion1 control");  //设置电机目标值
  command.add('M', doMotor1, "motor1");

  command.add('A', zeroBias_servo1, "my zeroBias_servo1");  //设置舵机1偏差
  command.add('B', zeroBias_servo2, "my zeroBias_servo2");  //
  command.add('C', zeroBias_servo3, "my zeroBias_servo3");  //
  command.add('D', zeroBias_servo4, "my zeroBias_servo4");  //

  command.add('H', ImuRATE_HZ, "my ImuRATE_HZ");
  command.add('Z', ImuLPF_CUTOFF_FREQ, "my ImuLPF_CUTOFF_FREQ");


  command.add('Q', TwoKp, "my TwoKp");  //MahonyFilter
  command.add('I', TwoKi, "my TwoKi");  //MahonyFilter

  command.add('K', KeyScalar, "my Select");
  command.add('E', KeyCalibration, "my CalibrationSelect");

  command.add('F', CutoffFreq, "my CutoffFreq");
  command.add('J', EnableDFilter, "my EnableDFilter");

#if AdjusParameter == 0
  command.add('P', CbAnglePid, "my AnglePid");
  command.add('S', CbSpeedPid, "my SpeedPid");
  command.add('Y', CbYawPid, "my YawPid");
  command.add('R', CbRollPid, "my RollPid");
  command.add('O', Target_Leg_Length, "my Target_Leg_Length");
#elif AdjusParameter == 1
  command.add('L', CbTouchXPid, "my CbTouchXPid");
  command.add('N', CbTouchYPid, "my CbTouchYPid");
  command.add('G', ControlTorqueCompensation, "my ControlTorqueCompensation");
#endif

  command.add('U', Pid_Parameter_Tuning, "my Pid_Parameter_Tuning");
}

void motor_init(void) {

  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

#if SensorSwitch == 1
  hspi = new SPIClass(HSPI);
  hspi->begin(18, 5, 17);  //(sck, miso, mosi)
  //initialise magnetic sensor1 hardware
  sensor1.init(hspi);
  sensor2.init(hspi);
#elif SensorSwitch == 2
  I2Cone.begin(4, 5, 400000);
  I2Ctwo.begin(41, 42, 400000);  //SDA1,SCL1
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
#endif


  // sensor1.min_elapsed_time = 0.0001; // 100us by default
  // sensor2.min_elapsed_time = 0.0001; // 100us by default

  // link the motor to the sensor
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 8.4;
  driver.init();

  driver2.voltage_power_supply = 8.4;
  driver2.init();
  // link driver
  motor1.linkDriver(&driver);
  motor2.linkDriver(&driver2);
  // link current sense and the driver
#if CurrentUser == 1
  current_sense1.linkDriver(&driver);
#endif

#if M2CurrentUser == 1
  current_sense2.linkDriver(&driver2);
#endif


  // control loop type and torque mode  velocity angle
  if (CurrentUser == 1)
    motor1.torque_controller = TorqueControlType::dc_current;  //foc_current   dc_current  voltage
  else
    motor1.torque_controller = TorqueControlType::voltage;

  if ((SwitchUser == 1) || (SwitchUser == 5))
    motor1.controller = MotionControlType::angle;
  else if (SwitchUser == 3)
    motor1.controller = MotionControlType::torque;
  else if (SwitchUser == 4)
    motor1.controller = MotionControlType::velocity;

  motor1.motion_downsample = 0.0;  //

  // velocity loop PID
  motor1.PID_velocity.P = 0.006;  //0.07;
  motor1.PID_velocity.I = 0;
  if (SwitchingPattern == 1)
    motor1.PID_velocity.I = 0.8;
  motor1.PID_velocity.D = 0.0;
  motor1.PID_velocity.output_ramp = 10000;
  motor1.PID_velocity.limit = 8.4;
  // Low pass filtering time constant
  motor1.LPF_velocity.Tf = 0.001;
  // angle loop PID
  motor1.P_angle.P = 15.0;
  motor1.P_angle.I = 22.0;
  motor1.P_angle.D = 0.0;
  motor1.P_angle.output_ramp = 10000;
  motor1.P_angle.limit = 111.0;
  // Low pass filtering time constant
  motor1.LPF_angle.Tf = 0.001;
  // current q loop PID
  motor1.PID_current_q.P = 2;
  motor1.PID_current_q.I = 222;
  motor1.PID_current_q.D = 0.0;
  motor1.PID_current_q.output_ramp = 11111;
  motor1.PID_current_q.limit = 8.4;
  // Low pass filtering time constant
  motor1.LPF_current_q.Tf = 0.01;
  // current d loop PID
  motor1.PID_current_d.P = motor1.PID_current_q.P;
  motor1.PID_current_d.I = motor1.PID_current_q.I;
  motor1.PID_current_d.D = motor1.PID_current_q.D;
  motor1.PID_current_d.output_ramp = motor1.PID_current_q.output_ramp;
  motor1.PID_current_d.limit = motor1.PID_current_q.limit;
  // Low pass filtering time constant
  motor1.LPF_current_d.Tf = motor1.LPF_current_q.Tf;
  // Limits
  motor1.velocity_limit = 88.0;
  motor1.voltage_limit = 8.4;
  motor1.current_limit = 5.0;
  // sensor zero offset - home position
  //motor1.sensor_offset = -68924.77299999999;
  // general settings
  // motor phase resistance
  motor1.phase_resistance = 22;
  // pwm modulation settings
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // 设置 PWM 调制为中心对齐模式
  motor1.modulation_centered = 1.0;

  if (M2CurrentUser == 1)
    // control loop type and torque mode velocity angle
    motor2.torque_controller = TorqueControlType::foc_current;  //foc_current   dc_current  voltage
  else
    // control loop type and torque mode velocity angle
    motor2.torque_controller = TorqueControlType::voltage;  //foc_current   dc_current  voltage


  if ((SwitchUser == 2) || (SwitchUser == 5))
    motor2.controller = MotionControlType::angle;
  else if (SwitchUser == 3)
    motor2.controller = MotionControlType::torque;
  else if (SwitchUser == 4)
    motor2.controller = MotionControlType::velocity;


  motor2.motion_downsample = 0.0;

  // velocity loop PID
  motor2.PID_velocity.P = 0.006;
  motor2.PID_velocity.I = 0;
  if (SwitchingPattern == 1)
    motor2.PID_velocity.I = 0.8;
  motor2.PID_velocity.D = 0;
  motor2.PID_velocity.output_ramp = 10000;
  motor2.PID_velocity.limit = 8.4;
  // Low pass filtering time constant
  motor2.LPF_velocity.Tf = 0.001;
  // angle loop PID
  motor2.P_angle.P = 22;
  motor2.P_angle.I = 111;
  motor2.P_angle.D = 0;
  motor2.P_angle.output_ramp = 10000;
  motor2.P_angle.limit = 88;
  // Low pass filtering time constant
  motor2.LPF_angle.Tf = 0.001;

  // current q loop PID
  motor2.PID_current_q.P = 2;
  motor2.PID_current_q.I = 222;
  motor2.PID_current_q.D = 0.0;
  motor2.PID_current_q.output_ramp = 11111;
  motor2.PID_current_q.limit = 8.4;
  // Low pass filtering time constant
  motor2.LPF_current_q.Tf = 0.01;
  // current d loop PID
  motor2.PID_current_d.P = motor2.PID_current_q.P;
  motor2.PID_current_d.I = motor2.PID_current_q.I;
  motor2.PID_current_d.D = motor2.PID_current_q.D;
  motor2.PID_current_d.output_ramp = motor2.PID_current_q.output_ramp;
  motor2.PID_current_d.limit = motor2.PID_current_q.limit;
  // Low pass filtering time constant
  motor2.LPF_current_d.Tf = motor2.LPF_current_q.Tf;

  // Limits
  motor2.velocity_limit = motor1.velocity_limit;
  motor2.voltage_limit = motor1.voltage_limit;
  motor2.current_limit = motor1.current_limit;
  // sensor zero offset - home position
  //motor2.sensor_offset = -68924.77299999999;
  // general settings
  // motor phase resistance
  motor2.phase_resistance = motor1.phase_resistance;
  // pwm modulation settings
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.modulation_centered = motor1.modulation_centered;

#if CurrentUser == 1
  // current sense init and linking
  current_sense1.init();
  motor1.linkCurrentSense(&current_sense1);
#endif

#if M2CurrentUser == 1
  // current sense init and linking
  current_sense2.init();
  motor2.linkCurrentSense(&current_sense2);
#endif



  // initialise motor
  motor1.init();
  motor2.init();
  // align encoder and start FOC

  if (SwitchUser == 0) {
    motor1.initFOC();
    motor2.initFOC();
    Serial.print("Sensor1 zero offset is:");
    Serial.print(motor1.zero_electric_angle, 6);  //初始电角度
    Serial.print("  Sensor1 natural direction is: ");
    Serial.println(motor1.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");  //电机转动方向（顺时针、逆时针）

    Serial.print("Sensor2 zero offset is:");
    Serial.print(motor2.zero_electric_angle, 6);  //初始电角度
    Serial.print("  Sensor2 natural direction is: ");
    Serial.println(motor2.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");  //电机转动方向（顺时针、逆时针）

    while (1)
      ;
  } else {

    //motor1.sensor_direction=Direction::CCW; // or Direction::CCW
    //motor1.zero_electric_angle=2.586293;   // use the real value!
    motor1.initFOC();

    //motor2.sensor_direction=Direction::CW; // or Direction::CCW
    //motor2.zero_electric_angle=4.166292;   // use the real value!
    motor2.initFOC();
  }


  // set the inital target value
  motor1.target = 0;
  motor2.target = 0;


  // comment out if not needed

  motor1.useMonitoring(Serial);
  motor1.monitor_downsample = 10;  // disable intially
  //motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; // monitor target velocity and angle
}

//映射
float mapf(long x, long in_min, long in_max, float out_min, float out_max) {
  long divisor = (in_max - in_min);
  if (divisor == 0) {
    return -1;  //AVR returns -1, SAM returns 0
  }
  return (x - in_min) * (out_max - out_min) / divisor + out_min;
}

void RXsbus() { //sbus接收处理函数

  //if (!remoteControlActive) return;  // 串口控制激活时跳过遥控器处理(这段是自己加的)

  static unsigned long now_ms = millis();

  sBus.FeedLine();
  
  if (sBus.toChannels == 1) { //有新的一帧数据可用
    remoteControlActive = true;  // 遥控器控制激活
    serialControlActive = false;  // 串口控制禁用
    sbus_dt_ms = millis() - now_ms;
    now_ms = millis();
    sBus.toChannels = 0;
    sBus.UpdateChannels();
    sBus.toChannels = 0;


    if(!sBus.Failsafe() == 0x00){
      serialControlActive = true;  // 串口控制激活
      remoteControlActive = false;  // 遥控器控制激活
      return;  // 如果遥控器进入失控状态，直接返回
    }

    MovementSpeed = mapf(sBus.channels[2], SBUS_chMin, SBUS_chMax, -15, 15);
    BodyTurn = -mapf(sBus.channels[3], SBUS_chMin, SBUS_chMax, -11, 11);
    sbus_swa = map(sBus.channels[4], SBUS_chMin, SBUS_chMax, 0, 2);
    sbus_swb = map(sBus.channels[5], SBUS_chMin, SBUS_chMax, 0, 1);
    sbus_swc = map(sBus.channels[6], SBUS_chMin, SBUS_chMax, 0, 1);
    sbus_swd = map(sBus.channels[7], SBUS_chMin, SBUS_chMax, 0, 2);
    sbuschx[8] = map(sBus.channels[8], SBUS_chMin, SBUS_chMax, 0, 100);
    sbuschx[9] = map(sBus.channels[9], SBUS_chMin, SBUS_chMax, 0, 100);


    if (sbus_swd == 0)  //姿态控制1
    {
      //SlideStep = mapf(sBus.channels[0],SBUS_chMin, SBUS_chMax, -0.05,0.05);//滑步
      if (sBus.channels[1] <= 992)
        LegLength = mapf(sBus.channels[1], SBUS_chMin, 992, 0.05, 0.06);  //双腿高
      else
        LegLength = mapf(sBus.channels[1], 993, SBUS_chMax, 0.06, 0.1);  //双腿高
    } else if (sbus_swd == 1)                                            //姿态控制2
    {
      //BodyRoll =  mapf(sBus.channels[0], SBUS_chMin, SBUS_chMax, -0.011, 0.011); //横滚
      //sbus_vrb    =  mapf(sBus.channels[9], SBUS_chMin, SBUS_chMax, -25, 25);
      BodyPitching = mapf(sBus.channels[1], SBUS_chMin, SBUS_chMax, -12, 12);  //俯仰     + sbus_vrb
    } else if (sbus_swd == 2)                                                  //姿态控制3
    {
      //LegLength = 0.06;//双腿高
      if (sBus.channels[1] <= 992)
        LegLength = mapf(sBus.channels[1], SBUS_chMin, 992, 0.05, 0.06);  //双腿高
      else
        LegLength = mapf(sBus.channels[1], 993, SBUS_chMax, 0.06, 0.07);  //双腿高
    }
    BodyRoll = mapf(sBus.channels[0], SBUS_chMin, SBUS_chMax, -0.011, 0.011);  //横滚
  }
}


float ArcToAngle(float arc)  //弧度转角度
{
  float angle = arc * (180 / PI);
  return angle;
}

float AngleToArc(float angle)  //角度转弧度
{
  float art = angle * (PI / 180);
  return art;
}


//五连杆动学逆解
int RightInverseKinematics(float x, float y, float p, float *ax) {
  x = constrain(x, -0.05, 0.05);
  y = constrain(y, 0.05, 0.1);

  int error = 0;     //坐标设置异常
  float AB = Thigh;  // 大腿长度m AB=ED
  float BC = Shank;  // 小腿长度m BC=DC

  float OA = 0.017f;  //
  float aOCF = 0;
  float aOCF2 = 0;
  float aAOC = 0;
  float OC = 0;
  float OF = 0;
  float FC = 0;
  float AC = 0;
  float aOAC = 0;
  float aOCA = 0;
  float aBAC = 0;
  float aBAG = 0;
  float OE = OA;  //
  float aEOC = 0;
  float EC = 0;
  float aOCE = 0;
  float aOEC = 0;
  float aDEC = 0;
  float aDEH = 0;

  float pitch, x1, y1;

  pitch = AngleToArc(p);  //俯仰角

  x1 = x * cosf(pitch) - y * sinf(pitch); //转换至有俯仰角的坐标系内
  y1 = x * sinf(pitch) + y * cosf(pitch);


  OF = x1;
  FC = y1;

  //关节1
  OC = sqrtf(pow(OF, 2) + pow(FC, 2));  //https://zh.numberempire.com/right_triangle_calculator.php
  aOCF = asinf(OF / OC);
  aAOC = AngleToArc(90) + aOCF;

  AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC));  //https://zh.numberempire.com/arbitrary_triangle_calculator.php?__cf_chl_tk=vr4WMdDsGMhU1V4wG4xVXMisQ2Q6fSgLDuKK6VlnV4o-1734502544-1.0.1.1-CoxTBWpWtqs3CDDFrz5boHxA3INdl0yYHMOzF5kZz8w
  aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
  aOAC = PI - aOCA - aAOC;
  aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
  aBAG = PI - aBAC - aOAC;
  ax[0] = ArcToAngle(aBAG);  //关节1角度

  //关节2
  aOCF2 = -aOCF;
  aEOC = AngleToArc(90) + aOCF2;

  EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
  aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
  aOEC = PI - aOCE - aEOC;
  aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
  aDEH = PI - aDEC - aOEC;
  ax[1] = ArcToAngle(aDEH);  //关节2角度


  if (AC >= (AB + BC))  //超出结构最大范围
    return error = 1;
  else if (EC >= (AB + BC))  //超出结构最大范围
    return error = 2;


  return error;
}


int LeftInverseKinematics(float x, float y, float p, float *ax) {
  x = constrain(x, -0.05, 0.05);
  y = constrain(y, 0.05, 0.1);

  x = -x;
  p = -p;
  int error = 0;     //坐标设置异常
  float AB = Thigh;  // 大腿长度m AB=ED
  float BC = Shank;  // 小腿长度m BC=DC

  float OA = 0.017f;  //
  float aOCF = 0;
  float aOCF2 = 0;
  float aAOC = 0;
  float OC = 0;
  float OF = 0;
  float FC = 0;
  float AC = 0;
  float aOAC = 0;
  float aOCA = 0;
  float aBAC = 0;
  float aBAG = 0;
  float OE = OA;  //
  float aEOC = 0;
  float EC = 0;
  float aOCE = 0;
  float aOEC = 0;
  float aDEC = 0;
  float aDEH = 0;

  float pitch, x1, y1;

  pitch = AngleToArc(p);  //俯仰角

  x1 = x * cosf(pitch) - y * sinf(pitch);
  y1 = x * sinf(pitch) + y * cosf(pitch);


  OF = -x1;
  FC = y1;

  //关节1
  OC = sqrtf(pow(OF, 2) + pow(FC, 2));  //https://zh.numberempire.com/right_triangle_calculator.php
  aOCF = asinf(OF / OC);
  aAOC = AngleToArc(90) + aOCF;

  AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC));  //https://zh.numberempire.com/arbitrary_triangle_calculator.php?__cf_chl_tk=vr4WMdDsGMhU1V4wG4xVXMisQ2Q6fSgLDuKK6VlnV4o-1734502544-1.0.1.1-CoxTBWpWtqs3CDDFrz5boHxA3INdl0yYHMOzF5kZz8w
  aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
  aOAC = PI - aOCA - aAOC;
  aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
  aBAG = PI - aBAC - aOAC;
  ax[0] = ArcToAngle(aBAG);  //关节1角度

  //关节2
  aOCF2 = -aOCF;
  aEOC = AngleToArc(90) + aOCF2;

  EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
  aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
  aOEC = PI - aOCE - aEOC;
  aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
  aDEH = PI - aDEC - aOEC;
  ax[1] = ArcToAngle(aDEH);  //关节1角度


  if (AC >= (AB + BC))  //超出结构最大范围
    return error = 1;
  else if (EC >= (AB + BC))  //超出结构最大范围
    return error = 2;


  return error;
}


void ImuUpdate(void) {
  unsigned long timestamp_now = micros();
  IMUtime_dt = (timestamp_now - timestamp_prev) * 1e-6f;  //计算间隔时间

  int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp;
  readIMUData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp);

  // 减去陀螺仪零偏值
  gyroX -= (int16_t)gyroBiasX;
  gyroY -= (int16_t)gyroBiasY;
  gyroZ -= (int16_t)gyroBiasZ;

  // 转换加速度计数据为 g 单位
  attitude.acc.x = (float)accelX * accelRange / 32768.0;
  attitude.acc.y = (float)accelY * accelRange / 32768.0;
  attitude.acc.z = (float)accelZ * accelRange / 32768.0;


  // 转换陀螺仪数据为 rad/s
  attitude.gyro.x = (float)gyroX * gyroRange / 32768.0 * (3.1415926f / 180.0f);
  attitude.gyro.y = (float)gyroY * gyroRange / 32768.0 * (3.1415926f / 180.0f);
  attitude.gyro.z = (float)gyroZ * gyroRange / 32768.0 * (3.1415926f / 180.0f);


  //软件二阶低通滤波
  attitude.gyrof.x = biquadFilterApply(&ImuFilterLPF[0], attitude.gyro.x);
  attitude.gyrof.y = biquadFilterApply(&ImuFilterLPF[1], attitude.gyro.y);
  attitude.gyrof.z = biquadFilterApply(&ImuFilterLPF[2], attitude.gyro.z);

  attitude.accf.x = biquadFilterApply(&ImuFilterLPF[3], attitude.acc.x);
  attitude.accf.y = biquadFilterApply(&ImuFilterLPF[4], attitude.acc.y);
  attitude.accf.z = biquadFilterApply(&ImuFilterLPF[5], attitude.acc.z);


  // 转换温度数据
  attitude.temp = (float)temp / 132.48 + 25;
  // 运行 Mahony 滤波算法，传入 dt
  //mahonyFilter.update(attitude.gyro.x, attitude.gyro.y, attitude.gyro.z, attitude.acc.x, attitude.acc.y, attitude.acc.z, IMUtime_dt);
  mahonyFilter.update(attitude.gyrof.x, attitude.gyrof.y, attitude.gyrof.z, attitude.accf.x, attitude.accf.y, attitude.accf.z, IMUtime_dt);

  // 获取滤波后的四元数
  float q0_out, q1_out, q2_out, q3_out;
  mahonyFilter.getQuaternion(q0_out, q1_out, q2_out, q3_out);
  // 四元数转欧拉角
  quaternionToEuler(q0_out, q1_out, q2_out, q3_out, &attitude.roll, &attitude.pitch, &attitude.yaw);

  roll_ok = attitude.roll - zeroBias.roll;
  pitch_ok = attitude.pitch - zeroBias.pitch;


  //////互补滤波//////
  angleAccX = atan2(attitude.acc.y, attitude.acc.z + abs(attitude.acc.x)) * 360 / 2.0 / PI;
  angleAccY = atan2(attitude.acc.x, attitude.acc.z + abs(attitude.acc.y)) * 360 / -2.0 / PI;

  gyroX = (float)gyroX * gyroRange / 32768.0;
  gyroY = (float)gyroY * gyroRange / 32768.0;
  gyroZ = (float)gyroZ * gyroRange / 32768.0;

  angleGyroX += gyroX * IMUtime_dt;
  angleGyroY += gyroY * IMUtime_dt;
  angleGyroZ += gyroZ * IMUtime_dt;

  angleX = (gyroCoef * (angleX + gyroX * IMUtime_dt)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * IMUtime_dt)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;


  timestamp_prev = timestamp_now;
}


void FlashInit(void) {
  preferences.begin("preferences", false);

  //读出数据  如果读取失败（即闪存中不存在该键名对应的数据），则使用默认值0.0
  zeroBias.roll = preferences.getFloat(zeroBiasKeys[0], 0.0);
  zeroBias.pitch = preferences.getFloat(zeroBiasKeys[1], 0.0);

  //读出数据  如果读取失败（即闪存中不存在该键名对应的数据），则使用默认值0.0
  gyroBiasX = preferences.getFloat(zeroBiasKeys[2], 0.0);
  gyroBiasY = preferences.getFloat(zeroBiasKeys[3], 0.0);
  gyroBiasZ = preferences.getFloat(zeroBiasKeys[4], 0.0);

  //舵机角度
  zeroBias.servo1 = preferences.getFloat(zeroBiasKeys[5], 0.0);
  zeroBias.servo2 = preferences.getFloat(zeroBiasKeys[6], 0.0);
  zeroBias.servo3 = preferences.getFloat(zeroBiasKeys[7], 0.0);
  zeroBias.servo4 = preferences.getFloat(zeroBiasKeys[8], 0.0);


  // 关闭闪存访问，释放相关资源
  preferences.end();

  Serial.println(" ");
  // 输出零偏
  Serial.print("  Roll Zero Bias: ");
  Serial.print(zeroBias.roll);
  Serial.print("  Pitch Zero Bias: ");
  Serial.println(zeroBias.pitch);

  // 输出零偏
  Serial.print("  gyroBiasX:");
  Serial.print(gyroBiasX);
  Serial.print("  gyroBiasY:");
  Serial.print(gyroBiasY);
  Serial.print("  gyroBiasZ:");
  Serial.println(gyroBiasZ);


  // 输出零偏
  Serial.print("  servo1:");
  Serial.print(zeroBias.servo1);
  Serial.print("  servo2:");
  Serial.print(zeroBias.servo2);
  Serial.print("  servo3:");
  Serial.print(zeroBias.servo3);
  Serial.print("  servo4:");
  Serial.println(zeroBias.servo4);
}


// 计算欧拉角零偏的函数
void calculateZeroBias() {
  // 初始化累加器
  static float rollSum = 0;
  static float pitchSum = 0;

  // 累加欧拉角
  rollSum += attitude.roll;
  pitchSum += attitude.pitch;

  // 增加调用计数器
  callCounter++;

  if (callCounter >= CALL_COUNT) {
    // 计算平均值，得到零偏
    zeroBias.roll = rollSum / CALL_COUNT;
    zeroBias.pitch = pitchSum / CALL_COUNT;

    // 初始化闪存访问，打开名为 "preferences" 的命名空间
    // 第二个参数为false，表示以可写模式打开命名空间
    preferences.begin("preferences", false);

    //写入数据
    zeroBiasFlash[0] = zeroBias.roll;
    preferences.putFloat(zeroBiasKeys[0], zeroBiasFlash[0]);
    zeroBiasFlash[1] = zeroBias.pitch;
    preferences.putFloat(zeroBiasKeys[1], zeroBiasFlash[1]);
    //读出数据  如果读取失败（即闪存中不存在该键名对应的数据），则使用默认值0.0
    zeroBias.roll = preferences.getFloat(zeroBiasKeys[0], 0.0);
    zeroBias.pitch = preferences.getFloat(zeroBiasKeys[1], 0.0);

    // 关闭闪存访问，释放相关资源
    preferences.end();

    // 输出零偏
    Serial.print("  Roll Zero Bias: ");
    Serial.print(zeroBias.roll);
    Serial.print("  Pitch Zero Bias: ");
    Serial.println(zeroBias.pitch);
    rollSum = 0;
    pitchSum = 0;
    callCounter = 0;        //清零下次用
    CalibrationSelect = 0;  //校准完整退出校准
  }
}


void FlashSave(int sw) {

  static float servo1_last = zeroBias.servo1;  //上一次偏差
  static float servo2_last = zeroBias.servo2;
  static float servo3_last = zeroBias.servo3;
  static float servo4_last = zeroBias.servo4;

  switch (sw) {
    case 1:
      // 进行陀螺仪校准
      calibrateGyro();
      preferences.begin("preferences", false);

      //写入数据
      preferences.putFloat(zeroBiasKeys[2], gyroBiasX);
      preferences.putFloat(zeroBiasKeys[3], gyroBiasY);
      preferences.putFloat(zeroBiasKeys[4], gyroBiasZ);

      //读出数据  如果读取失败（即闪存中不存在该键名对应的数据），则使用默认值0.0
      gyroBiasX = preferences.getFloat(zeroBiasKeys[2], 0.0);
      gyroBiasY = preferences.getFloat(zeroBiasKeys[3], 0.0);
      gyroBiasZ = preferences.getFloat(zeroBiasKeys[4], 0.0);

      // 关闭闪存访问，释放相关资源
      preferences.end();

      // 输出零偏
      Serial.print("  gyroBiasX:");
      Serial.print(gyroBiasX);
      Serial.print("  gyroBiasY:");
      Serial.print(gyroBiasY);
      Serial.print("  gyroBiasZ:");
      Serial.println(gyroBiasZ);


      CalibrationSelect = 0;  //校准结束
      break;

    case 2:
      // 计算欧拉角零偏的函数
      calculateZeroBias();

      break;

    case 3:

      preferences.begin("preferences", false);

      if (zeroBias.servo1 != servo1_last)  //参数调整过就保存
      {
        servo1_last = zeroBias.servo1;  //
        //写入数据
        preferences.putFloat(zeroBiasKeys[5], zeroBias.servo1);
        //读出舵机角度
        zeroBias.servo1 = preferences.getFloat(zeroBiasKeys[5], 0.0);
        //打印数据
        Serial.print("  zeroBias.servo1:");
        Serial.println(zeroBias.servo1);
      }

      if (zeroBias.servo2 != servo2_last)  //参数调整过就保存
      {
        servo2_last = zeroBias.servo2;  //
        //写入数据
        preferences.putFloat(zeroBiasKeys[6], zeroBias.servo2);
        //读出舵机角度
        zeroBias.servo2 = preferences.getFloat(zeroBiasKeys[6], 0.0);
        //打印数据
        Serial.print("  zeroBias.servo2:");
        Serial.println(zeroBias.servo2);
      }

      if (zeroBias.servo3 != servo3_last)  //参数调整过就保存
      {
        servo3_last = zeroBias.servo3;  //
        //写入数据
        preferences.putFloat(zeroBiasKeys[7], zeroBias.servo3);
        //读出舵机角度
        zeroBias.servo3 = preferences.getFloat(zeroBiasKeys[7], 0.0);
        //打印数据
        Serial.print("  zeroBias.servo3:");
        Serial.println(zeroBias.servo3);
      }

      if (zeroBias.servo4 != servo4_last)  //参数调整过就保存
      {
        servo4_last = zeroBias.servo4;  //
        //写入数据
        preferences.putFloat(zeroBiasKeys[8], zeroBias.servo4);
        //读出舵机角度
        zeroBias.servo4 = preferences.getFloat(zeroBiasKeys[8], 0.0);
        //打印数据
        Serial.print("  zeroBias.servo4:");
        Serial.println(zeroBias.servo4);
      }


      // 关闭闪存访问，释放相关资源
      preferences.end();
      //CalibrationSelect = 0;//手动设置结束校准

      break;



    default:

      break;
  }
}


void print_data(void) {
  switch ((int)Select) {
    case 1:
      // 输出欧拉角
      Serial.print("dt:");
      Serial.print(time_dt, 6);
      Serial.print(" Roll:");
      Serial.print(attitude.roll);
      Serial.print(" Pitch:");
      Serial.print(attitude.pitch);
      Serial.print(" Yaw:");
      Serial.println(attitude.yaw);

      break;

    case 2:
      // 输出acc
      Serial.print("dt:");
      Serial.print(time_dt, 6);
      Serial.print(" accx:");
      Serial.print(attitude.acc.x);
      Serial.print(" accy:");
      Serial.print(attitude.acc.y);
      Serial.print(" accz:");
      Serial.println(attitude.acc.z);

      break;

    case 3:
      // 输出
      Serial.print("dt:");
      Serial.print(time_dt, 6);
      Serial.print(" gyrox:");
      Serial.print(attitude.gyro.x, 4);
      Serial.print(" gyroy:");
      Serial.print(attitude.gyro.y, 4);
      Serial.print(" gyroz:");
      Serial.println(attitude.gyro.z, 4);

      break;

    case 4:
      // 输出欧拉角
      Serial.print("dt:");
      Serial.print(time_dt, 6);
      Serial.print(" Roll:");
      Serial.print(attitude.roll - zeroBias.roll);
      Serial.print(" Pitch:");
      Serial.print(attitude.pitch - zeroBias.pitch);
      Serial.print(" Yaw:");
      Serial.println(attitude.yaw - zeroBias.yaw);

      break;

    case 5:
      //
      Serial.print("dt:");
      Serial.print(time_dt, 6);
      Serial.print(" eRoll:");
      Serial.print(zeroBias.roll);
      Serial.print(" ePitch:");
      Serial.print(zeroBias.pitch);
      Serial.print(" eYaw:");
      Serial.println(zeroBias.yaw);

      break;

    case 6:
      //
      Serial.print(" v1:");
      Serial.print(Motor1_Velocity);
      Serial.print(" v2:");
      Serial.println(Motor2_Velocity);


      break;

    case 7:
      //
      Serial.print(" v1:");
      Serial.print(Motor1_Velocity);
      Serial.print(" v1f:");
      Serial.println(Motor1_Velocity_f);
      break;

    case 8:
      //
      for (int i = 0; i < 10; i++) {
        Serial.print(" ch:");
        Serial.print(sBus.channels[i]);
      }

      Serial.print(" sbus_dt_ms:");
      Serial.print(sbus_dt_ms);
      Serial.println(" ");
      break;

    case 9:
      //
      Serial.print(" PP:");
      Serial.print(Angle_Pid.Kp);
      Serial.print(" PI:");
      Serial.print(Angle_Pid.Ki);
      Serial.print(" PD:");
      Serial.print(Angle_Pid.Kd);

      Serial.print(" SP:");
      Serial.print(Speed_Pid.Kp);
      Serial.print(" SI:");
      Serial.print(Speed_Pid.Ki);
      Serial.print(" SD:");
      Serial.print(Speed_Pid.Kd);

      Serial.print(" YP:");
      Serial.print(Yaw_Pid.Kp);
      Serial.print(" YI:");
      Serial.print(Yaw_Pid.Ki);
      Serial.print(" YD:");
      Serial.print(Yaw_Pid.Kd);

      Serial.print("dt:");
      Serial.println(time_dt, 6);

      break;


    case 10:
      //
      Serial.print(" twoKp:");
      Serial.print(mahonyFilter.twoKp);
      Serial.print(" twoKi:");
      Serial.print(mahonyFilter.twoKi);

      Serial.print(" Roll:");
      Serial.print(attitude.roll);
      Serial.print(" Pitch:");
      Serial.print(attitude.pitch);

      Serial.print("IMUdt:");
      Serial.println(IMUtime_dt, 6);
      break;

    case 11:
      //

      Serial.print(" x:");
      Serial.print(angleX);
      Serial.print(" y:");
      Serial.print(angleY);
      Serial.print(" z:");
      Serial.print(angleZ);


      Serial.print(" gx:");
      Serial.print(angleGyroX);
      Serial.print(" gy:");
      Serial.print(angleGyroY);
      Serial.print(" gz:");
      Serial.print(angleGyroZ);


      Serial.print(" IMUdt:");
      Serial.println(IMUtime_dt, 6);

      break;

    case 12:
      //

      Serial.print(" x:");
      Serial.print(angleX);
      //Serial.print(" y:");
      //Serial.print( angleY);

      Serial.print(" Roll:");
      Serial.println(attitude.roll);
      //Serial.print(" Pitch:");
      //Serial.print( attitude.pitch);


      //Serial.print(" IMUdt:");
      //Serial.println(IMUtime_dt,6);

      break;


    case 13:
      // 输出
      Serial.print("dt:");
      Serial.print(time_dt, 6);
      Serial.print(" gyroxf:");
      Serial.print(attitude.gyrof.x);
      Serial.print(" gyroyf:");
      Serial.print(attitude.gyrof.y);
      Serial.print(" gyrozf:");
      Serial.println(attitude.gyrof.z);
      break;


    case 14:
      // 输出acc
      Serial.print("dt:");
      Serial.print(time_dt, 6);
      Serial.print(" accx:");
      Serial.print(attitude.accf.x);
      Serial.print(" accy:");
      Serial.print(attitude.accf.y);
      Serial.print(" accz:");
      Serial.println(attitude.accf.z);
      break;


    case 15:
      // 输出acc
      //Serial.print("dt:");
      //Serial.print(time_dt,6);
      Serial.print(" accy:");
      Serial.print(attitude.acc.y);
      Serial.print(" accyf:");
      Serial.println(attitude.accf.y);
      break;

    case 16:
      // 输出acc
      //Serial.print("dt:");
      //Serial.print(time_dt,6);
      Serial.print(" gyro:");
      Serial.print(attitude.gyro.y);
      Serial.print(" gyrof:");
      Serial.println(attitude.gyrof.y);
      break;


    case 17:
      // 输出acc
      //Serial.print("dt:");
      //Serial.print(time_dt,6);
      Serial.print(" current_sp:");
      Serial.println(motor2.current_sp, 6);
      break;

    case 18:
      // 输出acc
      Serial.print(" t:");
      Serial.print(motor2.target, 6);
      Serial.print(" a1:");
      Serial.print(sensor1.getAngle(), 6);
      Serial.print(" a11:");
      Serial.print(sensor1.getMechanicalAngle(), 6);

      Serial.print(" a2:");
      Serial.print(sensor2.getAngle(), 6);
      Serial.print(" a22:");
      Serial.println(sensor2.getMechanicalAngle(), 6);
      break;

    case 19:
      //
      Serial.print(" servo1:");
      Serial.print(zeroBias.servo1);
      Serial.print(" servo2:");
      Serial.print(zeroBias.servo2);
      Serial.print(" servo3:");
      Serial.print(zeroBias.servo3);
      Serial.print(" servo4:");
      Serial.println(zeroBias.servo4);
      break;

    case 20:
      Serial.print(" BodyRoll:");
      Serial.print(BodyRoll, 6);
      Serial.print(" LegLength:");
      Serial.println(LegLength, 6);
      break;

    case 21:
      Serial.print(" roll_ok:");
      Serial.print(roll_ok, 6);
      Serial.print(" BodyPitching:");
      Serial.println(BodyPitching, 6);
      break;


    case 22:
      /*
            Serial.print(" PP:");
            Serial.print(Angle_Pid.Kp, 5);
            Serial.print(" PI:");
            Serial.print(Angle_Pid.Ki, 5);
            Serial.print(" PD:");
            Serial.print(Angle_Pid.Kd, 5);   
            */
      Serial.print(" it:");
      Serial.print(Angle_Pid.iLimit, 5);
      Serial.print(" il:");
      Serial.print(Angle_Pid.integral, 5);
      Serial.print(" oI:");
      Serial.print(Angle_Pid.outI, 5);
      Serial.print(" out:");
      Serial.println(Angle_Pid.output, 5);
      break;


    case 23:
      /*
            Serial.print(" SP:");
            Serial.print(Speed_Pid.Kp, 5);
            Serial.print(" SI:");
            Serial.print(Speed_Pid.Ki, 5);
            */
      Serial.print(" it:");
      Serial.print(Speed_Pid.iLimit, 5);
      Serial.print(" il:");
      Serial.print(Speed_Pid.integral, 5);
      Serial.print(" oI:");
      Serial.print(Speed_Pid.outI, 5);
      Serial.print(" A:");
      Serial.print(BodyPitching_f, 5);
      Serial.print(" out:");
      Serial.println(Speed_Pid.output, 5);
      break;

    case 24:
      Serial.print(" EN:");
      Serial.print(enableDFilter);
      Serial.print(" HZ:");
      Serial.println(cutoffFreq, 5);
      break;

    case 25:
      Serial.print(" LpfOut:");
      Serial.print(BodyPitching_f, 6);
      Serial.print(" BodyPitching:");
      Serial.println(BodyPitching, 6);
      break;

    case 26:

      if (Touch.state == 1) {
        Serial.print("  aX:");
        Serial.print(Touch.XPdat);
        Serial.print("  aY:");
        Serial.println(Touch.YPdat);
      } else if (Touch.state == 0) {
        Serial.print("  tX:");
        Serial.print(Touch.XLdat);
        Serial.print("  tY:");
        Serial.println(Touch.YLdat);
      }
      break;


    case 27:

      Serial.print("  aX:");
      Serial.print(Touch.XPdat);
      Serial.print("  aY:");
      Serial.print(Touch.YPdat);
      Serial.print("  aXF:");
      Serial.print(Touch.XPdatF);
      Serial.print("  aYF:");
      Serial.println(Touch.YPdatF);
      break;

    case 28:

      Serial.print("  P:");
      Serial.print(BodyPitching_f);
      Serial.print("  R:");
      Serial.print(BodyRoll_f, 5);
      Serial.print("  H:");
      Serial.print(LegLength_f, 5);
      Serial.print("  S:");
      Serial.print(SlideStep_f);
      break;

    case 31:
      Serial.print(" E:");
      Serial.print(Roll_Pid.error, 6);
      Serial.print(" it:");
      Serial.print(Roll_Pid.iLimit, 5);
      Serial.print(" il:");
      Serial.print(Roll_Pid.integral, 5);
      Serial.print(" oI:");
      Serial.print(Roll_Pid.outI, 5);
      Serial.print(" out:");
      Serial.println(Roll_Pid.output, 5);
      break;

    case 32:

      Serial.print(" RP:");
      Serial.print(Roll_Pid.Kp, 6);
      Serial.print(" RI:");
      Serial.print(Roll_Pid.Ki, 6);
      Serial.print(" RD:");
      Serial.println(Roll_Pid.Kd, 6);
      break;

    case 33:
      Serial.print(" it:");
      Serial.print(Yaw_Pid.iLimit, 5);
      Serial.print(" il:");
      Serial.print(Yaw_Pid.integral, 5);
      Serial.print(" oI:");
      Serial.print(Yaw_Pid.outI, 5);
      Serial.print(" A:");
      Serial.print(BodyPitching_f, 5);
      Serial.print(" out:");
      Serial.println(Yaw_Pid.output, 5);
      break;

    case 34:
      Serial.print(" Kp:");
      Serial.print(TouchX_Pid.Kp, 6);
      Serial.print(" Ki:");
      Serial.print(TouchX_Pid.Ki, 6);
      Serial.print(" Kd:");
      Serial.print(TouchX_Pid.Kd, 6);

      Serial.print(" deriv:");
      Serial.print(TouchX_Pid.deriv);
      Serial.print(" out:");
      Serial.println(TouchX_Pid.output);
      break;

    case 35:
      Serial.print(" deriv:");
      Serial.println(TouchX_Pid.deriv);
      break;

    case 36:
      Serial.print(" state:");
      Serial.print(Touch.state);
      Serial.print(" start:");
      Serial.println(Touch.start);
      break;

    case 38:
      Serial.print(" X OUT:");
      Serial.print(BodyPitching);
      Serial.print(" Y OUT:");
      Serial.println(TouchY_Pid.output);
      break;


    case 39:
      Serial.print(" it:");
      Serial.print(TouchY_Pid.iLimit, 5);
      Serial.print(" il:");
      Serial.print(TouchY_Pid.integral, 5);
      Serial.print(" oI:");
      Serial.print(TouchY_Pid.outI, 5);
      Serial.print(" out:");
      Serial.println(TouchY_Pid.output, 5);
      break;


    case 40:

      if (Touch.state == 1) {
        Serial.print("  aX:");
        Serial.print(Touch.XPressDat);
        Serial.print("  aY:");
        Serial.println(Touch.YPressDat);
      } else if (Touch.state == 0) {
        Serial.print("  tX:");
        Serial.print(Touch.XPressDat);
        Serial.print("  tX:");
        Serial.println(Touch.YPressDat);
      }
      break;

    case 41:

      Serial.print(" roll_ok:");
      Serial.print(roll_ok, 5);
      Serial.print(" pa:");
      Serial.print(BodyPitching, 5);
      Serial.print(" out:");
      Serial.println(Speed_Pid.output, 5);
      break;

    case 42:

      Serial.print(" P:");
      Serial.print(roll_ok, 5);
      Serial.print(" P1:");
      Serial.print(BodyPitching, 5);
      Serial.print(" P3:");
      Serial.println(BodyPitchingCorrect(BodyPitching_f), 5);
      break;

    case 43:

      Serial.print(" Vdat:");
      Serial.print(VoltageADC);
      Serial.print(" Vdatf:");
      Serial.print(VoltageADCf);
      Serial.print(" V:");
      Serial.println(Voltage, 5);
      break;

    case 44:

      Serial.print(" PidParameterTuning:");
      Serial.print(PidParameterTuning);
      Serial.print(" TargetLegLength:");
      Serial.println(TargetLegLength, 6);

      break;

    case 45:

      Serial.print(" RobotTumble:");
      Serial.print(RobotTumble);
      Serial.print(" roll_ok:");
      Serial.print(roll_ok, 6);
      Serial.print(" Angle_Pid.error:");
      Serial.println(Angle_Pid.error, 6);

      break;

    case 46:



      break;

    case 47:
      Serial.print(" sbus_swb:");
      Serial.println(sbus_swb);

      break;

    case 48:
      Serial.print(" xo3:");
      Serial.print(body.xo3 * 100, 4);

      Serial.print(" zo3:");
      Serial.print(body.zo3 * 100, 4);
      Serial.print(" Ts:");
      Serial.println(body.Ts, 4);

      break;

    case 49:

      Serial.print(" xo4:");
      Serial.print(body.xo4 * 100, 4);

      Serial.print(" zo4:");
      Serial.print(body.zo4 * 100, 4);

      Serial.print(" Ts:");
      Serial.println(body.Ts, 4);

      break;

    case 50:

      Serial.print(" body.Ts:");
      Serial.print(body.Ts, 4);

      Serial.print(" bodyH:");
      Serial.println(sbus_vrb, 4);

      break;

    case 51:
      Serial.print(" mv1:");
      Serial.print(body.MotorVelocityF[0], 3);
      Serial.print(" mv2:");
      Serial.print(body.MotorVelocityF[1], 3);
      Serial.print(" mv3:");
      Serial.print(body.MotorVelocityF[2], 3);
      Serial.print(" mv4:");
      Serial.println(body.MotorVelocityF[3], 3);
      break;

    case 52:
      Serial.print(" xt:");
      Serial.print(body.xt, 4);
      Serial.print(" h:");
      Serial.print(body.h, 4);
      Serial.print(" Ts:");
      Serial.println(body.Ts, 4);
      break;

    case 54:
      Serial.print(" E:");
      Serial.print(Pitching_Pid.error, 6);
      Serial.print(" it:");
      Serial.print(Pitching_Pid.iLimit, 5);
      Serial.print(" il:");
      Serial.print(Pitching_Pid.integral, 5);
      Serial.print(" oI:");
      Serial.print(Pitching_Pid.outI, 5);
      Serial.print(" out:");
      Serial.println(Pitching_Pid.output, 5);
      break;

    case 55:
      //Serial.print("   MovementSpeed:");
      //Serial.print(MovementSpeed, 3);
      Serial.print("   roll_ok:");
      Serial.println(roll_ok, 3);
      break;

    case 56:
      Serial.print(" pp:");
      Serial.print(Angle_Pid.Kp, 6);
      Serial.print(" pi:");
      Serial.print(Angle_Pid.Ki, 6);
      Serial.print(" pd:");
      Serial.print(Angle_Pid.Kd, 6);
      Serial.print(" output:");
      Serial.println(Angle_Pid.output, 6);
      break;

    default:

      break;
  }
}


float BodyPitchingCorrect(float x)  //俯仰角校正
{
  float y = 0.000004 * x * x + 0.0004 * x - 0.0008;  //y = 4E-06x2 + 0.0004x - 0.0008    y = -2E-07x2 + 0.0002x - 0.0029
  return y;
}

/*PIDPID*/
void PidParameter(void) {
  if (sbus_swa == 1)  //不带触摸屏
  {
    //横滚角
    RollPid.P = 0.06;
    RollPid.I = 1.5;
    RollPid.D = 0.0028;
    RollPid.limit = 2;  //积分限幅

    //速度环
    SpeedPid.P = 0.1;
    SpeedPid.I = 0.04;
    SpeedPid.D = 0;
    SpeedPid.limit = 50;  //积分限幅

    //平衡环
    AnglePid.P = 11;
    AnglePid.I = 200;
    AnglePid.D = 0.2;
    AnglePid.limit = 0.1;  //积分限幅

  } else if (sbus_swa == 2)  //带触摸屏
  {
    //横滚角
    RollPid.P = 0.08;
    RollPid.I = 1.5;
    RollPid.D = 0.005;
    RollPid.limit = 2;  //积分限幅

    //速度环
    SpeedPid.P = 0.1;
    SpeedPid.I = 0.04;
    SpeedPid.D = 0;
    SpeedPid.limit = 50;  //积分限幅

    //平衡环
    AnglePid.P = 11;
    AnglePid.I = 200;
    AnglePid.D = 0.2;
    AnglePid.limit = 0.1;  //积分限幅
  }

  YawPid.P = 11;
  YawPid.I = 100;
  YawPid.D = 0;
  YawPid.limit = 0;
  
}


void PIDcontroller_posture(float dt) {
  if ((int)PidParameterTuning == 0)
    PidParameter();

  //横滚角
  Roll_Pid.Kp = RollPid.P / 100;
  Roll_Pid.Ki = RollPid.I / 100;
  Roll_Pid.Kd = RollPid.D / 100;
  Roll_Pid.iLimit = RollPid.limit;  //积分限幅

  float TargetBodyRoll = BodyRoll_f * 777;  //横滚
  if (sbus_swd == 2)                        //顶球禁止手动横滚
    TargetBodyRoll = 0;
  float RollError = (-pitch_ok) - (-TargetBodyRoll) - (-TouchY_Pid_outputF);
  if (sbus_swc == 1)  //横滚调平
  {
    Roll_Pid.compute(RollError, dt);
  } else {
    Roll_Pid.output = 0;
    Roll_Pid.integral = 0;
  }

  //速度环
  Speed_Pid.Kp = SpeedPid.P / 100;
  Speed_Pid.Ki = SpeedPid.I / 100;
  Speed_Pid.Kd = SpeedPid.D / 100;
  Speed_Pid.iLimit = SpeedPid.limit;  //积分限幅

  float speedError = (Motor1_Velocity_f + Motor2_Velocity_f) * 0.5 - MovementSpeed;  //测量值减去目标值
  BodyX = Speed_Pid.compute(speedError, dt) + BodyPitchingCorrect(BodyPitching_f);   //

  //平衡环
  Angle_Pid.Kp = AnglePid.P;
  Angle_Pid.Ki = AnglePid.I;
  Angle_Pid.Kd = AnglePid.D;
  Angle_Pid.iLimit = AnglePid.limit;  //积分限幅


  float angleError = roll_ok - (-BodyPitching_f);  //测量值减去目标值 此处roll_ok为俯仰角测量值
  float angleOutput = Angle_Pid.compute(angleError, dt);

  //转向环
  Yaw_Pid.Kp = YawPid.P;
  Yaw_Pid.Ki = YawPid.I;
  Yaw_Pid.Kd = YawPid.D;
  Yaw_Pid.iLimit = YawPid.limit;
  if (sbus_swc != 1)  //锁定航向角
  {
    //Yaw_Pid.Ki = 0;
    //Yaw_Pid.integral = 0;
  }

  float yawError = attitude.gyro.z - BodyTurn;  //测量值减去目标值
  float yawOutput = Yaw_Pid.compute(yawError, dt);

  float target1 = angleOutput - yawOutput;
  float target2 = angleOutput + yawOutput;

  if (control_torque_compensation != 0) {
    if (target1 > 0)
      target1 = target1 + control_torque_compensation;
    else if (target1 < 0)
      target1 = target1 + (-control_torque_compensation);

    if (target2 > 0)
      target2 = target2 + control_torque_compensation;
    else if (target2 < 0)
      target2 = target2 + (-control_torque_compensation);
  }

  motor1.target = target1;
  motor2.target = target2;
}


void RemoteControlFiltering(void)  //遥控器滤波
{
  static int enableDFilter_last = (int)enableDFilter;
  static int cutoffFreq_last = (int)cutoffFreq;


  if ((int)enableDFilter == 1) {
    if (body.MotorMode >= 3)
    else
      BodyPitching_f = biquadFilterApply(&FilterLPF[0], BodyPitching);  //

    
    BodyRoll_f = biquadFilterApply(&FilterLPF[1], BodyRoll);
    LegLength_f = biquadFilterApply(&FilterLPF[2], LegLength);
    SlideStep_f = biquadFilterApply(&FilterLPF[3], SlideStep);

  } else {
    BodyPitching_f = BodyPitching;
    BodyRoll_f = BodyRoll;
    LegLength_f = LegLength;
    SlideStep_f = SlideStep;
  }


  if (((int)enableDFilter != enableDFilter_last) || ((int)cutoffFreq != cutoffFreq_last)) {
    for (int i = 0; i < 6; i++) {
      biquadFilterInitLPF(&FilterLPF[i], 100, (unsigned int)cutoffFreq);  //遥控器滤波器
      //TouchscreenInit((unsigned int)cutoffFreq);
    }

    enableDFilter_last = (int)enableDFilter;
    cutoffFreq_last = (int)cutoffFreq;
    Serial.println(" ");
    Serial.println(" ok ");
    /****************************************************************/
    // 更新串口滤波器
    for (int i = 0; i < 4; i++) {
      biquadFilterInitLPF(&SerialFilterLPF[i], (unsigned int)cutoffFreq, 100);
    }
        
      Serial.println("Serial filters updated");
    /*******************************************************************/
  }
}


void ReadVoltage(void) {
  VoltageADC = analogRead(analogInPin);
  VoltageADCf = biquadFilterApply(&VoltageFilterLPF, VoltageADC);
  Voltage = (float)7.77 / 813.43 * VoltageADCf;
}


void Robot_Tumble(void) {
  static int x = 0;
  if (abs(roll_ok) >= 35) {
    x++;
    if (x >= 20) {
      x = 20;
      RobotTumble = 1;  //机器摔倒
    }
  } else {
    if ((RobotTumble == 1) && (abs(roll_ok) <= 5))  //机器摔倒后扶起
    {
      x--;
      if (x <= 0) {
        x = 0;
        RobotTumble = 0;
      }
    }
  }

  if (RobotTumble == 1)  //机器摔倒
  {
    bodyH = 0.06;
    bodyRoll = 0;
    BodyX = 0;
    bodyRoll = 0;
    BodyPitching_f = 0;
  }
}


void body_data_init(void)  
{
  // 步态参数
  body.delayTime = 0.005;  // 每一步的延迟时间
  body.CurrentSteps = 0;   //当前步数
  body.xt = 0.015;         // 起始位置
  body.xs1 = 0;            // 起始位置
  body.xf1 = body.xt;      // 终点位置
  body.xs2 = 0;            // 起始位置
  body.xf2 = body.xt;      // 终点位置
  body.xs3 = 0;            // 起始位置
  body.xf3 = body.xt;      // 终点位置
  body.xs4 = 0;            // 起始位置
  body.xf4 = body.xt;      // 终点位置
  body.h = 0.02;           // 最高位置
  body.zs = 0;             // 起始高度
  body.Ts = 0.5;           // 周期

  body.lambda[0] = 0.5;  // λ 参数
  body.lambda[1] = 1.0f;

  body.H_fron = 0.075;  //
  body.H_back = 0.075;
  ;  //

  body.MotorMode = 0;  //
}


void motor_task(void) {
  if (Communication_object == 1) {
    motor1.monitor();  //使用simpleFOC Studio上位机设置的时候，这句一定要打开。但是会影响程序执行速度
  } else if (Communication_object == 2) {
    motor2.target = motor1.target;
  }

  motor1.move();
  motor2.move();

  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();
}


void voltage_Indicator_Light(void)  //电压指示灯
{
  LED_count++;
  if (LED_count >= LED_dt) {
    LED_count = 0;
    if (LED_HL == 1) {
      digitalWrite(LED_Pin, LOW);  //亮
      LED_HL = 0;
    } else {
      digitalWrite(LED_Pin, HIGH);  //不亮
      LED_HL = 1;
    }
  }

  if (Voltage <= 7.4)
    LED_dt = 20;     //20ms闪一次
  else
    LED_dt = 100;
}

void LPF_Parameter_Init(void)  //滤波器调参初始化
{
  if (RATE_HZ != RATE_HZ_last) {
    //初始化二阶低通滤波
    for (int axis = 0; axis < 6; axis++) {
      biquadFilterInitLPF(&ImuFilterLPF[axis], (unsigned int)LPF_CUTOFF_FREQ, (unsigned int)RATE_HZ);
    }
    Serial.print(" RATE_HZ:");
    Serial.print(RATE_HZ);
    RATE_HZ_last = RATE_HZ;
  }
  if (LPF_CUTOFF_FREQ != LPF_CUTOFF_FREQ_last) {
    //初始化二阶低通滤波
    for (int axis = 0; axis < 6; axis++) {
      biquadFilterInitLPF(&ImuFilterLPF[axis], (unsigned int)LPF_CUTOFF_FREQ, (unsigned int)RATE_HZ);
    }
    Serial.print(" LPF_CUTOFF_FREQ:");
    Serial.print(LPF_CUTOFF_FREQ);
    LPF_CUTOFF_FREQ_last = LPF_CUTOFF_FREQ;
  }
}

void Update_Motor_Speed(void)  //更新电机速度
{
  Motor1_Velocity = (sensor1.getAngle() - Motor1_place_last) / 0.01f;
  Motor1_Velocity_f = Motor1_Velocity_filter(Motor1_Velocity);
  Motor1_place_last = sensor1.getAngle();

  Motor2_Velocity = -(sensor2.getAngle() - Motor2_place_last) / 0.01f;
  Motor2_Velocity_f = Motor2_Velocity_filter(Motor2_Velocity);
  Motor2_place_last = sensor2.getAngle();

  body.MotorVelocityF[0] = Motor1_Velocity_f;
  body.MotorVelocityF[1] = Motor2_Velocity_f;
}

void Robot_Stop(void) {
  if (Communication_object == 0 && SwitchUser != 1 && SwitchUser != 2)  //电机停止
  {
    motor1.target = 0;
    motor2.target = 0;
    /*******添加姿态归0***********/
    BodyRoll = 0;
    MovementSpeed = 0;
    BodyTurn = 0;
    LegLength = 0.06;
  }

  bodyH = 0.06;
  bodyRoll = 0;
  BodyX = 0;
  bodyRoll = 0;
  BodyPitching_f = 0;

//清空pid中的积分项，防止控制器积累误差，确保重新启动时控制平滑
  Angle_Pid.integral = 0;
  Speed_Pid.integral = 0;
  Yaw_Pid.integral = 0;

  body_data_init();
}

void Wheel_foot_controller(void)  //轮足控制器
{
  PIDcontroller_posture(time_dt);  //PID控制器

  if (sbus_swc == 1)  //手动横滚
    bodyRoll = Roll_Pid.output;

  if (TargetLegLength == 0)
    bodyH = LegLength_f;  //遥控设置腿长
  else
    bodyH = TargetLegLength;  //串口设腿长目标值
}

void inverse_kinematics_task(void)  //运动学逆解任务
{
  //打印腿长
  // Serial.print("Target Leg: ");
  // Serial.print(bodyH, 4);

  if (RightInverseKinematics(BarycenterX - BodyX, bodyH - bodyRoll, BodyPitching_f, Rax))
    Serial.println("RightInverseKinematics no");

  if (LeftInverseKinematics(BarycenterX - BodyX, bodyH + bodyRoll, BodyPitching_f, Lax))
    Serial.println("LeftInverseKinematics no");
}

void servo_task(void)  //舵机任务
{
  if (sbus_swb == 0)  //姿态
  {
    // 将四个舵机的角度设置为....
    servoControl.setServosAngle(1, Lax[0] - zeroBias.servo1, -1, Lax[1] - zeroBias.servo2, -1, Rax[0] - zeroBias.servo3, 1, Rax[1] - zeroBias.servo4, 1);
  } else  //装配位置与校准
  {
    if ((int)CalibrationSelect == 3)  //舵机校准
    {
      servoControl.setServosAngle(1, 0 - zeroBias.servo1, -1, 0 - zeroBias.servo2, -1, 0 - zeroBias.servo3, 1, 0 - zeroBias.servo4, 1);  //标定偏差
    } else {
      servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0, 1);  //
    }
  }
}

//解释和应用串口数据
// 2. 修改handleSerialCommands函数
void handleSerialCommands() {
  if (remoteControlActive) return;

  static String inputString;
  while (Serial.available() > 0) { //以单个字符为单位读取串口数据
    char inChar = Serial.read();

    // 调试输出（可保留）
    Serial.print("Received: ");
    Serial.println(inChar);
/*********/
    SwitchingPattern == 0;
/*********/
    if (inChar == '\n') { // 如果接收到换行符，表示命令结束
      Serial.print("Full command: ");
      Serial.println(inputString);

      int values[4];
      int count = sscanf(inputString.c_str(), "%d,%d,%d,%d", &values[0], &values[1], &values[2], &values[3]);

      if (count == 4) {
        serialCH1 = constrain(values[0], 1000, 2000);
        serialCH2 = constrain(values[1], 1000, 2000);
        serialCH3 = constrain(values[2], 1000, 2000);
        serialCH4 = constrain(values[3], 1000, 2000);

        // Serial.print("Parsed: ");
        // Serial.print(serialCH1);
        // Serial.print(", ");
        // Serial.print(serialCH2);
        // Serial.print(", ");
        // Serial.print(serialCH3);
        // Serial.print(", ");
        // Serial.println(serialCH4);

        newSerialData = true;
        serialControlActive = true;  // 激活串口控制
        remoteControlActive = false;  // 禁用遥控器控制

        lastSerialTime = millis();   // 更新最后活动时间
        lastSerialCommandTime = millis();  // 更新最后接收时间
      }
      inputString = "";
    } else if (isDigit(inChar) || inChar == ',') {
      inputString += inChar;
    }
  }
}

// 3. 添加超时检测函数
void checkSerialTimeout() {
  if (serialControlActive && (millis() - lastSerialCommandTime > SERIAL_TIMEOUT_MS)) {
    emergencyStop();
    serialControlActive = false;
    Serial.println("Serial timeout! Emergency stop activated.");
  }
}

void emergencyStop() {
  // 重置控制变量
  MovementSpeed = 0;
  BodyTurn = 0;
  LegLength = 0.06f;
  BodyRoll = 0;
  
  // // 清空PID积分项
  // Angle_Pid.integral = 0;
  // Speed_Pid.integral = 0;
  // Yaw_Pid.integral = 0;
  
  // 重置姿态
  bodyH = 0.06f;
  bodyRoll = 0;
  BodyX = 0;
  BodyPitching_f = 0;
}

void resetControlVariables() {
  LegLength = 0.06f;  // 目标腿长
  BodyRoll = 0;       // 横滚
  MovementSpeed = 0;  // 移动速度
  BodyTurn = 0;       // 转向
}

//腿长分段映射
float mapSegmented(float value, float inLow, float inMid, float inHigh, float outLow, float outMid, float outHigh) {
    if (value <= inMid) {
        // 低区间映射: inLow-inMid -> outLow-outMid
        return mapf(value, inLow, inMid, outLow, outMid);
    } else {
        // 高区间映射: inMid-inHigh -> outMid-outHigh
        return mapf(value, inMid, inHigh, outMid, outHigh);
    }
}

void applySerialCommands() {
    
    //这段是重新换了腿长映射之后的控制，没有滤波
    LegLength = mapSegmented(
        serialCH1, 
        1000, 1500, 2000,  // 输入范围
        0.05, 0.06, 0.1    // 输出范围
    );
    BodyRoll = mapf(serialCH2, 1000, 2000, -0.011, 0.011);
    MovementSpeed = mapf(serialCH3, 1000, 2000, -15, 15);
    BodyTurn = -mapf(serialCH4, 1000, 2000, -11, 11);
    // 强制设置开关状态
    sbus_swa = 1;
    sbus_swb = 0;
    sbus_swc = 0;
    sbus_swd = 0;
}

//实时滤波更新函数
void updateSerialFilters(float dt) {
    if (!serialRawCache.updated) return;
    
    // 使用固定时间步长进行滤波更新
    const float fixedDt = 0.005f; // 5ms固定步长
    
    // 腿长滤波
    serialFiltered.legLength = biquadFilterApply(&SerialFilterLPF[0], 
                                              serialRawCache.legLength);
    
    // 横滚滤波
    serialFiltered.bodyRoll = biquadFilterApply(&SerialFilterLPF[1], 
                                             serialRawCache.bodyRoll);
    
    // 速度滤波
    serialFiltered.movementSpeed = biquadFilterApply(&SerialFilterLPF[2], 
                                                   serialRawCache.movementSpeed);
    
    // 转向滤波
    serialFiltered.bodyTurn = biquadFilterApply(&SerialFilterLPF[3], 
                                              serialRawCache.bodyTurn);
    
    // 应用滤波后的值
    LegLength = serialFiltered.legLength;
    BodyRoll = serialFiltered.bodyRoll;
    MovementSpeed = serialFiltered.movementSpeed;
    BodyTurn = serialFiltered.bodyTurn;
    
    // 调试输出
    if (millis() - lastSerialUpdate > 100) { // 每100ms输出一次
        lastSerialUpdate = millis();
        Serial.print("Serial Filtered: L=");
        Serial.print(LegLength, 4);
        Serial.print(", R=");
        Serial.print(BodyRoll, 4);
        Serial.print(", S=");
        Serial.print(MovementSpeed, 4);
        Serial.print(", T=");
        Serial.println(BodyTurn, 4);
    }
}

void printControlValues() {
  Serial.print("运动控制值 | ");
  
  // 基本运动参数
  Serial.print("速度: "); Serial.print(MovementSpeed, 2); Serial.print(" m/s | ");
  Serial.print("转向: "); Serial.print(BodyTurn, 2); Serial.print(" deg | ");
  Serial.print("腿长: "); Serial.print(LegLength, 3); Serial.print(" m | ");
  
  // 姿态参数
  Serial.print("俯仰: "); Serial.print(roll_ok, 2); Serial.print(" deg | ");
  Serial.print("横滚设定: "); Serial.print(BodyRoll, 4); Serial.print(" rad | ");
  Serial.print("实际横滚: "); Serial.print(pitch_ok, 2); Serial.print(" deg | ");
  
  // 电源状态
  Serial.print(" | 电压: "); Serial.print(Voltage, 2); Serial.print(" V");
  
  Serial.println(); // 换行
}

void setup() {

  if ((MasterSlaveSelection == 0) && (SwitchingPattern == 1))  //从机 && 4轮模式
    Serial2.begin(1000000, SERIAL_8N1, RXD2, TXD2);            //
  else if (MasterSlaveSelection == 1)                          //主机
    Serial1.begin(1000000, SERIAL_8N1, RXD1, TXD1);

  Serial.begin(250000);
  
  Serial.println("ESP32-S3 Ready");
  
  FlashInit();  //读取闪存数据
  pinMode(LED_Pin, OUTPUT);
  digitalWrite(LED_Pin, LOW);  //亮

  body_data_init();
  //初始化二阶低通滤波
  for (int axis = 0; axis < 6; axis++) {
    biquadFilterInitLPF(&ImuFilterLPF[axis], (unsigned int)LPF_CUTOFF_FREQ, (unsigned int)RATE_HZ);
  }

  biquadFilterInitLPF(&VoltageFilterLPF, 50.0f, 100);  //电压滤波函数初始化

  // 初始化舵机
  servoControl.initialize();
  // 将四个舵机的初始角度设置为 0 度
  servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0, 1);
  _delay(555);

  servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0, 1);  //装配位置
  //IMU
  if (MasterSlaveSelection == 1)  //主机
  {
    if (!initICM42688()) {
      Serial.println("ICM42688 initialization failed!");
      while (1)
        ;
    }
    Serial.println("ICM42688 initialized successfully!");
  }

  // 进行陀螺仪校准
  //calibrateGyro();

  //遥控器
  if (MasterSlaveSelection == 1)  //主机使用
    sBus.begin();

  for (int i = 0; i < 6; i++)
    biquadFilterInitLPF(&FilterLPF[i], 100, (unsigned int)cutoffFreq);  //遥控器滤波器

  biquadFilterInitLPF(&FilterLPF[8], 50, (unsigned int)cutoffFreq);   //遥控器滤波器
  biquadFilterInitLPF(&FilterLPF[9], 50, (unsigned int)cutoffFreq);   //遥控器滤波器
  biquadFilterInitLPF(&FilterLPF[10], 200, (unsigned int)400);        //
  biquadFilterInitLPF(&FilterLPF[11], 200, (unsigned int)400);        //
  biquadFilterInitLPF(&FilterLPF[12], 50, (unsigned int)cutoffFreq);  //遥控器滤波器

  motor_init();  //FOC电机初始化

  command_init();  //调试命令初始化

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println("Motor ready.");
  timestamp_prev = micros();
}

void loop() {
  now_us = micros();
  motor_task();//FOCd电机任务
  command.run();//通讯调参

  if(MasterSlaveSelection==1)//主机使用
  {
    ImuUpdate();//更新IMU数据   
    RXsbus(); 
  }

  // 新增串口处理代码
  handleSerialCommands();  // 解析串口数据
  if (newSerialData) {     // 如果有新数据
    applySerialCommands(); // 应用串口控制值
    newSerialData = false; // 清除标志
  }
    
  time_dt = (now_us - now_us1) / 1000000.0f;
  if (time_dt >= 0.005f)  //执行间隔5ms
  {

      
    RemoteControlFiltering();//遥控信号滤波
    ReadVoltage();//电池
    print_data();//串口打印数据

    if(SwitchingPattern==0)//2轮模式

      Robot_Tumble();//机器摔倒检测

    voltage_Indicator_Light();//电压指示灯  

    LPF_Parameter_Init();//滤波器调参初始化

    FlashSave((int)CalibrationSelect);//校准保存数据

    Update_Motor_Speed();//更新电机速度

    bodyRoll = BodyRoll_f;//横滚角

    if(MasterSlaveSelection==1)//主机模式
    {
      // if((sbus_swa == 0)||(RobotTumble == 1))//遥控关闭或者机器摔倒
      if(RobotTumble == 1)//机器摔倒
      {
        Robot_Stop();//机器人停止
      }
      // else if((sbus_swa >= 1)&&(RobotTumble == 0))//遥控启动且机器没摔倒
      else if(RobotTumble == 0)//遥控启动且机器没摔倒
      {
        if(SwitchingPattern==0)//2轮模式
        {
          Wheel_foot_controller();//轮足控制器
        }
      }      
    }
    
    else //从机模式
    {

    }
    inverse_kinematics_task();//运动学逆解任务

    servo_task();//舵机任务

    // // //检查串口超时
    // checkSerialTimeout();
    if (serialControlActive && (millis() - lastSerialTime > SERIAL_TIMEOUT)) {
      resetControlVariables();       // 重置控制变量
      serialControlActive = false;   // 禁用串口控制
      // remoteControlActive = true;    // 启用遥控器控制
      // Serial.println("Timeout: Switch back to remote control");
  }
    now_us1 = now_us;
  }
}