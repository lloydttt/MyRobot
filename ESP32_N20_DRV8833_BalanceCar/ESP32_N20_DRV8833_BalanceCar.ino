//马达供电电压 9v

#include "Led.h"
#include "Motor.h"
#include <Wire.h>
#include "BluetoothSerial.h"
#include <MPU6050_tockn.h>

//Led led(22);                            // 板载LED    改    get
MPU6050 mpu6050(Wire);                  // 实例化mpu6050对象
BluetoothSerial SerialBT;               // 实例化esp32的蓝牙串口对象
Motor motorLF(13, 12), motorRT(26, 14); // 实例化左右马达      改    get

/*预设定值*/
float Balance_Angle_raw = -1.15;                            // 测试出的静态机械平衡角度
float ENERGY = 0.4, turn_ENERGY = 80;                     // 设定的前进后退使能幅度,转向幅度
float kp = 19.5, ki = 0.48, kd = 0.80;                      // 根据调试测试得到设置kp ki kd的值
float turn_kp = 0.6, turn_spd = 0.2; // 根据调试测试得到设置kp ki kd的值

int sdapin = 21, sclpin = 22;                          // 自定义ESP32的I2C引脚    改    get
int leftMotorPwmOffset = 18, rightMotorPwmOffset = 18; // 测试出的左右轮的启动pwm值，pwm达到一定电压马达才开始转动

/*调试 控制变量*/
float Keep_Angle, bias, integrate; // 保持角度，角度偏差，偏差积分变量
float turn_integrate;
float AngleX, GyroX, GyroZ;                    // mpu6050输出的角度值为浮点数，两位有效小数
int vertical_PWM, turn_PWM, PWM, L_PWM, R_PWM; // 各PWM值

/*状态设定，为真启动，为假关机*/
bool STATE = true;      

/*蓝牙串口调试控制*/
void serial_debug()
{
  if (SerialBT.available())
  {
    char DATA = SerialBT.read();
    switch (DATA)
    {
      /*机械平衡点调整*/
      case 'u': // 前倾
        Keep_Angle += 0.1;
        break;
      case 'd': // 后仰
        Keep_Angle -= 0.1;
        break;
      /*直立PID调试*/
      case '0':
        kp -= 0.5;
        break;
      case '1':
        kp += 0.5;
        break;
      case '2':
        ki -= 0.02;
        break;
      case '3':
        ki += 0.02;
        break;
      case '4':
        kd -= 0.02;
        break;
      case '5':
        kd += 0.02;
        break;
      /*转向比例项调试*/
      case '6':
        turn_kp -= 0.2;
        break;
      case '7':
        turn_kp += 0.2;
      case 'G'://机器启动
        STATE = true;
        break;
      case 'g'://机器关停
        STATE = false;
        break;
      /*运动控制*/
      case 's': // 停车
        Keep_Angle = Balance_Angle_raw;
        break;
      case 'a': // 前进
        Keep_Angle = Balance_Angle_raw - ENERGY;
        break;
      case 'b': // 后退
        Keep_Angle = Balance_Angle_raw + ENERGY;
        break;
      case 'z': // 不转
        turn_spd = 0;
        break;
      case 'l': // 左转
        turn_spd = turn_ENERGY;
        break;
      case 'r': // 右转
        turn_spd = -turn_ENERGY;
        break;
    }
    /*串口打印输出显示*/
    SerialBT.printf("Keep_Angle:%.2f\n", Keep_Angle);
    SerialBT.printf("kp:%.2f  ", kp);
    SerialBT.printf("ki:%.2f  ", ki);
    SerialBT.printf("kd:%.2f  ", kd);
    SerialBT.printf("turn_kp:%.2f\n", turn_kp);
    SerialBT.print("--------------------\n");
  }
}

void vertical_pwm_calculation() // 直立PMW计算
{
  AngleX = mpu6050.getAngleX();                           // 获取MPU6050角度
  GyroX = mpu6050.getGyroX();                             // 获取MPU6050角速度
  bias = AngleX - Keep_Angle;                             // 角度偏差bias
  integrate += bias;                                      // 偏差累积
  integrate = constrain(integrate, -500, 500);          // 积分限定
  vertical_PWM = kp * bias + ki * integrate + kd * GyroX; // 角速度直接作为偏差微分项
}

void turn_pwm_calculation() // 转向PMW计算
{
  GyroZ = mpu6050.getGyroZ(); // 获取Z轴角速度
  float turn_erro = turn_spd - GyroZ;
  turn_PWM = turn_kp * turn_erro; // 求转向pwm
}

void motor_control() // 马达控制
{
  if (STATE)
  {
    /*直立控制*/
    PWM = vertical_PWM;
    /*补偿马达死区*/
    if (PWM > 0)
    {
      L_PWM = PWM + leftMotorPwmOffset;
      R_PWM = -(PWM + rightMotorPwmOffset);
    }
    if (PWM < 0)
    {
      L_PWM = PWM - leftMotorPwmOffset;
      R_PWM = -(PWM - rightMotorPwmOffset);
    }
    /*转向控制*/
    L_PWM -= turn_PWM;
    R_PWM += turn_PWM;
    /*限定PWM区间在-255~255*/
    L_PWM = constrain(L_PWM, -225, 225);
    R_PWM = constrain(R_PWM, -225, 225);
  } else {
    L_PWM = 0;
    R_PWM = 0;
  }
  /*驱动马达*/
  motorLF.run(L_PWM);
  motorRT.run(R_PWM);
}

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("ESP32_CAR");           // 蓝牙从机
  Wire.begin(sdapin, sclpin);            // 自定义IIC引脚
  mpu6050.begin();                       // 启动mpu6050
  mpu6050.calcGyroOffsets(true); // 显示补偿值计算过程
  Keep_Angle = Balance_Angle_raw;        // 保持静态平衡角度
//  led.on();                              // 初始化完成点亮板载LED   改
}

void loop()
{
  serial_debug();             // 串口PID调试控制
  mpu6050.update();           // 陀螺仪刷新
  vertical_pwm_calculation(); // 直立环PWM计算
  turn_pwm_calculation();     // 转向PWM计算
  motor_control();            // 马达输出
}
