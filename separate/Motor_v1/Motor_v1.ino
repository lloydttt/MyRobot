#include <Arduino.h>
#include<Ticker.h>

#define ENCODER_L   5  //编码器采集引脚 每路2个 共4个
#define DIRECTION_L 18
#define ENCODER_R   4
#define DIRECTION_R 19
#define interrupt_time 10 // 中断时间

Ticker timer_read_encoder; 

int32_t Velocity_L, Velocity_R;   //左右轮编码器数据
int16_t Velocity_Left, Velocity_Right;     //左右轮速度

void READ_ENCODER_L(void);
void READ_ENCODER_R(void);
void myread(void);

void READ_ENCODER_L(void) {
  if (digitalRead(ENCODER_L) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //根据另外一相电平判定方向
    else      Velocity_L++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L++; //根据另外一相电平判定方向
    else     Velocity_L--; 
  }
}

void READ_ENCODER_R(void) {
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--;//根据另外一相电平判定方向
    else      Velocity_R++;
  }
  else {   //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++; //根据另外一相电平判定方向
    else     Velocity_R--;
  }
}

void myread(void)
{
/**************************************************************************
计算为转速n/s  500线光电编码器数据 定时时间为20MS
 转速 = num/500/20ms
 转速 = num*2/time
**************************************************************************/
    Velocity_Left = Velocity_L*2/interrupt_time;    Velocity_L = 0;  //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
    Velocity_Right = Velocity_R*2/interrupt_time;    Velocity_R = 0; //读取右轮编码器数据，并清零
   
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(ENCODER_L, INPUT);       //编码器引脚 输入模式
  pinMode(ENCODER_R, INPUT);       //编码器引脚 输入模式
  pinMode(DIRECTION_L, INPUT);     //编码器引脚 输入模式
  pinMode(DIRECTION_R, INPUT);     //编码器引脚 输入模式
  
  //编码器接口1 开启外部跳边沿中断 
  attachInterrupt(ENCODER_L, READ_ENCODER_L, CHANGE);  //中断函数READ_ENCODER_L
  //编码器接口2 开启外部跳边沿中断 
  attachInterrupt(ENCODER_R, READ_ENCODER_R, CHANGE);  //中断函数READ_ENCODER_R
    
  interrupts();                      //打开外部中断  
    timer_read_encoder.attach_ms(interrupt_time, myread); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("Velocity_L:");Serial.print(Velocity_L);
  Serial.print(" | Velocity_Left:");Serial.println(Velocity_Left);
  Serial.print("Velocity_R:");Serial.println(Velocity_R);
  Serial.print(" | Velocity_Right:");Serial.println(Velocity_Right);

}
