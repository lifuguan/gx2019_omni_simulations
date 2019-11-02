/*
 * @Author: your name
 * @Date: 2019-10-26 19:55:28
 * @LastEditTime: 2019-10-27 15:23:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /base-controller/base-controller.ino
 */

#include <ros.h>
#include <PID_v1.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <LobotServoController.h>
#include <gx2019_omni_simulations/arm_transport.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <LiquidCrystal.h>

LobotServoController mxarm(Serial2);

const int rs = 48, en = 46, d0 = 44, d1 = 42, d2 = 40, d3 = 38, d4 = 36, d5 = 34, d6 = 32, d7 = 30;
LiquidCrystal lcd(rs, en, d0, d1, d2, d3, d4, d5, d6, d7);

int atop(int angle_in)
{
  int angle_out;
  angle_out = map(angle_in, 0, 270, 500, 2500);
  return angle_out;
}

int IN1_AL = 23;
int IN2_AL = 25;
int IN3_AR = 27;
int IN4_AR = 29; //对应控制右轮L298N模块-1 IN1/2/3/4,用于控制电机方向与启停
int IN1_BL = 31;
int IN2_BL = 33;
int IN3_BR = 35;
int IN4_BR = 37; //对应控制左轮L298N模块-2 IN1/2/3/4,用于控制电机方向与启停
int h = 12;
int w = 19;
double arm[6] = {0};
long int longterm = 0; //长期控制的四轮平均值
String vel_read_str = "";
String omg_read_str = "";
String qrcode_msg = "";
String qrcode_msg_last = "";
double vel_read = 0;
double omg_read = 0;
double vel_in_x = 0.0;
double vel_in_y = 0.0;
double omg_in = 0.0;
double omg_in_arm = 0.0;
bool arm_moveit = false;
void test();

void velCallback(const geometry_msgs::Twist &vel)
{
  vel_in_x = vel.linear.x * 100;
  vel_in_y = vel.linear.y * 100;
  omg_in = vel.angular.z;
}

void armTransportCallback(const gx2019_omni_simulations::arm_transport &msg)
{
  omg_in_arm = msg.gimbal_rotate;
  arm_moveit = msg.arm_moveit;
}

void qrcodeCallBack(const std_msgs::String &qrcode_msg_)
{
  qrcode_msg = qrcode_msg_.data;
}

ros::NodeHandle b_c;
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);
ros::Subscriber<gx2019_omni_simulations::arm_transport> arm_transport_sub("arm_transport", armTransportCallback);
ros::Subscriber<std_msgs::String> qrcode_message_sub("qrcode_message", qrcodeCallBack);

std_msgs::Float32 float_msg;
ros::Publisher chatter("chatter", &float_msg);

geometry_msgs::Vector3 data;
ros::Publisher pos("pos",&data);

class motor_settings
{
private:
public:
  double Kp = 0.0, Ki = 0.0, Kd = 0.0; //P/I/D的值
  int status = 0;
  int hall = 0;                //arduino上传入霍尔传感器计数的引脚
  int PWM = 0;                 //arduino上控制分别控制左/右电机的引脚
  int counter = 0;             //次数int 用于计数
  long int counter_long = 0;   //长期次数long int 用于长期控制
  double counter_rotation = 0; //次数double 用于计算转速
  double input_PID = 80;       //次数double 用于计算转速Z
  double output_PID = 0;       //pid的输出值 由pid算法计算出 用于提供电机PWM[对应test的zkb(占空比)]
  double rad = 0;              //转速 由次数除以33[30(减速比)*11(编码器缺口数)/10(每秒采样10次)]
  double vel_out = 0.0;
  unsigned long times = 0;
  int take_time = 100;
  double SetPoint;

  PID myPID;

  void pid_process()
  {
    // if (abs(SetPoint) > 80)
    // {
    //   myPID.SetTunings(8, 2.2, 0.2);
    // }
    // if (abs(SetPoint) < 80)
    // {
    //   myPID.SetTunings(6.5, 2.0, 0.12);
    // }
    // if (abs(SetPoint) < 61)
    // {
    //   myPID.SetTunings(5, 1.8, 0.08);
    // }
    // if (abs(SetPoint) < 41)
    // {
    //   myPID.SetTunings(4, 1.5, 0.05);
    // }
    myPID.SetTunings(2, 0.4, 0.02);
    if (SetPoint < 0)
    {
      if (status == 1)
      {
        digitalWrite(IN3_AR, LOW);
        digitalWrite(IN4_AR, HIGH);
      }
      if (status == 2)
      {
        digitalWrite(IN1_AL, HIGH);
        digitalWrite(IN2_AL, LOW);
      }
      if (status == 3)
      {
        digitalWrite(IN1_BL, HIGH);
        digitalWrite(IN2_BL, LOW);
      }
      if (status == 4)
      {
        digitalWrite(IN3_BR, LOW);
        digitalWrite(IN4_BR, HIGH);
      }
    }
    else if (SetPoint >= 0)
    {
      if (status == 1)
      {
        digitalWrite(IN3_AR, HIGH);
        digitalWrite(IN4_AR, LOW);
      }
      if (status == 2)
      {
        digitalWrite(IN1_AL, LOW);
        digitalWrite(IN2_AL, HIGH);
      }
      if (status == 3)
      {
        digitalWrite(IN1_BL, LOW);
        digitalWrite(IN2_BL, HIGH);
      }
      if (status == 4)
      {
        digitalWrite(IN3_BR, HIGH);
        digitalWrite(IN4_BR, LOW);
      }
    }
    SetPoint = abs(SetPoint);
    input_PID = counter_rotation;
    if (millis() < 1000)
    {
      myPID.SetOutputLimits(0, 30);
    }
    else
    {
      myPID.SetOutputLimits(0, 250);
    }
    myPID.Compute(); //PID转换完成返回值为1
    analogWrite(PWM, output_PID);
    if (millis() >= times)
    {
      counter_rotation = counter;
      rad = counter_rotation / 33;

      times = millis() + take_time;
      counter = 0; //输出速度结果后清零，记录下一秒的触发次数
    }
  }
  void vel_process_mecanum()
  {
    if (status == 1)
    {
      vel_out = vel_in_x - vel_in_y + omg_in * (h + w);
      //Serial.print("rf : ");
      //Serial.println(vel_out);
    }
    else if (status == 2)
    {
      vel_out = vel_in_x + vel_in_y - omg_in * (h + w);
      //Serial.print("lf : ");
      //Serial.println(vel_out);
    }
    else if (status == 3)
    {
      vel_out = vel_in_x - vel_in_y - omg_in * (h + w);
      //Serial.print("lb : ");
      //Serial.println(vel_out);
    }
    else if (status == 4)
    {
      vel_out = vel_in_x + vel_in_y + omg_in * (h + w);
      //Serial.print("rb : ");
      //Serial.println(vel_out);
    }
  }

  void vel_process_normal()
  {
    if (status == 1 || status == 4)
    {
      vel_out = vel_in_x + omg_in * (h + w);
    }
    else if (status == 2 || status == 3)
    {
      vel_out = vel_in_x - omg_in * (h + w);
    }
  }
  void test()
  {
    float_msg.data = (float)input_PID;
    chatter.publish(&float_msg);
    // Serial.println();
    // Serial.print(input_PID);
    // Serial.println("-input_PID");
    // Serial.print(SetPoint);
    // Serial.println("-spt");
    // Serial.print(output_PID);
    // Serial.println("-zkb");
    // Serial.print(counter_rotation);
    // Serial.println("-cishu");
    // Serial.print(rad);
    // Serial.println("rad/s");
  }
  //用于在setup函数中设置各电机引脚状态
  motor_settings(int a, int b, int c) : hall(a), PWM(b), status(c), myPID(&input_PID, &output_PID, &SetPoint, Kp, Ki, Kd, DIRECT)
  {
    myPID.SetMode(AUTOMATIC);      //设置PID为自动模式
    myPID.SetSampleTime(100);      //设置PID采样频率为100ms
    myPID.SetOutputLimits(0, 250); // 输出在40-240之间
    pinMode(hall, INPUT);
    pinMode(PWM, OUTPUT);
  }
};

motor_settings right_front_wheel(18, 10, 1);
motor_settings left_front_wheel(19, 11, 2);
motor_settings right_back_wheel(2, 9, 3);
motor_settings left_back_wheel(3, 8, 4);

void left_front_count()
{
  left_front_wheel.counter++;
  left_front_wheel.counter_long++;
}
void right_front_count()
{
  right_front_wheel.counter++;
  right_front_wheel.counter_long++;
}
void left_back_count()
{
  left_back_wheel.counter++;
  left_back_wheel.counter_long++;
}
void right_back_count()
{
  right_back_wheel.counter++;
  right_back_wheel.counter_long++;
}

void longterm_ctrl()
{
  longterm = (left_front_wheel.counter_long +
              right_front_wheel.counter_long +
              left_back_wheel.counter_long +
              right_back_wheel.counter_long) /
             4;
}

void get_velomg() //此处程序为遥控车用,读取串口2收到的速度与角速度信息,格式为:线速度(m/s)v角速度(rad/s)o,示例:0.5v0.2o
{
  if (Serial2.available() > 0)
  {
    vel_read_str = "";
    omg_read_str = "";
    while (1)
    {
      if (Serial2.available() > 0 && Serial2.peek() != 118) //118对应字符v
      {
        vel_read_str += char(Serial2.read());
        delay(2);
      }

      else if (Serial2.peek() == 118)
      {
        Serial2.read();
        break;
      }
    }
    delay(5);
    while (1) //111对应字符o
    {
      if (Serial2.available() > 0 && Serial2.peek() != 111) //111对应字符o
      {
        omg_read_str += char(Serial2.read());
        delay(2);
      }
      else if (Serial2.peek() == 111)
      {
        Serial2.read();
        break;
      }
    }
  };
  //vel_in = atof(vel_read_str.c_str());gimbal_moveitgimbal_moveit
  //omg_in = atof(omg_read_str.c_str());
  //    Serial.print("vel_in=");
  //    Serial.print(vel_in);
  //    Serial.print("omg_in=");
  //    Serial.println(omg_in);
}

void shen_zhua() //伸机械臂抓物块
{
  arm[1] = 135 - 90;
  arm[2] = 135 - 42;
  arm[3] = 135 - 50;
  arm[5] = 180;
  mxarm.moveServos(4, 2000, 11, atop(arm[1]), 12, atop(arm[2]), 13, atop(arm[3]), 15, atop(arm[5]));
  delay(2200);
  arm[5] = 80;
  mxarm.moveServo(15, atop(arm[5]), 500);
}

void shen_fang() //伸机械臂放物块
{
  arm[1] = 135 - 90;
  arm[2] = 135 - 42;
  arm[3] = 135 - 50;
  arm[5] = 80;
  mxarm.moveServos(4, 2000, 11, atop(arm[1]), 12, atop(arm[2]), 13, atop(arm[3]), 15, atop(arm[5]));
  delay(2200);
  arm[5] = 180;
  mxarm.moveServo(15, atop(arm[5]), 1000);
}

void shou() //收机械臂并保持爪子之前状态
{
  arm[0] = 145;
  arm[1] = 210;
  arm[2] = 190;
  arm[3] = 155;
  mxarm.moveServo(10, atop(arm[0]), 2000); //大逆小顺
  mxarm.moveServo(11, atop(arm[1]), 2000); //大后小前
  mxarm.moveServo(12, atop(arm[2]), 3000); //大逆小顺
  mxarm.moveServo(13, atop(arm[3]), 3000); //大逆小顺
}

void zero() //机械臂归零
{

  arm[0] = 135;
  arm[1] = 135;
  arm[2] = 135;
  arm[3] = 135;
  arm[5] = 135;
  mxarm.moveServos(5, 1000, 10, atop(arm[0]), 11, atop(arm[1]), 12, atop(arm[2]), 13, atop(arm[3]), 15, atop(arm[5])); //归零
  arm[0] = 145;
  arm[1] = 210;
  arm[2] = 190;
  arm[3] = 155;
  arm[5] = 180;
  mxarm.moveServo(10, atop(arm[0]), 2000); //大逆小顺(从左往右看)
  mxarm.moveServo(11, atop(arm[1]), 2000); //大后小前
  mxarm.moveServo(12, atop(arm[2]), 2000); //大逆小顺
  mxarm.moveServo(13, atop(arm[3]), 2000); //大逆小顺
  mxarm.moveServo(15, atop(arm[5]), 2000); //大开小合
}

double omg_in_arm_last = 0;
 void turn()
 {
   if (arm_moveit == true)
   {
     shen_zhua();
     delay(1400);
     shou();
     return;
   }

   if (omg_in_arm == omg_in_arm_last)
   {
   }
   else
   {
     arm[0] -= omg_in_arm;
     omg_in_arm_last = omg_in_arm;
     float_msg.data = omg_in_arm;
     chatter.publish(&float_msg);
     if (arm[0] > 270)
     {
       arm[0] = 270;
     }
     if (arm[0] < 0)
     {
       arm[0] = 0;
     }
      mxarm.moveServo(10, atop(arm[0]), 200);
   }
 }

void disp()
{
  lcd.clear();
  lcd.print(qrcode_msg);
  qrcode_msg = qrcode_msg_last;
  qrcode_msg = "";
}

void setup()
{
  b_c.initNode();
  b_c.subscribe(sub);
  b_c.subscribe(arm_transport_sub);
  b_c.advertise(chatter);
  b_c.advertise(pos);
  //Serial.begin(9600);
  Serial2.begin(9600);
  lcd.begin(16, 2);
  pinMode(IN1_AL, OUTPUT);
  pinMode(IN2_AL, OUTPUT);
  pinMode(IN3_AR, OUTPUT);
  pinMode(IN4_AR, OUTPUT);

  pinMode(IN1_BL, OUTPUT);
  pinMode(IN2_BL, OUTPUT);
  pinMode(IN3_BR, OUTPUT);
  pinMode(IN4_BR, OUTPUT);

  digitalWrite(IN1_AL, HIGH);
  digitalWrite(IN2_AL, LOW);
  digitalWrite(IN3_AR, LOW);
  digitalWrite(IN4_AR, HIGH);

  digitalWrite(IN1_BL, HIGH);
  digitalWrite(IN2_BL, LOW);
  digitalWrite(IN3_BR, LOW);
  digitalWrite(IN4_BR, HIGH);

  zero();
  lcd.print("hello world");

  attachInterrupt(digitalPinToInterrupt(left_front_wheel.hall), left_front_count, FALLING);
  attachInterrupt(digitalPinToInterrupt(right_front_wheel.hall), right_front_count, FALLING);
  attachInterrupt(digitalPinToInterrupt(left_back_wheel.hall), left_back_count, FALLING);
  attachInterrupt(digitalPinToInterrupt(right_back_wheel.hall), right_back_count, FALLING); //中断函数 用于0计数
}
void loop()
{
  //get_velomg();                  //配合遥控建图使用
  // omg_in = 0;                    //转向速度输入
  // vel_in_x = 50;                //前进速度输入
  // vel_in_y = 0;
  // Serial.print("b_vel_in_x = ");
  // Serial.println(vel_in_x);

  turn();

  if(qrcode_msg != qrcode_msg_last)
  {
    disp();  
  }
   
  right_front_wheel.vel_process_mecanum();
  right_front_wheel.SetPoint = right_front_wheel.vel_out / (6.6 * 3.14) * 33; //填入的数字除以33即为转速/所需转速乘以33即为Setpoint_l
  right_front_wheel.pid_process();
  right_front_wheel.test();

  left_front_wheel.vel_process_mecanum();
  left_front_wheel.SetPoint = left_front_wheel.vel_out / (6.6 * 3.14) * 33; //填入的数字除以33即为转速/所需转速乘以33即为Setpoint_l
  left_front_wheel.pid_process();
  left_front_wheel.test();

  left_back_wheel.vel_process_mecanum();
  left_back_wheel.SetPoint = left_back_wheel.vel_out / (6.6 * 3.14) * 33; //填入的数字除以33即为转速/所需转速乘以33即为Setpoint_l
  left_back_wheel.pid_process();
  left_back_wheel.test();

  right_back_wheel.vel_process_mecanum();
  right_back_wheel.SetPoint = right_back_wheel.vel_out / (6.6 * 3.14) * 33; //填入的数字除以33即为转速/所需转速乘以33即为Setpoint_l
  right_back_wheel.pid_process();
  right_back_wheel.test;

  data.x = (right_back_wheel.counter_rotation + right_front_wheel.counter_rotation) / 2;
  data.y = (left_back_wheel.counter_rotation + left_front_wheel.counter_rotation) / 2;
  pos.publish(&data);
  b_c.spinOnce();
}
