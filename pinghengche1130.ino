#include  "Wire.h"`
#include <U8g2lib.h>
#include <SPI.h>
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 100; // Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    Wire.write(data, length);
    uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
    if (rcode) {
      Serial.print(F("i2cWrite failed: "));
      Serial.println(rcode);
    }
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
    uint32_t timeOutTimer;
    Wire.beginTransmission(IMUAddress);
    Wire.write(registerAddress);
    uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
    if (rcode) {
      Serial.print(F("i2cRead failed: "));
      Serial.println(rcode);
      return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
    }
    Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
    for (uint8_t i = 0; i < nbytes; i++) {
      if (Wire.available())
        data[i] = Wire.read();
      else {
        timeOutTimer = micros();
        while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
        if (Wire.available())
          data[i] = Wire.read();
        else {
          Serial.println(F("i2cRead timeout"));
          return 5; // This error value is not already taken by endTransmission
        }
      }
    }
    return 0; // Success
}

/******************************************************/

//2560 pin map  引脚定义好即可，然后改变一下PID的几个值（kp，kd，ksp，ksi）即可，剩下的全部都是固定的程序，
//可能小车会有一点重心不在中点的现象，加一下角度值或者减一点即可
//至于每个MPU6050的误差，自己调节一下即可，不是很难
//调试时先将速度环的ksp，ksi=0，调到基本可以站起来，然后可能会出现倒，或者自动跑起来的时候加上速度环
//这时就会很稳定的站起来，然后用小力气的手推不会倒。

int ENA=10;
int ENB=11;
int IN1=4;
int IN2=5;
int IN3=6;
int IN4=7;
int MAS,MBS;
/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
uint8_t i2cData[14]; // Buffer for I2C data
uint32_t timer;
unsigned long lastTime;      
/***************************************/
double P[2][2] = {{ 1, 0 },{ 0, 1 }};
double Pdot[4] ={ 0,0,0,0};
static const double Q_angle=0.001, Q_gyro=0.003, R_angle=0.5,dtt=0.005,C_0 = 1;
double q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
double angle,angle_dot,aaxdot,aax;
double position_dot,position_dot_filter,positiono;
/*-------------Encoder---------------*/

#define LF 0
#define RT 1

//The balance PID
float kp,kd,ksp,ksi;

int Lduration,Rduration;
boolean LcoderDir,RcoderDir;
const byte encoder0pinA = 2;
const byte encoder0pinB = 8;
byte encoder0PinALast;
const byte encoder1pinA = 3;
const byte encoder1pinB = 9;
byte encoder1PinALast;

int RotationCoder[2];
int turn_flag=0;
float move_flag=0;//////////////////////////////////
float right_need = 0, left_need = 0;;

int pwm;
int pwm_R,pwm_L;
float range;
float range_error_all;
float wheel_speed;
float last_wheel;
float error_a=0;
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 13, /* reset=*/ U8X8_PIN_NONE);   // 四角显示屏
void Kalman_Filter(double angle_m,double gyro_m)
{
    angle+=(gyro_m-q_bias) * dtt;
    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]=- P[1][1];
    Pdot[2]=- P[1][1];
    Pdot[3]=Q_gyro;
    P[0][0] += Pdot[0] * dtt;
    P[0][1] += Pdot[1] * dtt;
    P[1][0] += Pdot[2] * dtt;
    P[1][1] += Pdot[3] * dtt;
    angle_err = angle_m - angle;
    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];
    E = R_angle + C_0 * PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];
    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;
    angle+= K_0 * angle_err;
    q_bias += K_1 * angle_err;
    angle_dot = gyro_m-q_bias;//也许应该用last_angle-angle
}
 
void setup() {
    Wire.begin();
    u8g2.begin();   //选择U8G2模式，或者U8X8模式
    Serial.begin(9600);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);  
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
 
    i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
    while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
    while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
 
    while (i2cRead(0x75, i2cData, 1));
    if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
        Serial.print(F("Error reading sensor"));
        while (1);
    }

    delay(20); // Wait for sensor to stabilize

    while (i2cRead(0x3B, i2cData, 6));
    accX = (i2cData[0] << 8) | i2cData[1];
    accY = (i2cData[2] << 8) | i2cData[3];
    accZ = (i2cData[4] << 8) | i2cData[5];


    double roll  = atan2(accX, accZ) * RAD_TO_DEG;
    EnCoderInit();
    timer = micros();

      //The balance PID
    kp= 42;//42;//24.80;43
    kd= 2.0;//1.9;//9.66;1.4
    ksp=8.5;//8.5;//4.14;
    ksi=2.1;//2.1;//0.99; 0.550
     u8g2.firstPage();
  do {
     u8g2.setFont(u8g2_font_ncenB14_tr); //设置字体/////u8g2_font_5x7_tr
     u8g2.setCursor(5, 15);    //设置光标处
     u8g2.print("MiniBlack");  //输出内容
     u8g2.setCursor(0,40);    //设置光标处
     u8g2.print("Copyright@");  //输出内容
      u8g2.setCursor(70,60);    //设置光标处
     u8g2.print("PWZ");  //输出内容
  } while ( u8g2.nextPage() );
}

void EnCoderInit()
{
    pinMode(8,INPUT);
    pinMode(9,INPUT);
    attachInterrupt(LF, LwheelSpeed, RISING);
    attachInterrupt(RT, RwheelSpeed, RISING);
}

void pwm_calculate()
{
    unsigned long  now = millis();       // 当前时间(ms)
    int Time = now - lastTime;
    int range_error;
    range += (Lduration + Rduration) * 0.5;
    range *= 0.9;
    range_error = Lduration - Rduration;
    range_error_all += range_error;
    
    wheel_speed = range - last_wheel;   
    //wheel_speed = constrain(wheel_speed,-800,800);
    //Serial.println(wheel_speed);
    last_wheel = range;  
    pwm = (angle + 0.825) * kp + angle_dot * kd + range * ksp + wheel_speed * ksi;     
    if(pwm > 255)pwm = 255;
    if(pwm < -255)pwm = -255;
    
    if(turn_flag == 0)
    {
         pwm_R = pwm + range_error_all;
         pwm_L = pwm - range_error_all;
    }
    else if(turn_flag == 1)     //左转
    {
        pwm_R = pwm ;  //Direction PID control
        pwm_L = pwm + left_need * 68;
        range_error_all = 0;     //clean
    }
    else if(turn_flag == 2)
    {
        pwm_R = pwm + right_need * 68;  //Direction PID control
        pwm_L = pwm ;
        range_error_all = 0;     //clean
    }
       
       Lduration = 0;//clean
       Rduration = 0;
       lastTime = now;
}

void PWMD()
{  
      if(pwm>0)
      {
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, HIGH);    
      }
      else if(pwm<0)
      {
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
      }
      int PWMr = abs(pwm);
      int PWMl = abs(pwm);
    
      analogWrite(ENB, max(PWMl,51)); //PWM调速a==0-255  51
      analogWrite(ENA, max(PWMr,54)); //PWM调速a==0-255  54
      
}

void LwheelSpeed()
{
      if(digitalRead(encoder0pinB))
        Lduration++;
      else Lduration--;
}


void RwheelSpeed()
{
      if(digitalRead(encoder1pinB))
        Rduration--;
      else Rduration++;
}

void control()
{
    if(Serial.available()){
      int val;
      val=Serial.read();
      switch(val){
        case 'w':
          if(move_flag<5)move_flag += 0.5;
          else  move_flag = 0;
          break;
        case 's':
          if(move_flag<5)move_flag -= 0.5;
          else  move_flag = 0;
         Serial.println("back");
          break;
        case 'a':
          turn_flag = 1;
          left_need = 0.6;
          Serial.println("zuo");
          break;
        case 'd':
          turn_flag = 2;
          right_need = 0.6;
          Serial.println("you");
          break;
        case 't':
          move_flag=0;
          turn_flag=0;
          right_need = left_need = 0;
          Serial.println("stop");
          break;
        default:
          break;
          }
      }
}

void loop()
{

    control();
    while (i2cRead(0x3B, i2cData, 14));
    accX = ((i2cData[0] << 8) | i2cData[1]);
    //accY = ((i2cData[2] << 8) | i2cData[3]);
    accZ = ((i2cData[4] << 8) | i2cData[5]);
    //gyroX = (i2cData[8] << 8) | i2cData[9];
    gyroY = (i2cData[10] << 8) | i2cData[11];
    //gyroZ = (i2cData[12] << 8) | i2cData[13];

    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
 
    double roll  = atan2(accX, accZ) * RAD_TO_DEG - move_flag;


    double gyroXrate = gyroX / 131.0; // Convert to deg/s
    double gyroYrate = -gyroY / 131.0; // Convert to deg/s

    //gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    //gyroYangle += gyroYrate * dt;

    Kalman_Filter(roll,gyroYrate);   
    if(abs(angle)<35){
        //Serial.println(angle); 
       ////////////////////////////////////////////
       
        //////////////////////////////////////  
        pwm_calculate();
        PWMD();
    }
    else{
        analogWrite(ENB, 0); //PWM调速a==0-255
        analogWrite(ENA, 0); //PWM调速a==0-255
    }  
    delay(2);
}
