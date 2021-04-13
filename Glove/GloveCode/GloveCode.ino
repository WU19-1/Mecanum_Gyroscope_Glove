#include <SoftwareSerial.h>
#include "LobotServoController.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define BTH_RX 11
#define BTH_TX 12

float min_list[5] = {0, 0, 0, 0, 0};
float max_list[5] = {255, 255, 255, 255, 255};
float sampling[5] = {0, 0, 0, 0, 0};
//data 0 - 4 mulai dari jempol ke kelingking
float data[5] = {1500, 1500, 1500, 1500, 1500};
bool turn_on = true;
SoftwareSerial Bth(BTH_RX, BTH_TX);
LobotServoController lsc(Bth);

float float_map(float in, float left_in, float right_in, float left_out, float right_out)
{
  return (in - left_in) * (right_out - left_out) / (right_in - left_in) + left_out;
}

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
float ax0, ay0, az0;
float gx0, gy0, gz0;
float ax1, ay1, az1;
float gx1, gy1, gz1;
float radianX;
float radianY;
float radianZ;
float radianX_last; 
float radianY_last;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
bool key_state = false;
int mode = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(7, INPUT_PULLUP);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A6, INPUT);
  //LED 灯配置
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  Bth.begin(9600);
  Bth.print("AT+ROLE=M");
  delay(100);
  Bth.print("AT+RESET");
  delay(250);

  Wire.begin();
  accelgyro.initialize();
  accelgyro.setFullScaleGyroRange(3); 
  accelgyro.setFullScaleAccelRange(1); 
  delay(200);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  
  ax_offset = ax;  
  ay_offset = ay;  
  az_offset = az - 8192;  
  gx_offset = gx; 
  gy_offset = gy; 
  gz_offset = gz; 
}

//获取各个手指电位器数据
void finger() {
  // put your main code here, to run repeatedly:
  static uint32_t timer_sampling;
  static uint32_t timer_init;
  static uint32_t timer_lsc = 0;
  static uint8_t init_step = 0;
  if (timer_lsc == 0)
    timer_lsc = millis();
  if (timer_sampling <= millis())
  {
    for (int i = 14; i <= 18; i++)
    {
      if (i < 18)
        sampling[i - 14] += analogRead(i); 
      else
        sampling[i - 14] += analogRead(A6);  
      sampling[i - 14] = sampling[i - 14] / 2.0; 
      data[i - 14 ] = float_map( sampling[i - 14], min_list[i - 14], max_list[i - 14], 2500, 500); 
      data[i - 14] = data[i - 14] > 2500 ? 2500 : data[i - 14];  
      data[i - 14] = data[i - 14] < 500 ? 500 : data[ i - 14];   
    }
    //timer_sampling = millis() + 10;
  }

  if (turn_on && timer_init < millis())
  {
    switch (init_step)
    {
      case 0:
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        timer_init = millis() + 20;
        init_step++;
        break;
      case 1:
        digitalWrite(2, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(6, HIGH);
        timer_init = millis() + 200;
        init_step++;
        break;
      case 2:
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        timer_init = millis() + 50;
        init_step++;
        break;
      case 3:
        digitalWrite(2, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(6, HIGH);
        timer_init = millis() + 500;
        init_step++;
        Serial.print("max_list:");
        for (int i = 14; i <= 18; i++)
        {
          max_list[i - 14] = sampling[i - 14];
          Serial.print(max_list[i - 14]);
          Serial.print("-");
        }
        Serial.println();
        break;
      case 4:
        init_step++;
        break;
      case 5:
        if ((max_list[1] - sampling[1]) > 50)
        {
          init_step++;
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, LOW);
          timer_init = millis() + 2000;
        }
        break;
      case 6:
        digitalWrite(2, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(6, HIGH);
        timer_init = millis() + 200;
        init_step++;
        break;
      case 7:
        digitalWrite(2, LOW);
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
        digitalWrite(6, LOW);
        timer_init = millis() + 50;
        init_step++;
        break;
      case 8:
        digitalWrite(2, HIGH);
        digitalWrite(3, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(6, HIGH);
        timer_init = millis() + 500;
        init_step++;
        Serial.print("min_list:");
        for (int i = 14; i <= 18; i++)
        {
          min_list[i - 14] = sampling[i - 14];
          Serial.print(min_list[i - 14]);
          Serial.print("-");
        }
        Serial.println();
        lsc.runActionGroup(0, 1);
        turn_on = false;
        break;

      default:
        break;
    }
  }
}

void update_mpu6050()
{
  static uint32_t timer_u;
  if (timer_u < millis())
  {
    // put your main code here, to run repeatedly:
    timer_u = millis() + 20;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    ax0 = ((float)(ax)) * 0.3 + ax0 * 0.7;  
    ay0 = ((float)(ay)) * 0.3 + ay0 * 0.7;
    az0 = ((float)(az)) * 0.3 + az0 * 0.7;
    ax1 = (ax0 - ax_offset) /  8192.0;  
    ay1 = (ay0 - ay_offset) /  8192.0;
    az1 = (az0 - az_offset) /  8192.0;

    gx0 = ((float)(gx)) * 0.3 + gx0 * 0.7;  
    gy0 = ((float)(gy)) * 0.3 + gy0 * 0.7;
    gz0 = ((float)(gz)) * 0.3 + gz0 * 0.7;
    gx1 = (gx0 - gx_offset);  
    gy1 = (gy0 - gy_offset);
    gz1 = (gz0 - gz_offset);


    
    radianX = atan2(ay1, az1);
    radianX = radianX * 180.0 / 3.1415926;
    float radian_temp = (float)(gx1) / 16.4 * 0.02;
    radianX_last = 0.8 * (radianX_last + radian_temp) + (-radianX) * 0.2;

    
    radianY = atan2(ax1, az1);
    radianY = radianY * 180.0 / 3.1415926;
    radian_temp = (float)(gy1) / 16.4 * 0.01;
    radianY_last = 0.8 * (radianY_last + radian_temp) + (-radianY) * 0.2;
  }
}

void print_data()
{
  static uint32_t timer_p;
  static uint32_t timer_printlog;
  if ( 0 && timer_p < millis())
  {
    Serial.print("ax:"); Serial.print(ax1);
    Serial.print(", ay:"); Serial.print(ay1);
    Serial.print(", az:"); Serial.print(az1);
    Serial.print(", gx:"); Serial.print(gx1);
    Serial.print(", gy:"); Serial.print(gy1);
    Serial.print(", gz:"); Serial.print(gz1);
    Serial.print(", GX:"); Serial.print(radianX_last);
    Serial.print(", GY:"); Serial.print(radianY_last);
    //    Serial.print(", Data 0 : "); Serial.print(data[0]);
    //    Serial.print(", Data 1 : "); Serial.print(data[1]);
    //    Serial.print(", Data 2 : "); Serial.print(data[2]);
    //    Serial.print(", Data 3 : "); Serial.print(data[3]);
    //    Serial.print(", Data 4 : "); Serial.println(data[4]);
    timer_p = millis() + 500;
  }

  if (timer_printlog <= millis())
  {
    for (int i = 14; i <= 18; i++)
    {
      Serial.print(data[i - 14]);
      Serial.print("  ");
      // Serial.print(float_map(min_list[i-14], max_list[i-14], 500,2500,sampling[i-14]));
      Serial.print(" ");
      // Serial.print();
    }
    timer_printlog = millis() + 1000;
    Serial.println();
  }
}


void run2()
{
  static uint32_t timer;
  static uint32_t step;
  int act = 0;
  static int last_act;
  if (timer > millis())
    return;
  timer = millis() + 100;
  if (radianY_last < -30 && radianY_last > -90)//Go Right
  {
    act = 4;
  } 
  else if (radianY_last < 90 && radianY_last > 30) //Go Left
  {
    act = 3;
  } 
  else if ((data[1] > 1600 && data[2] > 1600 && data[3] > 1600 && data[4] > 1600)) //Go Forward
  {
    act = 1;
  }
  else if ((data[1] < 1000 && data[2] < 1000 && data[3] < 1000 && data[4] < 1000))  //Stop
  {
    act = 0;
  }
  else if(radianX_last > 40  && radianX_last < 100){
    act = 2;
  }
  if (1)
  {
    last_act = act;
    if (act == 0)
    {
      Serial.println("Stop");
      Bth.print("a,0,0,"); 
      return;
    }
    if (act == 1)
    {
      Serial.println("Go Forward");
      Bth.print("b,0,0,"); 
      return;
    }
    if (act == 2)
    {
      Serial.println("Go Backward");
      Bth.print("c,0,0"); 
      return;
    }
    if (act == 3)
    {
      Serial.println("Go Left");
      Bth.print("d,0,0,");
      return;
    }
    if (act == 4)
    {
      Serial.println("Go Right");
      Bth.print("e,0,0");
      return;
    }
  }
}


void loop() {
  finger();
  update_mpu6050();

  if (turn_on == false) 
  {
    if (key_state == true && digitalRead(7) == true)
    {
      delay(30);
      if (digitalRead(7) == true)
        key_state = false;
    }
    if (digitalRead(7) == false && key_state == false)
    {
      delay(30);
      if (digitalRead(7) == false)
      {
        key_state = true;
        if (mode == 5)
        {
          mode = 0;
        }
        else
          mode++;
        if (mode == 0)
        {
          digitalWrite(2, HIGH);
          digitalWrite(3, HIGH);
          digitalWrite(4, HIGH);
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
        }
        if (mode == 1)
        {
          digitalWrite(2, LOW);
          digitalWrite(3, HIGH);
          digitalWrite(4, HIGH);
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
        }
        if (mode == 2)
        {
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, HIGH);
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
        }
        if (mode == 3)
        {
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, HIGH);
          digitalWrite(6, HIGH);
        }

        if (mode == 4)
        {
          digitalWrite(2, LOW);
          digitalWrite(3, LOW);
          digitalWrite(4, LOW);
          digitalWrite(5, LOW);
          digitalWrite(6, HIGH);
        }
      }
    }
  }

  // run mekanum
  run2();
  // testing print data
  //print_data();
}
