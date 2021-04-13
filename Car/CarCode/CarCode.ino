#include <Wire.h>
#include <SPI.h>
#include <Encoder.h>
//#include "timer.h"

//This code below is for controlling motor
class SPDMotor {
  public:
  SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 );

  /// Set the PWM speed and direction pins.
  /// pwm = 0, stop (no active control)
  /// pwm = 1 to 255, proportion of CCW rotation
  /// pwm = -1 to -255, proportion of CW rotation
  void speed( int pwm );

  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;

    // Current speed setting.
    int _speed;
};

SPDMotor::SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 ) {
  _encoder = new Encoder(encoderA, encoderB);
  _encoderReversed = encoderReversed;

  _motorPWM = motorPWM;
  pinMode( _motorPWM, OUTPUT );
  _motorDir1 = motorDir1;
  pinMode( _motorDir1, OUTPUT );
  _motorDir2 = motorDir2;
  pinMode( _motorDir2, OUTPUT );
}

/// Set the PWM speed and direction pins.
/// pwm = 0, stop (no active control)
/// pwm = 1 to 255, proportion of CCW rotation
/// pwm = -1 to -255, proportion of CW rotation
void SPDMotor::speed( int speedPWM ) {
  _speed = speedPWM;
  if( speedPWM == 0 ) {
    digitalWrite(_motorDir1,LOW);
    digitalWrite(_motorDir2,LOW);
    analogWrite( _motorPWM, 255);
  } else if( speedPWM > 0 ) {
    digitalWrite(_motorDir1, LOW );
    digitalWrite(_motorDir2, HIGH );
    analogWrite( _motorPWM, speedPWM < 255 ? speedPWM : 255);
  } else if( speedPWM < 0 ) {
    digitalWrite(_motorDir1, HIGH );
    digitalWrite(_motorDir2, LOW );
    analogWrite( _motorPWM, (-speedPWM) < 255 ? (-speedPWM): 255);
  }
}

//Motor/Servo declaration
SPDMotor *motorLF = new SPDMotor(18, 31, true, 12, 34, 35); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, false, 8, 36, 37); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor *motorLR = new SPDMotor( 3, 49, true,  9, 43, 42); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor( 2, 23, false, 5, 27, 26); // <- NOTE: Motor Dir pins reversed for opposite operation

//Timer timer;

String directionAndroid;
char charFromAndroid;
int countIndexDirection=0;
int flagForRunToLocation=0;

void (*resetApps) (void)=0;

//Setting speed for each wheel
void moveForward() {
  motorLF->speed(65);
  motorLR->speed(65);
  motorRF->speed(62);
  motorRR->speed(62);
}
void moveBackward() {
  motorLF->speed(-65);
  motorLR->speed(-65);
  motorRF->speed(-63.3);
  motorRR->speed(-63.3);
}
void moveSidewaysRight() {
  motorLF->speed(69);
  motorLR->speed(-60);
  motorRF->speed(-64);
  motorRR->speed(63);
}
void moveSidewaysLeft() {
  motorLF->speed(-67.5);
  motorLR->speed(61);
  motorRF->speed(61);
  motorRR->speed(-61);
}
void moveRightForward() {
  motorLF->speed(37);
  motorLR->speed(0);
  motorRF->speed(0);
  motorRR->speed(30);
}
void moveLeftForward() {
  motorLF->speed(0);
  motorLR->speed(34);
  motorRF->speed(35);
  motorRR->speed(0);
}
void moveRightBackward() {
  motorLF->speed(0);
  motorLR->speed(-35);
  motorRF->speed(-35);
  motorRR->speed(0);
}
void moveLeftBackward() {
  motorLF->speed(-38);
  motorLR->speed(0);
  motorRF->speed(0);
  motorRR->speed(-32);
}
void stopMoving() {
  motorLF->speed(0);
  motorLR->speed(0);
  motorRF->speed(0);
  motorRR->speed(0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  Serial.println("Hello!");
  
  SPI.begin(); // Init SPI bus
}

void loop() {
  // put your main code here, to run repeatedly:
  int step = 0;
  char cmd = 'N';
  int  p1=0,p2=0;
  while(Serial2.available() > 0)
  {
    switch(step){
      case 0:
        cmd = Serial2.read();
        if(cmd >= 'A' && cmd <= 'z')        
        {
          delay(5);  
          Serial2.read();  
          step = 1;   
        }
        break;
     case 1:
        p1 = Serial2.parseInt(); 
        Serial2.read();         
        p2 = Serial2.parseInt();
        step = 2;       
        break;
      case 2:
//        Serial.print(cmd);       
//        Serial.print(':');
//        Serial.print(p1);
//        Serial.print(':');
//        Serial.println(p2);
        Serial2.print(cmd);       
        Serial2.print(':');
        Serial2.print(p1);
        Serial2.print(':');
        Serial2.println(p2);
        switch(cmd){
          case 'a':
            stopMoving();
            Serial.println("Stop");
            break;
          case 'b':
            moveForward();
            Serial.println("Move forward");
            break;
          case 'c':
            moveBackward();
            Serial.println("Move backward");
            break;
          case 'd':
            moveSidewaysLeft();
            Serial.println("Go left");
            break;
          case 'e':
            moveSidewaysRight();
            Serial.println("Go right");
            break;
          default:
            break;
        }
        step = 0;
        break;
      default:
        step = 0;
        break;
    }
  }
}
