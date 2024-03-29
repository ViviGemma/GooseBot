// Team 301 'GooseBot' psuedocode

// This document is meant to be the central script that will be uploaded to uc
// It is mostly psuedocode now, but should be replaced as time goes on


// Include Libraries
#include <Encoder.h>
#include <Servo.h>

// Define pins
#define Lidar2Pin A0
#define Lidar3Pin A2
#define WheelEN3 8 //Front Left
#define WheelEN4 7//Front Right
#define WheelEN1 9 //Back Left
#define WheelEN2 10 //Back Right
#define WheelIN3_2 22
#define WheelIN3_1 23
#define WheelIN4_2 24
#define WheelIN4_1 25
#define WheelIN2_2 26
#define WheelIN2_1 27
#define WheelIN1_1 39
#define WheelIN1_2 38
#define CombineInput1 50
#define CombineInput2 53
#define Wheel3EncB 18
#define Wheel3EncA 14
#define Wheel4EncA 19
#define Wheel4EncB 15
#define Wheel1EncA 20
#define Wheel1EncB 16
#define Wheel2EncA 21
#define Wheel2EncB 17
#define LiDAR1Pin 45
//Define Notes
#define C 262
#define D 294
#define F 349
#define G 392
#define ASharp 466

// Construct Encoder Objects
Encoder Wheel1Enc(Wheel1EncA,Wheel1EncB);
Encoder Wheel2Enc(Wheel2EncA,Wheel2EncB);
Encoder Wheel3Enc(Wheel3EncA,Wheel3EncB);
Encoder Wheel4Enc(Wheel4EncA,Wheel4EncB);
Servo servo;

// Declare Variables
double countsPerRev = 2256;
const int numSensors = 4; // Number of IR Sensors
int numSamples = 0; // Index for DFT Sample Buffer
const int sampleBufferSize = 12; // Size of DFT Sample Buffer
char IRAdcPin[numSensors] = {'A2','A3','A1','A0'}; // ADC Pins for IR Sensors
int sampleBuffer[numSensors][sampleBufferSize]; // Init DFT Sample Buffer
bool ledState = 0; // State of IR LED
double* IRSensorReading; // Array of IR Sensor readings
int blockCollectionState = 0; // System State During Block Collection Routine
int mainState = 0; // Main System State 
bool previousTurn;
//***FUNCTION DEFINITIONS***//
// Motor Initialization
  // PWM Initialization
  void InitMotors() {
    // Set Motor Control Pins to Outputs
    pinMode(WheelEN1, OUTPUT);
    pinMode(WheelEN2, OUTPUT);
    pinMode(WheelEN3, OUTPUT);
    pinMode(WheelEN4, OUTPUT);
    pinMode(WheelIN1_1, OUTPUT);
    pinMode(WheelIN1_2, OUTPUT);
    pinMode(WheelIN2_1, OUTPUT);
    pinMode(WheelIN2_2, OUTPUT);
    pinMode(WheelIN3_1, OUTPUT);
    pinMode(WheelIN3_2, OUTPUT);
    pinMode(WheelIN4_1, OUTPUT);
    pinMode(WheelIN4_2, OUTPUT);

  }
  // Encoder Initialization
  void InitEncoders() {
    // Set Encoder Pins to Inputs
    pinMode(Wheel1EncA,INPUT);
    pinMode(Wheel1EncB,INPUT);
    pinMode(Wheel2EncA,INPUT);
    pinMode(Wheel2EncB,INPUT);
    pinMode(Wheel3EncA,INPUT);
    pinMode(Wheel3EncB,INPUT);
    pinMode(Wheel4EncA,INPUT);
    pinMode(Wheel4EncB,INPUT);
  }
// Recognise Start
bool SenseStartLight() {
  // Some how sense the green light
}
// Block Collection Routine
  // Spin Wheels
   void SpinWheels(int duty1, int duty2, int duty3,int duty4){
    bool IN1P = 0;//Plus
    bool IN1M = 0;//Minus
    bool IN2P = 0;//Plus
    bool IN2M = 0;//Minus
    bool IN3P = 0;//Plus
    bool IN3M = 0;//Minus
    bool IN4P = 0;//Plus
    bool IN4M = 0;//Minus
    if(duty1>0){
        IN1P = 1;
    }
    else if(duty1<0){
        IN1M = 1;
    }
    
    if(duty2>0){
        IN2P = 1;
    }
    else if(duty2<0){
        IN2M = 1;
    }

    if(duty3>0){
        IN3P = 1;
    }
    else if(duty3<0){
      
        IN3M = 1;
    }
    if(duty4>0){
        IN4P = 1;
    }
    else if(duty4<0){
        IN4M = 1;
    }

    double realDuty1 = map(duty1,-100,100,-255,255);
    double realDuty2 = map(duty2,-100,100,-255,255);
    double realDuty3 = map(duty3,-100,100,-255,255);
    double realDuty4 = map(duty4,-100,100,-255,255);
    
    digitalWrite(WheelIN1_1,IN1P);
    digitalWrite(WheelIN1_2,IN1M);
    digitalWrite(WheelIN2_1,IN2P);
    digitalWrite(WheelIN2_2,IN2M);
    digitalWrite(WheelIN3_1,IN3P);
    digitalWrite(WheelIN3_2,IN3M);
    digitalWrite(WheelIN4_1,IN4P);
    digitalWrite(WheelIN4_2,IN4M);
    analogWrite(WheelEN1,abs(realDuty1));
    analogWrite(WheelEN2,abs(realDuty2));
    analogWrite(WheelEN3,abs(realDuty3));
    analogWrite(WheelEN4,abs(realDuty4));     
  }
  // Dynamic Brakeing
  void DynamicBrake(){
    // Write all motor driver inputs HIGH
    digitalWrite(WheelIN1_1,HIGH);
    digitalWrite(WheelIN1_2,HIGH);
    digitalWrite(WheelIN2_1,HIGH);
    digitalWrite(WheelIN2_2,HIGH);
    digitalWrite(WheelIN3_1,HIGH);
    digitalWrite(WheelIN3_2,HIGH);
    digitalWrite(WheelIN4_1,HIGH);
    digitalWrite(WheelIN4_2,HIGH);
    digitalWrite(WheelEN1,HIGH);
    digitalWrite(WheelEN2,HIGH);
    digitalWrite(WheelEN3,HIGH);
    digitalWrite(WheelEN4,HIGH);
    delay(100);
  }

  // Turn CCW
  void Turn90DegPointCCW(){
    Wheel1Enc.write(0); // Reset enc
    double wheel1Pos = 0;
    double wheel1Rev = 0;
    double wheel1Limit = -1.07; // max wheel 1 position
    
    while (wheel1Rev>wheel1Limit){
      wheel1Pos = Wheel1Enc.read();
      wheel1Rev = wheel1Pos/countsPerRev;
      SpinWheels(-60,-60,60,60); // turn
    }
    DynamicBrake();
  }
    void Turn180DegWideCCW(){
    Wheel3Enc.write(0); // Reset enc
    double wheel3Pos = 0;
    double wheel3Rev = 0;
    double wheel3Limit = -4.65; // max wheel 1 position
    while (wheel3Rev>wheel3Limit){
      wheel3Pos = Wheel3Enc.read();
      Serial.println(wheel3Rev);
      wheel3Rev = wheel3Pos/countsPerRev;
      SpinWheels(-20,-20,100,100); // turn
    }
    DynamicBrake();
  }

  void Turn90DegWideCCW(){
    Wheel3Enc.write(0); // Reset enc
    double wheel3Pos = 0;
    double wheel3Rev = 0;
    double wheel3Limit = -2.325; // max wheel 1 position
    while (wheel3Rev>wheel3Limit){
      wheel3Pos = Wheel3Enc.read();
      Serial.println(wheel3Rev);
      wheel3Rev = wheel3Pos/countsPerRev;
      SpinWheels(0,0,100,100); // turn
    }
    DynamicBrake();
  }
  // Turn CW
  void Turn90DegPointCW(){
    Wheel3Enc.write(0);
    double wheel1Pos = 0;
    double wheel1Rev = 0;
    double wheel1Limit = 1.07;
    while (wheel1Rev<wheel1Limit){
      wheel1Pos = Wheel1Enc.read();
      wheel1Rev = wheel1Pos/countsPerRev;
      SpinWheels(60,60,-60,-60);
    }
    DynamicBrake();
  }
  // Move Forward
  void MoveForward(double dist){ //in mm
    Wheel1Enc.write(0);
    double wheel1Pos = 0;
    double wheel1Rev = 0;
    double wheel1Limit = dist/(3.1415*60); // converts mm to rev
    while (wheel1Rev<wheel1Limit){
      wheel1Pos = Wheel1Enc.read();
      wheel1Rev = wheel1Pos/countsPerRev;
      SpinWheels(100,100,100,100);
    }
    DynamicBrake();
  }
  //Block Collection Routine
  void BlockCollectionRoutine(){
    IntakeCombine();
    MoveForward(200);
    int wheel1Pos = 0;
    Wheel1Enc.write(0);
    while (wheel1Pos<12000){
      wheel1Pos = Wheel1Enc.read();
      LineFollow();
    } //mm
    delay(10);
    DynamicBrake();
    Turn90DegPointCCW();
    delay(10);
    SpinWheels(-50,-50,-50,-50);
    delay(100);
    DynamicBrake();
    Turn90DegPointCCW();
    delay(10);
    while(digitalRead(LiDAR1Pin)){
      SpinWheels(75,75,75,75);
    }
    DynamicBrake();
    delay(10);
    Turn90DegPointCW();
    delay(10);
    StopCombine();  
    MoveForward(80); //mm
    delay(10);
    Turn90DegPointCW();
    digitalWrite(WheelEN1,LOW);
    digitalWrite(WheelEN2,LOW);
    digitalWrite(WheelEN3,LOW);
    digitalWrite(WheelEN4,LOW);
  }
  void BlockCollection2(){
    IntakeCombine();
    Turn90DegPointCW();
    MoveForward(50);
    SpinWheels(-50,-50,-50,-50);
    delay(100);
    SpinWheels(0,0,0,0);
    Turn90DegPointCW(); 
  }
  // Line Following
  void LineFollow(){
    bool sensor0 =digitalRead(Lidar2Pin);
    bool sensor1 = digitalRead(Lidar3Pin);
      if (sensor0 && sensor1){ // if Line is centered
        SpinWheels(100,100,100,100); //go straight
      }
      else if(!sensor0 && !sensor1) {// Line is centered
        SpinWheels(100,100,100,100); // go straight
      }
      else if(sensor0 && !sensor1){ //line slightly to right
        SpinWheels(100,100,60, 60); //turn slight right
      }
      else if(!sensor0 && sensor1){ //line slightly to right
        SpinWheels(60,60,100, 100); //turn slight right
      }
  }
  // Motor Control
 
  // Deliver Blocks
void BlockDelivery(){
    Wheel1Enc.write(0);
    double wheel1Pos = 0;
    while(wheel1Pos<15000){
      wheel1Pos = Wheel1Enc.read();
      LineFollow();
    }  
    DynamicBrake();
    delay(10);
    while(digitalRead(LiDAR1Pin)){
      SpinWheels(50,50,50,50);
    }
   
    DynamicBrake();
    OuttakeCombine();
    delay(10000);
    StopCombine();
    SpinWheels(-50,-50,-50,-50);
    delay(100);
    SpinWheels(0,0,0,0);
}
  // Recognize Crater Edge
//
// Combine Intake
void IntakeCombine(){     
  digitalWrite(CombineInput1,HIGH);
  digitalWrite(CombineInput2,LOW);
}
// Combine Outtake
void OuttakeCombine(){
  digitalWrite(CombineInput1,LOW);
  digitalWrite(CombineInput2,HIGH);
}
//Combine Stop
void StopCombine(){
  digitalWrite(CombineInput1,HIGH);
  digitalWrite(CombineInput2,HIGH);
  delay(50);
  digitalWrite(CombineInput1,LOW);
  digitalWrite(CombineInput2,LOW);
}
// Ramp Deployment
// Ramp Crossing
// Find Button
// Press Button
// School Spirit??
void Channel1PWM(unsigned char duty, double freq) {
  TCCR3A = 0x21;
  TCCR3B = 0x14;
  OCR3A = 0x7A12 / freq;
  OCR3B = OCR3A * (duty / 255.0);
}
void PlayWarChant(){
  Channel1PWM(50,G);
  delay(1500);
  Channel1PWM(50,F);
  delay(500);
  Channel1PWM(50,G);
  delay(125);
  Channel1PWM(50,F);
  delay(125);
  Channel1PWM(50,D);
  delay(1625);
  Channel1PWM(50,G);
  delay(125);
  Channel1PWM(50,F);
  delay(125);
  Channel1PWM(50,D);
  delay(1125);
  Channel1PWM(50,C);
  delay(500);
   Channel1PWM(50,C);
  delay(125);
  Channel1PWM(50,ASharp/2);
  delay(125);
  Channel1PWM(50,G/2);
  delay(1625);
}

// Setup
void setup() {
  Serial.begin(9600);
  pinMode(LiDAR1Pin, INPUT_PULLUP); 
  pinMode(Lidar2Pin,INPUT_PULLUP);
  pinMode(Lidar3Pin,INPUT_PULLUP);
  servo.attach(6);
  servo.write(0);

  InitMotors();
  InitEncoders();
}
//***MAIN LOOP***//
void loop(){  
  LineFollow();
  // MoveForward(5);
  // DynamicBrake();
  // // BlockCollectionRoutine();
  // BlockDelivery();  
  // delay(50);
  // Turn90DegPointCCW();
  // delay(50);
  // while(digitalRead(LiDAR1Pin)){
  //   SpinWHeels(75,75,75,75);
  // }
  // DynamicBrake();
  // delay(50);
  // Turn90DegPointCCW();
  // delay(50);
  // Wheel1Enc.write(0);
  // double wheel1Pos = 0;
  // // MoveForward(600);
  // while(wheel1Pos<8000){
  //   wheel1Pos = Wheel1Enc.read();
  //   LineFollow();
  // }
  // DynamicBrake();
  // servo.write(180);
  // SpinWheels(-50,-50,-50,-50);
  // delay(200);
  // DynamicBrake();
  // MoveForward(500);
  // DynamicBrake();
  // while (digitalRead(LiDAR1Pin)){
  //   LineFollow();
  // }
  // DynamicBrake();
  // Turn90DegPointCW();
  // MoveForward(100);
  // DynamicBrake();
  // SpinWheels(-50,-50,-50,-50);
  // delay(200);
  // DynamicBrake();
  // while(1){
  //   PlayWarChant();
  // }
//  mainState = 1;
//  switch (mainState){
//    case 0: // Initialization
//      int startFlag = 1;
//      if (startFlag)
//        mainState = 1;
//      else
//        mainState = 0; 
//      break;
//    case 1: // Block Collection
//      BlockCollectionRoutine();
//     break;
//    case 2: // Block Delivery
//      break;
//    case 3: // Thruster Collection and Delivery
//      break;
//    case 4: // Deploy Ramp
//      break;
//    case 5: // Cross Crater
//      break;
//    case 6: // Press Button
//      break;
//    case 7: // Celebration
//      break;
// }
}
