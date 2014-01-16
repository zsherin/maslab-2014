#include <Wire.h>

/*
MAPLE Microcontroller Code in Processing

Specs:
  The Code can only run for 7hours.  This is due to the nature of microsec() cycling every 7 hours.

TODO:
1 Byte math check.
2 Gyro misConfig Error.
3 Gyro OOP Data.
4 Add MC and BMP checks

Completed:
Componenet      OOP    SensorFilter   String(output)
Gyro            [x]         []              []
Motor/Encoder   [x]       [N/A]             []
Sonar           [x]       [EMWA]              []
Acc             [x]        

Author:  MASLAB2014 TeamSeven @2014
*/

/*
OOP specs

Object.sample:  sample & store 1 data sample,  usually filter is in this.

Object.readData:  for the main loop to grab the data.

Actuator.set:  to set the actuator in a specific state{Motor: voltage}
*/


// Acc Reader V1.0
//I2C addresses
byte GyroL3 = 0x69; //L3G4200D Gyro
byte AccAD = 0x53;  //ADXL345 Accelerometer
byte ComMC = 0x1E;  //HMC5883L Magnetic Compass
byte BaroBM = 0x77; //BMP085 Barometer + Thermometer
class Acc{
  //Acc scale = 8/1024 (8G) = 0.0078
  public:
  double dxt;
  double dyt;
  double heading;
  long AccTime;
  long GyroTime;
  Acc(){
    dxt = 0;
    dyt = 0;
    heading = 0;
    Wire.begin();
    //Begin I2C Testing.
    Wire.beginTransmission(AccAD);
    Wire.send(0x00);
    if(Wire.receive()!=0xE5){
      SerialUSB.println("Error ADSL345 Accelerometer: Not recognized in I2C.  Incorrect ID received.");
    }
    Wire.endTransmission();
    
    Wire.beginTransmission(GyroL3);
    Wire.send(0x0F);
    if(Wire.receive()!=0xD3){
      SerialUSB.println("Error L3G4200D Gyro: Not recognized in I2C.  Incorrect ID received.");
    }
    Wire.endTransmission();
    //TODO4 add check to MC and BMP
    AccTime = micros();
    GyroTime = micros();
  }
  
  void ComSample(){
    Wire.beginTransmission(ComMC);
    //X
    double x = 0;
    Wire.send(0x05);
    x += Wire.receive();
    Wire.send(0x06);;
    x = Wire.receive() + x*1024;
    double y = 0;
    Wire.send(0x07);
    y += Wire.receive();
    Wire.send(0x08);;
    y = Wire.receive() + y*1024;
    heading += 0.95*heading + 0.05*atan((float)x/y);
    Wire.endTransmission();
  }
  void GyroSample(){
    long Time = micros();
    long dt = Time - GyroTime;
    Wire.beginTransmission(GyroL3);
    uint8 out[] ={};
    Wire.send(0x2C);
    delay(1);
    out[0] = Wire.receive();
    Wire.send(0x2D);
    delay(1);
    out[1] =  Wire.receive();
    long ddz = out[0]*1024 + out[1];
    heading += ddz*(2*PI/365)*Time;
    Wire.endTransmission();
  }
  void AccSample(){
    long Time = micros();
    long dt = Time - AccTime;
    Wire.beginTransmission(AccAD);
    uint8 out[] = {};
    //begin reading from X axis.
    Wire.send(0x32);
    delay(1);
    out[0] =  Wire.receive();
    Wire.send(0x33);
    delay(1);
    out[1] = Wire.receive();
    double ddx = out[0]*1024 + out[1];
    dxt += ddx*dt*dt;
    //Y
    Wire.send(0x34);
    delay(1);
    out[2] = Wire.receive();
    Wire.send(0x35);
    delay(1);
    out[3] = Wire.receive();
    double ddy = out[2]*1024 + out[3];
    dyt += ddy*dt*dt;
    
    //Z
    Wire.send(0x36);
    delay(1);
    out[4] = Wire.receive();
    Wire.send(0x37);
    delay(1);
    out[5] = Wire.receive();
    AccTime = Time;
  }
  double readData(){
    return dxt;//TODO return 
  }
};



// Gryo Reader V1.0
HardwareSPI spi(1);
class FancyGyro{
public:
  double dx;
  double dy;
  double heading;
  uint8 writeBuf[4];
  uint8 readBuf[4];
  FancyGyro(){
    dx =0;
    dy = 0;
    heading =0;
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);
    spi.begin(SPI_4_5MHZ, MSBFIRST, SPI_MODE_0);
  }
  // TODO3 Understand Gyro output and oop data.
  void sample(){
    writeBuf[0] = 0x20;
    writeBuf[1] = 0x00;
    writeBuf[2] = 0x00;
    writeBuf[3] = 0x00;

    digitalWrite(9, LOW);
    delay(1);

    readBuf[0] = spi.transfer(writeBuf[0]);
    delay(1);
    //SerialUSB.println(readBuf[0]);
    readBuf[1] = spi.transfer(writeBuf[1]);
    delay(1);
    //SerialUSB.println(readBuf[1]);
    readBuf[2] = spi.transfer(writeBuf[2]);
    delay(1);
    //SerialUSB.println(readBuf[2]);
    readBuf[3] = spi.transfer(writeBuf[3]);
    delay(1);
    //SerialUSB.println(readBuf[3]);
    digitalWrite(9, HIGH);

    uint8 test = readBuf[0] & 0b00001100;
    if (test == 0b00000100) {
      uint16 temp0 = (uint16) readBuf[0];
      uint16 temp1 = (uint16) readBuf[1];
      uint16 unsignedData = (readBuf[2] >> 2);
      unsignedData += (temp1 << 6);
      unsignedData += (temp0 << 14);
      int16 signedData = (int16) unsignedData;
      SerialUSB.println(signedData);
    } 
    else {
      // not sensor data; could be a R/W error message
      // TODO2 add gyro Misconfig Error
    }
  }
};



// Motor Controller V1.0
class Motor {
  //Motor Control
public:
  uint8 pwmPin;
  uint8 dirPin;
  uint8 gndPin;
  uint8 encoder1Pin;
  uint8 encoder2Pin;
  boolean encoder;
  volatile unsigned int count;
  
  Motor(uint8 _gndPin, uint8 _pwmPin, uint8 _dirPin, uint8 _encoder1Pin, uint8 _encoder2Pin) : 
  gndPin(_gndPin), pwmPin(_pwmPin), dirPin(_dirPin), encoder1Pin(_encoder1Pin), encoder2Pin(_encoder2Pin){
    pinMode(gndPin, OUTPUT);
    pinMode(pwmPin, PWM );
    pinMode(dirPin, OUTPUT);
    pinMode(encoder1Pin, INPUT);
    pinMode(encoder2Pin, INPUT);
    digitalWrite(gndPin,LOW);
    encoder = true;
    int8 dir = 0;
    count =0;
    set(dir);
  }
  void sample() {
    if ( digitalRead(encoder2Pin)==HIGH )
      count++;
    else
      count--;
  }
  Motor(uint8 _gndPin, uint8 _pwmPin, uint8 _dirPin) : 
  gndPin(_gndPin), pwmPin(_pwmPin), dirPin(_dirPin){
    pinMode(gndPin, OUTPUT);
    pinMode(pwmPin, PWM);
    pinMode(dirPin, OUTPUT);
    digitalWrite(gndPin,LOW);
    encoder = false;
    int8 dir = 0;
    set(dir);
  }
  void set(int8 dir){
    uint16 dirMag = dir > 0 ? dir : -dir;
    uint16 pwm = (dirMag == 128) ? 65535 : dirMag << 9;
    SerialUSB.print(pwmPin);
    SerialUSB.print("||");
    SerialUSB.println(dir);
    digitalWrite(dirPin,(dir>0));//TODO1 Critical Assumption:  Byte division behavior need to be checked.
    pwmWrite(pwmPin,pwm);
  }
  int readData(){
    if (encoder)
      return count;
    else
      return 0;
    
  }
};


// Ultrasonic Range  Finder V1.0
class Ultra {
public:
  uint8 trig;
  uint8 echo;
  volatile unsigned int start;
  volatile unsigned int endx;
  uint8 data;
  Ultra(uint8 _trig, uint8 _echo) : 
  trig(_trig), echo(_echo) {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    start = 0;
    endx = 0;
    data = 0;
    //attachInterrupt(echo, this.sample, CHANGE);
  }

  void sample() {
    if ( digitalRead(echo)==HIGH ) {
      start = micros();
    } 
    else {
      endx = micros();      
      //25% EWMA filter
      float diff =  (endx - start)*0.034029;//in cm
      data = (uint8)(0.95*data + 0.05*diff);
    }
  }
  byte readData(){
    return data;
  }
};



//Main LOOP

Ultra ultra1 = Ultra(24,23);
Ultra ultra2 = Ultra(26,25);
Ultra ultra3 = Ultra(28,27);
Ultra ultra4 = Ultra(30,29);
Ultra ultra5 = Ultra(32,31);
Ultra ultra6 = Ultra(34,33);
Ultra ultra7 = Ultra(36,35);
Ultra sonars[] = {ultra1,ultra2,ultra3,ultra4,ultra5,ultra6,ultra7};
FancyGyro gyro = FancyGyro();
Motor motorL = Motor(4,3,2);
Motor motorR = Motor(7,6,5);
void ultra1ISR(){
  ultra1.sample();
}
void ultra2ISR(){
  ultra2.sample();
}
void ultra3ISR(){
  ultra3.sample();
}
void ultra4ISR(){
  ultra4.sample();
}
void ultra5ISR(){
  ultra5.sample();
}
void ultra6ISR(){
  ultra6.sample();
}
void ultra7ISR(){
  ultra7.sample();
}
int charCount;
byte state;
void setup() {
  noInterrupts();
  attachInterrupt(ultra1.echo, ultra1ISR, CHANGE);
  attachInterrupt(ultra2.echo, ultra2ISR, CHANGE);
  attachInterrupt(ultra3.echo, ultra3ISR, CHANGE);
  attachInterrupt(ultra4.echo, ultra4ISR, CHANGE);
  attachInterrupt(ultra5.echo, ultra5ISR, CHANGE);
  attachInterrupt(ultra6.echo, ultra6ISR, CHANGE);
  attachInterrupt(ultra7.echo, ultra7ISR, CHANGE);
  interrupts();
  //For Motor
  charCount = 0;
  state = 0x00;
}

char buf[4];
//sensory states
int8 sNum = 0; //sonar number
uint32 sTime = micros(); //sonar time;

void loop() {
  //Serial Communications
  if(SerialUSB.available()) {
    char ch = SerialUSB.read();
    SerialUSB.print(ch);
    SerialUSB.println(state);
    switch(state){
      case 0x00: //In Main
        if(ch == 'A') {//Motor Initializer
          SerialUSB.println("ToMotor!");
          state = 0x01;
          charCount=1;
          buf[0] = 'A';
        }
        break;
      case 0x01: //In Motor
        buf[charCount % 4] = ch;
        charCount++;
        if (charCount == 4) {
          if (ch == 'E') {
            motorL.set(buf[1]); 
            motorR.set(buf[2]);
          }
          charCount = 0;
          state = 0x00;
        }
        break;
    }
  }
  
  uint32 cTime = micros();
  //Internal Sensor Updates
  
  //Sonar
  if(sTime-cTime > 20){
    uint8 trigPin = sonars[sNum%7].trig;
    digitalWrite(trigPin,HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin,LOW);
    sNum++;
    sTime = cTime;
  }
  
  //Gyro
  
  
  //Acc
  
  
  
  
  
}



