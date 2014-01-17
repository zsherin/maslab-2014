
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

//Robot Model.



// Gryo Reader V1.0
HardwareSPI spi(1);
class FancyGyro{
public:
  double heading;
  uint8 writeBuf[4];
  uint8 readBuf[4];
  uint32 lTime;
  FancyGyro(){
    heading =0;
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);
    spi.begin(SPI_4_5MHZ, MSBFIRST, SPI_MODE_0);
    lTIme = micros();
  }
  // TODO3 Understand Gyro output and oop data.
  void sample(){
    writeBuf[0] = 0x20;
    writeBuf[1] = 0x00;
    writeBuf[2] = 0x00;
    writeBuf[3] = 0x00;
    digitalWrite(9, LOW);
    for(int i =0; i<4; i++){  
        delayMicroseconds(2);
        readBuf[i] = spi.transfer(writeBuf[i]);
    }
    digitalWrite(9, HIGH);
    uint8 test = readBuf[0] & 0b00001100;
    if (test == 0b00000100) {
      uint16 temp0 = (uint16) readBuf[0];
      uint16 temp1 = (uint16) readBuf[1];
      uint16 unsignedData = (readBuf[2] >> 2);
      unsignedData += (temp1 << 6);
      unsignedData += (temp0 << 14);
      int16 signedData = (int16) unsignedData;
      uint32 time = micros();
      heading += ((float)signedData)*(time-lTime)*0.000218166156/100000; //rad/s
      lTime=time;
      SerialUSB.println(heading);
    } 
    else {
      SerialUSB.print("SensorSetupFail:");
      SerialUSB.print(readBuf[0]);
      SerialUSB.print(readBuf[1]);
      SerialUSB.print(readBuf[2]);
      SerialUSB.println(readBuf[3]);
    }
  }
};

class Locator{
  public:
  int dx;
  int dy;
  int countR;
  int countL;
  Locator(){
    dx =0;
    dy =0;
    countR;
    countL;
  }
  void update(){
    int r = motorR.readData();
    int l = motorL.readData();
    int travel = ((r - countR)+(l - countL))/2;
    countR = r;
    countL = l;
    int heading = gryo.readData();
    dx += travel*sin(heading);
    dy += travel*cos(heading); 
    
  }
}


//Motor w/Encoder Controller V1.0
//TODO add PID for Motor w/ Encoder
class MotorE{
  public:
  uint8 pwmPin;
  uint8 dirPin;
  uint8 gndPin;
  uint8 encoder1Pin;
  uint8 encoder2Pin;
  boolean encoder;
  volatile int count;
/*  int8 ddir; //desired dir
  int8 cdir; //current dir
  //PID controls
  int32 eInt; // Integral sum of the last few times.
  int8  eDx; //Previous value
  int32 ltime; //Last time PID updated.
  */
  MotorE(uint8 _gndPin, uint8 _pwmPin, uint8 _dirPin, uint8 _encoder1Pin, uint8 _encoder2Pin) : 
  gndPin(_gndPin), pwmPin(_pwmPin), dirPin(_dirPin), encoder1Pin(_encoder1Pin), encoder2Pin(_encoder2Pin){
    pinMode(gndPin, OUTPUT);
    pinMode(pwmPin, PWM );
    pinMode(dirPin, OUTPUT);
    pinMode(encoder1Pin, INPUT);
    pinMode(encoder2Pin, INPUT);
    digitalWrite(gndPin,LOW);
    encoder = true;
    count =0;
  }
  void sample() {
    if ( digitalRead(encoder2Pin)==HIGH )
      count++;
    else
      count--;
  }
  void set(int8 dir){
    uint16 dirMag = dir > 0 ? dir : -dir;
    uint16 pwm = (dirMag == 128) ? 65535 : dirMag << 9;
    digitalWrite(dirPin,(dir>0));//TODO1 Critical Assumption:  Byte division behavior need to be checked.
    pwmWrite(pwmPin,pwm);
  }
  void update(){
  }
  int readData(){
    count * 0.0166601;//in cm
  }
};

// Motor Controller V1.0
class Motor {
  //Motor Control
public:
  uint8 pwmPin;
  uint8 dirPin;
  uint8 gndPin;
  
  Motor(uint8 _gndPin, uint8 _pwmPin, uint8 _dirPin) : 
  gndPin(_gndPin), pwmPin(_pwmPin), dirPin(_dirPin){
    pinMode(gndPin, OUTPUT);
    pinMode(pwmPin, PWM);
    pinMode(dirPin, OUTPUT);
    digitalWrite(gndPin,LOW);
    int8 dir = 0;
    set(dir);
  }
  void set(int8 dir){
    uint16 dirMag = dir > 0 ? dir : -dir;
    uint16 pwm = (dirMag == 128) ? 65535 : dirMag << 9;
    digitalWrite(dirPin,(dir>0));//TODO1 Critical Assumption:  Byte division behavior need to be checked.
    pwmWrite(pwmPin,pwm);
  }
  int readData(){
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
      float diff =  (endx - start);
      data = (uint8)(0.95*data + 0.05*diff);
    }
  }
  byte readData(){
    return data*0.034029;//in cm
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
MotorE motorL = MotorE(4,3,2,18,17);
MotorE motorR = MotorE(7,6,5,20,19);
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

void motorLISR(){
  motorL.sample();
}
void motorRISR(){
  motorR.sample();
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
  attachInterrupt(motorL.encoder1Pin,motorLISR,RISING);
  attachInterrupt(motorR.encoder1Pin,motorRISR,RISING);
  interrupts();
  //For Motor
  charCount = 0;
  state = 0x00;
}

char buf[4];
//sensory states
int8 sNum = 0; //sonar number
uint32 sTime = micros(); //sonar time;
void loop(){
  gyro.sample();
  delayMicroseconds(50);
}
/*
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
  
  //Relative Localization.
  
  
  
  
  
}

*/

