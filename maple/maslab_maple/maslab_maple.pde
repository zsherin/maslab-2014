
/*
MAPLE Microcontroller Code in Processing


TODO:
1 Byte math check.
2 Gyro misConfig Error.
3 Gyro OOP Data.

Completed:
Componenet      OOP    SensorFilter   String(output)
Gyro            [x]         []              []
Motor/Encoder   [x]       [N/A]             []
Sonar           [x]       [EMWA]              []

Author:  MASLAB2014 TeamSeven @2014
*/

/*
OOP specs

Object.sample:  sample & store 1 data sample,  usually filter is in this.

Object.readData:  for the main loop to grab the data.

Actuator.set:  to set the actuator in a specific state{Motor: voltage}
*/



#define LED_PIN BOARD_LED_PIN



// Gryo Reader V1.0
HardwareSPI spi(1);
class Gyro{
public:
  uint8 writeBuf[4];
  uint8 readBuf[4];
  Gyro(){
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
}



// Motor Controller V1.0
class Motor {
  //Motor Control
public:
  uint8 pwmPin;
  uint8 dirPin;
  uint8 encoder1Pin;
  uint8 encoder2Pin;
  byte dir;
  boolean encoder = false;
  volatile unsigned int count = 0;
  
  void sample() {
    if ( digitalRead(encoder2Pin)==HIGH )
      count++;
    else
      count--;
  }
  
  Motor(uint8 _pwmPin, uint8 _dirPin, uint8 _encoder1Pin, uint8 _encoder2Pin) : 
  pwmPin(_pwmPin), dirPin(_dirPin, encoder1Pin(_encoder1Pin), encoder2Pin(_encoder2Pin)){
    pinMode(pwnPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(encoder1Pin, INPUT);
    pinMode(encoder2Pin, INPUT);
    attachInterrupt(encoder1Pin,Motor:sample,RISING);
    encoder = true;
    dir = 0;
    Motor::set(dir);
  }
  Motor(uint8 _pwmPin, uint8 _dirPin) : 
  pwmPin(_pwmPin), dirPin(_dirPin){
    pinMode(pwnPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    dir = 0;
    Motor::set(dir);
  }
  void set(byte _dir): 
  dir(_dir){
    digitalWrite(dirPin,dir/abs(dir));//TODO1 Critical Assumption:  Byte division behavior need to be checked.
    pwmWrite(pwmPin,abs(dir));
  }
  int readData(){
    if (encoder)
      return count;
    else
      return null;
    
  }
}


// Ultrasonic Range  Finder V1.0
class Ultra {
public:
  uint8 trig;
  uint8 echo;
  volatile unsigned int start;
  volatile unsigned int endx;
  unsigned int data = 0;
  Ultra(uint8 _trig, uint8 _echo) : 
  trig(_trig), echo(_echo) {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    start = 0;
    endx = 0;
    attachInterrupt(echo, Ultra::sample, CHANGE);
  }

  void sample() {
    if ( digitalRead(echo)==HIGH ) {
      start = micros();
    } 
    else {
      endx = micros();      
      //25% EWMA filter
      int diff =  endx - start;
      data = 0.75*data + 0.25*diff;
    }
  }
  unsigned int readData(){
    return data;
  }
}



//Main LOOP

Ultra ultra;

void setup() {
  noInterrupts();

  // Set up the built-in LED pin as an output:
  pinMode(24, PWM);
  ultra = Ultra(30, 28);

  interrupts();
}

void loop() {
  digitalWrite(30,HIGH);
  delayMicroseconds(10);
  digitalWrite(30,LOW);
  delay(60);

  unsigned int range = ultra.readData();
  pwmWrite(24, range*3/2);
  SerialUSB.println(range);
}



