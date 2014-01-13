#include <Servo.h>

// TODO:
// [X] Serial Comm
// [ ] Dagu Motor Controller
// [X] Cytron Motor Controller
// [X] Encoders

// [X] Servo
// [X] HC-SR04 Sonic Range Finder
// [X] Analog read
// [X] Analog write
// [X] Digital read
// [X] Digital write

#define LED_PIN BOARD_LED_PIN

/*
// Serial test
 void setup() {
 pinMode(LED_PIN, OUTPUT);
 }
 
 void loop() {
 delay(100);
 while (SerialUSB.available()) {
 char input = SerialUSB.read();
 SerialUSB.println(input);
 toggleLED();
 }
 }
 */



/*
volatile unsigned int count = 0;
 
 void irh() {
 if ( digitalRead(28)==HIGH )
 count++;
 else
 count--;
 }
 
 void setup() {
 noInterrupts();
 
 // Set up the built-in LED pin as an output:
 pinMode(24, PWM);
 pinMode(26, INPUT);
 pinMode(28, INPUT);
 
 attachInterrupt(26,irh,RISING);
 interrupts();
 }
 
 void loop() {
 pwmWrite(24,count*10);
 }
 */
// Gryo Reader
HardwareSPI spi(1);
class Gryo{
public:
  uint8 writeBuf[4];
  uint8 readBuf[4];
  Gryo(){
    pinMode(9, OUTPUT);
    digitalWrite(9, HIGH);
    spi.begin(SPI_4_5MHZ, MSBFIRST, SPI_MODE_0);
  }
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
    }
  }


}



// Motor Controller


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
  
  void irh() {
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
    attachInterrupt(encoder1Pin,Motor:irh,RISING);
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
    digitalWrite(dirPin,dir/abs(dir));//TODO Critical Assumption:  Byte division behavior need to be checked.
    pwmWrite(pwmPin,abs(dir));
  }
  int sample(){
    if (encoder)
      return count;
    else
      return null;
  }
}

// Sonic Range Finder

class Ultra {
public:
  uint8 trig;
  uint8 echo;
  volatile unsigned int start;
  volatile unsigned int endx;

  Ultra(uint8 _trig, uint8 _echo) : 
  trig(_trig), echo(_echo) {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    start = 0;
    endx = 0;
    attachInterrupt(echo, Ultra::irh, CHANGE);
  }

  void irh() {
    if ( digitalRead(echo)==HIGH ) {
      start = micros();
    } 
    else {
      endx = micros();
    }
  }

  unsigned int diff() {
    return endx - start;
  }
}

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

  unsigned int diff = ultra.diff();
  pwmWrite(24, diff*3/2);
  SerialUSB.println(diff);
}


/*
// Servo
 Servo servo;
 
 void setup() {
 servo.attach(1);
 
 // Set up the built-in LED pin as an output:
 //pinMode(24, PWM);
 }
 
 int i = 0;
 void loop() {
 //pwmWrite(24,i);
 servo.write(i);
 delay(1000);
 i += 5;
 if ( i > 180 ) i = 0;
 }
 */

/*
// Analog Write
 void setup() {
 // Set up the built-in LED pin as an output:
 pinMode(24, PWM);
 }
 
 int i = 0;
 void loop() {
 pwmWrite(24,i*100);
 delay(10);
 i += 2;
 }
 */

/*
// Analog Read
 // Requires port labeled with AIN (from what I can tell)
 void setup() {
 pinMode(2, INPUT_ANALOG);
 }
 
 int i = 0;
 void loop() {
 uint16 val = analogRead(2);
 SerialUSB.print((char) (val >> 8));
 delay(100);
 }
 */

/*
// Digital Write
 void setup() {
 // Set up the built-in LED pin as an output:
 pinMode(24, OUTPUT);
 }
 
 int i = 0;
 void loop() {
 digitalWrite(24,HIGH);
 delay(500);
 digitalWrite(24,LOW);
 delay(500);
 }
 */

/*
// Digital Read
 void setup() {
 // Set up the built-in LED pin as an output:
 pinMode(24, OUTPUT);
 pinMode(23, INPUT);
 }
 
 int i = 0;
 void loop() {
 int val = digitalRead(23);
 digitalWrite(24,val);
 delay(50);
 }
 */

