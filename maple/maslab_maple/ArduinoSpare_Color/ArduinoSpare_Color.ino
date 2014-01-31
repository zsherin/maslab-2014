#include <avr/pgmspace.h>
#include <Servo.h>
#define DATA_1 (PORTC |=  0X01)    // DATA 1    // for UNO
#define DATA_0 (PORTC &=  0XFE)    // DATA 0    // for UNO
#define STRIP_PINOUT (DDRC=0xFF)    // for UNO
int s0=3,s1=4,s2=5,s3=6;
int offsetR=0;
int offsetG=0;
int flag=0;
int counter=0;
int countR=0,countG=0,countB=0;
Servo sortServo;
PROGMEM const unsigned long pattern_test_rainbow[10][10]={
  {0xff0000,0xff7f00,0xffff00,0x00ff00,0x0000ff,0x6f00ff,0x8f00ff,0x000000,0x000000,0x000000},
  {0x000000,0xff0000,0xff7f00,0xffff00,0x00ff00,0x0000ff,0x6f00ff,0x8f00ff,0x000000,0x000000},
  {0x000000,0x000000,0xff0000,0xff7f00,0xffff00,0x00ff00,0x0000ff,0x6f00ff,0x8f00ff,0x000000},
  {0x000000,0x000000,0x000000,0xff0000,0xff7f00,0xffff00,0x00ff00,0x0000ff,0x6f00ff,0x8f00ff},
  {0x8f00ff,0x000000,0x000000,0x000000,0xff0000,0xff7f00,0xffff00,0x00ff00,0x0000ff,0x6f00ff},
  {0x6f00ff,0x8f00ff,0x000000,0x000000,0x000000,0xff0000,0xff7f00,0xffff00,0x00ff00,0x0000ff},
  {0x0000ff,0x6f00ff,0x8f00ff,0x000000,0x000000,0x000000,0xff0000,0xff7f00,0xffff00,0x00ff00},
  {0x00ff00,0x0000ff,0x6f00ff,0x8f00ff,0x000000,0x000000,0x000000,0xff0000,0xff7f00,0xffff00},
  {0xffff00,0x00ff00,0x0000ff,0x6f00ff,0x8f00ff,0x000000,0x000000,0x000000,0xff0000,0xff7f00},
  {0xff7f00,0xffff00,0x00ff00,0x0000ff,0x6f00ff,0x8f00ff,0x000000,0x000000,0x000000,0xff0000},
};
void setup() {                
  STRIP_PINOUT;        // set output pin - DEBUG: should auto detect which mother board for use

  reset_strip();
  pinMode(s0,OUTPUT);
  pinMode(s1,OUTPUT); 
  pinMode(s2,OUTPUT);
  pinMode(s3,OUTPUT);
  sortServo.attach(9);
  Serial.begin(9600);
  //noInterrupts();
  TCS();
  delay(2000);
  offsetR = countR;
  offsetG = countG;
}

void TCS()
{
  digitalWrite(s1,HIGH);
  digitalWrite(s0,HIGH);
  flag=0;
  attachInterrupt(0, ISR_INTO, CHANGE);
  timer2_init();
}
void ISR_INTO()
{
  counter++;
}
void timer2_init(void)
{
  TCCR2A=0x00;
  TCCR2B=0x07; //the clock frequency source 1024 points
  TCNT2= 100;    //10 ms overflow again
  TIMSK2 = 0x01; //allow interrupt
}
int i=0;
ISR(TIMER2_OVF_vect)//the timer 2, 10ms interrupt overflow again. Internal overflow interrupt executive function
{
TCNT2=100;
flag++;
if(flag==1)
 {
   counter=0;
 }
else if(flag==2)
  {
   digitalWrite(s2,LOW);
   digitalWrite(s3,LOW); 
   countR=counter/1.051-offsetR;
   Serial.print("red=");
   Serial.println(countR,DEC);
   digitalWrite(s2,HIGH);
   digitalWrite(s3,HIGH);   
  }
else if(flag==3)
   {
    countG=counter/1.0157-offsetG;
   Serial.print("green=");
   Serial.println(countG,DEC);
    digitalWrite(s2,LOW);
    digitalWrite(s3,HIGH); 
    }
else if(flag==4)
   {
    countB=counter/1.114;
   //Serial.print("blue=");
   //Serial.println(countB,DEC);
    digitalWrite(s2,LOW);
    digitalWrite(s3,LOW);
    }
else
    {
    flag=0; 
     TIMSK2 = 0x00;
    }
    counter=0;
    delay(2);
}
void loop()
{
  sortServo.write(90);
  send_1M_pattern(pattern_test_rainbow, 10, 70);
  TCS();
  if((countR>100)||(countG>40))
  {
     if(countG>=40){//GreenID
        Serial.println("Green");
        Serial.println("");
        sortServo.write(150);
        delay(3000);
      }else if(countR>=100){//RedID
        Serial.println("Red");
        Serial.println("");
        sortServo.write(0);
        delay(3000);
     }
   }
  else 
  {
    delay(300);       
  }
}


void send_1M_pattern(const unsigned long data[][10], int pattern_no, int frame_rate)
{
  int i=0;
  int j=0;
  uint32_t temp_data;

  for (i=0;i<pattern_no;i++)
  {
    noInterrupts();
    for (j=0;j<10;j++)
    {
      temp_data=pgm_read_dword_near(&data[i][j]);
      send_strip(temp_data);
    }
    interrupts();

    delay(frame_rate);

  }
}
  
void send_strip(uint32_t data)
{
  int i;
  unsigned long j=0x800000;
  
 
  for (i=0;i<24;i++)
  {
    if (data & j)
    {
      DATA_1;
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");    
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      
/*----------------------------*/
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");  
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");  
      __asm__("nop\n\t");  
      __asm__("nop\n\t");        
/*----------------------------*/      
      DATA_0;
    }
    else
    {
      DATA_1;
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");    
      DATA_0;
/*----------------------------*/      
       __asm__("nop\n\t");
      __asm__("nop\n\t");
      __asm__("nop\n\t");      
/*----------------------------*/         
    }

    j>>=1;
  }


  
}
void	reset_strip()
{
  DATA_0;
  delayMicroseconds(20);
}
