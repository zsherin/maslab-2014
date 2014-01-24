  
boolean turning;
class Motor{
  private:
  double power;
  public:
  int InAPin;
  int InBPin;
  int PWMPin;
  Motor(int _InAPin, int _InBPin, int _PWMPin){
    power = 0;
    InAPin = _InAPin;
    InBPin = _InBPin;
    PWMPin = _PWMPin;
    pinMode(InAPin, OUTPUT);
    pinMode(InBPin, OUTPUT);
    pinMode(PWMPin, OUTPUT);
  }
  void set(double input){
    power = input;
    if(power >0){
      digitalWrite(InAPin, HIGH);
      digitalWrite(InBPin, LOW);
    }else{
      digitalWrite(InAPin, LOW);
      digitalWrite(InBPin, HIGH);
    }
    
    analogWrite(PWMPin,abs(power));
  }
};
Motor mL = Motor(6,7,5);
Motor mR = Motor(2,4,3);
void setup(){
  //IR
  pinMode(A1, INPUT);
  pinMode(A0, INPUT);
}

void loop(){
  int dist = analogRead(A1) + analogRead(A0);
  
  if(dist < 1200)
  {
    mL.set(210);
    mR.set(210);
  }
  else
  {
    mL.set(210);
    mR.set(-210);
    delay(10);
    mL.set(0);
    mR.set(0);
  }
  
  
  
}
