  

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
  }
}
Motor mL;
Motor mR;
void setup(){
  mL = Motor();
  mR = Motor();
  //IR
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
}

void loop(){
  int sL = analogRead(A1);
  int sR = analogRead(A2);
  
}
