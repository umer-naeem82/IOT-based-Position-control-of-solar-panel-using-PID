// This alternate version of the code does not require
// atomic.h. Instead, interrupts() and noInterrupts() 
// are used. Please use this code if your 
// platform does not support ATOMIC_BLOCK.

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 5
#define IN2 6
#define IN1 7
#include <SoftwareSerial.h> 
SoftwareSerial s(8,4); //RX,TX

volatile int temp, counter = 0;
volatile int posi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
long prevT = 0;
float eprev = 0;
float eintegral = 0;
String readString;
int User_Input = 0;
float target = 0;
float pp=0;

void setup() {
  Serial.begin(9600);
  s.begin(9600);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
 // attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  attachInterrupt(0, ai0, RISING);
  attachInterrupt(1, ai1, RISING);
  
}

void loop() {
  while(s.available ()>0)
    {
      target=(s.read())*9.4;
      pp=target;
    }
  //target=target*2.83;
  Serial.print(target);
  //target=-target*3.3334;


  // set target position

  //int target = 250*sin(prevT/1e6);

  // PID constants
  float kp = 4.0;
  float kd = 0.02;
  float ki = 0.003;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position
  int pos = 0; 
  noInterrupts(); // disable interrupts temporarily while reading
  pos = posi;
  interrupts(); // turn interrupts back on
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);


  // store previous error
  eprev = e;


  //Serial.print(target);
  Serial.print(" ");
  //Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void ai0() {
  if (digitalRead(3) == LOW) {
    counter--;  // Reverse the increment and decrement operations
  } else {
    counter++;
  }
}

void ai1() {
  if (digitalRead(2) == LOW) {
    counter++;
    posi=counter;
  } else {
    counter--;  // Reverse the increment and decrement operations
    posi=counter;
  }
}