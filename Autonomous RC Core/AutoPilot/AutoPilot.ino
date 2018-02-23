#include<IntervalTimer.h>

unsigned int pwm_centre_throttle = 9830;  //  15% duty cycle - corresponds to zero velocity, zero steering
unsigned int pwm_forwardMin = 10090;   //  15.38% duty cycle - correspond to minimum moving forward velocity
unsigned int pwm_backwardMin = 9503;   //  14.52% duty cycle - correspond to minimum moving backward velocity
unsigned int pwm_backwardMax = 6554;    //  10% duty cycle - corresponds to max reverse velocity
unsigned int pwm_forwardMax = 13107;   //  20% duty cycle - corresponds to max forward velocity

unsigned int pwm_center_steer = 10350; //  Calibrated center steering value in PWM
unsigned int pwm_upperlimit_steer=13107;
unsigned int pwm_lowerlimit_steer=6554;

unsigned int pwm_steer = pwm_center_steer;

volatile float errorRPM=0, errorRPMPrevious=0, errorRPMIntegral=0;

volatile unsigned int pwm_throttle=pwm_centre_throttle;
volatile float targetSpeed;

unsigned short speedPin = 3;             //  pin 3 is for detecting speed
unsigned short LEDPin = 2;
float mass = 4.5;             //  car's mass is 4.5 kg
unsigned int i = 10100;
bool estopState = 0;

bool ledState=1;
unsigned short count=1;

float speed=0;
float wheelRadius=0.089;//m

long unsigned int revCount=0;
unsigned long int revPrevious=0;
unsigned long int revCurrent=micros();

bool armed=0;

long int controlInterval=10000;
IntervalTimer controlTimer;

void speedISR();
void controlLoop();
float throttlePID();

float position[] ={0,0,0};

volatile float Kp=10;
volatile float Ki=1;
volatile float Kd=1;

bool tuningMode=false;

void setup() {
  Serial.begin(115200);
  // Need to produce PWM signals so we need to setup the PWM registers. This setup happens next.
  analogWriteFrequency(5, 100);     //  freq at which PWM signals is generated at pin 5.
  analogWriteFrequency(6, 100); 
  analogWriteResolution(16);        // Resolution for the PWM signal
  
  analogWrite(5,pwm_center_steer);  // Setup zero velocity and steering.
  analogWrite(6,pwm_centre_throttle);
  
  pinMode(13,OUTPUT);               // Teensy's onboard LED pin. 
  pinMode(LEDPin,OUTPUT);
  
  digitalWrite(13,HIGH);            
  delay(500);
  digitalWrite(13,LOW);            
  delay(100);
  digitalWrite(13,HIGH);            
  delay(500);
  digitalWrite(13,LOW);            
  delay(100);
  digitalWrite(13,HIGH);           
  delay(500);
  digitalWrite(13,LOW);            
  delay(100);
  //Startup blinking.
  
  pinMode(speedPin, INPUT_PULLUP);  // Setup pin that read speed in RPS
  attachInterrupt(speedPin, speedISR, RISING);

  controlTimer.priority(0);
  controlTimer.begin(controlLoop, controlInterval);
}

void estop(){
  analogWrite(5,pwm_center_steer);  // Setup zero velocity and steering.
  analogWrite(6,pwm_centre_throttle);

  targetSpeed=0;

  Serial.println("Emergency Stop!!");
}

void controlLoop(){
  if(armed){
    noInterrupts();
    if(count%100==0){
      digitalWrite(13,ledState);    
      count=0;
      ledState=!ledState;
    }
    
    pwm_throttle=mapSignal(throttlePID(), pwm_forwardMin, pwm_forwardMax, pwm_centre_throttle, pwm_backwardMin, pwm_backwardMax);
    interrupts();
  
    speed=1000000*(2*(3.14159265359/3)*wheelRadius)/(revCurrent-revPrevious);
    speed=speed*(speed>=0.18);
  
    if((micros()-revCurrent)>(revCurrent-revPrevious)){
      speed=0; 
    }
    errorRPMPrevious=errorRPM;
    errorRPM=targetSpeed-speed;
    errorRPMIntegral+=errorRPMPrevious*0.01;
    count++;
    estopState=0;
    if(tuningMode){
      Serial.println("Heart Beat: "+String(micros())+", Speed: "+String(speed)+", Desired Speed: "+String(targetSpeed)+", Error: "+String(errorRPM));
    }
    else{
      Serial.println("Heart Beat: "+String(micros())+", Speed: "+String(speed));
    }
    analogWrite(5,pwm_steer);  // Setup zero velocity and steering.
    analogWrite(6,pwm_throttle);
    
    if(estopState){
      estop();
      estopState=0;
    }
  }
}

void speedISR(){
  revPrevious=revCurrent;
  revCurrent=micros();
}

int mapSignal(int sig, int minForward, int maxForward, int centre, int minBackward, int maxBackward){
  if (sig > 0.0){
      return (int)map(sig, 0, 100, minForward, maxForward);
    }
    else if(sig < 0.0){
      return (int)map(sig, -100, 0, maxBackward, minBackward);
    }
    else{
      return pwm_centre_throttle;
    }
}

float throttlePID(){
  
  return Kp*errorRPM + Kd*0.01*(errorRPM-errorRPMPrevious)+ Ki*errorRPMIntegral ;
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()){
    String header=Serial.readString();

    int PWM;
    bool Estop;
    
    if(header.equals("Are you the autopilot?")){  //Possible 
      Serial.println("Yes");
    }
    if(header.equals("Arm")){  //Possible 
      armed=1;
    }
    if(header.equals("Disarm")){
      armed=0;
      analogWrite(5,pwm_center_steer);  // Setup zero velocity and steering.
      analogWrite(6,pwm_centre_throttle);
      digitalWrite(13,0);  
      Serial.flush();  
    }
    else{
      String element;
      while(!element.equals(header)){
        element=header.substring(0, header.indexOf(','));
        header=header.substring(header.indexOf(',')+1);
        if(element.substring(0,element.indexOf(':')).equals("T")){
          targetSpeed=element.substring(element.indexOf(':')+1).toFloat();
        }
        if(element.substring(0,element.indexOf(':')).equals("S")){
          PWM=element.substring(element.indexOf(':')+1).toFloat();
        }
        if(element.substring(0,element.indexOf(':')).equals("E")){
          Estop=element.substring(element.indexOf(':')+1).toFloat()==1;
        }
        if(element.substring(0,element.indexOf(':')).equals("Kp")){
          Kp=element.substring(element.indexOf(':')+1).toFloat();
        }
        if(element.substring(0,element.indexOf(':')).equals("Ki")){
          Ki=element.substring(element.indexOf(':')+1).toFloat();
        }
        if(element.substring(0,element.indexOf(':')).equals("Kd")){
          Kd=element.substring(element.indexOf(':')+1).toFloat();
        }
        if(element.substring(0,element.indexOf(':')).equals("TM")){
          Serial.println(element.substring(element.indexOf(':')+1));
          tuningMode=element.substring(element.indexOf(':')+1)=="1";
        }
      }
  
      pwm_steer=mapSignal(PWM, pwm_center_steer, pwm_upperlimit_steer, pwm_center_steer, pwm_center_steer, pwm_lowerlimit_steer);
      estopState=Estop;
    }
  }
}
