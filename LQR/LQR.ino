#include <ESP32Encoder.h>
ESP32Encoder MotoEncoder;
ESP32Encoder PendEncoder;
/* create a hardware timer */
hw_timer_t * timer = NULL;

int freq = 2000;
int channel = 0;
int resolution = 8;
boolean toggled = false;

volatile boolean flag = false;
int pulPin = 17;
int dirPin = 16;
int enPin = 4;
int LSW = 14;
int RSW = 15;
float cart = 0.0;
uint32_t u = 0;
boolean ON = 0;
boolean zacal = true;
boolean mid = false;
int pulls = 0;


int direct = 1;
float kc1 = 29.514661;
float kc2 = 4.560546;
float kc3 = -2.681323*1.9;
float kc4 = -3.282847;

float dT = 0.008;
float av1 = 0.0;
float av2 = 0.0;
float av3 = 0.0;
float av4 = 0.0;

float a = 0.0;
float vk = 0.0;
float vk1 = 0.0;
float uk = 0.0;
float uk1 = 0.0;

float y = 0.0;
float yk = 0.0;
float Theta = 0.0;
float vr = 0.0;
float cartk = 0.0;

void IRAM_ATTR onTimer(){
  flag = !flag;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(20);
  Serial.setTimeout(5);
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);
  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &onTimer, true);
  /* Set alarm to call onTimer function every second 1 tick is 1us
  => 1 second is 8000us */
  /* Repeat the alarm (third parameter) */
  timerAlarmWrite(timer, 8000, true);
  /* Start an alarm */
  timerAlarmEnable(timer);

  ledcSetup(channel, freq, resolution);
  ledcAttachPin(pulPin, channel);
  
  // use pin 19 and 18 for the first encoder
  MotoEncoder.attachFullQuad(27, 26);
  // use pin 17 and 16 for the second encoder
  PendEncoder.attachFullQuad(33, 32);
  pinMode(pulPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  
  digitalWrite(pulPin, LOW);
  digitalWrite(dirPin, HIGH);
  digitalWrite(enPin, LOW);
  digitalWrite(dirPin,(direct>0?0:1));
}

void loop() {
  
    int pulls = PendEncoder.getCount();
    y = ((float)pulls+800)*2.0*3.14159/1600.0; // Pendulum angle in rad
    cart = ((float)MotoEncoder.getCount())/20/1000; // Cart velocity in m
    
    //Serial.println(String(y) + " " + String(cart));
    if(zacal){
      if(pulls <= -798 && pulls >= -802){
        ON = true;
        zacal = false;
        av1 = 0;
        av2 = 0;
        av3 = 0;
        av4 = 0;
        yk = 0;
        vk = 0;
        vk1 = 0;
      }
    }
    if(flag and ON){
    if(abs(y)<1 && (!digitalRead(LSW) && !digitalRead(RSW))){
        if(abs(y)>0.003){

        Theta = (y-yk)/dT;
        Theta = (Theta + av1 + av2 + av3 + av4)/5;
        av1 = Theta;
        av2 = av1;
        av3 = av2;
        av4 = av3;
        
        uk = - (kc1*y + kc2*Theta + kc3*cart + kc4*vk/1000);  
        
        yk = y;
        a = constrain(uk, -25, 25);
        uk1 = uk;
        vk = a*dT*1000+vk1;
        
        digitalWrite(dirPin,(vk>0?0:1));
        vk1 = vk;
        //pul = constrain(round(vk/0.025025), -32000, 32000);
        u = constrain(abs(round(vk*40)), 0, 2000000);
        //tone(pinTone, u);
        ledcWriteTone(channel, u);
        Serial.println(u);
        }
      }
     else{
      ledcWriteTone(channel, 0);
      digitalWrite(pulPin, LOW);
     }
    
    flag = 0;
  }
}
