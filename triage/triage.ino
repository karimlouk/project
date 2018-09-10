#include <Servo.h>
Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int retard=3000;
int open_tim=6000;
int somm_time=retard+open_tim;
const int pin=10;

int pos = 0;    // variable to store the servo position
int cons=0;
unsigned long Time;
unsigned long old_Time;
unsigned long req_Time;
int c=0;
const int s0 = 3;  
const int s1 = 4;  
const int s2 = 5;  
const int s3 = 6;  
const int out = 2;   
bool choose=0;
// LED pins connected to Arduino

// Variables  
int red = 0;  
int green = 0;  
int blue = 0;  
bool open_close;
    
void setup()   
{  
  Serial.begin(9600); 
  pinMode(s0, OUTPUT);  
  pinMode(s1, OUTPUT);  
  pinMode(s2, OUTPUT);  
  pinMode(s3, OUTPUT);  
  pinMode(out, INPUT);   
  pinMode(choose, OUTPUT);  
  digitalWrite(s0, HIGH);  
  digitalWrite(s1, HIGH);  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(0);
}  
    
void loop() 
{  
  choose=digitalRead(pin);
  color(); 
  Serial.print("R Intensity:");  
  Serial.print(red, DEC);  
  Serial.print(" G Intensity: ");  
  Serial.print(green, DEC);  
  Serial.print(" B Intensity : ");  
  Serial.print(blue, DEC);  
  //Serial.println();  
if (choose==1){
  if (red < blue && red < green && red < 30)
  {  
   Serial.println(" - (Red Color)");  
    triage(0); 
  }  


  else if (green < red && green < blue)  
  {  
   Serial.println(" - (Green Color)");  
    triage(1); 
  }  
  else{
  Serial.println(); 
     triage(0);  
  }
}
else {
    if (red < blue && red < green && red < 30)
  {  
   Serial.println(" - (Red Color)");
   triage(1);  
 
  }  


  else if (green < red && green < blue)  
  {  
   Serial.println(" - (Green Color)");  
  triage(0); 
  }  
  else{
  Serial.println();  
    triage(0);
  }
  }

  
 }  
    
void color()  
{    
  digitalWrite(s2, LOW);  
  digitalWrite(s3, LOW);  
  //count OUT, pRed, RED  
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s3, HIGH);  
  //count OUT, pBLUE, BLUE  
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
  digitalWrite(s2, HIGH);  
  //count OUT, pGreen, GREEN  
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);  
}
void triage(bool sensor)
{
if((sensor)&&(c==0)) {old_Time=millis();c=1;}
if((!sensor)&&(c==1))c=0;
Time=millis();

if(!c)req_Time=Time-old_Time;
Serial.println(req_Time);

if((req_Time>retard)&&(req_Time<somm_time)){myservo.write(180);}
if((req_Time>=0)&&(req_Time<retard)||(req_Time>somm_time)) myservo.write(0);
 }
