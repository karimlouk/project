#include <Wire.h>
#include <math.h>
#include <MPU6050.h>
/******************ULTRA SOUND************************/
//                  d_ultrasond[2]                   
//                         *
//      d_ultrasond[1] *   *   * d_ultrasond[3]
//                      *  *  *
//                       * * *
// d_ultrasond[0] ********* *********d_ultrasond[4]
/******************ULTRA SOUND*******************/
const int trig[5]={12,4,6,8,10};
const int echo[5]={13,5,7,9,11};
int i;
double d_ultrasond[5] = {0.0};
double module_obstacle={0.0};
double angle_obstacle;
double U_ax,U_ay;

void setup() 
{
  
  Serial.begin(115200); 
  attachInterrupt(2,interruption,RISING);
for(i=0;i<5;i++){
                  pinMode(trig[i],OUTPUT);
                  pinMode(echo[i],INPUT);
                  } 
}


void loop()
{ 
 for(i=0;i<5;i++)
  {
    d_ultrasond[i]=ultraSound(i);
    if((d_ultrasond[i]>30)||(d_ultrasond[i]==0))d_ultrasond[i]=0;
    d_ultrasond[i]=d_ultrasond[i]*10;
  }
  
  U_ax=0.7071*d_ultrasond[1]+d_ultrasond[2]+0.7071*d_ultrasond[3];
  U_ay=d_ultrasond[0]+0.7071*d_ultrasond[1]-0.7071*d_ultrasond[3]-d_ultrasond[4];
  module_obstacle=sqrt(U_ax*U_ax+U_ay*U_ay);
  angle_obstacle=atan2(U_ay,U_ax);
}

void interruption(){
  
  
  Serial.print('d');
  Serial.print(module_obstacle);
  Serial.print('\0');

  Serial.print('o');
  Serial.print(angle_obstacle/0.0174444444);
  Serial.print('\0');
}

double ultraSound(int n)
   {
    float d;
    float duration = 0;
  digitalWrite(trig[n] , HIGH);
  delayMicroseconds(200);
  digitalWrite(trig[n], LOW);
  duration = pulseIn(echo[n], HIGH,4000);
  d=(duration/2)/ 28.5 ;
return ((double)d);
   }
