#include <Wire.h>
#include <math.h>
/******************ULTRA SOUND************************/
//                  d_ultrasond[2]                   
//                         *
//      d_ultrasond[1] *   *   * d_ultrasond[3]
//                      *  *  *
//                       * * *
// d_ultrasond[0] ********* *********d_ultrasond[4]
/******************ULTRA SOUND*******************/
const int trig[5]={2,4,6,8,10};
const int echo[5]={3,5,7,9,11};
int i;

double d_ultrasond[5] = {0.0};
double module_obstacle={0.0};
double angle_obstacle;
double U_ax,U_ay;


void setup() 
{
 Wire.begin(8);                // join i2c bus with address #8
 Wire.onRequest(requestEvent); // register event
for(i=0;i<5;i++){
                  pinMode(trig[i],OUTPUT);
                  pinMode(echo[i],INPUT);
                  }
  
Serial.begin(115200);
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
   
void requestEvent() 
{
  // as expected by master
   Wire.write('d');
   delay(100);
//   Wire.write((int)module_obstacle);
//   Wire.write('\0');
//   
//   Wire.write('o');
//   Wire.write((int)(angle_obstacle/0.0174533));
//   Wire.write('\0');
}
