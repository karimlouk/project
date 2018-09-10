#include <math.h>
/******************ULTRA SOUND************************/
//                  d_ultrasond[2]                   
//                         *
//      d_ultrasond[1] *   *   * d_ultrasond[3]
//                      *  *  *
//                       * * *
// d_ultrasond[0] ********* *********d_ultrasond[4]

const int trig[5]={4,6,8,10,12};
const int echo[5]={5,7,9,11,13};
int i;

double d_ultrasond[5] = {0.0};
double module_obstacle={0.0};
double angle_obstacle;
double U_ax,U_ay;


//*************************************************************//

void setup() 
{
  Serial.begin(115200);
 
  //Ultra sound configuration 
  for(i=0;i<5;i++){
                  pinMode(trig[i],OUTPUT);
                  pinMode(echo[i],INPUT);
                  }
                  
           
  attachInterrupt(0,sendData1, RISING);//pin 2
  attachInterrupt(1,sendData2, RISING);//pin 3

            
}

void loop()
{
     ultra();
//  sendData1();
//  sendData2();
//  delay(100);
}

//************************************************************************************
void ultra()
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
    angle_obstacle=angle_obstacle/0.017453; 
 }
//******************************************************************************************* 
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
//*******************************************************************************************
void sendData1()
{  
   Serial.print(module_obstacle);
   Serial.print('\0');
   //Serial.print("----");
   
 }
 void sendData2()
{  
  
   Serial.print((int)((angle_obstacle)*10));
   Serial.print('\0');
   
 }

