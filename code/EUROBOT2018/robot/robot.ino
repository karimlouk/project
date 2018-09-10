#include<math.h>

#include <MPU6050.h>
MPU6050 mpu;

unsigned long timer = 0;
float timeStep = 0.01;
float pitch = 0;
float roll = 0;
float yaw = 0;


float R;  // wheels radious
float L;  // distance between the wheels
int N;    //tiks number 
float phi;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}


void Diff_drive_modelle();
{
float Dc,Dl,Dr;
Dl=0.233*N2;
Dr=0.199*N2;
Dc=(Dl+Dr)/2;  
  x=x+Dc*cos(phi);
  y=y+Dc*sin(phi);
  phi=yaw;
}

