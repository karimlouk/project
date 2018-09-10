#include <MPU6050.h>
#include <Wire.h>
#include <math.h>
#include <AFMotor.h>
float v=0;
float A;
double set_point,MES,commande;
double Erreure_Angle,Erreure_preced_Angle,somme_Erreure_Angle,delta_Erreure_Angle;
double cmd;
AF_DCMotor motor1(1); // define motor on channel 4 with 1KHz default PWM
AF_DCMotor motor2(2); // define motor on channel 4 with 1KHz default PWM
AF_DCMotor motor3(3); // define motor on channel 4 with 1KHz default PWM
AF_DCMotor motor4(4); // define motor on channel 4 with 1KHz default PWM
float vl=0,vr=0;
double  omega;
int  oldcomptD=0;
int  oldcomptG=0;
int oldOrientation;
double Dl,Dr,Dc;
int Nl=0,Nr=0;
double xr,yr;
float Orientation;
int deltaCommande;
const double coeffGLong = 0.233;
const double coeffDLong = 0.199;  
long comptD = 0;
long comptG = 0;
double xc =800;
double yc =-800;
double consigneOrientation = 0.0;
double distanceCible = 0.0;
double yaw_gyro,distance_obstacle,ongle_obstacle;  //valeur retourner par la communication serie avec NANO
void setup()
{
  Serial.begin(115200);
  Serial3.begin(115200);  
  pinMode(28, INPUT);  
  pinMode(30, INPUT);
  pinMode(19, INPUT);           
  pinMode(18, INPUT);
  digitalWrite(28, HIGH);       
  digitalWrite(30, HIGH);       
  digitalWrite(19, HIGH);       
  digitalWrite(18, HIGH);
  
  // Nano interrupt pins
  pinMode(31,OUTPUT);
  pinMode(32,OUTPUT);
  pinMode(33,OUTPUT);
  digitalWrite(31, LOW);
  digitalWrite(32, LOW);
  digitalWrite(33, LOW);
           
     
  attachInterrupt(4, Reagir, RISING);//19
  attachInterrupt(5, ReagirBis, RISING);//18
}

/**************************Programme Principale****************************/
void loop()
 {

        motor3.run(FORWARD);
        motor3.setSpeed(150); 
        motor4.run(FORWARD);
        motor4.setSpeed(50);
//coordonner();
//yaw_gyro=revoir_data(32)*0.0017;
//distance_obstacle=revoir_data(33);
//ongle_obstacle=revoir_data(31)/10;

//distanceCible = sqrt((xc-xr)*(xc-xr)+(yc-yr)*(yc-yr));
//consigneOrientation =atan2((yc-yr),(xc-xr));
//
//Orientation=yaw_gyro;
//omega=PID_Reguator_Angle((double)consigneOrientation,(double)Orientation,0.01,100,0.1);
//v=gain_distance((double)distanceCible);
//commandmotor();
//
//if (v<10)
//{
//  motor1.run(FORWARD);
//      motor1.setSpeed(0); 
//      motor2.run(FORWARD);
//      motor2.setSpeed(0);
//      xc =0;
//        yc =0;
//      delay(5000);       
//}
//Serial.print(" vl==");
//Serial.print( vl);
//Serial.print(" vr ==");
//Serial.print( vr);
//Serial.print(" v ==");
//Serial.print( v);
//Serial.print(" Orientation ==");
//Serial.print( Orientation);
//Serial.print(" consigneOrientation ==");
//Serial.print( consigneOrientation);
//Serial.print(" omega ==");
//Serial.print( omega);
//Serial.print(" distanceCible ==");
//Serial.println( distanceCible );
//Serial.print(yaw_gyro);
//Serial.print("****");
//Serial.print(distance_obstacle);
//Serial.print("****");
//Serial.println(ongle_obstacle);

}
/**************************Inerruption1****************************/
void Reagir()
{ 
     if(digitalRead(30) == HIGH)
       {
         comptG--;        
       }
       else
       {
         comptG++;        
       }
       
     Serial.print("Gauche=");
     Serial.println(comptG);
     
}
/**************************Inerruption2****************************/
void ReagirBis()
{    
    if(digitalRead(28) == HIGH)
      {
        comptD ++;
      }
      else
      {
        comptD --;
      }
     Serial.print("Droite=");
     Serial.println(comptD);
     
}
/**************************valeurs NANO****************************/
  int revoir_data(int interrupt)
{
  digitalWrite(interrupt, HIGH);
  char message[20],character_recu;
  int indice=0,data=0;
                                   
         do{
            if(Serial3.available()){
              character_recu=Serial3.read();
              message[indice]=character_recu;             // enregistrement de chaque charactere sur une case memoire de la table message
              indice++;
              } 
             }while(character_recu!='\0');            //arret de la reception et de lenregistrement des donnees
             data=atoi(message);
digitalWrite(interrupt, LOW);

return data;
}
///**************************calcule coordonner****************************/
void coordonner()
{
    Nl=comptD-oldcomptD;
    Nr=comptG-oldcomptG;
    oldcomptD=comptD;
    oldcomptG=comptG;
    Dl=coeffDLong*Nl;
    Dr=coeffGLong*Nr;
    Dc=(Dr+Dl)/2;
    xr=xr+Dc*cos(Orientation);
    yr=yr+Dc*sin(Orientation);
}
//
///**************************Regulation Angle***************************/    
double PID_Reguator_Angle(double consigne,double mesure ,float Ki,float Kp,float Kd) //consigne,mesure,ki,kp,kd
{
   
  Erreure_Angle=consigne-mesure;
  somme_Erreure_Angle+=Erreure_Angle;
  delta_Erreure_Angle=Erreure_Angle-Erreure_preced_Angle;
  Erreure_preced_Angle=Erreure_Angle;
  cmd=Kp*Erreure_Angle+Ki*somme_Erreure_Angle+Kd*delta_Erreure_Angle;
  if(cmd<-100) cmd=-100;
  else if (cmd>100) cmd=100;
  return (cmd);
}
/**************************Regulation distance****************************/
float gain_distance(double dictance_erreure)
{
  double abs_error,k;
  double V0=180.0;
  float alpha=0.0001;
  abs_error=abs(dictance_erreure); 
  k=V0*(1-exp(-alpha*abs_error*abs_error));

  return k;
}
/**************************commande moteurs****************************/
void commandmotor()
{
  vl=((2*v+omega*180)/2);
  vr=((2*v-omega*180)/2);
  if(vr>255)
      {
         vr = 255;
      }
  else if(vr < 0)
      {
      vr = 0;
      }                 
  if(vl>255)
      {
      vl= 255;
      }
  else if(vl < 0)
      {
      vl= 0;
      }
  if(vl>0)
      { 
      motor1.run(FORWARD);
      motor1.setSpeed(vl);
      }
   else if (vl<0)
      { 
      motor1.run(BACKWARD);
      motor1.setSpeed((-vl));
      }
  if(vr>0)
      { 
      motor2.run(FORWARD);
      motor2.setSpeed(vr);
      }
  else if (vr<0)
      { 
      motor2.run(BACKWARD);
      motor2.setSpeed((-vr));
      }
}


