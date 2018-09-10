
  double set_point,MES,commande;
  double Erreure,Erreure_preced,somme_Erreure,delta_Erreure;
  double cmd;
  

double PID_Reguator(double consigne,double mesure ,float Ki,float Kp,float Kd) //consigne,mesure,ki,kp,kd
{
   
  Erreure=consigne-mesure;
  somme_Erreure+=Erreure;
  delta_Erreure=Erreure-Erreure_preced;
  Erreure_preced=Erreure;
  
  cmd=Kp*Erreure+Ki*somme_Erreure+Kd*delta_Erreure;

  if(cmd<-255) cmd=-255;
  else if (cmd>255) cmd=255;
  
  return cmd;
  }

