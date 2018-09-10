
int comptG;
void setup()
{
  Serial.begin(115200);
  
   pinMode(19, INPUT);  
  pinMode(30, INPUT);
        
  digitalWrite(19, HIGH);       
  digitalWrite(30, HIGH);       
 
  attachInterrupt(4, Reagir, RISING);//19
}

/**************************Programme Principale****************************/
void loop()
 { 
while(1);
}
/**************************Inerruption1****************************/
void Reagir()
{ 
     if(digitalRead(30) == HIGH)
       {
         comptG++;        
       }
       else
       {
         comptG--;        
       }
       Serial.println(comptG);
}

