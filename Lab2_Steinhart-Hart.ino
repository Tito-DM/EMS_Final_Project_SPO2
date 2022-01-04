#include<math.h>
#define AnalogPin A0
float R1=5000.0;
float R6=R1;
//float R4=1000.0;
//float R3=2200.0;
//float R5=5800.0;
float Vth=1.57;
float Rth=6504.0;
float Vcc=5.0;
float R_NTC=0.0;
float Vo=0.0;
//float Ro=10000;
//float To=25+273.15;
float T=0.0;
float A=0.0015936558304898885;
float B=0.00017023091250123744;
float C=2.4634823231312137*pow(10,-7);
float X=0;

float T_SH(float R){
  
    float A=0.0015936558304898885;
    float B=0.00017023091250123744;
    float C=2.4634823231312137*pow(10,-7);
    
    float Y = A+B*log(R)+C*pow(log(R),3);
    float X=(1/Y-273.15);
    return X;
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  
  Vo = analogRead(AnalogPin)*5.0/1023.0;
  R_NTC = (float)R1*(Vcc*(1+R6/Rth)/(Vo+R6*Vth/Rth)-1);

  Serial.println("-----------");
  Serial.print("Vo:");
  Serial.println(Vo);
  Serial.print("R_NTC:");
  Serial.println(R_NTC);
 
 //Steihart-Hart Model
  T = T_SH(R_NTC);
  Serial.print("T: ");
  Serial.println(T);
  delay(1000);
}
