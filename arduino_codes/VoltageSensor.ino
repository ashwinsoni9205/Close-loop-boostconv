#define Vin A0
float Vcc = 5.0000;
float Vmin = 0.05;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  float R1 = 19.5600; 
  float Req = 1030.0000;
  float Vdetect = 0.0000;
  float analogAvg = 0.0000;
  analogAvg = (analogRead(Vin));
  Vdetect = (5.0000*analogAvg)/1023.0000;
  float Vactual = ((Vdetect*Req)/R1);
  if(Vdetect < Vmin)
  {
    Serial.println(analogRead(Vin));
    Serial.println("No Voltage");
  }
  else
  {
    Serial.println(analogRead(Vin));
    Serial.println(Vdetect);
    Serial.println(Vactual);
  }
delay(1000);
}
