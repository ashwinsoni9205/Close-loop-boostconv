#define Vin A0
float Vcc = 5.0;
float sensitivity = 0.1000; // 0.100 for 20A and 0.66 for 30A;
float V0out = 0.5*Vcc; // 0 current output voltage from the sensor;
float Voltage;
float Icutoff = 0.5000;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // this is to initialize the serial monitor, so that we can see out results;
}

void loop() {
  // put your main code here, to run repeatedly:
  float analogAvg = 0;

  for (int i = 0 ; i<10 ; i++)
  {
    analogAvg = analogAvg + analogRead(Vin);
    delay(10);
  }
  float rawVoltage = (5.0000*analogAvg)/10230.0000; // this will read the voltage from the sensor and will give us a raw voltage;
  Voltage = rawVoltage - V0out ; // error factors (0.011 and 0.019);
  float current = Voltage / sensitivity;

  if(abs(current) > Icutoff)// checking the absolute value of the current;
  {
    Serial.println(analogRead(Vin));
    // Serial.println(rawVoltage,4);
    // Serial.println(Voltage,4);
    Serial.println(current,4);
  }
  else
  {
    Serial.println(analogRead(Vin));
    // Serial.println(rawVoltage,4);
    // Serial.println(Voltage,4);
    Serial.println("No current");
  }
  delay(500); // to give delay of 500ms before running the loop again;
}
