// Implemented on Arduino Nano Every with USB control
// PID not tuned yet
#include <Adafruit_MAX31865.h>

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31865 thermo = Adafruit_MAX31865(A0, A1, A2, A3);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31865 thermo = Adafruit_MAX31865(10);

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF      4300.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  1000.0

#define SSR_TEMP    8
#define SSR_MAGNETS 7

float aerror[20]; // 2 point should be enough
float ainterval[20];
float temp = 0;
float ratio = 0;
int k = 0;
float error = 0;
float derror = 0;
float ierror = 0;
float time_tot = 1;
float time_0[20];
float interval = millis();
bool heatingEnable = true;
bool magnetEnable = true;
float Kp = 1.0;
float Kd = 150.0; //PID parameters to be tuned
float Ki = 0.01;
float setpoint = 37;

bool stringComplete = false;
String dataString = "";
String inputString = "";

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Adafruit MAX31865 PT1000 Sensor Test!");
  thermo.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  
  pinMode(SSR_TEMP,OUTPUT);
  digitalWrite(SSR_TEMP,LOW);
  pinMode(SSR_MAGNETS,OUTPUT);
  digitalWrite(SSR_MAGNETS,LOW);
  delay(500);
}


void loop() {

  uint16_t rtd = thermo.readRTD();
  
  //Serial.print("RTD value: "); Serial.println(rtd);
  ratio = rtd;
  temp = thermo.temperature(RNOMINAL,RREF);
  error = setpoint-temp;
  interval = millis()-interval;
  ratio /= 32768;
  //Serial.print("Ratio = "); Serial.println(ratio,8);
  //Serial.print("Resistance = "); Serial.println(RREF*ratio,8);
  //Serial.print("Temperature = "); Serial.println(thermo.temperature(RNOMINAL, RREF));

  // Check and print any faults
  uint8_t fault = thermo.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    thermo.clearFault();
  }
  //delay(200);

  //Temperature Control PID ----------------------------------------------------------------
  if(k<20){
    aerror[k] = error; //array_error °C
    ainterval[k] = interval; //array_interval ms
    //time_0[k] = time_0[k-1] + interval;
    k++;
  }
  else{
    for(int i = 0; i<19;i++){
      aerror[i] = aerror[i+1];
      ainterval[i] = ainterval[i+1];
      time_tot += ainterval[i];
      time_0[i] = time_tot;
    }
    aerror[19] = error;
    ainterval[19] = interval;
    time_tot += interval;
    time_0[19] = time_tot;
    derror = (aerror[19]-aerror[10])/(time_0[19]-time_0[10])*1000;
    ierror = (aerror[19]*ainterval[19]+aerror[18]*ainterval[18])/(ainterval[19]+ainterval[18]);
  }
  // PID controller = Kp*e + Ki*integral(e) + Kd*de/dt
  // error > 0 when temp < setpoint
  //&& !digitalRead(PIN_ENABLE)
  Serial.print("Setpoint : ");Serial.print(Kp*error + Kd*derror + Ki*ierror + temp);
  Serial.print(" / Temperature : ");Serial.print(temp);
  Serial.print(" / derror : ");Serial.print(Kd*derror);
  Serial.print(" / error : ");Serial.print(error);
  Serial.println();
  //if(Kp*error + Kd*derror + Ki*ierror > 0 && digitalRead(PIN_ENABLE)){
  if(Kp*error + Kd*derror + Ki*ierror > 0 && heatingEnable){
    digitalWrite(SSR_TEMP,HIGH);
    Serial.print("Heating On / ");
  }
  else{digitalWrite(SSR_TEMP,LOW);
  Serial.print("Heating Off / ");
  }
  delay(250); //Wait 250 ms to be sure that the SSR was activated or deactivated
  //Magnet control ----------------------------------------------------------------------------
  if(magnetEnable){
    digitalWrite(SSR_MAGNETS,HIGH);
    Serial.println("Magnets ON");
  }
  else {
    digitalWrite(SSR_MAGNETS,LOW);
    Serial.println("Magnets OFF");
  }
  //SerialEvent -------------------------------------------------------------------
  if(Serial.available()){
    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // if the incoming character is a newline, set a flag so the main loop can do something about it:
      if (inChar == '\n') {
          stringComplete = true;}
      else {    
      // add it to the inputString:
      inputString += inChar;}
    }
  }
  //if SerialEvent occured
  if(stringComplete){
    Serial.println(inputString);
    dataString = inputString;
    stringComplete = false;

    if(dataString == "Magnets ON")
      magnetEnable = true;
    if(dataString == "Magnets OFF")
      magnetEnable = false;
    if(dataString == "Heating ON")
      heatingEnable = true;
    if(dataString == "Heating OFF")
      heatingEnable = false;
    if(dataString == "Reset Controller")
      software_reset();

    dataString = "";
    inputString = "";
  }
}

//Reset Arduino
void software_reset() {
  asm volatile (" jmp 0");  
}
