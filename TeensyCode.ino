#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SD.h>

boolean testMode = false;
boolean runStarted = false;

unsigned long current  = 0;

int bankb = 0;
int bankr = 0;

//Actuators
#define DIG1 24
#define DIG2 25
#define DIG3 26
#define DIG4 27
#define DIG5 28
#define DIG6 29
#define DIG7 30
#define DIG8 31
#define DIG9 32
#define DIG10 2
#define DIG11 3
#define DIG12 4
#define DIG13 5
#define DIG14 6
#define DIG15 14
#define DIG16 15

//Encoders
#define ENC1 17
#define ENC2 18
#define ENC3 7
#define ENC4 8

//Potentiometers
#define POT1 A14
#define POT2 A15
#define POT3 A16
#define POT4 A17
#define POT5 A18
#define POT6 A19
#define POT7 A20
#define POT8 A21

#define TEMP A7 //Tempereture
#define PRESSURE A8 //Pressure
#define PUSHER 5 //Distance from Pusher
#define BOTTOM A22 //Distance to Bottom


int pot1 = 0;
int pot2 = 0;
int pot3 = 0;
int pot4 = 0;
int pot5 = 0;
int pot6 = 0;
int pot7 = 0;
int pot8 = 0;

int maxPot = 700;
int minPot = 300;
int minPot6 = 310;

int maxPot1 = 1010;
int maxPot2 = 980;
int maxPot3 = 1005;
int maxPot4 = 1015;
int maxPot5 = 920;
int maxPot6 = 895;
int maxPot7 = 970;
int maxPot8 = 1015;

//Pressure Sensor Variables and Constants
/*double pressureVsupply = 3.3;
double pressureMaxADC = 1023; //for 10-bit ADC
double Pmax = 14.5; //14.5 PSI for SSCSANN001BAAA3 pressure sensor
double Pmin = 0; //0 PSI for SSCSANN001BAAA3 pressure sensor
*/

//Distance to Bottom Variables
double Vsupply = 3.3;
double maxADC = 1023; //for 10-bit ADC

//Tempereature
int reading;
float myVoltage = 0;

//Encoder  
unsigned int pulses[4] = {0, 0, 0, 0};
unsigned int pulsesf[4] = {0, 0, 0, 0};
int pulsesdt = 0;
unsigned int pulsest = 0;
float velocitydt = 0;
float velocityprev = 0;
unsigned int dtime = 500;
float dt = 0;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
unsigned long prev1,prev2,prev3,prev4 = 0;

//SD Card Variables
const int chipSelect = BUILTIN_SDCARD;
String dataString = "";

//Wireless Networking Variables
byte teensyMAC[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
unsigned int teensyPort = 5000;
IPAddress teensyIP(192, 168, 0, 9);
unsigned int computerPort = 6000;
const char *computerIP = "192.168.0.8";
EthernetUDP Udp;
char packetBuffer[1];

//Thresholds
unsigned int timeOutThreshold = 5000;   //How long to wait for message until Pod thinks its disconnected
int distPusherThreshold = 20;            //Distance from the Pusher in cm
unsigned int timePushThreshold = 0;  //Time after leaving Pusher in milliseconds
unsigned int runStartThreshold = 10000;  //Time after starting run
int podDistanceThreshold = 200;        //Distance at which we brake (in encoder counts)

int podState; //State of the Pod
String c; //Serial Variables
String d; //Serial Variables

boolean good; //good to go from idle to good?
boolean halt; //start breaking?
boolean uplink; //connected to computer?

//Sensor Values
float acceleration = 0;
float acceleration1 = 0;
float acceleration2 = 0;
float accelerationEncoder = 0;
float velocity = 0; //  pulses/s
int pos = 0;
float temperature = 0;
float distanceFromBottom = 0;
float distanceFromPusher = 0;
float pressure = 0;

//Timing variables
unsigned long lastCommunicationTime = 0;
unsigned long lastPushTime = 0;
unsigned long runStartTime = 0;

void send(float stat, float accel, float veloc, float posit, float distFromBot, float distFromPush, float temp, float pressur, int pot1, int pot2, int pot3, int pot4, int pot5, int pot6, int pot7, int pot8) {
  // send a reply to the IP address and port that sent us the packet we received
  Udp.beginPacket(computerIP, computerPort);
  String fin = "[";
  fin = fin + stat + ','; //Pod Status
  fin = fin + accel + ','; // m/s^2 (units)
  fin = fin + veloc + ','; // m/s (units)
  fin = fin + posit + ','; // encoder counts
  fin = fin + distFromBot + ','; // cm
  fin = fin + distFromPush + ','; // cm
  fin = fin + temp + ','; // degrees celsius
  fin = fin + pressur + ','; // psi
  fin = fin + pot1 + ',';
  fin = fin + pot2 + ',';
  fin = fin + pot3 + ',';
  fin = fin + pot4 + ',';
  fin = fin + pot5 + ',';
  fin = fin + pot6 + ',';
  fin = fin + pot7 + ',';
  fin = fin + pot8;
  fin = fin + ']' + '\n';
  char charBuff[fin.length() + 1];
  fin.toCharArray(charBuff, fin.length() + 1);
  Serial.print(fin);
  Udp.write(charBuff);
  Udp.endPacket();
}
//Initialization functions
void configPins(){
  
  pinMode(ENC1, INPUT_PULLUP);
  pinMode(ENC2, INPUT_PULLUP);
  pinMode(ENC3, INPUT_PULLUP);
  pinMode(ENC4, INPUT_PULLUP);

  pinMode(POT1, INPUT);
  pinMode(POT2, INPUT);
  pinMode(POT3, INPUT);
  pinMode(POT4, INPUT);
  pinMode(POT5, INPUT);
  pinMode(POT6, INPUT);
  pinMode(POT7, INPUT);
  pinMode(POT8, INPUT);

  pinMode(DIG1, OUTPUT);
  pinMode(DIG2, OUTPUT);
  pinMode(DIG3, OUTPUT);
  pinMode(DIG4, OUTPUT);
  pinMode(DIG5, OUTPUT);
  pinMode(DIG6, OUTPUT);
  pinMode(DIG7, OUTPUT);
  pinMode(DIG8, OUTPUT);
  pinMode(DIG9, OUTPUT);
  pinMode(DIG10, OUTPUT);
  pinMode(DIG11, OUTPUT);
  pinMode(DIG12, OUTPUT);
  pinMode(DIG13, OUTPUT);
  pinMode(DIG14, OUTPUT);
  pinMode(DIG15, OUTPUT);
  pinMode(DIG16, OUTPUT);

  pinMode(TEMP, INPUT);
  pinMode(PRESSURE, INPUT);
  pinMode(A5, INPUT);
  pinMode(BOTTOM, INPUT);
  
}
void initVariables() {
  podState = 0;

  acceleration1 = 0;
  acceleration2 = 0;
  accelerationEncoder = 0;
  velocity = 0;
  pos = 0;
  temperature = 0;
  distanceFromBottom = 0;
  distanceFromPusher = 0;
  pressure = 0;

  good = false; //Ready to transition form Idle to Ready state
  halt = false;
  uplink = false;
}
void initSDCard() {
  current = millis();
  while(millis() - current < 30000){
    if(SD.begin(chipSelect)){
      while(millis() - current < 30000){
        File DataBase = SD.open("datalog.txt", FILE_WRITE);
        Serial.println("datalog.txt initialized.");
        if(DataBase){
          DataBase.println("___________________________Purdue Hyperloop Master Unit DATALOG___________________________");
          DataBase.close();
          Serial.println("Card initialized.");
          break;
        }
      }
      break;    
    }
  }  
}
void initWireless() {
  Ethernet.begin(teensyMAC, teensyIP);
  Udp.begin(teensyPort);
  Serial.println("Wireless Initialized");
}
void initEncoders() {
  attachInterrupt(digitalPinToInterrupt(ENC1), encoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2), encoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC3), encoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC4), encoder4, CHANGE);
}
//Encoder Counting

void encoder1() {
  if(micros()-prev1>3347){
    pulses[0]++;
    prev1 = micros();
  }
}
void encoder2() {
  if(micros()-prev1>3347){
    pulses[1]++;
    prev2 = micros();
  }
}
void encoder3() {
  if(micros()-prev1>3347){
    pulses[2]++;    
    prev3 = micros();
  }
}
void encoder4() {
  if(micros()-prev1>3347){
    pulses[3]++;
    prev4 = micros();
  }
}

void determineDistanceFromPusher() {
  if (testMode) {
    Serial.print("Enter distance from pusher: ");
    distanceFromPusher = (Serial.readStringUntil('$')).toFloat();
    Serial.println(distanceFromPusher);
  }
  else {   
   int newSenseValue;
   newSenseValue = analogRead(A5);
   distanceFromPusher = (23.391*pow(2.718,-0.003*newSenseValue))*2.54;
   //Serial.println(distanceFromPusher);
  }
}
void determinePositionVelocity() {
  
  if (testMode) {
    Serial.print("Enter total distance: ");
    pos = (Serial.readStringUntil('$')).toFloat();
    Serial.println(pos);
    Serial.print("Enter velocity: ");
    velocity = (Serial.readStringUntil('$')).toFloat();
    Serial.println(velocity);
  }
  else {
    
    currentMillis = millis();
    
    if (currentMillis - previousMillis >= dtime) {      
     
      dt = currentMillis - previousMillis;
      previousMillis = currentMillis;
      
      for (unsigned int j = 0; j <= 3; j++) {
          pulsesf[j] = pulses[j];
          pulses[j] = 0;      
      }
      
      for (unsigned int i = 0; i <= 3; i++) {       
        if (pulsesf[i] > pulsesdt) {
          pulsesdt = pulsesf[i];
        }
        pulsesf[i] = 0;
      }
      
      dt = dt/1000;
      velocity = pulsesdt / dt;
      velocitydt = velocity - velocityprev;      
      accelerationEncoder  = velocitydt / dt;
      accelerationEncoder = accelerationEncoder * 0.2394;
      velocityprev = velocity;
      pulsest = pulsest + pulsesdt;
      pulsesdt = 0;
      pos = pulsest;
      
      if(velocity >= 20){
        dtime = 100;      
      } else {
        dtime = 500;
      }

    }
          
  }
}

void readSlave() {
  if (testMode) {
    Serial.print("Enter acceleration: ");
    acceleration1 = (Serial.readStringUntil('$')).toFloat();
    Serial.println(acceleration1);
  }
  else {
    if (Serial1.available()) {
      c = Serial1.readStringUntil('\n');
      int commaLoc1 = c.indexOf(",");
      acceleration1 = (c.substring(0, commaLoc1)).toFloat();
      acceleration2 = (c.substring(commaLoc1 + 1)).toFloat();
    }
  }
}

void medianAcceleration(){

  float orderAcceleration[3] = {acceleration1,acceleration2, accelerationEncoder};
  sort(orderAcceleration,3);
  acceleration = orderAcceleration[1];
  
}

void sort(float a[], int size) {
    for(int i=0; i<(size-1); i++) {
        for(int o=0; o<(size-(i+1)); o++) {
                if(a[o] > a[o+1]) {
                    int t = a[o];
                    a[o] = a[o+1];
                    a[o+1] = t;
                }
        }
    }
}

void determineTemperature() {
  int i = 0;
  for(i = 0; i<100; i++){
    reading = reading + analogRead(TEMP);    
  }
  reading = reading / 101;
  myVoltage = (myVoltage + ((reading*3.226)/1000))/2;
  temperature = (temperature + ((myVoltage-1.25)/0.005))/2;
}

void determineDistanceFromBottom() {

  int senseValue = 0;
  double senseDistance = 0;

  senseValue = analogRead(BOTTOM);
  senseDistance = 16.706*pow(2.718,-0.004*senseValue);
  distanceFromBottom = senseDistance;
  //Serial.println(distanceFromBottom);
}
void readpins() {
  pot1 = analogRead(POT1);
  pot2 = analogRead(POT2);
  pot3 = analogRead(POT3);
  pot4 = analogRead(POT4);
  pot5 = analogRead(POT5);
  pot6 = analogRead(POT6);
  pot7 = analogRead(POT7);
  pot8 = analogRead(POT8);
  Serial.print(pot1);
  Serial.print(",");
  Serial.print(pot2);
  Serial.print(",");
  Serial.print(pot3);
  Serial.print(",");
  Serial.print(pot4);
  Serial.print(",");
  Serial.print(pot5);
  Serial.print(",");
  Serial.print(pot6);
  Serial.print(",");
  Serial.print(pot7);
  Serial.print(",");
  Serial.println(pot8);
}
void logData() {
  File DataBase = SD.open("datalog.txt", FILE_WRITE);
  dataString = String(podState);
  dataString = dataString + ","; 
  dataString = dataString + String(acceleration);
  dataString = dataString + ",";
  dataString = dataString + String(velocity);
  dataString = dataString + ",";
  dataString = dataString + String(pulsest);
  dataString = dataString + ",";
  dataString = dataString + String(distanceFromBottom);
  dataString = dataString + ",";
  dataString = dataString + String(distanceFromPusher);
  dataString = dataString + ",";
  dataString = dataString + String(temperature);
  dataString = dataString + ",";
  dataString = dataString + String(pressure);
  
  if (DataBase) { // if the file is available, write to it:
    DataBase.println(dataString);
    DataBase.close();
    Serial.print(dataString);
    Serial.print("___________");
    Serial.println(pot7);
  }
  else { // if the file isn't open, pop up an error:
    //Serial.println("SD Card Error");
  }

}
void updateWireless() {
  //Get heartbeat or good/break message and check if connection lost
  //Get flags for connectivity, distanceFromPusher, reachedpodDistanceThreshold, acceleration > 0
  if(testMode){
    Serial.print("Connected? ");
    uplink = (Serial.readStringUntil('$')).toInt();
    if(uplink){Serial.println("Yes");}
    else{Serial.println("No");}
    
    Serial.print("Ready to Start Run? ");
    good = (Serial.readStringUntil('$')).toInt();
    if(good){Serial.println("Yes");}
    else{Serial.println("No");}
    
    Serial.print("Emergency Brake? ");
    halt = (Serial.readStringUntil('$')).toInt();
    if(halt){Serial.println("Engaged");}
    else{Serial.println("Released");}
    
  } else {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      uplink = true;
      lastCommunicationTime = millis();
      Udp.read(packetBuffer, 1);
      if (packetBuffer[0] == 'r') {
        good = true; // good to go into "good" state
      }
      else if (packetBuffer[0] == 'b') {
        halt = true;
      }
      else if (packetBuffer[0] == 'x') {
        podState = -2;

      }
      else if (packetBuffer[0] == 'y') {
        podState = -3;

      } 
      else if (packetBuffer[0] == 'z'){
        podState = 0;
      }
    }
    else if (!testMode && millis() - lastCommunicationTime >= timeOutThreshold) {
      uplink = false;
    }
    send(podState, acceleration, velocity, pos, distanceFromBottom, distanceFromPusher, temperature, pressure, pot1, pot2, pot3, pot4, pot5, pot6, pot7, pot8);
  }
}
void braking(){
  bankr = 0;
  if(bankb==0){
    pot1 = analogRead(POT1);
    if (pot1 < maxPot1) {
      digitalWrite(DIG1, HIGH);
      digitalWrite(DIG2, LOW);
    } else {
      digitalWrite(DIG1, LOW);
      digitalWrite(DIG2, LOW);
    }
    pot3 = analogRead(POT3);
    if (pot3 < maxPot3) {
      digitalWrite(DIG5, HIGH);
      digitalWrite(DIG6, LOW);
    } else {
      digitalWrite(DIG5, LOW);
      digitalWrite(DIG6, LOW);
    }
    pot6 = analogRead(POT6);
    if (pot6 < maxPot6) {
      digitalWrite(DIG11, HIGH);
      digitalWrite(DIG12, LOW);
    } else {;
      digitalWrite(DIG11, LOW);
      digitalWrite(DIG12, LOW);
    }
    pot8 = analogRead(POT8);
    if (pot8 < maxPot8) {
      digitalWrite(DIG15, HIGH);
      digitalWrite(DIG16, LOW);
    } else {
      digitalWrite(DIG15, LOW);
      digitalWrite(DIG16, LOW);
    }
    if(pot1 >= maxPot1 && pot3 >= maxPot3 && pot6 >= maxPot6 && pot8 >= maxPot8){
      bankb=1;
    }
  } else if(bankb ==1){
        
    pot7 = analogRead(POT7);
    if (pot7 < maxPot7) {
      digitalWrite(DIG13, HIGH);
      digitalWrite(DIG14, LOW);
    } else {
      digitalWrite(DIG13, LOW);
      digitalWrite(DIG14, LOW);
    }
    pot4 = analogRead(POT4);
    if (pot4 < maxPot4) {
      digitalWrite(DIG7, HIGH);
      digitalWrite(DIG8, LOW);
    } else {
      digitalWrite(DIG7, LOW);
      digitalWrite(DIG8, LOW);
    }
    pot5 = analogRead(POT5);
    if (pot5 < maxPot5) {
      digitalWrite(DIG9, HIGH);
      digitalWrite(DIG10, LOW);
    } else {
      digitalWrite(DIG9, LOW);
      digitalWrite(DIG10, LOW);
    }
      pot2 = analogRead(POT2);
    if (pot2 < maxPot2) {
      digitalWrite(DIG3, HIGH);
      digitalWrite(DIG4, LOW);
    } else {
      digitalWrite(DIG3, LOW);
      digitalWrite(DIG4, LOW);
    }
    if(pot7 >= maxPot7 && pot4 >= maxPot4 && pot5 >= maxPot5 && pot2 >= maxPot2){
      bankb=2;
    }   
    
  }
}
void releasebrakes(){
  bankb = 0;
  if(bankr==0){
    pot1 = analogRead(POT1);
    if (pot1 > minPot) {
      digitalWrite(DIG1, LOW);
      digitalWrite(DIG2, HIGH);
    } else {
      digitalWrite(DIG1, LOW);
      digitalWrite(DIG2, LOW);
    }
    pot3 = analogRead(POT3);
    if (pot3 > minPot) {
      digitalWrite(DIG5, LOW);
      digitalWrite(DIG6, HIGH);
    } else {
      digitalWrite(DIG5, LOW);
      digitalWrite(DIG6, LOW);
    }
    pot6 = analogRead(POT6);
    if (pot6 > minPot6) {
      digitalWrite(DIG11, LOW);
      digitalWrite(DIG12, HIGH);
    } else {
      digitalWrite(DIG11, LOW);
      digitalWrite(DIG12, LOW);
    }
    pot8 = analogRead(POT8);
    if (pot8 > minPot) {
      digitalWrite(DIG15, LOW);
      digitalWrite(DIG16, HIGH);
    } else {
      digitalWrite(DIG15, LOW);
      digitalWrite(DIG16, LOW);
    }
    if(pot1 <= minPot && pot3 <= minPot && pot6 <= minPot6 && pot8 <= minPot){
      bankr=1;
    }
  } else if(bankr == 1) {
        
    pot7 = analogRead(POT7);
    if (pot7 > minPot) {
      digitalWrite(DIG13, LOW);
      digitalWrite(DIG14, HIGH);
    } else {
      digitalWrite(DIG13, LOW);
      digitalWrite(DIG14, LOW);
    }
    pot4 = analogRead(POT4);
    if (pot4 > minPot) {
      digitalWrite(DIG7, LOW);
      digitalWrite(DIG8, HIGH);
    } else {
      digitalWrite(DIG7, LOW);
      digitalWrite(DIG8, LOW);
    }
    pot5 = analogRead(POT5);
    if (pot5 > minPot) {
      digitalWrite(DIG9, LOW);
      digitalWrite(DIG10, HIGH);
    } else {
      digitalWrite(DIG9, LOW);
      digitalWrite(DIG10, LOW);
    }
      pot2 = analogRead(POT2);
    if (pot2 > minPot) {
      digitalWrite(DIG3, LOW);
      digitalWrite(DIG4, HIGH);
    } else {
      digitalWrite(DIG3, LOW);
      digitalWrite(DIG4, LOW);
    }
    if(pot7 <= minPot && pot4 <= minPot && pot5 <= minPot && pot2 <= minPot){
      bankr=2;
    }   
    
  }
}

void updateState() {
  switch (podState) {
    case -3: //Debug Release
      break;
    case -2: //Debug Brake
      break;
    case -1: //Fault brake      
      break;
    case 0: //Fault
      if (!halt && uplink && !runStarted){
        podState = 1;
      }
      else if (distanceFromPusher > distPusherThreshold && acceleration <= 0 && runStarted){// 
        //If we are not touching the pusher and the run has started ...
        if(millis() - runStartTime > runStartThreshold){
          //If we have passed our time threshold 
          podState = -1;
        }
        else {
          podState = 0;
        }
      }
      break;
    case 1: //Idle
      if (!uplink || halt){
        podState = 0;
      }
      else if (good){
        podState = 2;
        good = false;
        pulsest = 0;
        pos = 0;
      }
      break;
    case 2: //Ready    
      runStarted = true;
      if (!uplink || halt){
        podState = 0;
      }
      else if (acceleration > 0.25){ //&& distanceFromPusher < distPusherThreshold
        podState = 3;
        runStartTime = millis();
      }
      break;
    case 3: //Pushing
      if (!uplink || halt){
        podState = 0;
      }
      else if (distanceFromPusher > distPusherThreshold && acceleration <= 0 && millis() - runStartTime > runStartThreshold ) { //(
          podState = 4;
      } else {
        lastPushTime = millis();
      }
      break;
    case 4: //Coasting
      if (!uplink || halt){
        podState = 0;
      }
      else if (pos > podDistanceThreshold){

          podState = 5;
      }
      break; 
    case 5: //Breaking
      if (!uplink || halt){ 
        podState = 0;
      }
      break;
    default:
      Serial.print("Updated states incorrectly");
      podState = 0;
      break;
  }
  Serial.println(podState);
}

void performState() {
  switch (podState) {
    case -3:
      releasebrakes();
      break;
    case -2:
      braking();
      break;
    case -1: //Fault break
      braking();
      Serial.println("Emergency Braking");
      break;
    case 0:
      releasebrakes();
      Serial.println("Emergency Stopped");
      break;
    case 1:
      Serial.println("Idle");
      releasebrakes();
      break;
    case 2:
      Serial.println("Ready To Be Pushed");
      releasebrakes();
      break;
    case 3:
      Serial.println("Pushing");
      releasebrakes();
      break;
    case 4:
      Serial.println("Coasting");
      releasebrakes();
      break;
    case 5:
      Serial.println("Braking");
      braking();
      break;
    default:
      Serial.print("Updated states incorrectly");
      break;
  }
}

void printDiagnostics(){
    Serial.println("Current Values");
    Serial.print("Acceleration: ");
    Serial.println(acceleration);
    Serial.print("Velocity: ");
    Serial.println(velocity);
    Serial.print("Distance: ");
    Serial.println(pos);
    Serial.print("Distance from Pusher: ");
    Serial.println(distanceFromPusher);
    Serial.print("Distance from Bottom: ");
    Serial.println(distanceFromBottom);
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Connection: ");
    Serial.println(uplink);
    Serial.print("Ready to Start: ");
    Serial.println(good);
    Serial.print("Emergency Stop: ");
    Serial.println(halt);
    Serial.println("");
}
void setup() {
  //setup code
  if(testMode){ Serial.setTimeout(-1); }

  Serial.begin(230400);
  Serial1.begin(230400);
  configPins();
  initVariables();
  initWireless();
  initSDCard();
  initEncoders();

}
void loop() {
  //main loop
  if(!testMode){

    readpins();
    updateWireless();
    determineDistanceFromPusher();
    determinePositionVelocity();
    medianAcceleration();
    readSlave();
    determineDistanceFromBottom();
    determineTemperature();
    updateState();
    performState();
    logData();
    //printDiagnostics();
    
  } else {
    
    if(Serial.readStringUntil('$') == "l"){
      updateWireless();
      determineDistanceFromPusher();
      determinePositionVelocity();
      readSlave();
      determineDistanceFromBottom(); 
      determineTemperature();
      medianAcceleration();
      updateState();
      //readpins();
      performState();
      //logData();
    }
    else if(Serial.readStringUntil('$') == "d"){ printDiagnostics(); }
  }
}

