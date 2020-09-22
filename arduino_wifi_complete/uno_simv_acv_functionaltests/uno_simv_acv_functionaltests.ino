// ========================
//          SETUP
// ========================
#include <Servo.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <Nextion.h>
#include <SPI.h>
#include <SD.h>
#include<Arduino.h>

//====== Serial Connection with NODEMCU =====
SoftwareSerial SUART(2, 3); //SRX=Dpin-2; STX-DPin-3

// setup servo
Servo servo;
float pos = 0;

// ==== Analog Pins =====
int potpinIE_ratio = 0;
int potpinTidVol = 1;
int potpinBPM = 2;
int pinMask = 5;
int pinDiff = 4;
int ledState = LOW;

// ==== Digital Pins =====
const int buzzerPin = 5;
const int ledPin = 6;

// ==== Sensor Offset =====
float constPressureMask = 512 + 24.1;
float slopePressureMask = 204.8;
float constPressureDiff = 512 + 23;
float slopePressureDiff = 204.8;

//  Other vars 
float IE_ratio;
float insTime;
float expTime;
float TidVol;
float BPM;
float separation;
float sensorvalue;
float maskPressure;
float diffPressure;
float volFlow;
float diffPressureArr[90];
float maskPressureArr[90];
float area_1 = 0.0002835287370;
float area_2 = 0.00007853981634;
float rho = 1.225;
float timeNow;

// Monitored Parameters 
float peakPressure = 0;
float peakFlow = 0;
float minuteVentilation = 0;
float cycleVolume;
float totVolume = 0;
String acvLabel = "acv";
String simvLabel = "simv";


// ======= Mode changing vars =========
uint32_t lastPrint = millis();

// Declare your Nextion objects - Example (page id = 0, component id = 1, component name = "b0")
//====== LCD Variables ==============
int start = 0;
int id_1 = 4;
int ch = 0;
int id_2 = 3;
int id_3 = 2;
String rawMsg, msg, pageNum;
String ad = "add ";
static char buffer_1[10] = {};
static char buffer_2[10] = {};
static char buffer_3[10] = {};
static char buffer_mode[10] = {};
String set_mode = "";
NexText IERatioText_Page0 = NexText(0, 13, "t_ie_ratio");
NexText BPMText_Page0 = NexText(0, 14, "t_bpm");
NexText TidVolText_Page0 = NexText(0, 15, "t_tidvol");
NexText IERatioText_Page1 = NexText(1, 11, "t_ie_ratio");
NexText BPMText_Page1 = NexText(1, 12, "t_bpm");
NexText TidVolText_Page1 = NexText(1, 13, "t_tidvol");

NexText t_mode = NexText(0, 4, "t_mode");
NexButton SIMVButton = NexButton(0, 2, "SIMVButton");
NexButton ACVButton = NexButton(0, 1, "ACVButton");
NexButton PSVButton = NexButton(0, 9, "PSVButton");
NexButton NoModeButton = NexButton(0, 10, "NoModeButton");
NexButton startButton = NexButton(0, 11, "startButton");
NexButton stopButton = NexButton(1, 1, "stopButton");
NexTouch *nex_listen_list[] = {&SIMVButton, &ACVButton, &PSVButton, &NoModeButton, &startButton, &stopButton, NULL};





//======= SD Card File ===========
File myFile;

void setup()
{
  servo.attach(6);
  Serial.begin(9600);
  SUART.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(10, OUTPUT);
  nexInit();
  SIMVButton.attachPop(SIMVButtonPopCallback, &SIMVButton);
  ACVButton.attachPop(ACVButtonPopCallback, &ACVButton);
  PSVButton.attachPop(PSVButtonPopCallback, &PSVButton);
  NoModeButton.attachPop(NoModeButtonPopCallback, &NoModeButton);
  startButton.attachPop(startButtonPopCallback, &startButton);
  stopButton.attachPop(stopButtonPopCallback, &stopButton);

  //if (!SD.begin(4)) {
  // Serial.println("initialization failed!");
  // return;
  //}
  //Serial.println("SD Card Initialization done.");
  /* myFile = SD.open("example.txt", FILE_WRITE);
    myFile.close(); */
}

//////////////////
////// Loop //////
//////////////////
// ===========================================
//              RUN RESPIRATOR
// ===========================================
void loop()
{

fetchPotValues();
send_to_screen_values();
delay(500);
nexLoop(nex_listen_list);
//if(set_mode == "SIMV" && start == 1)
//{
//  simv_mode();    
//}
//if(set_mode == "ACV" && start == 1)
//{
start = 1;
  
acv_mode();    
//}


    //acv_mode();
    //Serial.println(set_mode);
    //Serial.println(start);
  //if(set_mode == "ACV" && start == true)
  //{
  //    acv_mode();
  //}

  //send_to_screen_values();
  //}

  //delay(500);
  //send_to_screen_values();

  /*   send_to_screen_values();
    if (set_mode == "ACV") {
    //myFile = SD.open("example.txt", FILE_WRITE);
    acv_mode();
    //myFile.close();
    }
    else if (set_mode == "SIMV") {
    //myFile = SD.open("example.txt", FILE_WRITE);
    simv_mode();
    //myFile.close();
    } */

}
// ***************** END RUN RESPIRATOR  *******************

// ============ LAYER 1 FUNCTIONS ===============
// =================================================
//                 SIMV MODE Function
// =================================================
void simv_mode()
{
  uint32_t cycleEndTime;
  bool firstRun = true;
  int breathsInitiated = 0;
  int seperationBreaths = 0;
  float breathPercent; 
  uint32_t startTime;
  while(start == 1)
  {
    // Fetch all potentiometer values
    fetchPotValues();

    // ==== Initiate the cycle =====
    if (firstRun)
    {
      cycleEndTime = inspiration(TidVol);
      delay(15);
      expiration(TidVol, IE_ratio);
      firstRun = false;
      startTime = millis();
      
    }

    // ========= Triggered Breaths =============

    if (maskPressure < -2)
    { 
      breathsInitiated = breathsInitiated + 1;
      cycleEndTime = inspiration(TidVol);
      minuteVentilation += totVolume;
      delay(15);
      expiration(TidVol, IE_ratio);
    }
   
    // ========= Seperation Breaths =============
   
    if (millis() - cycleEndTime >= (uint32_t)separation)
    {
      cycleEndTime = inspiration(TidVol);
      minuteVentilation += totVolume;
      delay(15);
      expiration(TidVol, IE_ratio);
      seperationBreaths = seperationBreaths + 1;
    }
  
    // ======= Analytics Record every minute ==========
    if((millis() - startTime)*1000 >= 60)
    {
      // === minute ventilation ===
      minuteVentilation = minuteVentilation/(millis()-startTime)*1000*60;

      // === % of breaths initiated ===
      breathPercent = (breathsInitiated/(breathsInitiated+seperationBreaths))*100;
      breathsInitiated = 0;
      seperationBreaths = 0;
      
      // record minuteVentilation and breathPercent in sd card and wifi
      minuteVentilation = 0;
      startTime = millis();
      }

      maskPressure = pressureFromAnalog(pinMask,1000);
      diffPressure = pressureFromAnalog(pinDiff,1000); 
      //Serial.println(diffPressure);
      send_to_screen_values();
      send_to_screen_graph();
      nexLoop(nex_listen_list); 

  }
  return;
}


// ===========================================
//            ACV MODE Function
// ===========================================
void acv_mode()
{
  uint32_t cycleEndTime;
  uint32_t startTime;
  bool firstRun = true;
  int breathsInitiated = 0;
  int seperationBreaths = 0;
  float breathPercent; 
  while(start == 1)
  {
    // Fetch all potentiometer values
    fetchPotValues();
    diffSensnorCalibration();

    // ==== Initiate the cycle =====
    if (firstRun)
    {
      cycleEndTime = inspiration(TidVol);
      delay(500);
      expiration(TidVol, IE_ratio);
      firstRun = false;
      startTime = millis();
    }
    // ========= Regular Breaths =============
    if (millis() - cycleEndTime >= expTime)
    { 
      cycleEndTime = inspiration(TidVol);
      delay(500);
      expiration(TidVol, IE_ratio);
      minuteVentilation += totVolume;
      seperationBreaths = seperationBreaths + 1;

    }
    
    // ========= Triggered Breaths =============
    if (maskPressure < -2)
    {
      cycleEndTime = inspiration(TidVol);
      delay(500);
      expiration(TidVol, IE_ratio);
      breathsInitiated = breathsInitiated + 1;
      minuteVentilation += totVolume;
    }

    // ======= Analytics Record every minute ==========
    if((millis() - startTime)*1000 >= 60)
    {
      // === minute ventilation ===
      minuteVentilation = minuteVentilation/(millis()-startTime)*1000*60;
      
      // record minuteVentilation and breathPercent in sd card and wifi
      minuteVentilation = 0;
      startTime = millis();
      }

      // === % of breaths initiated ===
      breathPercent = (breathsInitiated/(breathsInitiated+seperationBreaths))*100;
      maskPressure = pressureFromAnalog(pinMask,1000);
      diffPressure = pressureFromAnalog(pinDiff,1000); 
      //send_to_screen_values();
      //send_to_screen_graph();
      //nexLoop(nex_listen_list); 
  }
  return;
}

// ============ LAYER 2 FUNCTIONS ================

// =======================
// Average Pressure Function
// =======================

float average_maskPressure()
{
  maskPressure = 0;
  for (int i = 0; i < 5; i++)
  {
    maskPressure = maskPressure + pressureFromAnalog(pinMask, 1);
    delay(3);
  }
  return (maskPressure / 5);
}


// =======================
// Inspiration Function
// =======================
 
uint32_t inspiration(float TidVol)
{ int count = 0;
  timeNow = millis();
  totVolume = 0;
  peakPressure = 0;
  peakFlow = 0;
  float posInc = 0;
  uint32_t recTime = millis();

  if (insTime>2500){ posInc = (insTime/2500)*TidVol/750;} //linear
  else { posInc = (19.62517 - 0.02755329*insTime + 0.00001542717*pow(insTime,2) - 2.891608e-9*pow(insTime,3))*TidVol/750;} 
  Serial.println(insTime);
  Serial.println(posInc);
  for (pos = 0; pos <= TidVol/5.75; pos += posInc) // goes from 0 degrees to 180 degrees
  { // in steps of 1 degree

    servo.write(pos);
    delay(0);
    // ============ Update pressure values =========
    maskPressure = pressureFromAnalog(pinMask, count);
    diffPressure = pressureFromAnalog(pinDiff, count);
    computePrintVolFlow();
    //String data = set_mode + "," + String(maskPressure) + "," + String(volFlow) + "," + String(totVolume) + ";";
    //Serial.println(set_mode + "," + String(maskPressure) + "," + String(volFlow) + "," + String(totVolume) + ";");
    //myFile.println(data);
    send_to_screen_graph();
    count++;
    // === Calculating Peak inspiratory pressure====
    if (peakPressure < maskPressure) peakPressure = maskPressure;
    // === Calculating Peak inspiratory flow====
    if (peakFlow < volFlow) peakFlow = volFlow;  
  }
  recTime = millis() - recTime;
  Serial.println(recTime);
  cycleVolume = totVolume;
  return millis();
}

// =====================
// Expiration Function
// =====================

void expiration(float TidVol, float IE_ratio)
 {
  timeNow = millis();
  for (int pos = TidVol/5.75; pos >= 0; pos -= 1) // goes from 180 degrees to 0 degrees
  {
    servo.write(pos);
    delay(7);
  }
  return;
}

// ======================================================
// Fetch Potentiometer values and adjust separation time
// ======================================================
void fetchPotValues()
{
  // Fetch all potentiometer values
  IE_ratio = map(analogRead(potpinIE_ratio), 0, 1023, 1.00, 3.00);
  TidVol = map(analogRead(potpinTidVol), 0, 1023, 300.00, 750.00);
  BPM = map(analogRead(potpinBPM), 0, 1023, 8.00, 30.00);
  insTime = (1/(1+IE_ratio))*(60/BPM)*1000;
  expTime = (IE_ratio/(1+IE_ratio))*(60/BPM)*1000;

}


// ================= LAYER 3 FUNCTIONS =============

// =============================
// Calibration Function
// =============================
void diffSensnorCalibration()

{ 
  diffPressure = pressureFromAnalog(pinDiff,1000); 

  while(diffPressure < -10 || diffPressure > 10)
  { 
      if(diffPressure > 10){constPressureDiff += 1;}
      if(diffPressure < -10){constPressureDiff -= 1;}
      diffPressure = pressureFromAnalog(pinDiff,1000); 
    }
    return;
} 


// =============================
// Pressure from Analog Function
// =============================
float pressureFromAnalog(int pin, int count)
{ float pressure;
  pressure = analogRead(pin);
  // Differential pressure sensor - output Pascal
  if (pin == pinDiff)
  {
    pressure = (pressure - constPressureDiff) * 1000 / slopePressureDiff;
    if(pressure>-10 && pressure < 10) pressure = 0;
    if (count != 1000) {
      diffPressureArr[count] = pressure;
    }
  }
  // Guage pressure sensor - output cmH20
  if (pin == pinMask)
  {
    pressure = (pressure - constPressureMask) * 10.1972 / slopePressureMask;;
    if (count != 1000) {
      maskPressureArr[count] = pressure;
    }
  }

  return pressure;
}


// =============================
// Alarm Sanity Check
// =============================

void sanityCheckBuzzer()
{
  float SDPressure;
  SDPressure = calcSD(maskPressureArr);
  //Serial.println(SDPressure);
  //Serial.println(SDPressure);
  if (SDPressure < 0.01) buzzAlarm(true);
  if (SDPressure >= 0.01) buzzAlarm(false);
}

// ================= LAYER 4 FUNCTIONS =============

// =============================
// Calculate standard deviation
// =============================
float calcSD(float data[])
{
  float avg = 0.0, SD = 0.0;
  int length = sizeof(data);

  for (int i = 0; i < length; i++)
  {
    avg += data[i];
  }
  avg = avg / length;

  for (int i = 0; i < length; i++)
  {
    SD += pow(data[i] - avg, 2);
  }
  return sqrt(SD / length);
}

// ================================================================
// Buzz Alarm if there is a problem; stop alarm if problem resolved
// ================================================================
void buzzAlarm(bool turnOn)
{
  if (turnOn == true)
  {
    ledState = HIGH;
    tone(buzzerPin, 500);
  }

  if (turnOn == false)
  {
    ledState = LOW;
    noTone(buzzerPin);
  }

}

// =======================
// Copmute Vol Flow
// =======================
void computePrintVolFlow()
{
  volFlow =  1000 * sqrt((abs(diffPressure) * 2 * rho) / ((1 / (pow(area_2, 2))) - (1 / (pow(area_1, 2))))) / rho;
  totVolume = totVolume + volFlow * (millis() - timeNow);
  timeNow = millis();
  //Serial.println(diffPressure);
}
// =======================
// Nextion Screen Functions
// =======================
void SIMVButtonPopCallback(void *ptr) {
  set_mode = "SIMV";
  String data = "page0.t_mode.txt=\"" + set_mode  + "\""; writeString(data);
}

void ACVButtonPopCallback(void *ptr) {
  set_mode = "ACV";
  String data = "page0.t_mode.txt=\"" + set_mode  + "\""; writeString(data);
}
void PSVButtonPopCallback(void *ptr) {
  set_mode = "PSV";
  String data = "page0.t_mode.txt=\"" + set_mode  + "\""; writeString(data);
}
void NoModeButtonPopCallback(void *ptr) {
  set_mode = "NoMode";
  String data = "page0.t_mode.txt=\"" + set_mode  + "\""; writeString(data);
}
void startButtonPopCallback(void *ptr) {
  start = 1;

}
void stopButtonPopCallback(void *ptr) {
  start = 0;
}


// =====================
// Transmit to DB
// =====================
void transmit() {
  Serial.println("Inside the transmit function");
  String message = "";
  boolean messageReady = false;
  while (SUART.available()) {
    message = SUART.readString();
    Serial.println(message);
    messageReady = true;
  }
  if (messageReady) {
    const int capacity_data = JSON_OBJECT_SIZE(7);
    StaticJsonBuffer<capacity_data> jb_data;
    JsonObject& doc = jb_data.createObject();
    doc["type"].set("response");
    doc["BPM"].set(BPM);
    doc["IE_ratio"].set(IE_ratio);
    doc["maskPressure"].set(maskPressure);
    doc["flowRate"].set(volFlow);
    doc["TidVol"].set(TidVol);
    doc["totVolume"].set(totVolume);
    doc.printTo(SUART);
    doc.printTo(Serial);
  }
}


// ==========================
// Print to Screen Functions
// ==========================
void send_to_screen_graph() {
    
  String to_send_p_mask = ad + id_1 + "," + ch + "," + 20;
  //Serial.println(int(map(maskPressure, -10,24,0,80)));
  print_screen(to_send_p_mask);
  String to_send_volFlow = ad + id_2 + "," + ch + "," + int(map(volFlow*100, -10,150,0.0,80.0));
  print_screen(to_send_volFlow);
  String to_send_totVolume = ad + id_3 + "," + ch + "," + int(map(totVolume, 0,800,0,80));
  print_screen(to_send_totVolume);
}

void send_to_screen_values() {
  String data = "page0.t_bpm.txt=\"" + String(BPM)  + "\""; writeString(data);
  data = "page0.t_ie_ratio.txt=\"" + String(IE_ratio)  + "\""; writeString(data);
  data = "page0.t_tidvol.txt=\"" + String(TidVol)  + "\""; writeString(data);
}

void print_screen(String to_send) {
  Serial.print(to_send);
  Serial.print( "\xFF\xFF\xFF");

}

void writeString(String stringData) {
  //Function to send commands to the Nextion display.
  for (int i = 0; i < stringData.length(); i++) {
    Serial.write(stringData[i]);
  }
  Serial.write(0xff); Serial.write(0xff); Serial.write(0xff);
}