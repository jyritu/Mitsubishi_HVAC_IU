// Controls mitsubishi kirigamine HVAC system through raspberry and mosquitto mqtt server // 20.1.2018
// Uses mix of different code sources, mainly related IR handling.
// Use at your own risk.

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//WIFI Constants
const char* ssid = "ssid";
const char* password = "password";  

//MQTT constants
const char* mqttServer = "192.168.1.xxx"; //Your MQTT-server address
const int mqttPort = 1883;


//REMOTE OFF:
/*                  23    CB    26    01    00    00    08    08    C0    61    70    00   00    00    10    40     00   06
 * 
 */
 //REMOTE ON:
 /*                 23    CB    26    01    00    20    08    08    C0    61    70    00   00    00    10    40     00   26
  *                  
  */
// Initial values, that are about to change when topics call them
byte data[18] = { 0x23, 0xCB, 0x26, 0x01, 0x00, 0x20, 0x48, 0x0A, 0x30, 0x79, 0x21, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x61};
  //                0     1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    17

//MQTT Topics

const char* mqttTopicTemp = "cmnd/hvac/settemp";
const String mqttTopicMode = "cmnd/hvac/mode";
const String mqttTopicFan = "cmnd/hvac/setfan";
const String mqttTopicOn = "cmnd/hvac/setpower";
const String mqttTopicVane = "cmnd/hvac/vane";
const String mqttTopicPlama = "cmnd/hvac/plasma";
const String mqttTopicGo = "cmnd/hvac/go";


//Define and then instantiate the wifi / MQTT Client
WiFiClient client;
PubSubClient mqttClient(client);

//Global IR Declarations 

int IRpin=14; //D5
int khz=38;

int halfPeriodicTime=500/khz;


// HVAC MITSUBISHI Heavy Industries IR Mark & Space timings. These values are tweaked for my unit with AnalysIR
#define HVAC_MITSUBISHI_HI_HDR_MARK    3400
#define HVAC_MITSUBISHI_HI_HDR_SPACE   1700
#define HVAC_MITSUBISHI_HI_BIT_MARK    433
#define HVAC_MITSUBISHI_HI_ONE_SPACE   1299
#define HVAC_MISTUBISHI_HI_ZERO_SPACE  433
#define HVAC_MITSUBISHI_HI_RPT_MARK    433 
#define HVAC_MITSUBISHI_HI_RPT_SPACE   17100 // Above original iremote limit

long previousMillis = 0;
long interval = 5000;

/* callback
 * function for reading incoming message
 */
void callback(char* topic, byte* payload, unsigned int length){

  Serial.println("");
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  
  String sTopic(topic);
  String sMessy = (String)(char*)payload;
  String sMessage = sMessy.substring(0, length);
  

  Serial.println("Char array");
  Serial.println((char*)payload);
  Serial.println("Payload lenght:");
  Serial.println(length);
  Serial.print("sMessage: ");
  Serial.print(sMessage);
  Serial.println("");
  
  //Update HVAC settings based on messages in sub-routines, unless Go message received then send the IR code. 
  if(sTopic == mqttTopicOn){setSwitch(sMessage);}
  else if (sTopic == mqttTopicTemp)  { setTemp(sMessage) ;}
  else if (sTopic == mqttTopicFan)   { setFan(sMessage) ;}
  else if (sTopic == mqttTopicVane)  { setVane(sMessage) ;}
  else if (sTopic == mqttTopicMode)  { setMode(sMessage) ;}
  else if (sTopic == mqttTopicPlama) { setPlasma(sMessage) ;}
  else if (sTopic == mqttTopicGo)    { sendHvacMitsubishiHI(); }
  else {}
}

/* setSwitch
 *  function to modify IR array bytes 6 & 7 for On/off
 */
void setSwitch(String sMessage){
  // Byte 6 & 7 - On / Off and mode
  if (sMessage.toInt() == 0) {
    //Switch Off in Auto mode
    data[5] = (byte) 0x00; 
  } else {
    //Switch On in Auto mode
    data[5] = (byte) 0x20; 
  }
  }


/* setMode
 * function to modify bytes 6 & 7 of the IR array to reflect the requested mode
 */
void setMode(String sMessage){

  Serial.println();
  Serial.println("SMessage:");
  Serial.println(sMessage);
  Serial.println();

  switch (sMessage.toInt())
  {
    case 0:       data[6] = (byte) 0x20; data[7] = (byte) 0x07; data[8] = (byte) 0xC6; break; //set to fan speed 1
    case 1:       data[6] = (byte) 0x18; data[7] = (byte) 0x06; data[8] = (byte) 0xC6; break; //set to fan speed 2
    case 2:       data[6] = (byte) 0x10; data[7] = (byte) 0x08; data[8] = (byte) 0xC2; break; //set to fan speed 3
    case 3:       data[6] = (byte) 0x08; data[7] = (byte) 0x09; data[8] = (byte) 0xC0; break; //set to fan speed 4

    default: break;
  }
  }

/*  setTemp
 *  function to modify bytes 8 & 9 of the IR array to reflect the requested temp
 */
void setTemp(String sMessage){
  // Byte 8 & 9 - Temperature
  // Check Min Max For Hot Mode
  int HVAC_Temp = sMessage.toInt();
  int iTemp;
  if (HVAC_Temp > 30) { iTemp = 30;}
  else if (HVAC_Temp < 18) { iTemp = 18; } 
  else { iTemp = HVAC_Temp; };
  
  data[7] = (byte) (255 - (iTemp - 17));

  }
  

/*  setFan
 *  function to modify bytes 10 & 11 of the IR array to reflect the requested fan setting
 */
void setFan(String sMessage){
    
  // Byte 10 & 11 - FAN
  switch (sMessage.toInt())
  {
    case 0:       data[9] = (byte) 0x51; break; //set to fan speed 1
    case 1:       data[9] = (byte) 0x52; break; //set to fan speed 2
    case 2:       data[9] = (byte) 0x53; break; //set to fan speed 3
    case 3:       data[9] = (byte) 0x54; break; //set to fan speed 4

    default: break;
  }
 
  }

// Plasma function

void setPlasma(String sMessage){
    
  // Byte 15 - PLASMA
  switch (sMessage.toInt())
  {
    case 0:       data[15] = (byte) 0x00; break; //set plasma off
    case 1:       data[15] = (byte) 0x40; break; //set plasma on


    default: break;
  }
  
  }

/*  setProfile
 *  function to modify bytes 10 & 11 of the IR array to reflect the requested Profile setting(e.g. Power / Eco)setting
 *  This overwrites the Fan Setting
 */

void setVane(String sMessage){
    switch (sMessage.toInt())
  {
    case 0:       data[9] = (byte) 0x69; break;
    case 1:       data[9] = (byte) 0x61; break; 
    case 2:       data[9] = (byte) 0x59; break; 
    case 3:       data[9] = (byte) 0x51; break; 
    case 4:       data[9] = (byte) 0x49; break; 

    default: break;
  }
}

/* reconnect
 * Function to connect the MQTT client using factoryish pattern
 */
void reconnect(){

  //loop until reconnected
  while (!mqttClient.connected()){
    Serial.print("Attempting MQTT connection...");
    //attempt to connect
    if (mqttClient.connect("HVAC","","")){
      Serial.println("connected");
      //Subscribe to the settings
      mqttClient.subscribe("cmnd/hvac/settemp");
      mqttClient.subscribe("cmnd/hvac/mode");
      mqttClient.subscribe("cmnd/hvac/setfan");
      mqttClient.subscribe("cmnd/hvac/setpower");
      mqttClient.subscribe("cmnd/hvac/setprofile");
      mqttClient.subscribe("cmnd/hvac/setplasma");
      mqttClient.subscribe("cmnd/hvac/go");

      char myIpString[24];

    IPAddress myIp = WiFi.localIP();
    sprintf(myIpString, "%d.%d.%d.%d", myIp[0], myIp[1], myIp[2], myIp[3]);
  
      mqttClient.publish("cmnd/hvac/ip", myIpString);

    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" , try again in 1 seconds...");
    }
  }
}

/*
 * setup
 * Function that runs on start up
 * Used to establish Valid IR Array.
 */
void setup() {

Serial.println("Setup...");
Serial.begin(115200);
Serial.println("Serial started!");

 
delay(100);
WiFi.begin(ssid, password);
while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
delay(10);
Serial.print("WiFi connected, IP address: ");
Serial.println(WiFi.localIP());

Serial.println("Wifi Connected!");

  //Set IR PIN and PIN mode
 
pinMode(IRpin, OUTPUT);

  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);

  Serial.println("Starting IU");
}

/*
 * Continiuosly executes
 */
void loop() {

  //If no connection exists establish one.
  if (!mqttClient.connected()){
    reconnect();
  }   
  mqttClient.loop();
 
  // Non-blocking delay to slow down the temp refreshes to avoud flooding the MQTT client. 
  // To be removed once the ESP sleep is used 
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > interval){
    previousMillis = currentMillis;
    
  }

}

/*****************************************************************************************
** Function to Send IR commands to a Mitsubishi Heavy Industries Split system AC, 
** Remote model: RLA502A700B. Note IRAnalysIR reports that it is using Sanyo152AC.
*****************************************************************************************/
void sendHvacMitsubishiHI()
{
  pinMode(2, OUTPUT);
//uint8_t checksum = 0x00;

#define  HVAC_MITSUBISHI_DEBUG;  // Un comment to access DEBUG information through Serial Interface

byte mask = 1; //our bitmask
byte i;

 data[17] = 0;
  for (i = 0; i < 17; i++) {
    data[17] = (byte) data[i] + data[17];  // CRC is a simple bits addition
}
  
#ifdef HVAC_MITSUBISHI_DEBUG
  Serial.println("Packet to send: ");
  for (i = 0; i < 18; i++) {
    Serial.print("_"); Serial.print(data[i], HEX);
  }
  Serial.println("");
#endif

  enableIROut(38);  // 38khz
  space(0);
  for (int j = 0; j < 2; j++) {
  // Header for the Packet
  mark(HVAC_MITSUBISHI_HI_HDR_MARK); //3400
  space(HVAC_MITSUBISHI_HI_HDR_SPACE); //1750
  
  //loop through each hex value in the array.   
  for (i = 0; i < 18; i++) {
    
    // Send all Bits from Byte Data
    mask = 00000001;        
    //iterate through the byte by bit. 
    for(int k=0; k<8; k++){

      //Compare the byte against the target bit using the mask. 
      if (data[i] & mask) { // Bit ONE
 //       Serial.print("1");
        mark(HVAC_MITSUBISHI_HI_BIT_MARK); //450
        space(HVAC_MITSUBISHI_HI_ONE_SPACE); //1300
      }//bit one
      else { // Bit ZERO
  //      Serial.print("0");
        mark(HVAC_MITSUBISHI_HI_BIT_MARK); //450
        space(HVAC_MISTUBISHI_HI_ZERO_SPACE); //420
      }//bit zero
      
      //Shift the mask one place to the right, adding a zero to the front of the number.
      mask <<= 1;
    }//for(int k=0; k<8; k++){
//    Serial.println("");
  }//for (i = 0; i < 18; i++)
    
  // End of Packet and retransmission of the Packet
  mark(HVAC_MITSUBISHI_HI_RPT_MARK); //440
  space(HVAC_MITSUBISHI_HI_RPT_SPACE); //17100
  space(0); // Just to be sure

}// for (int j = 0; j < 2; j++)

      mqttClient.publish("cmnd/hvac/heartbeat", "1");
      delay(1000);
      mqttClient.publish("cmnd/hvac/heartbeat", "0");
      
}//sendHvacMitsubishiHI

/****************************************************************************
/* enableIROut : Set global Variable for Frequency IR Emission
/***************************************************************************/ 
void enableIROut(int khz) {
  // Enables IR output.  The khz value controls the modulation frequency in kilohertz.
  halfPeriodicTime = 500/khz; // T = 1/f but we need T/2 in microsecond and f is in kHz
}

/****************************************************************************
/* mark ( int time) 
/***************************************************************************/ 
void mark(int time) {
  // Sends an IR mark for the specified number of microseconds.
  // The mark output is modulated at the PWM frequency.
  long beginning = micros();
  while(micros() - beginning < time){
    digitalWrite(IRpin, HIGH);
    delayMicroseconds(halfPeriodicTime);
    digitalWrite(IRpin, LOW);
    delayMicroseconds(halfPeriodicTime); //38 kHz -> T = 26.31 microsec (periodic time), half of it is 13
  }
}

/****************************************************************************
/* space ( int time) 
/***************************************************************************/ 
/* Leave pin off for time (given in microseconds) */
void space(int time) {
  // Sends an IR space for the specified number of microseconds.
  // A space is no output, so the PWM output is disabled.
  digitalWrite(IRpin, LOW);
  if (time > 0) delayMicroseconds(time);
}
