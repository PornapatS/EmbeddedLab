/**
 * Bassinet Project
 * Embedded Lab Project 2019
 * Code for NETPIE 
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <MicroGear.h>

#define APPID "BassinetEmbeddedLab"
#define KEY "WMuA2ANcimOaf81"
#define SECRET "7aTnk0NArBItN6vRMupIDNNAw"
#define ALIAS "esp8266"

const char* ssid = "ttk";
const char* password = "123456789aaffee";
int timer = 0;

//Status variable
int babyStatus = 0; //2 = Died,1 = Near Death ,0 = Alive
int musicStatus = 0; //1 = On, 0 = Off

//Buffer for UART
//char *buf = "A";

WiFiClient client;
MicroGear microgear(client);

//FUNCTIONS
void publishStatus(){
  microgear.publish("/bassinet/musicstatus",String(musicStatus));
  microgear.publish("/bassinet/babystatus",String(babyStatus));
}

//void uartTransmit(String str){
//  Serial.println("Transmit str : "+str);
//  Serial.write(str);
//}

//boolean uartPolling(){
//  if (Serial.available() > 0){
//    buf = Serial.read();
//    return true;
//  }
//  return false;
//}

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen){
  Serial.print("Incoming message --> ");
  msg[msglen] = '\0';
  Serial.println((char *)msg);
  Serial.println(topic);
  if (strcmp(topic,"/BassinetEmbeddedLab/bassinet/statusrequest") == 0){
    if (strcmp((char *)msg,"A") == 0){ //Publish all status to /bassinet/...
        publishStatus();
    }
    else if (strcmp((char *)msg,"B") == 0) { //Turn on music + Publish status
      musicStatus = 1;
      publishStatus();
    }
    else if (strcmp((char *)msg,"C") == 0){ //Turn off music + Publish status
      musicStatus = 0;
      publishStatus();
    }
  }
}

void onConnected(char *attribute, uint8_t* msg, unsigned int msglen){
  Serial.println("Connecting to NETPIE...");
  microgear.setAlias(ALIAS);
  if (microgear.connected()){
    Serial.println("NETPIE Connected!");
    microgear.subscribe("/bassinet/statusrequest");
  }
}

void setup() {
  microgear.on(MESSAGE,onMsghandler);
  microgear.on(CONNECTED,onConnected);
  

  Serial.begin(115200);

  Serial.print("Initializing WIFI connection");
  for (int i = 0; i < 3; ++i){Serial.print(".");}
  Serial.println();
  
  WiFi.begin(ssid,password);
  Serial.println("Connecting to Wifi");
  
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println("Connection Failed, RECONNECTING");
  }
  Serial.println("Connecting Success!");

  //Microgear Initialize
  microgear.init(KEY,SECRET,ALIAS);
  microgear.connect(APPID);
  

  
}

void loop() {
  
//  if (uartPolling() == true){
    if (microgear.connected()){
      microgear.loop();

    }else{
      if (timer >= 3000){
        Serial.println("Connection to NETPIE lost...");
        microgear.connect(APPID);
        timer = 0;
      }else{
        timer += 100;
      }
    }
    delay(100);
//  }
}
