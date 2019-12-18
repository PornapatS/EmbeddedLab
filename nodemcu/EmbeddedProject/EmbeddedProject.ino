/**
 * Bassinet Project
 * Embedded Lab Project 2019
 * Code for esp8266 nodeMCU
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
char buf = 'A';

WiFiClient client;
MicroGear microgear(client);

//FUNCTIONS
void publishStatus(){
  microgear.publish("/bassinet/musicstatus",String(musicStatus));
  microgear.publish("/bassinet/babystatus",String(babyStatus));
}

boolean uartPolling(){ //Recieve only "0","1","2"
  if (Serial.available() > 0 && buf == 'A'){
    buf = (char)Serial.read();
    blinkk();
    return true;
  }
  return false;
}

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen){
  msg[msglen] = '\0';
  blinkk();
  if (strcmp(topic,"/BassinetEmbeddedLab/bassinet/statusrequest") == 0){
    blinkk();
    if (strcmp((char *)msg,"A") == 0){ //Publish all status to /bassinet/...
        blinkk();
        publishStatus();
    }
    else if (strcmp((char *)msg,"B") == 0) { //Turn on music + Publish status
      musicStatus = 1;
      Serial.print('8');
      publishStatus();
    }
    else if (strcmp((char *)msg,"C") == 0){ //Turn off music + Publish status
      musicStatus = 0;
      Serial.print('9');
      publishStatus();
    }
  }
}

void onConnected(char *attribute, uint8_t* msg, unsigned int msglen){
  microgear.setAlias(ALIAS);
  if (microgear.connected()){
    digitalWrite(LED_BUILTIN,LOW);
    microgear.subscribe("/bassinet/statusrequest");
  }
}

void blinkk(){ //For debugging without UART
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  microgear.on(MESSAGE,onMsghandler);
  microgear.on(CONNECTED,onConnected);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  
  WiFi.begin(ssid,password);
  
  while (WiFi.status() != WL_CONNECTED){
    delay(1000);
  }

  //Microgear Initialize
  microgear.init(KEY,SECRET,ALIAS);
  microgear.connect(APPID); 
}

void loop() {
  if (microgear.connected()){
    microgear.loop();

    if (uartPolling() == true){
      if (buf == '0'){
        microgear.publish("/bassinet/babystatus","0");
      }
      else if (buf == '1'){
        microgear.publish("/bassinet/babystatus","1");
      }
      else if (buf == '2'){
        microgear.publish("/bassinet/babystatus","2");
      }
      else if (buf == '3'){
        microgear.publish("/bassinet/babystatus","3");
      }
      buf = 'A';
    }

  }else{
    if (timer >= 3000){
      microgear.connect(APPID);
      timer = 0;
    }else{
      timer += 100;
    }
  }
  delay(100);
}
