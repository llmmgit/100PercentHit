/* 100 Percent Hit ESP32 V5
   功能：保留WiFi UDP 4210/4211兼容M100，同时增加3W有线UART控制。
   重要硬件调整：LD14改为直接接Zero 3W，ESP32 UART0运行期用于3W。
   UART0: RX=GPIO32(来自3W TX), TX=GPIO1(到3W RX)
   UART1: RX=GPIO26 BNO085 UART-RVC
   UART2: RX=GPIO16/TX=GPIO17 STM32
*/
#include <WiFi.h>
#include <WiFiUdp.h>
const char* WIFI_SSID="YOUR_WIFI_NAME_HERE";const char* WIFI_PASS="YOUR_WIFI_PASSWORD_HERE";
const uint16_t CMD_PORT=4210,TELEM_PORT=4211;WiFiUDP udp;IPAddress pcIP;bool hasPC=false;
HardwareSerial STM32Serial(2),BNO(1);const int STM_RX=16,STM_TX=17,BNO_RX=26,BNO_TX=27,HOST_RX=32,HOST_TX=1;
const uint8_t STOP_FRAME[]={0xCD,0x0A,0x01,0,0,0,0,0,0,0,0,0,1};const uint32_t TIMEOUT_MS=500,STOP_REPEAT_MS=100;uint32_t lastCmd=0,lastStop=0,lastImu=0,lastAdc=0;
uint8_t rvc[19],rp=0;float yaw=0,pitch=0,roll=0,ax=0,ay=0,az=0;bool bnoOK=false;uint32_t lastRvc=0;const int BAT=34;const float RT=38700, RB=9660;
int16_t le16(const uint8_t*p){return(int16_t)(p[0]|(p[1]<<8));}bool rvcOK(const uint8_t*p){uint8_t s=0;for(int i=2;i<18;i++)s+=p[i];return s==p[18];}
void feedRvc(uint8_t b){if(rp==0){if(b==0xAA)rvc[rp++]=b;return;}if(rp==1){if(b==0xAA)rvc[rp++]=b;else rp=0;return;}rvc[rp++]=b;if(rp==19){if(rvcOK(rvc)){yaw=le16(&rvc[3])/100.0;pitch=le16(&rvc[5])/100.0;roll=le16(&rvc[7])/100.0;ax=le16(&rvc[9])/1000.0*9.80665;ay=le16(&rvc[11])/1000.0*9.80665;az=le16(&rvc[13])/1000.0*9.80665;bnoOK=true;lastRvc=millis();}rp=0;}}
void sendBoth(const uint8_t*d,size_t n){Serial.write(d,n);if(hasPC){udp.beginPacket(pcIP,TELEM_PORT);udp.write(d,n);udp.endPacket();}}
void sendText(const char*s){sendBoth((const uint8_t*)s,strlen(s));}void stopNow(){STM32Serial.write(STOP_FRAME,sizeof(STOP_FRAME));}
void handleHost(){while(Serial.available()){static uint8_t b[256];int n=Serial.readBytes(b,min(Serial.available(),255));if(n){STM32Serial.write(b,n);lastCmd=millis();}}}
void handleUdp(){int n=udp.parsePacket();while(n>0){uint8_t b[512];int k=udp.read(b,sizeof(b));if(k>0){pcIP=udp.remoteIP();hasPC=true;if(k>=4&&!memcmp(b,"PING",4))sendText("PONG\n");else{STM32Serial.write(b,k);lastCmd=millis();}}n=udp.parsePacket();}}
void setup(){Serial.begin(115200,SERIAL_8N1,HOST_RX,HOST_TX);STM32Serial.begin(115200,SERIAL_8N1,STM_RX,STM_TX);BNO.begin(115200,SERIAL_8N1,BNO_RX,BNO_TX);analogReadResolution(12);analogSetPinAttenuation(BAT,ADC_11db);WiFi.mode(WIFI_STA);WiFi.setSleep(false);WiFi.begin(WIFI_SSID,WIFI_PASS);uint32_t t=millis();while(WiFi.status()!=WL_CONNECTED&&millis()-t<15000)delay(200);udp.begin(CMD_PORT);stopNow();lastCmd=millis();}
void loop(){handleHost();handleUdp();while(BNO.available())feedRvc(BNO.read());if(bnoOK&&millis()-lastRvc>500)bnoOK=false;if(STM32Serial.available()){uint8_t b[256];int n=STM32Serial.readBytes(b,min(STM32Serial.available(),255));if(n){sendBoth((const uint8_t*)"STM:",4);sendBoth(b,n);}}uint32_t now=millis();if(now-lastCmd>TIMEOUT_MS&&now-lastStop>STOP_REPEAT_MS){lastStop=now;stopNow();}if(now-lastImu>=50){lastImu=now;char m[220];snprintf(m,sizeof(m),"IMU,ms=%lu,bno=%d,yaw=%.2f,pitch=%.2f,roll=%.2f,ax=%.5f,ay=%.5f,az=%.5f\n",now,bnoOK,yaw,pitch,roll,ax,ay,az);sendText(m);}if(now-lastAdc>=100){lastAdc=now;uint32_t mv=analogReadMilliVolts(BAT);float vb=mv/1000.0*(RT+RB)/RB;char m[100];snprintf(m,sizeof(m),"ADC,ms=%lu,mv=%lu,vbat=%.3f\n",now,mv,vb);sendText(m);}}
