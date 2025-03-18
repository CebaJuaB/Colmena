/*                           GPIO      
ESP32           5.0V                      
                3.3V                      
                GND                       
I2C             SDA           21          
                SCL           22
LCD             SDA              
                SCL              
                5V               
                GND              
MicroSD         CS            5 
                MOSI          23 
                CLK           18
                MISO          19
DHT22           Colmena       4 
DHT22           Exterior      15
Pulsador        Interrupción  34          
Sensor de luz   Exterior      36
Panel solar     +V
                -V
Bateria         +Bat
                -Bat                  */

#include <Arduino.h>
#include "sdkconfig.h"
#include <esp_system.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include <esp_system.h>
#include <esp_log.h>
#include <ESPping.h>
#include <esp_task_wdt.h>
#include <driver/gpio.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <uRTCLib.h>
#include <WiFi.h>
#include <PicoMQTT.h>
#include <PicoWebsocket.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "time.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#define   SEALEVELPRESSURE_HPA      (1013.25)                     // Presión atmosférica al nivel del mar
#define   SENSOR_PERIOD             24000                         // Periodo en milisegundos de lectura de sensores
#define   SD_PERIOD                 36000                         // Periodo en milisegundos de escritura a la SD
#define   MQTT_PERIOD               54000                         // Periodo en milisegundos de envio a MQTT
#define   HOST_PERIOD               48000                         // Periodo en milisegundos de POST al HOST
#define   WEB_PERIOD                72000                         // Periodo en milisegundos de POST a la Web (ThingSpeak)
#define   TIMER_PERIOD              24000                         // Periodo en milisegundos del temporizador luz LCD
#define   WDT_RST_PERIOD            9000                          // Periodo en milisegundos reinicia watchdog
#define   WDT_TIMEOUT               1200                          // Tiempo en segundos para el watchdog reinicia procesador
#define   ML_PERIOD                 120000                        // Periodo en milisegundos lazo principal antes de dormir
#define   TIME_TO_SLEEP             900                           // Tiempo en segundos que duerme el sistema
#define   uS_TO_S_FACTOR            1000000                       // Constante para convertir microsegundos a segundos
#define   LED                       GPIO_NUM_2                    // Salida para encender LED EPS32
#define   INPUT_PIN                 GPIO_NUM_34                   // Pulsador para encender luz de fondo del LCD
#define   L_PIN                     GPIO_NUM_36                   // Sensor de luz
#define   DHT_1_PIN                 GPIO_NUM_4                    // Sensor humedad y temperatura colmena         
#define   DHT_2_PIN                 GPIO_NUM_15                   // Sensor humedad y temperatura exterior
#define   DHTTYPE_1                 DHT22                         // Tipo del sensor de humedad y temperatura colmena
#define   DHTTYPE_2                 DHT22                         // Tipo del sensor de humedad y temperatura exterior
DHT               dht_1(DHT_1_PIN, DHTTYPE_1);
DHT               dht_2(DHT_2_PIN, DHTTYPE_2);
uRTCLib           rtc(0x68);                                      // Dirección I2C RTC
Adafruit_BME280   bme;                                            // Tipo sensor de presión
int lcdColumns                      = 20;
int lcdRows                         = 4;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);                 // Dirección I2C LCD
byte degree[8] = {
  0b00100,
  0b01010,
  0b10001,
  0b01010,
  0b00100,
  0b00000,
  0b00000,
  0b00000
};
const UBaseType_t Priority_1        =   10;
const UBaseType_t Priority_2        =   1;
SemaphoreHandle_t xSemaforo_Serial  =   NULL;
SemaphoreHandle_t xSemaforo_WiFi    =   NULL;
SemaphoreHandle_t xSemaforo_Datos   =   NULL;
const             IPAddress         host_mac(192,168,18,212);
/* Credenciales WokWi
const char* ssid                    = "Wokwi-GUEST";              //Simulador WokWi
const char* password                = ""; */
// Credenciales WiFi local
const char*       ssid              = "HomeNet";
const char*       password          = "Ana_Isabel_Ce";
// Parámetros POST servidor MacBook
const char*       lserverName       = "http://192.168.18.212/post-esp-data.php";
String            apiKeyValue       = "tPmAT5Ab3j7F9";
WiFiClient        lespClient;
PubSubClient      lclient(lespClient);
// Parámetros POST para Thingspeak
const char*       serverName        = "http://api.thingspeak.com/update"; 
String            apiKey            = "G0OYYV3AOQUPCMG5";                
WiFiClient        espClient;
PubSubClient      client(espClient);
// Parámetros MQTT
const char        *mqtt_broker      = "broker.emqx.io";
const char        *topic            = "emqx/esp32";
const char        *mqtt_username    = "emqx";
const char        *mqtt_password    = "public";
const int         mqtt_port         = 1883;
WiFiClient        mespClient;
PubSubClient      mclient(mespClient);
// Parámetros RTC
const char*       ntpServer         = "pool.ntp.org";             //Fuente tiempo RTC
const long        gmtOffset_sec     = -18000;                     //Colombia GMT-5
const int     daylightOffset_sec    = 0;                          //Colombia no usa DST
const char    daysOfTheWeek[7][12]  = {"Domingo", "Lunes",  "Martes", "Miércoles", "Jueves", "Viernes", "Sábado"};
// Datos que se publican en cada iteracción                                  
float       temperatura_colmena     = 0;
float       humedad_colmena         = 0;
float       temperatura_ambiente    = 0;
float       humedad_ambiente        = 0;
double      presion_atmosferica     = 0;
double      indice_luminico         = 0;
int         hora                    = 0;
int         minuto                  = 0;
int         segundo                 = 0;
int         year                    = 0;
int         month                   = 0;
int         day                     = 0;
int         dayOfWeek               = 0;
uint32_t    cpt                     = 0;                          // Contador de interrupciones del pulsador
RTC_DATA_ATTR int bootCount         = 0;                          // Number of reboots

static void IRAM_ATTR gpio_interrupt_handler(void *args){
  cpt++;
}

void autoReloadTimerCallback(TimerHandle_t xTimer){
  if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
    Serial.print("Temporizador que apaga luz de fondo del LCD\n");
    xSemaphoreGive(xSemaforo_Serial);
  }
  lcd.noBacklight();
}

void connectToWiFi() {
  WiFi.mode(WIFI_STA);                        
  WiFi.begin(ssid, password);
  vTaskDelay(3000);
  Serial.printf("Conectando a %s", ssid);
  lcd.setCursor(6, 2);
  lcd.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("ESP32 tiene el IP %s\n", WiFi.localIP().toString().c_str());
  Serial.print("IP del Gateway: ");
  Serial.println(WiFi.gatewayIP());
  Serial.print(host_mac);
  if (Ping.ping(host_mac) > 0){
    Serial.printf(" tiempo de respuesta del HOST MacBook : %d/%.2f/%d ms\n", Ping.minTime(), Ping.averageTime(), Ping.maxTime());
  } else {
    Serial.println(" Error de Ping al HOST MacBook !");
  }    
  lcd.setCursor(6, 0);
  lcd.print(WiFi.localIP().toString().c_str());
}

void reconnect() {
  while (!mclient.connected()) {
    Serial.print("Intentando re-conexión al MQTT... ");
    if (mclient.connect("ESP32Client")) {
      Serial.println("Conectado al servidor de MQTT");
    } else {
      Serial.print("falló, rc= ");
      Serial.print(mclient.state());
      Serial.println(". Probando nuevamente en 5 segundos");
      vTaskDelay(5000);
    }
  }
  mclient.subscribe("emqx/esp32");
}

void leer_rtc(){
  char buff[5];
  rtc.refresh();
  hora      = rtc.hour();
  minuto    = rtc.minute();
  segundo   = rtc.second();
  year      = rtc.year();
  month     = rtc.month();
  day       = rtc.day();
  dayOfWeek = rtc.dayOfWeek()-1;

  int decminuto = int(minuto/10);
  int uniminuto = int(minuto%10);
  
  if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
    Serial.print("Tiempo del RTC: ");
    Serial.print(year);
    Serial.print('/');
    Serial.print(month);
    Serial.print('/');
    Serial.print(day);
    Serial.print(" ");
    Serial.print(" (");
    Serial.print(daysOfTheWeek[dayOfWeek]);
    Serial.print(") ");
    Serial.print(hora);
    Serial.print(':');
    Serial.print(minuto);
    Serial.print(':');
    Serial.println(segundo);
    xSemaphoreGive(xSemaforo_Serial);
  }
  lcd.setCursor(0, 0);
  sprintf(buff, "%2d:%d%d", hora, decminuto, uniminuto);
  lcd.print(buff);
}

void display_LCD(){
  lcd.setCursor(2, 0);
  lcd.print(" ");
  lcd.setCursor(0, 1);
  lcd.print("Hive  ");
  lcd.print(F(""));
  lcd.print(humedad_colmena,0);
  lcd.print(F("%  "));
  lcd.print(temperatura_colmena,1);
  lcd.write(0);
  lcd.print(F("C"));
  lcd.setCursor(0, 2);
  lcd.print("Ext.  ");
  lcd.print(F(""));
  lcd.print(humedad_ambiente,0);
  lcd.print(F("%  "));
  lcd.print(temperatura_ambiente,1);
  lcd.write(0);
  lcd.print(F("C"));
  lcd.setCursor(0, 3);
  lcd.print(F("P "));
  lcd.print(presion_atmosferica);
  lcd.print(F("hPa"));
  lcd.setCursor(12, 3);
  lcd.print(F("L.A "));
  lcd.print(indice_luminico);
  lcd.setCursor(2, 0);
  lcd.print(":");
}

void LeerSensores(void *pvParameters){
  for (;;){
    if (xSemaphoreTake(xSemaforo_Datos, (TickType_t) 5 ) == pdTRUE){
      if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
        Serial.println("Leer Datos");
        xSemaphoreGive(xSemaforo_Serial);
      }
      digitalWrite(LED, HIGH);
      // printLocalTime();
      leer_rtc();
      temperatura_colmena       = dht_1.readTemperature();
      humedad_colmena           = dht_1.readHumidity();
      temperatura_ambiente      = dht_2.readTemperature();
      humedad_ambiente          = dht_2.readHumidity();
      presion_atmosferica       = bme.readPressure() / 100.0F;
      indice_luminico           = 1.0-analogRead(L_PIN) / 4095.0;
      xSemaphoreGive(xSemaforo_Datos);
    }
    if (isnan(temperatura_colmena) || isnan(humedad_colmena)) {
      Serial.println(F("No hay lectura del sensor de la colmena"));
    } else {
      if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
        Serial.print(F("Humedad colmena: "));
        Serial.print(humedad_colmena);
        Serial.print(F("%  Temperatura colmena: "));
        Serial.print(temperatura_colmena);
        Serial.println(F("°C"));
        xSemaphoreGive(xSemaforo_Serial);
      }
    }
    if (isnan(temperatura_ambiente) || isnan(humedad_ambiente)) {
      Serial.println(F("No hay lectura del sensor de humedad del ambiente"));
    } else {
      if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
        Serial.print(F("Humedad ambiente: "));
        Serial.print(humedad_ambiente);
        Serial.print(F("%  Temperatura externa: "));
        Serial.print(temperatura_ambiente);
        Serial.println(F("°C"));
        xSemaphoreGive(xSemaforo_Serial);
      }
    }
    if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
      Serial.print(F("Luz ambiente (normalizada entre 0 y 1): "));
      Serial.println(indice_luminico); 
      Serial.print("Presión atmosférica: ");
      Serial.print(presion_atmosferica);
      Serial.println("hPa");
      Serial.println();
      xSemaphoreGive(xSemaforo_Serial);
    }
    display_LCD();
    digitalWrite(LED, LOW);
    vTaskDelay(SENSOR_PERIOD);
  }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
    Serial.printf("Adicionando al archivo: %s\n", path);
    xSemaphoreGive(xSemaforo_Serial);
  }
  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
    if(file.print(message)){
      Serial.println("Lectura adicionada al SD");
    } else {
      Serial.println("No se pudo adicionar lectura en el SD");
    }
    xSemaphoreGive(xSemaforo_Serial);
  }
  file.close();
}

void MQTT(void *pvParameters){
  char    yearString[8];
  char    monthString[8];
  char    dayString[8];
  char    hourString[8];
  char    minuteString[8];
  char    secondString[8];
  char    tempString[8];
  char    humString[8];
  char    temp_a_String[8];
  char    hum_a_String[8];
  char    press_String[8];
  char    indice_String[8];
  char    frame[49];
  // String  trama;
  for (;;){
    mclient.loop();
    frame[0]= '\0';
    if (xSemaphoreTake(xSemaforo_Datos, (TickType_t) 5 ) == pdTRUE){
      dtostrf(year, 1, 0, yearString);
      dtostrf(month, 1, 0, monthString);
      dtostrf(day, 1, 0, dayString);
      dtostrf(hora, 1, 0, hourString);
      dtostrf(minuto, 1, 0, minuteString);
      dtostrf(segundo, 1, 0, secondString);
      dtostrf(temperatura_colmena, 1, 1, tempString);
      dtostrf(humedad_colmena, 1, 0, humString);
      dtostrf(temperatura_ambiente, 1, 1, temp_a_String);
      dtostrf(humedad_ambiente, 1, 0, hum_a_String);
      dtostrf(presion_atmosferica, 1, 2, press_String);
      dtostrf(indice_luminico, 1, 2, indice_String);
      strcat(frame, yearString);
      strcat(frame, ",");
      strcat(frame, monthString);
      strcat(frame, ",");
      strcat(frame, dayString);
      strcat(frame, ",");
      strcat(frame, hourString);
      strcat(frame, ",");
      strcat(frame, minuteString);
      strcat(frame, ",");
      strcat(frame, secondString);
      strcat(frame, ",");
      strcat(frame, tempString);
      strcat(frame, ",");
      strcat(frame, humString);
      strcat(frame, ",");
      strcat(frame, temp_a_String);
      strcat(frame, ",");
      strcat(frame, hum_a_String);
      strcat(frame, ",");
      strcat(frame, press_String);
      strcat(frame, ",");
      strcat(frame, indice_String);
      strcat(frame, "\n");
      strcat(frame, "\0");
      xSemaphoreGive(xSemaforo_Datos);
    }
    if (xSemaphoreTake(xSemaforo_WiFi, (TickType_t) 5 ) == pdTRUE){
      if (!mclient.connected()) {
        reconnect();
      }
      mclient.publish("emqx/temperatura_colmena", tempString);
      mclient.publish("emqx/humedad_colmena", humString);
      mclient.publish("emqx/temperatura_ambiente", temp_a_String);
      mclient.publish("emqx/humedad_ambiente", hum_a_String);
      mclient.publish("emqx/presion", press_String);
      mclient.publish("emqx/indice", indice_String);
      mclient.publish("emqx/trama", frame);
      if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
        Serial.print("Enviar MQTT: ");
        Serial.println(frame);
        xSemaphoreGive(xSemaforo_Serial);
      }
      xSemaphoreGive(xSemaforo_WiFi);
    }
    vTaskDelay(MQTT_PERIOD);
  }
}

void PostWeb(void *pvParameters){
  String      httpRequestData;
  int         httpResponseCode;
  WiFiClient  client;
  HTTPClient  http;
  for (;;){
      if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
        Serial.println("Enviar POST a la Web");
        xSemaphoreGive(xSemaforo_Serial);
      }
      if (xSemaphoreTake(xSemaforo_Datos, (TickType_t) 5 ) == pdTRUE){
        httpRequestData = "api_key="  + apiKey + 
                          "&field1="  + String(temperatura_colmena)   + 
                          "&field2="  + String(humedad_colmena)       + 
                          "&field4="  + String(temperatura_ambiente)  + 
                          "&field5="  + String(humedad_ambiente)      + 
                          "&field7="  + String(presion_atmosferica)   +
                          "&field8="  + String(indice_luminico);    
        xSemaphoreGive(xSemaforo_Datos);
      }
      if (xSemaphoreTake(xSemaforo_WiFi, (TickType_t) 5 ) == pdTRUE){
        http.begin(client, serverName);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        httpResponseCode = http.POST(httpRequestData);
        if (httpResponseCode>0) {
          Serial.print("Respuesta HTTP al POST del servidor remoto: ");
          Serial.println(httpResponseCode);
        } else {
          Serial.print("Código de error del servidor remoto: ");
          Serial.println(httpResponseCode);
        }
        http.end();
        xSemaphoreGive(xSemaforo_WiFi);
      }
    vTaskDelay(WEB_PERIOD);
  }
}

void PostDB(void *pvParameters){
  String      httpRequestData;
  int         httpResponseCode;
  WiFiClient  lclient;
  HTTPClient  http;
  for (;;){
      if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
        Serial.println("Enviar POST al HOST de la base de datos");
        xSemaphoreGive(xSemaforo_Serial);
      }
      if (xSemaphoreTake(xSemaforo_Datos, (TickType_t) 5 ) == pdTRUE){
        httpRequestData = "api_key="        + apiKeyValue + 
                            "&t_colmena="   + String(temperatura_colmena)       + 
                            "&h_colmena="   + String(humedad_colmena)           + 
                            "&t_ambiente="  + String(temperatura_ambiente)      + 
                            "&h_ambiente="  + String(humedad_ambiente)          + 
                            "&presion="     + String(presion_atmosferica)       +
                            "&luz="         + String(indice_luminico);        
        xSemaphoreGive(xSemaforo_Datos);
      }
      if (xSemaphoreTake(xSemaforo_WiFi, (TickType_t) 5 ) == pdTRUE){
        http.begin(lclient, lserverName);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        httpResponseCode = http.POST(httpRequestData);
        if (httpResponseCode>0) {
          Serial.print("Respuesta HTTP al POST del servidor local: ");
          Serial.println(httpResponseCode);
        } else {
          Serial.print("Código de error del servidor local: ");
          Serial.println(httpResponseCode);
        }
        http.end();
        xSemaphoreGive(xSemaforo_WiFi);
      }
    vTaskDelay(HOST_PERIOD);
  }
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);
  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }
  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Leyendo archivo: %s\n", path);
  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.println("Contenido del archivo: ");
  if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
    while(file.available()){
      Serial.write(file.read());
    }
    xSemaphoreGive(xSemaforo_Serial);
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }
  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Mensaje recibido para el topico: ");
  Serial.println(topic);
  Serial.print(". Mensaje: ");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
    messageTemp += (char)payload[i];
  }
  Serial.println();
  Serial.println("-----------------------");
  if (String(topic) == "emqx/esp32") {
    if(messageTemp == "on"){
      lcd.backlight();
      Serial.println("Prender luz de fondo del LCD ");
    } else if(messageTemp == "off"){
      lcd.noBacklight();
      Serial.println("Apagar luz de fondo del LCD ");
    } else if(messageTemp == "reboot"){
      ESP.restart();
    }
  }
}

void setMQTT(){
    mclient.setServer(mqtt_broker, mqtt_port);
    mclient.setCallback(callback);
    while (!mclient.connected()) {
      String client_id = "esp32-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("El cliente %s está conectado al MQTT broker\n", client_id.c_str());
      if (mclient.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        Serial.println("Conectado exitosamente a EMQX");
      } else {
        Serial.print("Falló con el siguiente estado ");
        Serial.print(mclient.state());
        delay(2000);
      }
    }
    mclient.publish(topic, "Colmena Altos de Cantaclaro");
    mclient.publish(topic, "Juan Bernardo Ceballos");
    mclient.publish(topic, "Latitud: 2°30'40.1 Norte   Longitud: 76°33'20.5 Oeste  Elevacion: 1820 metros");
    mclient.publish("emqx/trama", "Year, Month, Day, Hour, Minute, Second, Hive Temperature (°C), Hive Humidity (%), Ambient Temperature (°C), Ambient Humidity (%), Atmospheric presure (hPa), Luminic index (normalized)");
    mclient.subscribe(topic);
}

void setBME(){
  bool status = bme.begin(0x76);  
  if (!status) {
    Serial.println("No es un sensor BME280 válido");
    return;
  }
}

void setSD(){
  Serial.println();
  if(!SD.begin(5)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }
  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  listDir(SD, "/", 0);
  // Create a file on the SD card and write the data labels
  // deleteFile(SD, "/colmena.csv");
  File file = SD.open("/colmena.csv");
  if(!file) {
    if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
      Serial.println("El archivo colmena.csv no existe");
      xSemaphoreGive(xSemaforo_Serial);
    }
    writeFile(SD, "/colmena.csv", "Colmena Altos de Cantaclaro\n");
    appendFile(SD, "/colmena.csv", "Juan Bernardo Ceballos\n");
    appendFile(SD, "/colmena.csv", "Latitud: 2°30'40.1 Norte   Longitud: 76°33'20.5 Oeste  Elevacion: 1820 metros\n");
    appendFile(SD, "/colmena.csv",  "Year, Month, Day, Hour, Minute, Second, Hive Temperature (°C), Hive Humidity (%), Ambient Temperature (°C), Ambient Humidity (%), Atmospheric presure hPa, Luminic index (normalized) \n");
  }
  else {
    if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
      Serial.println("Archivo ya existe"); 
      xSemaphoreGive(xSemaforo_Serial);
    } 
  }
  file.close();
  readFile(SD, "/colmena.csv");
  if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
    Serial.println();
    xSemaphoreGive(xSemaforo_Serial);
  }
}

void setWDT(){
  esp_task_wdt_init(WDT_TIMEOUT, true);                                         // Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                                                       // Add current thread to WDT watch
}

void setINT(){
  gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);                               // Configure INPUT_PIN as input
  gpio_pulldown_en(INPUT_PIN);                                                  // Enable the pull-down for INPUT_PIN
  gpio_pullup_dis(INPUT_PIN);                                                   // Disable the pull-up for INPUT_PIN
  gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);                             // Interrupt triggers when state of the INPUT_PIN goes from LOW to HIGH
  gpio_install_isr_service(0);                                                  // Install  GPIO ISR service, which allows per-pin GPIO interrupt handlers
  gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);   // Configure gpio_interrupt_handler function has ISR handler for INPUT_PIN
}

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
      Serial.println("No se obtuvo el tiempo de la red");
      xSemaphoreGive(xSemaforo_Serial);
    }
    return;
  }
  if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
    Serial.print("Tiempo del ESP32: ");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
    xSemaphoreGive(xSemaforo_Serial);
  }
}

void setRTC(){
  struct tm timeinfo;
  URTCLIB_WIRE.begin();                                                           //Start RTC
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if(!getLocalTime(&timeinfo)){
    Serial.println("No se obtuvo el tiempo");
    return;
  }
  rtc.set(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour, timeinfo.tm_wday+1, timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year-100);
  printLocalTime();
}

void WriteSD(void *pvParameters){
  String      SD_Data;
  for (;;){
      if (xSemaphoreTake(xSemaforo_Datos, (TickType_t) 5 ) == pdTRUE){
        SD_Data = String(year)                + ","   +
                  String(month)               + ","   +
                  String(day)                 + ","   +
                  String(hora)                + ","   +
                  String(minuto)              + ","   +
                  String(segundo)             + ","   +
                  String(temperatura_colmena) + ","   + 
                  String(humedad_colmena)     + ","   +
                  String(temperatura_ambiente)+ ","   +
                  String(humedad_ambiente)    + ","   +
                  String(presion_atmosferica) + ","   +
                  String(indice_luminico)     + "\r\n";        
        xSemaphoreGive(xSemaforo_Datos);
      }
    appendFile(SD, "/colmena.csv", SD_Data.c_str());
    if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
      Serial.print("Adicionado a la SD");
      Serial.println(SD_Data.c_str());
      xSemaphoreGive(xSemaforo_Serial);
    }
    vTaskDelay(SD_PERIOD);
  }
}

void WDT(void *pvParameters){
  TimerHandle_t xAutoReloadTimer;
  xAutoReloadTimer = xTimerCreate("AutoReloadTimer",                            // Name of the timer
    pdMS_TO_TICKS(TIMER_PERIOD),                                                // The period of the timer specified in ticks
    pdFALSE,                                                                    // The timer will not auto-reload when it expires
    0,                                                                          // Identifier of the timer
    autoReloadTimerCallback);                                                   // Callback function
  xTimerStart(xAutoReloadTimer, 0);                                             // Temporizador para apagar luz del fondo del LCD
  for (;;){
    if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
      Serial.println("Reiniciando el temporizador del Watchdog");
      xSemaphoreGive(xSemaforo_Serial);
    }
    esp_task_wdt_reset();
    if(cpt>0){
      lcd.backlight();
      if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
        Serial.println("Interrupción recibida\n");
        xSemaphoreGive(xSemaforo_Serial);
      }
      xTimerStart(xAutoReloadTimer, 0);
      cpt=0;
      Serial.println();
      readFile(SD, "/colmena.csv");
      Serial.println(); 

    }
    if ((SD.totalBytes()*.99) <  SD.usedBytes()){
      deleteFile(SD, "/colmena.csv");
      Serial.println("SD llena, el archivo colmena.csv ha sido borrado.");
      File file = SD.open("/colmena.csv");
      writeFile(SD, "/colmena.csv", "Colmena Altos de Cantaclaro\n");
      appendFile(SD, "/colmena.csv", "Juan Bernardo Ceballos\n");
      appendFile(SD, "/colmena.csv", "Latitud: 2°30'40.1 Norte   Longitud: 76°33'20.5 Oeste  Elevacion: 1820 metros\n");
      appendFile(SD, "/colmena.csv",  "Year, Month, Day, Hour, Minute, Second, Hive Temperature (°C), Hive Humidity (%), Ambient Temperature (°C), Ambient Humidity (%), Atmospheric presure hPa, Luminic index (normalized) \n");
      readFile(SD, "/colmena.csv");
      Serial.printf("Espacio total de la SD: %lluMB\n", SD.totalBytes() / (1024 * 1024));
      Serial.printf("Espacio utilizado en la SD: %lluMB\n", SD.usedBytes() / (1024 * 1024));
      Serial.println();
      file.close();
    }
    vTaskDelay(WDT_RST_PERIOD);
  }
}

void setup(){
  Serial.begin(115200);

  lcd.init();  
  lcd.backlight();
  lcd.createChar(0, degree);
  lcd.setCursor(0, 0);
  lcd.print(" Juan Ceballos 2025");

  xSemaforo_Serial  = xSemaphoreCreateMutex();
  xSemaforo_WiFi    = xSemaphoreCreateMutex();
  xSemaforo_Datos   = xSemaphoreCreateMutex();

  TaskHandle_t  xHandleSensores   = NULL;
  TaskHandle_t  xHandleMQTT       = NULL;
  TaskHandle_t  xHandleWeb        = NULL;
  TaskHandle_t  xHandleDB         = NULL;
  TaskHandle_t  xHandleSD         = NULL;
  TaskHandle_t  xHandleWDT        = NULL;

  dht_1.begin();                                                                // Iniciar sensor humedad y temperatura colmena
  dht_2.begin();                                                                // Iniciar sensor humedad y temperatura externo
  setBME();                                                                     // Iniciar sensor de presión atmosférica
  pinMode(LED, OUTPUT);                                                         // LED azul se enciende cuando se adquieren datos
  setINT();

  connectToWiFi();
  
  setRTC();                                                                     // Obtener tiempo de la  Internet e iniciar RTC
  xTaskCreatePinnedToCore(
                LeerSensores,                                                   // Entry function of the task
                "Sensores",                                                     // Name of the task
                2048,                                                           // The number of words to allocate for use as the task's stack 
                NULL,                                                           // No parameter passed to the task
                Priority_1,                                           
                &xHandleSensores,
                0);                                                             // Task executed on core 0

  setSD();                                                                      // Iniciart tarjeta de memoria SD
  xTaskCreatePinnedToCore(
                WriteSD,                                                        // Entry function of the task
                "SD",                                                           // Name of the task
                4096,                                                           // The number of words to allocate for use as the task's stack 
                NULL,                                                           // No parameter passed to the task
                Priority_2,                                           
                &xHandleSD,
                0);                                                             // Task executed on core 0

  xTaskCreatePinnedToCore(
                PostWeb,                                                        // Entry function of the task
                "POST_WEB",                                                     // Name of the task
                4096,                                                           // The number of words to allocate for use as the task's stack 
                NULL,                                                           // No parameter passed to the task
                Priority_2,                                                     // Priority of the task
                &xHandleWeb,
                1);                                                             // Task executed on core 1
  xTaskCreatePinnedToCore(
                PostDB,                                                         // Entry function of the task
                "POST_DB",                                                      // Name of the task
                4096,                                                           // The number of words to allocate for use as the task's stack 
                NULL,                                                           // No parameter passed to the task
                Priority_2,                                                     // Priority of the task
                &xHandleDB,
                1);                                                             // Task executed on core 1

  setMQTT();                                                                    // Conectar al MQTT broker
  xTaskCreatePinnedToCore(
                MQTT,                                                           // Entry function of the task
                "MQTT",                                                         // Name of the task
                4096,                                                           // The number of words to allocate for use as the task's stack 
                NULL,                                                           // No parameter passed to the task
                Priority_2,                                                     // Priority of the task
                &xHandleMQTT,
                1);                                                             // Task executed on core 1
        
  setWDT();                                                                     // Iniciar el temporizador del Watchdog
  xTaskCreatePinnedToCore(
                WDT,                                                            // Entry function of the task
                "WDT",                                                          // Name of the task
                4096,                                                           // The number of words to allocate for use as the task's stack 
                NULL,                                                           // No parameter passed to the task
                Priority_1,                                                     // Priority of the task
                &xHandleWDT,
                0);                                                             // Task executed on core 0
                                                                       
  for (;;){
    vTaskDelay(ML_PERIOD); 
    ++bootCount;
    if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
      Serial.print("Número de reinicios del ESP32: "); 
      Serial.println(bootCount); 
      Serial.println("ESP32 entrando en sueño profundo"); 
      xSemaphoreGive(xSemaforo_Serial);
    }
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    Serial.flush();
    esp_deep_sleep_start();
  };
}

void loop(){
}