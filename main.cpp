#include <Arduino.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_log.h>
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
#include "time.h"
#include <PicoMQTT.h>
#include <PicoWebsocket.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ESPping.h>

#define   SEALEVELPRESSURE_HPA      (1013.25)                     // Presión atmosférica al nivel del mar

#define   L_PIN                     36                            // LED
#define   PUSH_BUTTON_PIN           34                            // Boton de control

#define   DHT_1_PIN                 4
#define   DHT_2_PIN                 5
#define   DHTTYPE_1                 DHT22
#define   DHTTYPE_2                 DHT11

DHT                                 dht_1(DHT_1_PIN, DHTTYPE_1);
DHT                                 dht_2(DHT_2_PIN, DHTTYPE_2);

uRTCLib                             rtc(0x68);                    //Dirección I2C RTC
Adafruit_BME280                     bme;
gpio_num_t    pin                   = GPIO_NUM_2;                 //Pin conectado al LED
LiquidCrystal_I2C                   lcd(0x27, 20, 4);             //Dirección I2C LCD

const         IPAddress             host_mac(192,168,18,212);
// const char* ssid                  = "Wokwi-GUEST";              //Simulador WokWi
// const char* password              = ""; 
// Credenciales WiFi local
const char*   ssid                  = "HomeNet";
const char*   password              = "Ana_Isabel_Ce";

// Parámetros POST servidor MacBook
const char*   lserverName           = "http://192.168.18.212/post-esp-data.php";
String        apiKeyValue           = "tPmAT5Ab3j7F9";
WiFiClient    lespClient;
PubSubClient  lclient(lespClient);

// Parámetros POST para Thingspeak
const char*   serverName            = "http://api.thingspeak.com/update"; 
String        apiKey                = "G0OYYV3AOQUPCMG5";                
WiFiClient    espClient;
PubSubClient  client(espClient);

// Parámetros MQTT
const char    *mqtt_broker          = "broker.emqx.io";
const char    *topic                = "emqx/esp32";
const char    *mqtt_username        = "emqx";
const char    *mqtt_password        = "public";
const int     mqtt_port             = 1883;
WiFiClient    mespClient;
PubSubClient  mclient(mespClient);

const char*   ntpServer             = "pool.ntp.org";             //Fuente tiempo RTC

// Parámetros RTC
const long    gmtOffset_sec         = -18000;                     //Colombia GMT-5
const int     daylightOffset_sec    = 0;                          //Colombia no usa DST
const char    daysOfTheWeek[7][12]  = {
  "Domingo", 
  "Lunes", 
  "Martes", 
  "Miércoles", 
  "Jueves", 
  "Viernes", 
  "Sábado"
};

const unsigned long timerDelay        = 20000;                               // Tiempo envio paquetes en milisegundos
const unsigned long LCDdelay          =  4000;                               // Tiempo actualización LCD, en milisegundos
unsigned long lastTime                =     0;                               // Para enviar POST

// Control de la luz del fondo del LCD
int           buttonState               = 1;                                    // Botón de control LCD
int           backlightLCD              = 1;                                    // Estado del backlight LCD

// Datos que se publican en cada iteracción                                  
float         temperatura_colmena       = 0;
float         humedad_colmena           = 0;
float         temperatura_ambiente      = 0;
float         humedad_ambiente          = 0;
double        presion_atmosferica       = 0;
double        indice_luminico           = 0;

// Function Declarations
void        connectToWiFi();
void        setRTC();
void        setBME();
void        setLED();
void        setMQTT();

void setup() {
  lcd.init();  
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(" Juan Ceballos 2025");

  Serial.begin(115200);

  connectToWiFi();
  setRTC();                         // Obtain time from Internet and starts RTC
  setMQTT();                        // Connect to MQTT broker

  dht_1.begin();
  dht_2.begin();
  setBME();                         // Set up pressure sensor
  setLED();
  pinMode(PUSH_BUTTON_PIN, INPUT);  // Set up button for display´s backlight
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
  mclient.publish(topic, "Hola, soy el ESP32 de Juan Ceballos");
  mclient.subscribe(topic);
}

void reconnect() {
  while (!mclient.connected()) {
    Serial.print("Intentando re-conexión al MQTT... ");
    if (mclient.connect("ESP32Client")) {
      Serial.println("Conectado");
    } else {
      Serial.print("falló, rc= ");
      Serial.print(mclient.state());
      Serial.println(". Probando nuevamente en 5 segundos");
      delay(5000);
    }
  }
  mclient.subscribe("emqx/esp32");
}

void send_mqtt(float temperatura_colmena, float humedad_colmena, float temperatura_ambiente, float humedad_ambiente, float presion, float indice){
  char    tempString[8];
  char    humString[8];
  char    temp_a_String[8];
  char    hum_a_String[8];
  char    press_String[8];
  char    indice_String[8];

  if (!mclient.connected()) {
    reconnect();
  }

  dtostrf(temperatura_colmena, 1, 2, tempString);
  dtostrf(humedad_colmena, 1, 2, humString);
  dtostrf(temperatura_ambiente, 1, 2, temp_a_String);
  dtostrf(humedad_ambiente, 1, 2, hum_a_String);
  dtostrf(presion, 1, 2, press_String);
  dtostrf(indice, 1, 2, indice_String);
  mclient.publish("emqx/temperatura_colmena", tempString);
  mclient.publish("emqx/humedad_colmena", humString);
  mclient.publish("emqx/temperatura_ambiente", temp_a_String);
  mclient.publish("emqx/humedad_ambiente", hum_a_String);
  mclient.publish("emqx/presion", press_String);
  mclient.publish("emqx/indice", indice_String);
}

void setLED(){
  gpio_config_t config;
    config.pin_bit_mask = (1ULL << pin);
    config.mode         = GPIO_MODE_OUTPUT;
    config.pull_up_en   = GPIO_PULLUP_DISABLE;
    config.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&config);     
}

void setBME(){
  bool status = bme.begin(0x76);  
  if (!status) {
    Serial.println("Not a valid BME280 sensor");
    return;
  }
}

void connectToWiFi() {
  WiFi.mode(WIFI_STA);      //WiFi in station mode
  WiFi.begin(ssid, password);
  delay(3000);
  Serial.printf("Conectando a %s", ssid);
  lcd.setCursor(6, 2);
  lcd.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
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

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No se obtuvo el tiempo de la red");
    return;
  }
  Serial.print("Tiempo del ESP32: ");
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void setRTC(){
  struct tm timeinfo;
  URTCLIB_WIRE.begin();     //Start RTC
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  rtc.set(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour, timeinfo.tm_wday+1, timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year-100);
  printLocalTime();
}

void leer_rtc(){
  char buff[5];
  rtc.refresh();
  int hora=rtc.hour();
  int minuto=rtc.minute();
  int decminuto=int(minuto/10);
  int uniminuto=int(minuto%10);
  Serial.print("Tiempo del RTC: ");
  Serial.print(rtc.year());
  Serial.print('/');
  Serial.print(rtc.month());
  Serial.print('/');
  Serial.print(rtc.day());
  Serial.print(" ");
  Serial.print(" (");
  Serial.print(daysOfTheWeek[rtc.dayOfWeek()-1]);
  Serial.print(") ");
  Serial.print(hora);
  Serial.print(':');
  Serial.print(minuto);
  Serial.print(':');
  Serial.println(rtc.second());
  Serial.print("Temperatura RTC: ");
  Serial.print(rtc.temp()  / 100);
  Serial.println("°C");
  lcd.setCursor(0, 0);
  sprintf(buff, "%2d:%d%d", hora, decminuto, uniminuto);
  lcd.print(buff);
}

void prender_LED(){
    gpio_set_level(pin, 1);
    lcd.setCursor(2, 0);
    lcd.print(":");
}

void apagar_LED(){
    gpio_set_level(pin, 0);
    lcd.setCursor(2, 0);
    lcd.print(" ");
}

void leer_boton(){
  buttonState = digitalRead(PUSH_BUTTON_PIN);
  if (buttonState == HIGH) {
    lcd.backlight();
    Serial.println("Luz del despliegue LCD encendida");
  } else {
    lcd.noBacklight();
  }
}

void enviar_post(float temperatura_colmena, float humedad_colmena, float temperatura_ambiente, float humedad_ambiente, float presion, float indice){
  WiFiClient client;
  HTTPClient http;
  http.begin(client, serverName);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String httpRequestData = "api_key="   + apiKey + 
                            "&field1="  + String(temperatura_colmena)   + 
                            "&field2="  + String(humedad_colmena)       + 
                            "&field4="  + String(temperatura_ambiente)  + 
                            "&field5="  + String(humedad_ambiente)      + 
                            "&field7="  + String(presion_atmosferica)   +
                            "&field8="  + String(indice_luminico);          
  int httpResponseCode = http.POST(httpRequestData);
  if (httpResponseCode>0) {
    Serial.print("Respuesta HTTP al POST del servidor remoto: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Código de error del servidor remoto: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

void guardar_db(float temperatura_colmena, float humedad_colmena, float temperatura_ambiente, float humedad_ambiente, float presion, float indice){
  WiFiClient lclient;
  HTTPClient http;

  http.begin(lclient,lserverName);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String httpRequestData = "api_key="       + apiKeyValue                       + 
                            "&t_colmena="   + String(temperatura_colmena)       + 
                            "&h_colmena="   + String(humedad_colmena)           + 
                            "&t_ambiente="  + String(temperatura_ambiente)      + 
                            "&h_ambiente="  + String(humedad_ambiente)          + 
                            "&presion="     + String(presion)                   +
                            "&luz="         + String(indice);          
  int httpResponseCode = http.POST(httpRequestData);
  if (httpResponseCode>0) {
    Serial.print("Respuesta HTTP al POST del servidor local: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("Código de error del servidor local: ");
    Serial.println(httpResponseCode);
  }
  http.end();
}

void diplay_LCD(float temperatura_colmena, float humedad_colmena, float temperatura_ambiente, float humedad_ambiente, float presion, float indice){
  lcd.setCursor(0, 1);
  lcd.print("Hive  ");
  lcd.print(F(""));
  lcd.print(humedad_colmena);
  lcd.print(F("%  "));
  lcd.print(temperatura_colmena);
  lcd.print(F("C"));

  lcd.setCursor(0, 2);
  lcd.print("Ext.  ");
  lcd.print(F(""));
  lcd.print(humedad_ambiente);
  lcd.print(F("%  "));
  lcd.print(temperatura_ambiente);
  lcd.print(F("C"));

  lcd.setCursor(0, 3);
  lcd.print(F("P "));
  lcd.print(presion);
  lcd.print(F("hPa"));

  lcd.setCursor(12, 3);
  lcd.print(F("L.A "));
  lcd.print(indice);
}

void loop(){
  if ((millis() - lastTime) > timerDelay) {
    leer_boton();
    printLocalTime();
    prender_LED();
    leer_rtc();
    temperatura_colmena       = dht_1.readTemperature();
    humedad_colmena           = dht_1.readHumidity();
    temperatura_ambiente      = dht_2.readTemperature();
    humedad_ambiente          = dht_2.readHumidity();
    presion_atmosferica       = bme.readPressure() / 100.0F;
    indice_luminico           = 1.0-analogRead(L_PIN) / 4095.0;

    if (isnan(temperatura_colmena) || isnan(humedad_colmena)) {
      Serial.println(F("No hay lectura del sensor de la colmena"));
    } else {
      Serial.print(F("Humedad colmena: "));
      Serial.print(humedad_colmena);
      Serial.print(F("%  Temperatura colmena: "));
      Serial.print(temperatura_colmena);
      Serial.println(F("°C"));
    }

    if (isnan(temperatura_ambiente) || isnan(humedad_ambiente)) {
      Serial.println(F("No hay lectura del sensor ambiental"));
    } else {
      Serial.print(F("Humedad ambiente: "));
      Serial.print(humedad_ambiente);
      Serial.print(F("%  Temperatura externa: "));
      Serial.print(temperatura_ambiente);
      Serial.println(F("°C"));
    }

    Serial.print(F("Luz ambiente (normalizada entre 0 y 1): "));
    Serial.println(indice_luminico); 

    Serial.print("Presión atmosférica: ");
    Serial.print(presion_atmosferica);
    Serial.println("hPa");
  
    diplay_LCD(temperatura_colmena,  humedad_colmena, temperatura_ambiente, humedad_ambiente, presion_atmosferica, indice_luminico);

    if(WiFi.status()== WL_CONNECTED){
      send_mqtt(temperatura_colmena,  humedad_colmena, temperatura_ambiente, humedad_ambiente, presion_atmosferica, indice_luminico);
      enviar_post(temperatura_colmena,  humedad_colmena, temperatura_ambiente, humedad_ambiente, presion_atmosferica, indice_luminico);
      guardar_db(temperatura_colmena,  humedad_colmena, temperatura_ambiente, humedad_ambiente, presion_atmosferica, indice_luminico);
    }
    else {
      Serial.println("WiFi desconectado");
    }

    Serial.println();
    lastTime = millis();
    apagar_LED();
  }

  mclient.loop(); 

  vTaskDelay(LCDdelay / portTICK_PERIOD_MS);
  leer_boton();
  vTaskDelay(LCDdelay / portTICK_PERIOD_MS);
} 