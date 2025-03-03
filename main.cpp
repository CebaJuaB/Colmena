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

#define   SEALEVELPRESSURE_HPA      (1013.25)                     // Presión atmosférica al nivel del mar
#define   WDT_TIMEOUT               600                           // Tiempo en segundos para el watchdog
#define   L_PIN                     36                            // Sensor de luz
#define   LED                       GPIO_NUM_2
#define   DHT_1_PIN                 4
#define   DHT_2_PIN                 5
#define   DHTTYPE_1                 DHT22
#define   DHTTYPE_2                 DHT11
#define   INPUT_PIN                 GPIO_NUM_34

#define   SENSOR_PERIOD             60000                           // Periodo en milisegundos de lectura de sensores
#define   MQTT_PERIOD               360000                          // Periodo en milisegundos de envio a MQTT
#define   HOST_PERIOD               480000                          // Periodo en milisegundos de POST al HOST
#define   WEB_PERIOD                240000                          // Periodo en milisegundos de POST a la Web (ThingSpeak)
#define   WDT_RST_PERIOD            600000                          // Periodo en milisegundos reinicia watchdog

const UBaseType_t Priority_1        = 1;
const UBaseType_t Priority_2        = 2;
      uint32_t    cpt               = 0;
SemaphoreHandle_t xSemaforo_Serial  = NULL;
SemaphoreHandle_t xSemaforo_WiFi    = NULL;
SemaphoreHandle_t xSemaforo_Datos   = NULL;

DHT                                 dht_1(DHT_1_PIN, DHTTYPE_1);
DHT                                 dht_2(DHT_2_PIN, DHTTYPE_2);

uRTCLib                             rtc(0x68);                    //Dirección I2C RTC
Adafruit_BME280                     bme;
LiquidCrystal_I2C                   lcd(0x27, 20, 4);             //Dirección I2C LCD

const             IPAddress         host_mac(192,168,18,212);
// const char* ssid                  = "Wokwi-GUEST";              //Simulador WokWi
// const char* password              = ""; 
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
float         temperatura_colmena   = 0;
float         humedad_colmena       = 0;
float         temperatura_ambiente  = 0;
float         humedad_ambiente      = 0;
double        presion_atmosferica   = 0;
double        indice_luminico       = 0;
int           hora                  = 0;
int           minuto                = 0;
int           segundo               = 0;
int           year                  = 0;
int           month                 = 0;
int           day                   = 0;
int           dayOfWeek             = 0;

// Function Declarations
void        connectToWiFi();
void        setRTC();
void        setBME();
void        setMQTT();
void        printLocalTime();
void        leer_rtc();

static void IRAM_ATTR gpio_interrupt_handler(void *args){
    // Interrupt is raised, increase the counter
    cpt++;
}


void autoReloadTimerCallback(TimerHandle_t xTimer){
  Serial.print("Timer callback\n");
}

void diplay_LCD(){
  lcd.setCursor(2, 0);
  lcd.print(" ");
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
    if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
      Serial.println("Leer Datos");
      xSemaphoreGive(xSemaforo_Serial);
    }
    digitalWrite(LED, HIGH);
    printLocalTime();
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
      Serial.println(F("No hay lectura del sensor de humedad del ambiente"));
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
    Serial.println();
    diplay_LCD();
    digitalWrite(LED, LOW);
    vTaskDelay(SENSOR_PERIOD);
  }
}

void MQTT(void *pvParameters){
  for (;;){
    /*
    temperatura_colmena       = dht_1.readTemperature();
    humedad_colmena           = dht_1.readHumidity();
    temperatura_ambiente      = dht_2.readTemperature();
    humedad_ambiente          = dht_2.readHumidity();
    presion_atmosferica       = bme.readPressure() / 100.0F;
    indice_luminico           = 1.0-analogRead(L_PIN) / 4095.0;
    */
    if (xSemaphoreTake(xSemaforo_Serial, (TickType_t) 5 ) == pdTRUE){
      Serial.println("Enviar MQTT");
      xSemaphoreGive(xSemaforo_Serial);
    }
    vTaskDelay(MQTT_PERIOD);
  }
}

void setup(){
  Serial.begin(115200);
  lcd.init();  
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(" Juan Ceballos 2025");

  xSemaforo_Serial  = xSemaphoreCreateMutex();
  xSemaforo_WiFi    = xSemaphoreCreateMutex();
  xSemaforo_Datos   = xSemaphoreCreateMutex();

  connectToWiFi();
  setRTC();                                       // Obtain time from Internet and starts RTC
  setMQTT();                                      // Connect to MQTT broker

  dht_1.begin();
  dht_2.begin();
  setBME();                                       // Set up pressure sensor
  pinMode(LED, OUTPUT);
  // pinMode(PUSH_BUTTON_PIN, INPUT);                // Set up button for display´s backlight

  Serial.println("Configurando temporizador del Watchdog");   // Watchdog timer
  esp_task_wdt_init(WDT_TIMEOUT, true);                       // Enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                                     // Add current thread to WDT watch

  TaskHandle_t  xHandleSensores   = NULL;
  TaskHandle_t  xHandle2          = NULL;

  xTaskCreatePinnedToCore(
                LeerSensores,   // Entry function of the task
                "Sensores",     // Name of the task
                10000,          // The number of words to allocate for use as the task's stack (arbitrary size enough for this task)
                NULL,           // No parameter passed to the task
                Priority_2,                                           
                &xHandleSensores,
                0);             // Task executed on core 1
  xTaskCreatePinnedToCore(
                MQTT,           // Entry function of the task
                "Task2",        // Name of the task
                10000,          // The number of words to allocate for use as the task's stack (arbitrary size enough for this task)
                NULL,           // No parameter passed to the task
                Priority_1,      // Priority of the task
                &xHandle2,
                1);             // Task executed on core 0
  /*
  gpio_set_direction(INPUT_PIN, GPIO_MODE_INPUT);                               // Configure INPUT_PIN as input
  gpio_pulldown_en(INPUT_PIN);                                                  // Enable the pull-down for INPUT_PIN
  gpio_pullup_dis(INPUT_PIN);                                                   // Disable the pull-up for INPUT_PIN
  gpio_set_intr_type(INPUT_PIN, GPIO_INTR_POSEDGE);                             // Interrupt triggers when state of the INPUT_PIN goes from LOW to HIGH
  gpio_install_isr_service(0);                                                  // Install  GPIO ISR service, which allows per-pin GPIO interrupt handlers
  gpio_isr_handler_add(INPUT_PIN, gpio_interrupt_handler, (void *)INPUT_PIN);   // Configure gpio_interrupt_handler function has ISR handler for INPUT_PIN
  
  TimerHandle_t xAutoReloadTimer;
  
  xAutoReloadTimer = xTimerCreate("AutoReloadTimer",          // Name of the timer
                                    pdMS_TO_TICKS(60000),     // The period of the timer specified in ticks
                                    pdTRUE,                   // The timer will auto-reload when it expires
                                    0,                        // Identifier of the timer
                                    autoReloadTimerCallback); // Callback function
  */
  // xTimerStart(xAutoReloadTimer, 0);
  for (;;){
    Serial.println("Reiniciando el temporizador del Watchdog");
    esp_task_wdt_reset();
    vTaskDelay(WDT_RST_PERIOD);  
  };
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

void connectToWiFi() {
  WiFi.mode(WIFI_STA);                        
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

void setBME(){
  bool status = bme.begin(0x76);  
  if (!status) {
    Serial.println("No es un sensor BME280 válido");
    return;
  }
}

void setRTC(){
  struct tm timeinfo;
  URTCLIB_WIRE.begin();                   //Start RTC
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if(!getLocalTime(&timeinfo)){
    Serial.println("No se obtuvo el tiempo");
    return;
  }
  rtc.set(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour, timeinfo.tm_wday+1, timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year-100);
  printLocalTime();
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

void loop(){
}