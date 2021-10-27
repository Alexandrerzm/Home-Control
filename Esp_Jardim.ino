#define BLYNK_TEMPLATE_ID           "TMPLpDYaYBJO"
#define BLYNK_DEVICE_NAME           "Smart Home"
#define BLYNK_AUTH_TOKEN            "b3aEvqrSvVhR9xIAzbT48-On0R83UvfM"

#include <esp_now.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" 
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include "RTClib.h"

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "TP-Link_9360";
char pass[] = "97005476";

RTC_DS1307 rtc;
BlynkTimer timer;
WidgetTerminal terminal(V4);

uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0xEF, 0x58, 0x68};

QueueHandle_t xQueue_liga;
QueueHandle_t xQueue_blynk;
QueueHandle_t xQueue_receiver;
QueueHandle_t xQueue_sender;

void vTask1( void *pvParameters );
void vTask2( void *pvParameters );
void vTask3( void *pvParameters );

BLYNK_WRITE(V0)
{
  char liga;
  char ligar;
  int value = param.asInt();
  if (value == 1)
  {
    liga = 1;
  } else
  {
    liga = 0;
  }
  xQueueOverwrite( xQueue_liga, &liga);
  xQueuePeek( xQueue_blynk, &ligar, 0);
    if (ligar == 1 || ligar == 2)
  {
    Blynk.virtualWrite(V0, 1);
  }else
  {
    Blynk.virtualWrite(V0, 0);
  }
  // atribui o valor de ligar ao pino v0
} 

BLYNK_WRITE(V1)
{
  char liga;
  char ligar;
  int value = param.asInt();
  if (value == 1)
  {
    liga = 2;
  } else
  {
    liga = 3;
  }
  xQueueOverwrite( xQueue_liga, &liga);
  xQueuePeek( xQueue_blynk, &ligar, 0);
  if (ligar == 0 || ligar == 1)
  {
    Blynk.virtualWrite(V1, 0);
  }else
  {
    Blynk.virtualWrite(V1, 1);
  }
  // atribui o valor de ligar ao pino v1
}

BLYNK_WRITE(V4)
{
  char ligado;
  xQueuePeek( xQueue_receiver, &ligado, 0 );

  if (ligado == 0) 
  {
    terminal.println("Sistema desligado.") ;
  } 
  else if (ligado >= 1 && ligado <= 3) 
  {
    terminal.println("Irrigando o Pomar...");
  }
  else if (ligado >= 4 && ligado <= 10) 
  {
    terminal.println("Irrigando o Jardim...");
  }
  else if (ligado == 11) 
  {
    terminal.println("Sistema pausado...");
  }
  else
  {
    terminal.print("ERRO!!! Ligado = ");
    terminal.println(ligado);
  }
  terminal.flush();
}
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char ligado;
  memcpy(&ligado, incomingData, sizeof(ligado));
  xQueueOverwrite( xQueue_receiver, &ligado);
}
 
void setup() {
  pinMode(25, OUTPUT);  // válvula 1
  pinMode(26, OUTPUT); // válvula 2
  pinMode(27, OUTPUT); // válvula 3
  pinMode(14, INPUT);  // sensor do portão
  pinMode(12, INPUT);  // sensor de chuva
  digitalWrite(25, LOW);  // válvula 1
  digitalWrite(26, LOW); // válvula 2
  digitalWrite(27, LOW); // válvula 3
  Serial.begin(9600);
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0; 
  peerInfo.encrypt = false;   
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
  }
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
  }
  xQueue_liga = xQueueCreate( 1, sizeof( char ) );
  xQueue_blynk = xQueueCreate( 1, sizeof( char ) );
  xQueue_receiver = xQueueCreate( 1, sizeof( char ) );
  xQueue_sender = xQueueCreate( 1, sizeof( char ) );

  xTaskCreatePinnedToCore( vTask1, "Task 1", 10000, NULL, 3, NULL, 0 );  
  xTaskCreatePinnedToCore( vTask2, "Task 2", 10000, NULL, 2, NULL, 0 );
  xTaskCreatePinnedToCore( vTask3, "Task 3", 10000, NULL, 2, NULL, 0 );
}

void loop() {
  Blynk.run();
  timer.run();
}

void vTask1( void *pvParameters )
{
  char liga;
  for(;;)
  {
    xQueuePeek( xQueue_sender, &liga, 0);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &liga, sizeof(liga));  
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      vTaskDelay( 500 / portTICK_PERIOD_MS );
    }
    else {
      Serial.println("Error sending the data");
      vTaskDelay( 50 / portTICK_PERIOD_MS );
    }    
  }
}

void vTask2( void *pvParameters )
{
  char ligado;
  for(;;)
  {
    xQueuePeek( xQueue_receiver, &ligado, 0);
    switch (ligado)
    {
      case 4:
        digitalWrite(25, HIGH);  // válvula 1
        digitalWrite(26, HIGH); // válvula 2
        digitalWrite(27, HIGH); // válvula 3
        break;
      case 5:
        digitalWrite(25, LOW);  // válvula 1
        digitalWrite(26, HIGH); // válvula 2
        digitalWrite(27, HIGH); // válvula 3
        break;
      case 6:
        digitalWrite(25, HIGH);  // válvula 1
        digitalWrite(26, LOW); // válvula 2
        digitalWrite(27, HIGH); // válvula 3
        break;
      case 7:
        digitalWrite(25, HIGH);  // válvula 1
        digitalWrite(26, HIGH); // válvula 2
        digitalWrite(27, LOW); // válvula 3
        break;
      case 8:
        digitalWrite(25, LOW);  // válvula 1
        digitalWrite(26, LOW); // válvula 2
        digitalWrite(27, HIGH); // válvula 3
        break;
      case 9:
        digitalWrite(25, LOW);  // válvula 1
        digitalWrite(26, HIGH); // válvula 2
        digitalWrite(27, LOW); // válvula 3
        break;
      case 10:
        digitalWrite(25, HIGH);  // válvula 1
        digitalWrite(26, LOW); // válvula 2
        digitalWrite(27, LOW); // válvula 3
        break;
      default :
        digitalWrite(25, LOW);  // válvula 1
        digitalWrite(26, LOW); // válvula 2
        digitalWrite(27, LOW); // válvula 3     
    }
    vTaskDelay( 500 / portTICK_PERIOD_MS );
  }
}

void vTask3( void *pvParameters )
{
  int timeControl;
  char liga;
  int init_Time;
  int ligado;
  char ligar;
  int portao;
  int chuva;
  int count;
  char blynk_Send;

  for(;;)
  {
    ligado = 0;
    portao = digitalRead(14);
    chuva = digitalRead(12);
    xQueuePeek( xQueue_liga, &ligar, 0);

    DateTime now = rtc.now();
    //now.unixtime();
    timeControl = now.unixtime() % 86400;
    if (timeControl  > 38900 && timeControl < 47300 && chuva == 0)
    {
      ligar = 1;
    }

    if (ligar == 1 && ligado == 0)
    {
      ligado == 1;
      init_Time = now.unixtime();
      while (ligado == 1)
      {
        if (now.unixtime() - init_Time < 1200)
        {
          liga = 1;
          blynk_Send = 1;
        }
        else if (now.unixtime() - init_Time >= 1200 && now.unixtime() - init_Time  < 2400)
        {
          liga = 2;
        }
        else if (now.unixtime() - init_Time >= 2400 && now.unixtime() - init_Time  < 3600)
        {
          liga = 3;
        }
        else if (now.unixtime() - init_Time >= 3600 && now.unixtime() - init_Time  < 4200)
        {
          liga = 4;
          blynk_Send = 2;
        }
        else if (now.unixtime() - init_Time >= 4200 && now.unixtime() - init_Time  < 4800)
        {
          liga = 5;
        }
        else if (now.unixtime() - init_Time >= 4800 && now.unixtime() - init_Time  < 5400)
        {
          liga = 6;
        }
        else if (now.unixtime() - init_Time >= 5400 && now.unixtime() - init_Time  < 6000)
        {
          liga = 7;
        }
        else if (now.unixtime() - init_Time >= 6000 && now.unixtime() - init_Time  < 6600)
        {
          liga = 8;
        }
        else if (now.unixtime() - init_Time >= 6600 && now.unixtime() - init_Time  < 7200)
        {
          liga = 9;
        }
        else if (now.unixtime() - init_Time >= 7200 && now.unixtime() - init_Time  < 7800)
        {
          liga = 10;
        }
        else
        {
          liga = 0;
          ligado = 0;
          blynk_Send = 0;
        }
        xQueuePeek( xQueue_liga, &ligar, 0);
        portao = digitalRead(14);
        if (ligar ==  0)
        {
          liga = 0;
          ligado = 0;
          blynk_Send = 0;
        }
        if ((ligar == 2 || portao == 0) && blynk_Send == 2)
        {
          liga = 11;
          blynk_Send = 1;
          xQueueOverwrite( xQueue_sender, &liga);
          xQueueOverwrite( xQueue_blynk, &blynk_Send);
          count = 0;
          while (count < 240)
          {
            xQueuePeek( xQueue_liga, &ligar, 0);
            if (ligar == 3) break;
            vTaskDelay( 500 / portTICK_PERIOD_MS );
            count++;
          }
          blynk_Send = 2;  
        }
        xQueueOverwrite( xQueue_blynk, &blynk_Send);
        xQueueOverwrite( xQueue_sender, &liga);
        vTaskDelay( 500 / portTICK_PERIOD_MS );
      }  
    }
    vTaskDelay( 500 / portTICK_PERIOD_MS );
  }
}
