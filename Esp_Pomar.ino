#include <esp_now.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h" 

QueueHandle_t xQueue_receiver;
QueueHandle_t xQueue_sender;

void vTask1( void *pvParameters );
void vTask2( void *pvParameters );

uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0xEF, 0x58, 0x68};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  char liga;
  memcpy(&liga, incomingData, sizeof(liga));
  xQueueOverwrite( xQueue_receiver, &liga);
}
 
void setup() {
  pinMode(25, OUTPUT); // válvula 1
  pinMode(26, OUTPUT); // válvula 2
  pinMode(27, OUTPUT); // válvula 3
  pinMode(14, OUTPUT); // bomba
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
  esp_now_register_recv_cb(OnDataRecv);
  xQueue_sender = xQueueCreate( 1, sizeof( char ) );
  xQueue_receiver = xQueueCreate( 1, sizeof( char ) ); 
  xTaskCreatePinnedToCore( vTask1, "Task 1", 10000, NULL, 2, NULL, 0 );
  xTaskCreatePinnedToCore( vTask2, "Task 2", 10000, NULL, 3, NULL, 0 );  
}

void vTask1( void *pvParameters )
{
  char ligado;
  char liga;
  for(;;)
  {
    xQueuePeek( xQueue_receiver, &liga, 0 );
    if(liga == 1)
    {
      digitalWrite(25, HIGH);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digitalWrite(14, HIGH);
      digitalWrite(26, LOW);
      digitalWrite(27, LOW);
      ligado = liga;    
    }
    else if(liga == 2)
    {
      digitalWrite(26, HIGH);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digitalWrite(14, HIGH);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digitalWrite(25, LOW);
      digitalWrite(27, LOW);
      ligado = liga;    
    }
    else if(liga == 3)
    {
      digitalWrite(27, HIGH);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digitalWrite(14, HIGH);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digitalWrite(25, LOW);
      digitalWrite(26, LOW);
      ligado = liga;    
    }
    else if(liga >= 4 && liga <= 10)
    {
      digitalWrite(14, HIGH);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digitalWrite(25, LOW);
      digitalWrite(26, LOW);
      digitalWrite(27, LOW);
      ligado = liga;    
    }
    else if(liga == 11)
    {
      digitalWrite(14, HIGH);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digitalWrite(25, HIGH);
      digitalWrite(26, HIGH);
      digitalWrite(27, HIGH);
      xQueueOverwrite( xQueue_sender, &ligado);
      ligado = liga;   
    }
    else
    {
      digitalWrite(14, LOW);
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digitalWrite(25, LOW);
      digitalWrite(26, LOW);
      digitalWrite(27, LOW);
      ligado = 0;
    }
    xQueueOverwrite( xQueue_sender, &ligado);
    vTaskDelay( 500 / portTICK_PERIOD_MS );
  }
}

void vTask2( void *pvParameters )
{
  char ligado;
  for(;;)
  {
    xQueuePeek( xQueue_sender, &ligado, 0);
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &ligado, sizeof(ligado));  
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

void loop() {
  vTaskDelay( 500 / portTICK_PERIOD_MS );
}