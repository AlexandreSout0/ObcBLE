#include <Arduino.h>
#include <iostream>
#include <string.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_task_wdt.h"

#define SERVICE_UUID "0716bf69-27fa-44bd-b636-4ab49725c6b0"
#define PACOTE_UUID  "0716bf69-27fa-44bd-b636-4ab49725c6b1"
#define RX_UUID      "4ac8a682-9736-4e5d-932b-e9b31405049c"


#define BAUD_RATE   9600
#define TX    GPIO_NUM_1
#define RX    GPIO_NUM_3 
#define UART  UART_NUM_0 
#define BUF_SIZE    1024

#define ED1   GPIO_NUM_5
#define ED2   GPIO_NUM_18
#define ED3   GPIO_NUM_19
#define ED4   GPIO_NUM_21
#define RPM   GPIO_NUM_34
#define PULSE GPIO_NUM_35

volatile int pulse_count = 0;


String checksum(String data);
String char_to_hex(char x);
unsigned int readRPM(gpio_num_t rpm);
unsigned int readPulse(gpio_num_t pulse);
static void IRAM_ATTR pulse_isr_handler(void* arg);
bool analogDigital(gpio_num_t ed);
void inicioBLE();
void uart_init(int baudRate, int tx_io_num, int rx_io_num, uart_port_t uart_num);
void send_data_over_ble(String package);


xQueueHandle QueuePackages;

struct obc_frame 
{
  unsigned int rpm ;
  unsigned int digital1;
  unsigned int digital2;
  unsigned int digital3;
  unsigned int digital4;
  unsigned int pulse1;
}frame = {1,0,0,0,0,0};

BLEServer *server = nullptr; //Ponteiro para uma variável tipo BLEserver
BLECharacteristic *pacote = nullptr; //Ponteiro para caracteristicas do serviço do periferico
BLECharacteristic *pacote_rx = nullptr; //Ponteiro para caracteristicas do serviço do periferico
BLEService *service = nullptr; //Ponteiro para variável tipo BLEService


void send_data_over_ble(String package)
{
    if(false)//!deviceConnected
    {
      return;
    }

    //std::string data(buff.c_str(), buff.length());
    pacote -> setValue(package.c_str());
    pacote -> notify();
}

void uart_init(int baudRate, int tx_io_num, int rx_io_num, uart_port_t uart_num)
{
    #if CONFIG_UART_ISR_IN_IRAM
      intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    uart_config_t uart_config = 
    {
      .baud_rate = baudRate,
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
   int intr_alloc_flags = 0;
   uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags);
   uart_param_config(uart_num, &uart_config);
   uart_set_pin(uart_num, tx_io_num, rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

}


//callback  para envendos das características
class CharacteristicCallbacks: public BLECharacteristicCallbacks 
{
    std::string buffer = "";

    void onWrite(BLECharacteristic *pacote_rx) 
    {
      //retorna ponteiro para o registrador contendo o valor atual da caracteristica
      std::string rxValue = pacote_rx->getValue(); 
      //verifica se existe dados (tamanho maior que zero)
      buffer += rxValue.c_str();
      std::string rxValueCheck = "!";
      rxValueCheck.c_str();

      if (rxValue == rxValueCheck)
      {
        uart_write_bytes(UART, (const char *) buffer.c_str(), strlen(buffer.c_str()));
      }

      if(buffer == "$POK!")
      {
         uart_write_bytes(UART, (const char *) "HandShake", strlen("HandShake"));
      }
  }

};

class ServerCallbacks: public BLEServerCallbacks // Classe para herdar os serviços de callback BLE
{
  void onConnect(BLEServer *s)
  {
    BLEDevice::startAdvertising(); // Mesmo que esteja alguém conectado o Advertinsing é chamado novamente e permite conecções com outros dispositivos simultaneos
    uart_write_bytes(UART, (const char *) "Device Connected\n", strlen("Device Connected\n"));
  }

  void onDisconnect(BLEServer *s)
  {
    uart_write_bytes(UART, (const char *) "Device Disconnected\n", strlen("Device Disconnected\n"));
    service -> stop();
  }
};


void inicioBLE()
{
  
  BLEDevice::init("OBC"); //inicio o dispositivo/Periferico
  server = BLEDevice::createServer(); //crio um servidor e coloco seu endereço no ponteiro server
  // Callback BLE
  server -> setCallbacks(new ServerCallbacks()); // cria uma instancia do serviço de callback
  // Serviços do Periferico BLE
  service = server -> createService(SERVICE_UUID); //crio um serviço com o UUID e guardo seu endereço no ponteiro
  pacote = service -> createCharacteristic(PACOTE_UUID,BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);   // Habilita a assinatura do serviço para receber alteraçoes de pacote
  pacote_rx = service -> createCharacteristic(RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_READ);  // Create a BLE Characteristic para recebimento de dados
  pacote_rx -> setCallbacks(new CharacteristicCallbacks());
  service -> start(); // inicia o serviço
  // Criação do Advertising para poder ser descoberto
  BLEAdvertising *advertising = nullptr; 
  advertising = BLEDevice::getAdvertising(); // Crio um advertising e coloco seu endereço no ponteiro advertising
  advertising -> addServiceUUID(SERVICE_UUID);
  advertising -> setScanResponse(false); //Configurações de Advertising
  advertising -> setMinPreferred(0x06); //Configurações de Advertising
  BLEDevice::startAdvertising(); //inicia o Advertising  
}

bool analogDigital(gpio_num_t ed)
{
    
    bool edState;
    gpio_pad_select_gpio(ed); //Configuração do pino
    gpio_set_direction(ed, GPIO_MODE_OUTPUT); //Direção do pino
    edState = gpio_get_level(ed);
    return edState;
}


static void IRAM_ATTR pulse_isr_handler(void* arg)
{
    pulse_count++;
}

unsigned int readPulse(gpio_num_t pulse)
{

    gpio_set_direction(pulse, GPIO_MODE_INPUT);
    gpio_set_intr_type(pulse, GPIO_INTR_POSEDGE);
    gpio_pulldown_dis(pulse);
    gpio_pullup_dis(pulse);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(pulse , pulse_isr_handler, (void*) pulse);

    int current_pulse_count = pulse_count;

    return current_pulse_count;
}

unsigned int readRPM(gpio_num_t rpm)
{

    gpio_set_direction(rpm, GPIO_MODE_INPUT);
    gpio_pulldown_dis(rpm);
    gpio_pullup_dis(rpm);

    uint32_t start = xthal_get_ccount();
    while (gpio_get_level(rpm) == 0) {}    // Espera até que o sinal mude para alto
    while (gpio_get_level(GPIO_NUM_0) == 1) {} // Conta o tempo até que o sinal mude para baixo novamente
    uint32_t end = xthal_get_ccount();
    float duration = (end - start) / (float)configTICK_RATE_HZ; // Calcula a frequência do sinal como 1 / duração
    float frequency = 1.0 / duration;
    int freq = static_cast<int>(frequency); // cast de float para int

    return freq;
}

String char_to_hex(char x){
    char hex[3];
    sprintf(hex,"%02X",x);
    hex[2] = '\0';
    return (String)hex;
}

String checksum(String data){
    String result;
    char csum = data[0] ^ data[1];
    for (int i = 2; i != data.length(); i++){
        csum = csum ^ data[i];
    }
    result = data +  "," + char_to_hex(csum);
    return result;
}

void readSignals(void * params)
{
  while(true)
  {

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

}


void setup() 
{
  inicioBLE();
  uart_init(BAUD_RATE, TX, RX, UART);
  uart_write_bytes(UART, (const char *) "Solinftec - OBC \n", strlen("Solinftec - OBC \n"));
  xTaskCreate(&readSignals, "estado da porta", 2048, NULL, 1, NULL);
}

__attribute__((unused)) void loop()
{
    vTaskDelay(100 / portTICK_PERIOD_MS);
    esp_task_wdt_reset();
}




