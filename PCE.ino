#include <Arduino.h>
#include "LoRaWan-Arduino.h"
#include <SPI.h>
#include <stdio.h>

#include "mbed.h"
#include "rtos.h"

using namespace std::chrono_literals;
using namespace std::chrono;

// LoRaWAN handlers
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void lorawan_unconf_finished(void);
static void lorawan_conf_finished(bool result);
static void send_lora_frame(void);

// OTAA keys and settings
bool doOTAA = true;
#define LORAWAN_DATERATE DR_0
#define LORAWAN_TX_POWER TX_POWER_5
#define JOINREQ_NBTRIALS 3
#define LORAWAN_APP_DATA_BUFF_SIZE 64
#define LORAWAN_APP_INTERVAL 60000  // 60 seconds

uint8_t nodeDeviceEUI[8] = {0x60, 0x81, 0xF9, 0x17, 0x5E, 0x8B};
uint8_t nodeAppEUI[8] = {0x60, 0x81, 0xF9, 0xA1, 0x8A, 0xEB, 0x66, 0xCB};
uint8_t nodeAppKey[16] = {0x45, 0x72, 0x24, 0x6F, , 0xD4};

DeviceClass_t g_CurrentClass = CLASS_A;
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_EU868;
lmh_confirm g_CurrentConfirm = LMH_UNCONFIRMED_MSG;
uint8_t gAppPort = 2;

static lmh_param_t g_lora_param_init = {
  LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK,
  JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF
};
static lmh_callback_t g_lora_callbacks = {
  BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
  lorawan_rx_handler, lorawan_has_joined_handler,
  lorawan_confirm_class_handler, lorawan_join_failed_handler,
  lorawan_unconf_finished, lorawan_conf_finished
};

static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0};

mbed::Ticker appTimer;
void tx_lora_periodic_handler(void);
bool send_now = false;

#define SENSOR_POWER_PIN WB_IO1
#define ANEMOMETER_PIN WB_IO2

volatile int interruptCounter = 0;
volatile uint32_t lastInterruptTime = 0;

unsigned long lastWindMeasure = 0;
const unsigned long windMeasureInterval = 10000; // 10 seconds

float windSpeedKmh = 0;
float windSpeedSum = 0;
float maxWindSpeed = 0;
int windSpeedSamples = 0;

unsigned long lastResetTime = 0;
const unsigned long resetInterval = 60000; // 1 minute

uint16_t readBatteryVoltage() {
  analogReadResolution(12);
  uint16_t raw = analogRead(WB_A0);
  float voltage = (raw / 4095.0) * 3.3 * 2.0;
  return (uint16_t)(voltage * 100);
}

void countup() {
  uint32_t currentMillis = millis();
  if (currentMillis - lastInterruptTime > 50) {
    interruptCounter++;
    lastInterruptTime = currentMillis;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  pinMode(ANEMOMETER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ANEMOMETER_PIN), countup, CHANGE);

  Serial.println("RAK11300 LoRaWAN - Wind Speed Monitor (m/s)");

  lora_rak11300_init();
  if (doOTAA) {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }

  uint32_t err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  if (err_code != 0) {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }
  lmh_join();
}

void loop() {
  if (millis() - lastWindMeasure >= windMeasureInterval) {
    lastWindMeasure = millis();

    noInterrupts();
    int currentInterrupts = interruptCounter;
    interruptCounter = 0;
    interrupts();

    float windHz = currentInterrupts / 10.0;
    windSpeedKmh = currentInterrupts == 0 ? 0 : (windHz * 0.8) + 3;

    if (windSpeedKmh > 0) {
      windSpeedSum += windSpeedKmh;
      if (windSpeedKmh > maxWindSpeed) maxWindSpeed = windSpeedKmh;
      windSpeedSamples++;
    }

    float windSpeedMs = windSpeedKmh / 3.6;
    float avgMs = windSpeedSamples > 0 ? (windSpeedSum / windSpeedSamples) / 3.6 : 0;
    float maxMs = maxWindSpeed / 3.6;

    Serial.printf("Current: %.2f m/s | Avg: %.2f m/s | Max: %.2f m/s\n", windSpeedMs, avgMs, maxMs);
  }

  if (millis() - lastResetTime >= resetInterval) {
    lastResetTime = millis();
    send_now = true;
  }

  if (send_now) {
    send_now = false;
    send_lora_frame();
  }

  delay(100);
}

void send_lora_frame() {
  if (lmh_join_status_get() != LMH_SET) return;

  float avgWindSpeedMs = windSpeedSamples > 0 ? (windSpeedSum / windSpeedSamples) / 3.6 : 0;
  float maxWindSpeedMs = maxWindSpeed / 3.6;

  Serial.printf(">>> 1-min Avg: %.2f m/s | Max: %.2f m/s | Battery: %.2f V\n",
    avgWindSpeedMs, maxWindSpeedMs, readBatteryVoltage() / 100.0);

  uint16_t avgEnc = (uint16_t)(avgWindSpeedMs * 100);
  uint16_t maxEnc = (uint16_t)(maxWindSpeedMs * 100);
  uint16_t battery = readBatteryVoltage();

  m_lora_app_data.buffer[0] = avgEnc >> 8;
  m_lora_app_data.buffer[1] = avgEnc & 0xFF;
  m_lora_app_data.buffer[2] = maxEnc >> 8;
  m_lora_app_data.buffer[3] = maxEnc & 0xFF;
  m_lora_app_data.buffer[4] = battery >> 8;
  m_lora_app_data.buffer[5] = battery & 0xFF;

  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffsize = 6;

  windSpeedSum = 0;
  windSpeedSamples = 0;
  maxWindSpeed = 0;

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
  if (error == LMH_SUCCESS) Serial.println("lmh_send OK");
  else Serial.println("lmh_send FAIL");
}

void tx_lora_periodic_handler(void) {
  appTimer.attach(tx_lora_periodic_handler, (std::chrono::microseconds)(LORAWAN_APP_INTERVAL * 1000));
  send_now = true;
}

void lorawan_has_joined_handler(void) {
  Serial.println("LoRaWAN Joined!");
  if (lmh_class_request(g_CurrentClass) == LMH_SUCCESS) {
    delay(1000);
    appTimer.attach(tx_lora_periodic_handler, (std::chrono::microseconds)(LORAWAN_APP_INTERVAL * 1000));
  }
}

void lorawan_join_failed_handler(void) {
  Serial.println("Join failed! Check keys and coverage.");
}

void lorawan_rx_handler(lmh_app_data_t *app_data) {
  Serial.printf("RX on port %d, size:%d, rssi:%d, snr:%d\n",
    app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);
}

void lorawan_confirm_class_handler(DeviceClass_t Class) {
  Serial.printf("Switched to Class %c\n", "ABC"[Class]);
}

void lorawan_unconf_finished(void) {
  Serial.println("Unconfirmed TX complete");
}

void lorawan_conf_finished(bool result) {
  Serial.printf("Confirmed TX %s\n", result ? "success" : "fail");
}
