#include "esp_camera.h"
#include <WiFi.h>
#include <ESPmDNS.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
struct {
  const char* ssid;
  const char* pass;
} networks[] = {
  {"YOUR_WIFI_SSID", "YOUR_WIFI_PASSWORD"}, // CAMBIAR ANTES DE SUBIR
  // {"OTRA_RED", "PASSWORD"},      // Puedes agregar más redes aquí
};
#define NET_COUNT (sizeof(networks) / sizeof(networks[0]))

void startCameraServer();
void setupLedFlash();

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Desactivar detector de bajo voltaje
  
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);
    s->set_brightness(s, 1);
    s->set_saturation(s, -2);
  }
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  WiFi.setSleep(false);
  WiFi.setHostname("hermes-camera");
  
  bool connected = false;
  for (int i = 0; i < NET_COUNT; i++) {
    Serial.printf("\n[WIFI] Intentando conectar a: %s ", networks[i].ssid);
    WiFi.begin(networks[i].ssid, networks[i].pass);
    
    int retry = 20;
    while (WiFi.status() != WL_CONNECTED && retry > 0) {
      delay(500);
      Serial.print(".");
      retry--;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
      connected = true;
      Serial.println("\n[WIFI] ¡Conectado!");
      break;
    }
  }

  if (!connected) {
    Serial.println("\n[WIFI] Error. Reiniciando...");
    delay(5000);
    ESP.restart();
  }

  // Iniciar mDNS para que el GCS lo encuentre como hermes-camera.local
  if (MDNS.begin("hermes-camera")) {
    Serial.println("[MDNS] Respondinedo como hermes-camera.local");
  }

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  delay(10000);
}
