// LilyGO T-Camera S3 (v1.6): live camera feed on the 128x64 SSD1306 OLED.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_camera.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "utilities.h"

#define OLED_ADDR     0x3C

#define FLIP_DISPLAY     false   // rotate OLED 180 degrees
#define MIRROR_CAMERA    false   // mirror image left/right
#define FLIP_CAMERA      false   // flip image top/bottom
#define INVERT_IMAGE     false   // swap black/white

Adafruit_SSD1306 display(128, 64, &Wire, -1);
XPowersPMU PMU;

// 4x4 ordered dither matrix (values 0..15)
static const uint8_t bayer4[4][4] = {
  { 0,  8,  2, 10},
  {12,  4, 14,  6},
  { 3, 11,  1,  9},
  {15,  7, 13,  5}
};

bool initPower() {
  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, BOARD_I2C_SDA, BOARD_I2C_SCL)) {
    Serial.println("PMU init failed");
    return false;
  }
  PMU.setALDO1Voltage(1800);  PMU.enableALDO1();   // CAM DVDD
  PMU.setALDO2Voltage(2800);  PMU.enableALDO2();   // CAM DVDD
  PMU.setALDO4Voltage(3000);  PMU.enableALDO4();   // CAM AVDD
  PMU.disableTSPinMeasure();
  delay(100);
  return true;
}

bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk  = XCLK_GPIO_NUM;
  c.pin_pclk  = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href  = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn  = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;   // 8-bit luminance
  c.frame_size   = FRAMESIZE_QQVGA;       // 160x120
  c.jpeg_quality = 12;                    // unused for grayscale, harmless
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: %s (0x%x)\n", esp_err_to_name(err), err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("Sensor PID: 0x%x\n", s->id.PID);
  s->set_hmirror(s, MIRROR_CAMERA);
  s->set_vflip(s, FLIP_CAMERA);
  return true;
}

static void showError(const char *l1, const char *l2) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(l1);
  display.println(l2);
  display.display();
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  uint32_t t = millis();
  while (!Serial && millis() - t < 3000) delay(10);
  Serial.println("Boot OK");
  Serial.printf("PSRAM: %u bytes\n", ESP.getPsramSize());

  // The PMU begin() also starts the I2C bus on SDA/SCL
  bool powerOk = initPower();
  Wire.setClock(400000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed");
    while (true) delay(1000);
  }
  display.setRotation(FLIP_DISPLAY ? 2 : 0);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Starting camera...");
  display.display();

  if (!powerOk) {
    showError("PMU init FAILED", "Camera has no power");
    while (true) delay(1000);
  }
  if (!initCamera()) {
    showError("Camera init FAILED", "See serial log");
    while (true) delay(1000);
  }
}

void loop() {
  uint32_t t0 = millis();

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Capture failed");
    return;
  }

  // Source 160x120 (4:3) -> display 128x64 (2:1).
  // Crop a centered 160x80 band, scale by 1.25 in both axes (no stretch).
  const int W     = fb->width;                // 160
  const int H     = fb->height;               // 120
  const int cropH = (W * 64) / 128;           // 80 rows
  const int yOff  = (H - cropH) / 2;          // 20

  display.clearDisplay();
  for (int y = 0; y < 64; y++) {
    int sy = yOff + ((y * W) >> 7);
    const uint8_t *row = fb->buf + sy * W;
    for (int x = 0; x < 128; x++) {
      int sx = (x * W) >> 7;
      uint8_t v = row[sx] >> 4;               // 0..15
      bool on = v > bayer4[y & 3][x & 3];
      if (INVERT_IMAGE) on = !on;
      if (on) display.drawPixel(x, y, SSD1306_WHITE);
    }
  }
  esp_camera_fb_return(fb);
  display.display();

  Serial.printf("frame time: %lu ms\n", millis() - t0);
}