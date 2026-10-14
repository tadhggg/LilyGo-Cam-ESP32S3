// LilyGO T-Camera S3 (v1.6): live camera feed on the 128x64 SSD1306 OLED.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_camera.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"
#include "utilities.h"
#include "doggyBun.h"


#define OLED_ADDR     0x3C

#define FLIP_DISPLAY     false   // rotate OLED 180 degrees
#define MIRROR_CAMERA    false   // mirror image left/right
#define FLIP_CAMERA      false   // flip image top/bottom
#define INVERT_IMAGE     false   // swap black/white

Adafruit_SSD1306 display(128, 64, &Wire, -1);
XPowersPMU PMU;

// 4x4 ordered dither matrix (values 0..15)
static const uint8_t bayer4DitherPattern[4][4] = {
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
  camera_config_t camera = {};
  camera.ledc_channel = LEDC_CHANNEL_0;
  camera.ledc_timer   = LEDC_TIMER_0;
  camera.pin_d0 = Y2_GPIO_NUM;
  camera.pin_d1 = Y3_GPIO_NUM;
  camera.pin_d2 = Y4_GPIO_NUM;
  camera.pin_d3 = Y5_GPIO_NUM;
  camera.pin_d4 = Y6_GPIO_NUM;
  camera.pin_d5 = Y7_GPIO_NUM;
  camera.pin_d6 = Y8_GPIO_NUM;
  camera.pin_d7 = Y9_GPIO_NUM;
  camera.pin_xclk  = XCLK_GPIO_NUM;
  camera.pin_pclk  = PCLK_GPIO_NUM;
  camera.pin_vsync = VSYNC_GPIO_NUM;
  camera.pin_href  = HREF_GPIO_NUM;
  camera.pin_sccb_sda = SIOD_GPIO_NUM;
  camera.pin_sccb_scl = SIOC_GPIO_NUM;
  camera.pin_pwdn  = PWDN_GPIO_NUM;
  camera.pin_reset = RESET_GPIO_NUM;
  camera.xclk_freq_hz = 20000000;
  camera.pixel_format = PIXFORMAT_GRAYSCALE;   // 8-bit luminance
  camera.frame_size   = FRAMESIZE_QQVGA;       // 160x120
  camera.jpeg_quality = 12;                    // unused for grayscale, harmless
  camera.fb_count     = 2;
  camera.fb_location  = CAMERA_FB_IN_PSRAM;
  camera.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t currentError = esp_camera_init(&camera);
  if (currentError != ESP_OK) {
    Serial.printf("Camera init failed: %s (0x%x)\n", esp_err_to_name(currentError), currentError);
    return false;
  }

  sensor_t *sensor = esp_camera_sensor_get();
  Serial.printf("Sensor PID: 0x%x\n", sensor->id.PID);
  sensor->set_hmirror(sensor, MIRROR_CAMERA);
  sensor->set_vflip(sensor, FLIP_CAMERA);
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
  uint32_t currentTime = millis();
  while (!Serial && millis() - currentTime < 3000) delay(10);
  Serial.println("Boot OK");
  Serial.printf("PSRAM: %u bytes\n", ESP.getPsramSize());

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

void updatePixelRows(int frameWidth, int frameHeight, int croppedHeight, int heightOffset, camera_fb_t *frameBuffer)
{
  for (int currentRow = 0; currentRow < 64; currentRow++ )
  {
    int originalRow = heightOffset + ((frameWidth * currentRow) >> 7);
    const uint8_t *currentRowPointer =  frameBuffer->buf + originalRow * frameWidth;

    for(int currentColumn = 0; currentColumn < 128; currentColumn++)
    {
      int originalColumn = ((currentColumn * frameWidth) >> 7);

      uint8_t pixelBrightness = currentRowPointer[originalColumn] >> 4;

      bool lightUpPixel = pixelBrightness > bayer4DitherPattern[currentRow & 3][currentColumn & 3];

      if(INVERT_IMAGE){
        lightUpPixel = !lightUpPixel;
      }

      if(lightUpPixel)
      {
        display.drawPixel(currentColumn, currentRow, SSD1306_WHITE);
      }

    }
  }
}

void loop() {
  uint32_t currentLoopTime = millis();

  camera_fb_t *frameBuffer = esp_camera_fb_get();
  if (!frameBuffer) {
    Serial.println("Capture failed");
    return;
  }

  // Source 160x120 (4:3) -> display 128x64 (2:1).
  // Crop a centered 160x80 band, scale by 1.25 in both axes (no stretch).
  const int frameWidth     = frameBuffer->width;                // 160
  const int frameHeight     = frameBuffer->height;               // 120
  const int croppedHeight = (frameWidth * 64) / 128;           // 80 rows
  const int heightOffset  = (frameHeight - croppedHeight) / 2;          // 20

  display.clearDisplay();
  updatePixelRows(frameWidth, frameHeight, croppedHeight, heightOffset, frameBuffer);
  
  

  esp_camera_fb_return(frameBuffer);

  int spriteX = 40;
  int spriteY = 20;
  display.drawBitmap(spriteX, spriteY, dogyBun_data,       32, 32, SSD1306_WHITE);
  display.drawBitmap(spriteX, spriteY, dogyBun_black_data, 32, 32, SSD1306_BLACK);
  display.display();

  Serial.printf("frame time: %lu ms\n", millis() - currentLoopTime);
}