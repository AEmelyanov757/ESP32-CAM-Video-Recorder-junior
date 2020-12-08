/*
 Проект видео/ловушка на базе платы AI-Thinker ESP32-CAM с видео-чипом OV2640, с трансляцией видео/потока(MJPEG по HTTP) по Wi-Fi.
 Версия 11.0 с доплеровским датчиком движения на модуле RCWL-0516 (аналог HW-MS03).

 Данный код разработан для ID Arduino ver. 1.8.10(и выше), платформа(менеджер плат) ESP32 ver. 1.0.4, необходимые установки для ESP32-CAM:
    - Плата: "ESP32 Wrover Module"
    - Схема разделов: "Huge APP (3MB No OTA)"

 Пины: 
 GPIO12 - сигнальный для датчика движения: "Высокий уровень" - движение, "Низкий уровень" - нет движения. Подробности см. в "Схеме проекта".

  Ссылки на источники:
    - Первичная ветка, ставшая основой проекта:
      https://github.com/jameszah/ESP32-CAM-Video-Recorder-junior.git
    - Постоянный репозиторий проекта:
      https://github.com/AEmelyanov757/ESP32-CAM-Video-Recorder-junior.git
    - Мануалы разработчиков чипа, платы, платформы:
      https://docs.espressif.com/projects/esp-idf/en/latest/index.html - интерактивный справочник по ESP32 от компании Espressif.
      https://wiki.ai-thinker.com/esp32-cam - интерактивный справочник по ESP32-CAM от компании AI-Thinker (китайский язык).
      https://www.arduino.cc/reference/en/ - интерактивный справочник по Arduino от Arduino AG.
    - Cерия статей от Руи Сантос (Rui Santos) и компании студентов про основы программирования ESP32-CAM в среде Arduino:
      https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/ - про порты вводы/вывода.
      https://randomnerdtutorials.com/esp32-touch-pins-arduino-ide/ - про особенность сенсорных кнопок ESP32.
      https://makeradvisor.com/esp32-cam-ov2640-camera/ - вводная статья о ESP32-CAM.
      https://randomnerdtutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/ - организация Wi-Fi видое стриминга.
      https://randomnerdtutorials.com/esp32-cam-video-streaming-face-recognition-arduino-ide/ - стриминг и распознование лиц.
      https://randomnerdtutorials.com/esp32-cam-take-photo-save-microsd-card/ - сохранение фото на SD-карты, преодоление проблемы "4 Гбайт".
    - Полезные репозитории кода на GitHub по ESP32-CAM:
      https://github.com/espressif/esp32-camera - репозиторий от Espressif.
      https://github.com/donny681/ESP32_CAMERA_QR - репозиторий кода распознования QR-кода (можно организовать чтение имени и пароля Wi-Fi "на лету").
      https://github.com/raphaelbs/esp32-cam-ai-thinker - репозиторий кода Wi-Fi стрим камеры.
      https://github.com/ArduCAM/ArduCAM_ESP32S_UNO - бибилиотека Arduino Camera, адаптированный для ESP32. Есть пример кода записи видео (320x240) на SD-карту.
      https://github.com/dproldan/Esp32AutoCamera - репозиторий кода Wi-Fi стрим камеры.
      https://github.com/tsaarni/esp32-micropython-webcam - репозиторий кода Wi-Fi стрим камеры на MicroPython (первый шаг в сторону OpenMV).
      https://github.com/openmv/openmv - репозиторий кода библиотеки машинного зрения OpenMV. Требуется произвести адаптацию быстрых математически функций. 
      https://github.com/jameszah/ESP32-CAM-Video-Recorder.git - таймламп видео рекордер.
*/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// user edits here:
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

static const char vernum[] = "v11";
static const char devname[] = "esp32cam";         // name of your camera for mDNS, Router, and filenames

#define IncludeInternet 1               // if you want internet/wifi, change the to 1, and put in your wifi name/pass             
const char* ssid = "SSID";
const char* password = "password";

// https://sites.google.com/a/usapiens.com/opnode/time-zones  -- find your timezone here
#define TIMEZONE "UTC-12:00" // Временная зона Asia/Kamchatka

// two configurations :
int c1_framesize = 7;                // SVGA
int c1_quality = 10;                 // quality on the 1..63 scale  - lower is better quality and bigger files - must be higher than the jpeg_quality in camera_config
int c1_avi_length = 180;             // how long a movie in seconds -- 180 sec = 3 min

int c2_framesize = 5;                // CIF
int c2_quality = 1;                  // quality on the 1..63 scale  - lower is better quality and bigger files - must be higher than the jpeg_quality in camera_config
int c2_avi_length = 180;             // how long a movie in seconds -- 180 sec = 3 min

int c1_or_c2     = 1;
int c1_or_c2_now = c1_or_c2;
int framesize = c1_framesize;
int quality = c1_quality;
int avi_length = c1_avi_length;

int MagicNumber = 100;                // change this number to reset the eprom in your esp32 for file numbers

float most_recent_fps = 0;
int most_recent_avg_framesize = 0;

uint8_t* framebuffer;
int framebuffer_len;

uint8_t framebuffer_static[64 * 1024 + 20];
int framebuffer_len_static;

//#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_camera.h"

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

camera_fb_t * fb_curr = NULL;
camera_fb_t * fb_next = NULL;

#include <stdio.h>

//#include <sys/types.h>
//#include <sys/stat.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "soc/soc.h"
#include "soc/cpu.h"
#include "soc/rtc_cntl_reg.h"

typedef struct{
        size_t size; //number of values used for filtering
        size_t index; //current value index
        size_t count; //value count
        int sum;
        int * values; //array to be filled with values
} ra_filter_t;

static esp_err_t cam_err;

int first = 1;
long frame_start = 0;
long frame_end = 0;
long frame_total = 0;
long frame_average = 0;
long loop_average = 0;
long loop_total = 0;
long total_frame_data = 0;
long last_frame_length = 0;
int done = 0;
long avi_start_time = 0;
long avi_end_time = 0;
int moveStop = 0;

int we_are_already_stopped = 0;
long total_delay = 0;
long bytes_before_last_100_frames = 0;
long time_before_last_100_frames = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  Avi Writer Stuff here
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// MicroSD
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
#include <SD_MMC.h>

FILE *avifile = NULL;
FILE *idxfile = NULL;

long bp;
long ap;
long bw;
long aw;

int diskspeed = 0;

char fname[100];
char pname[100];
char strftime_buf[64];

static int i = 0;
uint8_t temp = 0, temp_last = 0;
unsigned long fileposition = 0;
uint16_t frame_cnt = 0;
uint16_t frame_mov = 0;
uint16_t remnant = 0;
uint32_t length = 0;
uint32_t startms;
uint32_t elapsedms;
uint32_t uVideoLen = 0;
bool is_header = false;
int bad_jpg = 0;
int extend_jpg = 0;
int normal_jpg = 0;

int file_number = 0;
int file_group = 0;
long boot_time = 0;

long totalp;
long totalw;
float avgp;
float avgw;

#define BUFFSIZE 512

uint8_t buf[BUFFSIZE];

#define AVIOFFSET 240 // AVI main header length

unsigned long movi_size = 0;
unsigned long jpeg_size = 0;
unsigned long idx_offset = 0;

uint8_t zero_buf[4] = {0x00, 0x00, 0x00, 0x00};
uint8_t dc_buf[4] = {0x30, 0x30, 0x64, 0x63};    // "00dc"
uint8_t dc_and_zero_buf[8] = {0x30, 0x30, 0x64, 0x63, 0x00, 0x00, 0x00, 0x00};

uint8_t avi1_buf[4] = {0x41, 0x56, 0x49, 0x31};    // "AVI1"
uint8_t idx1_buf[4] = {0x69, 0x64, 0x78, 0x31};    // "idx1"

uint8_t  vga_w[2] = {0x80, 0x02}; // 640
uint8_t  vga_h[2] = {0xE0, 0x01}; // 480
uint8_t  cif_w[2] = {0x90, 0x01}; // 400
uint8_t  cif_h[2] = {0x28, 0x01}; // 296
uint8_t svga_w[2] = {0x20, 0x03}; // 800
uint8_t svga_h[2] = {0x58, 0x02}; // 600
uint8_t sxga_w[2] = {0x00, 0x05}; // 1280
uint8_t sxga_h[2] = {0x00, 0x04}; // 1024
uint8_t uxga_w[2] = {0x40, 0x06}; // 1600
uint8_t uxga_h[2] = {0xB0, 0x04}; // 1200

const int avi_header[AVIOFFSET] PROGMEM = {
  0x52, 0x49, 0x46, 0x46, 0xD8, 0x01, 0x0E, 0x00, 0x41, 0x56, 0x49, 0x20, 0x4C, 0x49, 0x53, 0x54,
  0xD0, 0x00, 0x00, 0x00, 0x68, 0x64, 0x72, 0x6C, 0x61, 0x76, 0x69, 0x68, 0x38, 0x00, 0x00, 0x00,
  0xA0, 0x86, 0x01, 0x00, 0x80, 0x66, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4C, 0x49, 0x53, 0x54, 0x84, 0x00, 0x00, 0x00,
  0x73, 0x74, 0x72, 0x6C, 0x73, 0x74, 0x72, 0x68, 0x30, 0x00, 0x00, 0x00, 0x76, 0x69, 0x64, 0x73,
  0x4D, 0x4A, 0x50, 0x47, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x73, 0x74, 0x72, 0x66,
  0x28, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x80, 0x02, 0x00, 0x00, 0xe0, 0x01, 0x00, 0x00,
  0x01, 0x00, 0x18, 0x00, 0x4D, 0x4A, 0x50, 0x47, 0x00, 0x84, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0x4E, 0x46, 0x4F,
  0x10, 0x00, 0x00, 0x00, 0x6A, 0x61, 0x6D, 0x65, 0x73, 0x7A, 0x61, 0x68, 0x61, 0x72, 0x79, 0x20,
  0x76, 0x31, 0x30, 0x20, 0x4C, 0x49, 0x53, 0x54, 0x00, 0x01, 0x0E, 0x00, 0x6D, 0x6F, 0x76, 0x69,
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Time
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include "time.h"
time_t now;
struct tm timeinfo;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Writes an uint32_t in Big Endian at current file position
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static void inline print_quartet(unsigned long i, FILE * fd)
{
  uint8_t y[4];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  size_t i1_err = fwrite(y , 1, 4, fd);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Writes 2 uint32_t in Big Endian at current file position
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static void inline print_2quartet(unsigned long i, unsigned long j, FILE * fd)
{
  uint8_t y[8];
  y[0] = i % 0x100;
  y[1] = (i >> 8) % 0x100;
  y[2] = (i >> 16) % 0x100;
  y[3] = (i >> 24) % 0x100;
  y[4] = j % 0x100;
  y[5] = (j >> 8) % 0x100;
  y[6] = (j >> 16) % 0x100;
  y[7] = (j >> 24) % 0x100;
  size_t i1_err = fwrite(y , 1, 8, fd);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// if we have no camera, or sd card, then flash rear led on and off to warn the human SOS - SOS
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void major_fail() {
  Serial.println(" ");

  for  (int i = 0;  i < 10; i++) {                 // 10 loops or about 100 seconds then reboot
    for (int j = 0; j < 3; j++) {
      digitalWrite(33, LOW);   delay(150);
      digitalWrite(33, HIGH);  delay(150);
    }
    delay(1000);

    for (int j = 0; j < 3; j++) {
      digitalWrite(33, LOW);  delay(500);
      digitalWrite(33, HIGH); delay(500);
    }
    delay(1000);
    Serial.print("Major Fail  "); Serial.print(i); Serial.print(" / "); Serial.println(10);
  }

  ESP.restart();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static esp_err_t config_camera() {
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 10000000;     // 10000000 or 20000000 -- 10 is faster !!

  config.pixel_format = PIXFORMAT_JPEG;

  //config.frame_size = FRAMESIZE_UXGA; // edit in framesizes below -- this must be better than the framesize specified at the top
  /*
      FRAMESIZE_96X96,    // 96x96
      FRAMESIZE_QQVGA,    // 160x120
      FRAMESIZE_QCIF,     // 176x144
      FRAMESIZE_HQVGA,    // 240x176
      FRAMESIZE_240X240,  // 240x240
      FRAMESIZE_QVGA,     // 320x240
      FRAMESIZE_CIF,      // 400x296     5
      FRAMESIZE_HVGA,     // 480x320
      FRAMESIZE_VGA,      // 640x480     6
      FRAMESIZE_SVGA,     // 800x600     7
      FRAMESIZE_XGA,      // 1024x768
      FRAMESIZE_HD,       // 1280x720
      FRAMESIZE_SXGA,     // 1280x1024   9
      FRAMESIZE_UXGA,     // 1600x1200   10
  */

  config.frame_size = FRAMESIZE_UXGA; // edit in framesizes below
  config.jpeg_quality = 6;  // 1 to 63 - smaller number is higher quality and more data - must be lower rhat the quality parameter at the top
  config.fb_count = 7;

  esp_err_t cam_err = ESP_FAIL;
  int attempt = 2;
  while (attempt && cam_err != ESP_OK) {
    cam_err = esp_camera_init(&config);
    if (cam_err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", cam_err);
      digitalWrite(PWDN_GPIO_NUM, 1);
      delay(100);
      digitalWrite(PWDN_GPIO_NUM, 0); // power cycle the camera (OV2640)
      attempt--;
    }
  }

  if (cam_err != ESP_OK) {
    major_fail();
  }

  sensor_t * ss = esp_camera_sensor_get();

  Serial.printf("\nCamera started correctly, Type is %x (hex) of 9650, 7725, 2640, 3660, 5640\n\n", ss->id.PID);

  ss->set_quality(ss, quality);
  ss->set_framesize(ss, (framesize_t)framesize);

  ss->set_brightness(ss, 1);  //up the blightness just a bit
  ss->set_saturation(ss, -2); //lower the saturation

  delay(800);
  for (int j = 0; j < 4; j++) {
    camera_fb_t * fb = esp_camera_fb_get();
    esp_camera_fb_return(fb);
    delay(50);
  }
}

static esp_err_t init_sdcard()
{
  //pinMode(13, PULLUP);

  esp_err_t ret = ESP_FAIL;
  sdmmc_host_t host = SDMMC_HOST_DEFAULT();
  host.flags = SDMMC_HOST_FLAG_1BIT;                       // using 1 bit mode
  host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
  diskspeed = host.max_freq_khz;
  sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
  slot_config.width = 1;                                   // using 1 bit mode
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false,
    .max_files = 8,
  };

  sdmmc_card_t *card;

  ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

  if (ret == ESP_OK) {
    Serial.println("SD card mount successfully!");
  }  else  {
    Serial.printf("Failed to mount SD card VFAT filesystem. Error: %s\n\n", esp_err_to_name(ret));
    Serial.println("Do you have an SD Card installed?");
    Serial.println("Check pin 12 and 13, not grounded, or grounded with 10k resistors!\n");
    major_fail();
  }

  sdmmc_card_print_info(stdout, card);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  get_good_jpeg()  - take a picture and make sure it has a good jpeg
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
camera_fb_t *  get_good_jpeg() {
  camera_fb_t * fb;

  do {
    bp = millis();
    fb = esp_camera_fb_get();
    totalp = totalp + millis() - bp;

    int x = fb->len;
    int foundffd9 = 0;

    for (int j = 1; j <= 1025; j++) {
      if (fb->buf[x - j] != 0xD9) {
        // no d9, try next for
      } else {
        if (fb->buf[x - j - 1] == 0xFF ) {
          //Serial.print("Found the FFD9, junk is "); Serial.println(j);
          if (j == 1) {
            normal_jpg++;
          } else {
            extend_jpg++;
          }
          if (j > 1000) { //  never happens. but > 1 does, usually 400-500
            Serial.print("Frame "); Serial.print(frame_cnt);
            Serial.print(", Len = "); Serial.print(x);
            Serial.print(", Correct Len = "); Serial.print(x - j + 1);
            Serial.print(", Extra Bytes = "); Serial.println( j - 1);
          }
          foundffd9 = 1;
          break;
        }
      }
    }

    if (!foundffd9) {
      bad_jpg++;
      Serial.print("Bad jpeg, Len = "); Serial.println(x);
      esp_camera_fb_return(fb);
    } else {
      break;
      // count up the useless bytes
    }

  } while (1);

  return fb;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  eprom functions  - increment the file_group, so files are always unique
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <EEPROM.h>

struct eprom_data {
  int eprom_good;
  int file_group;
};

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void do_eprom_read() {

  eprom_data ed;

  EEPROM.begin(200);
  EEPROM.get(0, ed);

  if (ed.eprom_good == MagicNumber) {
    Serial.println("Good settings in the EPROM ");
    file_group = ed.file_group;
    file_group++;
    Serial.print("New File Group "); Serial.println(file_group );
  } else {
    Serial.println("No settings in EPROM - Starting with File Group 1 ");
    file_group = 1;
  }
  do_eprom_write();
  file_number = 1;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void do_eprom_write() {
  eprom_data ed;

  ed.eprom_good = MagicNumber;
  ed.file_group  = file_group;

  Serial.println("Writing to EPROM ...");

  EEPROM.begin(200);
  EEPROM.put(0, ed);
  EEPROM.commit();
  EEPROM.end();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Make the avi functions
//
//   start_avi() - open the file and write headers
//   another_pic_avi() - write one more frame of movie
//   end_avi() - write the final parameters and close the file
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// start_avi - open the files and write in headers
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static esp_err_t start_avi() {
  Serial.println("Starting an avi ");

  time(&now);
  localtime_r(&now, &timeinfo);

  strftime(strftime_buf, sizeof(strftime_buf), "%Y_%m_%d__%H-%M-%S", &timeinfo);
  sprintf(fname, "/sdcard/%s.avi", strftime_buf);

  file_number++;

  avifile = fopen(fname, "w");
  idxfile = fopen("/sdcard/idx.tmp", "w");

  if (avifile != NULL)  {
    Serial.printf("File open: %s\n", fname);
  }  else  {
    Serial.println("Could not open file");
    major_fail();
  }

  if (idxfile != NULL)  {
    Serial.printf("File open: %s\n", "/sdcard/idx.tmp");
  }  else  {
    Serial.println("Could not open file");
    major_fail();
  }

  for ( i = 0; i < AVIOFFSET; i++)
  {
    char ch = pgm_read_byte(&avi_header[i]);
    buf[i] = ch;
  }

  size_t err = fwrite(buf, 1, AVIOFFSET, avifile);

  if (framesize == 6) {
    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(vga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(vga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(vga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(vga_h, 1, 2, avifile);
  } else if (framesize == 10) {
    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(uxga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(uxga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(uxga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(uxga_h, 1, 2, avifile);
  } else if (framesize == 9) {
    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(sxga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(sxga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(sxga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(sxga_h, 1, 2, avifile);
  } else if (framesize == 7) {
    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(svga_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(svga_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(svga_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(svga_h, 1, 2, avifile);

  }  else if (framesize == 5) {
    fseek(avifile, 0x40, SEEK_SET);
    err = fwrite(cif_w, 1, 2, avifile);
    fseek(avifile, 0xA8, SEEK_SET);
    err = fwrite(cif_w, 1, 2, avifile);
    fseek(avifile, 0x44, SEEK_SET);
    err = fwrite(cif_h, 1, 2, avifile);
    fseek(avifile, 0xAC, SEEK_SET);
    err = fwrite(cif_h, 1, 2, avifile);
  }
  fseek(avifile, AVIOFFSET, SEEK_SET);

  Serial.print(F("\nRecording "));
  Serial.print(avi_length);
  Serial.println(" seconds.");

  startms = millis();

  totalp = 0;
  totalw = 0;

  jpeg_size = 0;
  movi_size = 0;
  uVideoLen = 0;
  idx_offset = 4;

  //frame_cnt = 0;

  bad_jpg = 0;
  extend_jpg = 0;
  normal_jpg = 0;

} // end of start avi

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  another_save_avi saves another frame to the avi file, uodates index
//           -- pass in a fb pointer to the frame to add
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static esp_err_t another_save_avi(camera_fb_t * fb ) {
  int fblen;
  fblen = fb->len;

  int fb_block_length;
  uint8_t* fb_block_start;

  jpeg_size = fblen;
  movi_size += jpeg_size;
  uVideoLen += jpeg_size;

  remnant = (4 - (jpeg_size & 0x00000003)) & 0x00000003;

  bw = millis();
  long frame_write_start = millis();

  framebuffer_static[3] = 0x63;
  framebuffer_static[2] = 0x64;
  framebuffer_static[1] = 0x30;
  framebuffer_static[0] = 0x30;

  int jpeg_size_rem = jpeg_size + remnant;

  framebuffer_static[4] = jpeg_size_rem % 0x100;
  framebuffer_static[5] = (jpeg_size_rem >> 8) % 0x100;
  framebuffer_static[6] = (jpeg_size_rem >> 16) % 0x100;
  framebuffer_static[7] = (jpeg_size_rem >> 24) % 0x100;
  fb_block_start = fb->buf;

  if (fblen > 64 * 1024 - 8 ) {
    fb_block_length = 64 * 1024;
    fblen = fblen - (64 * 1024 - 8);
    memcpy(framebuffer_static + 8, fb_block_start, fb_block_length - 8);
    fb_block_start = fb_block_start + fb_block_length - 8;

  } else {
    fb_block_length = fblen + 8  + remnant;
    memcpy(framebuffer_static + 8, fb_block_start,  fblen);
    fblen = 0;
  }

  size_t err = fwrite(framebuffer_static, 1, fb_block_length, avifile);
  if (err != fb_block_length) {
    Serial.print("Error on avi write: err = "); Serial.print(err);
    Serial.print(" len = "); Serial.println(fb_block_length);
  }

  while (fblen > 0) {
    if (fblen > 64 * 1024) {
      fb_block_length = 64 * 1024;
      fblen = fblen - fb_block_length;
    } else {
      fb_block_length = fblen  + remnant;
      fblen = 0;
    }

    memcpy(framebuffer_static, fb_block_start, fb_block_length);

    size_t err = fwrite(framebuffer_static, 1, fb_block_length, avifile);
    if (err != fb_block_length) {
      Serial.print("Error on avi write: err = "); Serial.print(err);
      Serial.print(" len = "); Serial.println(fb_block_length);
    }

    fb_block_start = fb_block_start + fb_block_length;
  }
  long frame_write_end = millis();

  print_2quartet(idx_offset, jpeg_size, idxfile);
  idx_offset = idx_offset + jpeg_size + remnant + 8;
  movi_size = movi_size + remnant;
  totalw = totalw + millis() - bw;
} // end of another_pic_avi

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  end_avi writes the index, and closes the files
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static esp_err_t end_avi() {
  unsigned long current_end = 0;

  current_end = ftell (avifile);
  Serial.println("End of avi - closing the files");

  if (frame_cnt <  10 ) {
    Serial.println("Recording screwed up, less than 10 frames, forget index\n");
    fclose(idxfile);
    fclose(avifile);
    int xx = remove("/sdcard/idx.tmp");
    int yy = remove(fname);
  } else {
    elapsedms = millis() - startms;

    float fRealFPS = (1000.0f * (float)frame_cnt) / ((float)elapsedms);

    float fmicroseconds_per_frame = 1000000.0f / fRealFPS;
    uint8_t iAttainedFPS = round(fRealFPS);
    uint32_t us_per_frame = round(fmicroseconds_per_frame);

    //Modify the MJPEG header from the beginning of the file, overwriting various placeholders

    fseek(avifile, 4 , SEEK_SET);
    print_quartet(movi_size + 240 + 16 * frame_cnt + 8 * frame_cnt, avifile);

    fseek(avifile, 0x20 , SEEK_SET);
    print_quartet(us_per_frame, avifile);

    unsigned long max_bytes_per_sec = movi_size * iAttainedFPS / frame_cnt;

    fseek(avifile, 0x24 , SEEK_SET);
    print_quartet(max_bytes_per_sec, avifile);

    fseek(avifile, 0x30 , SEEK_SET);
    print_quartet(frame_cnt, avifile);

    fseek(avifile, 0x8c , SEEK_SET);
    print_quartet(frame_cnt, avifile);

    fseek(avifile, 0x84 , SEEK_SET);
    print_quartet((int)iAttainedFPS, avifile);

    fseek(avifile, 0xe8 , SEEK_SET);
    print_quartet(movi_size + frame_cnt * 8 + 4, avifile);

    Serial.println(F("\n*** Video recorded and saved ***\n"));
    Serial.print(F("Recorded "));
    Serial.print(elapsedms / 1000);
    Serial.print(F("s in "));
    Serial.print(frame_cnt);
    Serial.print(F(" frames\nFile size is "));
    Serial.print(movi_size + 12 * frame_cnt + 4);
    Serial.print(F(" bytes\nActual FPS is "));
    Serial.print(fRealFPS, 2);
    Serial.print(F("\nMax data rate is "));
    Serial.print(max_bytes_per_sec);
    Serial.print(F(" byte/s\nFrame duration is "));  Serial.print(us_per_frame);  Serial.println(F(" us"));
    Serial.print(F("Average frame length is "));  Serial.print(uVideoLen / frame_cnt);  Serial.println(F(" bytes"));
    Serial.print("Average picture time (ms) "); Serial.println( 1.0 * totalp / frame_cnt);
    Serial.print("Average write time (ms)   "); Serial.println( 1.0 * totalw / frame_cnt );
    Serial.print("Normal jpg % ");  Serial.println( 100.0 * normal_jpg / frame_cnt, 1 );
    Serial.print("Extend jpg % ");  Serial.println( 100.0 * extend_jpg / frame_cnt, 1 );
    Serial.print("Bad    jpg % ");  Serial.println( 100.0 * bad_jpg / frame_cnt, 5 );


    Serial.printf("Writng the index, %d frames\n", frame_cnt);
    fseek(avifile, current_end, SEEK_SET);

    fclose(idxfile);

    size_t i1_err = fwrite(idx1_buf, 1, 4, avifile);

    print_quartet(frame_cnt * 16, avifile);

    idxfile = fopen("/sdcard/idx.tmp", "r");

    if (idxfile != NULL)  {
      Serial.printf("File open: %s\n", "/sdcard/idx.tmp");
    }  else  {
      Serial.println("Could not open index file");
      major_fail();
    }

    char * AteBytes;
    AteBytes = (char*) malloc (8);

    for (int i = 0; i < frame_cnt; i++) {
      size_t res = fread ( AteBytes, 1, 8, idxfile);
      size_t i1_err = fwrite(dc_buf, 1, 4, avifile);
      size_t i2_err = fwrite(zero_buf, 1, 4, avifile);
      size_t i3_err = fwrite(AteBytes, 1, 8, avifile);
    }

    free(AteBytes);

    fclose(idxfile);
    fclose(avifile);
    int xx = remove("/sdcard/idx.tmp");
  }
  Serial.println("---");
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <WiFi.h>
#include <ESPmDNS.h>

char localip[20];

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool init_wifi()
{
  int connAttempts = 0;

  uint32_t brown_reg_temp = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(devname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED ) {
    delay(1000);
    Serial.print(".");
    if (connAttempts++ == 5) break;     // try for 5 seconds to get internet, then give up
  }

  Serial.printf("\nInternet status: %d\n", WiFi.status());

  if (!MDNS.begin(devname)) {
    Serial.println("Error setting up MDNS responder!");
  } else {
    Serial.printf("mDNS responder started '%s'\n", devname);
  }

  configTime(0, 0, "pool.ntp.org");

  setenv("TZ", TIMEZONE, 1);  // mountain time zone from #define at top
  tzset();

  time(&now);
  localtime_r(&now, &timeinfo);

  while (now < 10) {        // try for 5 seconds to get the time, then give up - 10 seconds after boot
    delay(1000);
    Serial.print("o");
    time(&now);
    localtime_r(&now, &timeinfo);
  }

  Serial.print("Local time: "); Serial.print(ctime(&now));
  sprintf(localip, "%s", WiFi.localIP().toString().c_str());
  Serial.print("IP: "); Serial.println(localip); Serial.println(" ");

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, brown_reg_temp);
  return true;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <HTTPClient.h>
#include "camera_index.h"

static ra_filter_t ra_filter;
httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

char the_page[4000];

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static ra_filter_t * ra_filter_init(ra_filter_t * filter, size_t sample_size){
    memset(filter, 0, sizeof(ra_filter_t));

    filter->values = (int *)malloc(sample_size * sizeof(int));
    if(!filter->values){
        return NULL;
    }
    memset(filter->values, 0, sample_size * sizeof(int));

    filter->size = sample_size;
    return filter;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static int ra_filter_run(ra_filter_t * filter, int value){
    if(!filter->values){
        return value;
    }
    filter->sum -= filter->values[filter->index];
    filter->values[filter->index] = value;
    filter->sum += filter->values[filter->index];
    filter->index++;
    filter->index = filter->index % filter->size;
    if (filter->count < filter->size) {
        filter->count++;
    }
    return filter->sum / filter->count;
}

// Cтраница изменения параметров работы видео сенсора
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static esp_err_t cmd_handler(httpd_req_t *req){
    char*  buf;
    size_t buf_len;
    char variable[32] = {0,};
    char value[32] = {0,};

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    int val = atoi(value);
    sensor_t * s = esp_camera_sensor_get();
    int res = 0;

    if(!strcmp(variable, "framesize")) {
      if(val == 7){
        c1_or_c2 = 1;
      } else c1_or_c2 = 2;
    }
    else if(!strcmp(variable, "contrast")) res = s->set_contrast(s, val);
    else if(!strcmp(variable, "brightness")) res = s->set_brightness(s, val);
    else if(!strcmp(variable, "saturation")) res = s->set_saturation(s, val);
    else if(!strcmp(variable, "gainceiling")) res = s->set_gainceiling(s, (gainceiling_t)val);
    else if(!strcmp(variable, "colorbar")) res = s->set_colorbar(s, val);
    else if(!strcmp(variable, "awb")) res = s->set_whitebal(s, val);
    else if(!strcmp(variable, "agc")) res = s->set_gain_ctrl(s, val);
    else if(!strcmp(variable, "aec")) res = s->set_exposure_ctrl(s, val);
    else if(!strcmp(variable, "hmirror")) res = s->set_hmirror(s, val);
    else if(!strcmp(variable, "vflip")) res = s->set_vflip(s, val);
    else if(!strcmp(variable, "awb_gain")) res = s->set_awb_gain(s, val);
    else if(!strcmp(variable, "agc_gain")) res = s->set_agc_gain(s, val);
    else if(!strcmp(variable, "aec_value")) res = s->set_aec_value(s, val);
    else if(!strcmp(variable, "aec2")) res = s->set_aec2(s, val);
    else if(!strcmp(variable, "dcw")) res = s->set_dcw(s, val);
    else if(!strcmp(variable, "bpc")) res = s->set_bpc(s, val);
    else if(!strcmp(variable, "wpc")) res = s->set_wpc(s, val);
    else if(!strcmp(variable, "raw_gma")) res = s->set_raw_gma(s, val);
    else if(!strcmp(variable, "lenc")) res = s->set_lenc(s, val);
    else if(!strcmp(variable, "special_effect")) res = s->set_special_effect(s, val);
    else if(!strcmp(variable, "wb_mode")) res = s->set_wb_mode(s, val);
    else if(!strcmp(variable, "ae_level")) res = s->set_ae_level(s, val);
    else {
        res = -1;
    }

    if(res){
        return httpd_resp_send_500(req);
    }

    delay(800);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Cтраница получения параметров работы видео сенсора
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static esp_err_t status_handler(httpd_req_t *req){
    static char json_response[1024];

    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;

    *p++ = '{';
    p+=sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p+=sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p+=sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p+=sprintf(p, "\"saturation\":%d,", s->status.saturation);
    p+=sprintf(p, "\"sharpness\":%d,", s->status.sharpness);
    p+=sprintf(p, "\"special_effect\":%u,", s->status.special_effect);
    p+=sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);
    p+=sprintf(p, "\"awb\":%u,", s->status.awb);
    p+=sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);
    p+=sprintf(p, "\"aec\":%u,", s->status.aec);
    p+=sprintf(p, "\"aec2\":%u,", s->status.aec2);
    p+=sprintf(p, "\"ae_level\":%d,", s->status.ae_level);
    p+=sprintf(p, "\"aec_value\":%u,", s->status.aec_value);
    p+=sprintf(p, "\"agc\":%u,", s->status.agc);
    p+=sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);
    p+=sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);
    p+=sprintf(p, "\"bpc\":%u,", s->status.bpc);
    p+=sprintf(p, "\"wpc\":%u,", s->status.wpc);
    p+=sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);
    p+=sprintf(p, "\"lenc\":%u,", s->status.lenc);
    p+=sprintf(p, "\"vflip\":%u,", s->status.vflip);
    p+=sprintf(p, "\"hmirror\":%u,", s->status.hmirror);
    p+=sprintf(p, "\"dcw\":%u,", s->status.dcw);
    p+=sprintf(p, "\"colorbar\":%u,", s->status.colorbar);
    *p++ = '}';
    *p++ = 0;

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Главная страница
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    sensor_t * s = esp_camera_sensor_get();
    return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Streaming stuff based on Random Nerd
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define PART_BOUNDARY "123456789000000000000987654321"

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Страница видео/потока, MJPEG по HTTP
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  Serial.print("stream_handler, core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {

    if (fb_next == NULL) {
      fb = esp_camera_fb_get();
      framebuffer_len = fb->len;
      memcpy(framebuffer, fb->buf, framebuffer_len);
      esp_camera_fb_return(fb);

    } else {
      fb = fb_next;
      framebuffer_len = fb->len;
      memcpy(framebuffer, fb->buf, framebuffer_len);
    }
    _jpg_buf_len = framebuffer_len;
    _jpg_buf = framebuffer;

    size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
    res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    if (res != ESP_OK) {
      return res;
    }

    res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res != ESP_OK) {
      return res;
    }

    res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    if (res != ESP_OK) {
      return res;
    }

    //if (fb_next == NULL) {
    //  esp_camera_fb_return(fb);
    //}

    delay(200);       // 200 ms = 5 fps !!!
  }

  return res;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Инициализация Web-сервера
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();

  Serial.print("http task prio: "); Serial.println(config.task_priority);

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t status_uri = {
    .uri       = "/status",
    .method    = HTTP_GET,
    .handler   = status_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri       = "/control",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  ra_filter_init(&ra_filter, 20);

  config.task_priority = 0; //минимальный приоритет выполнения!
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
  }

  config.task_priority = 3; //максимальный приоритет выполнения только для потока видео!
  config.server_port += 1;
  config.ctrl_port += 1;
  if(httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
  
  Serial.println("Camera http started");
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Функция настройки модуля
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
  Serial.begin(115200);
  Serial.println("\n\n---");

  pinMode(33, OUTPUT);             // little red led on back of chip
  digitalWrite(33, LOW);           // turn on the red LED on the back of chip

  pinMode(4, OUTPUT);               // Blinding Disk-Avtive Light
  digitalWrite(4, LOW);             // turn off

  pinMode(12, INPUT_PULLDOWN);               // pull this down to stop recording

  Serial.println("                                    ");
  Serial.println("-------------------------------------");
  Serial.printf("ESP32-CAM-Video-Recorder %s\n", vernum);
  Serial.println("-------------------------------------");

  Serial.print("setup, core ");  Serial.print(xPortGetCoreID());
  Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));

  if (IncludeInternet) {
    Serial.println("Starting the wifi ...");
    init_wifi();
  }

  Serial.println("Setting up the camera ...");
  config_camera();

  // SD camera init
  esp_err_t card_err = init_sdcard();
  if (card_err != ESP_OK) {
    Serial.printf("SD Card init failed with error 0x%x", card_err);
    major_fail();
    return;
  }

  digitalWrite(33, HIGH);         // red light turns off when setup is complete
  Serial.println("Warming up the camera ... here are some frames sizes ...");

  for (int i = 0; i < 10; i++) {
    camera_fb_t * fb = esp_camera_fb_get();
    Serial.printf("frame %d, len %d\n", i, fb->len);
    esp_camera_fb_return(fb);
    delay(100);
  }

  do_eprom_read();

  if (IncludeInternet) {
    Serial.println("Start Web ...");
    startCameraServer();
  }

  framebuffer = (uint8_t*)ps_malloc(512 * 1024); // buffer to store a jpg in motion
  Serial.println("  End of setup()\n\n");
  boot_time = millis();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// loop() - остатки Ардуино... главная задача
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {
  // первичный запуск задачи...
  if (first) {
    Serial.print("the loop, core ");  Serial.print(xPortGetCoreID());
    Serial.print(", priority = "); Serial.println(uxTaskPriorityGet(NULL));
    first = 0;
    moveStop = 0;
    frame_cnt = 0;
    frame_mov = 0;
  }

  // счетчики кадров: 
  frame_cnt = frame_cnt + 1; // длина всей записи
  frame_mov = frame_mov + 1; // отсчитывание "0.5 минуты" записи
  
  // быстрый способ обнаружить движение/"окончание движения"
  if(frame_mov > 720){
    frame_mov = 0;
    moveStop = 0;

    for(int i = 0; i < 3; i++){
      moveStop = moveStop+digitalRead(12);
      delay(10);
    }
    
    Serial.println(moveStop);
    moveStop = moveStop/3;// измерений движения = 100%  
  }
    
  // нечего не делаем... "ждем начало движения"
  if(frame_cnt == 1 && moveStop < 1){
    frame_cnt = 0;
    frame_mov = 0;
    if(we_are_already_stopped == 0) Serial.println("\n\nDisconnect Pin 12 from GND to start recording.\n\n");
    we_are_already_stopped = 1;

    // точный способ обнаружить движение    
    for(int i = 0; i < 10; i++){
      moveStop = moveStop+digitalRead(12);
      delay(50);
    }
    Serial.println(moveStop);
    moveStop = moveStop/7; // измерений движения > 70% 

    // есть возможность изменить параметры работы матрицы
    if((c1_or_c2 == 2 ) && (c1_or_c2_now != c1_or_c2){
        c1_or_c2_now = c1_or_c2;
        framesize = c2_framesize;
        quality = c2_quality;
        avi_length = c2_avi_length;

        sensor_t * ss = esp_camera_sensor_get();
        ss->set_quality(ss, quality);
        ss->set_framesize(ss, (framesize_t) framesize);

        delay(800);
        for (int j = 0; j < 4; j++) {
          camera_fb_t * fb = esp_camera_fb_get();
          esp_camera_fb_return(fb);
          delay(50);
        }
    } else {
      if((c1_or_c2 == 1 ) && (c1_or_c2_now != c1_or_c2)){
        c1_or_c2_now = c1_or_c2;
        framesize = c1_framesize;
        quality = c1_quality;
        avi_length = c1_avi_length;

        sensor_t * ss = esp_camera_sensor_get();
        ss->set_quality(ss, quality);
        ss->set_framesize(ss, (framesize_t)framesize);

        delay(800);
        for (int j = 0; j < 4; j++) {
          camera_fb_t * fb = esp_camera_fb_get();
          esp_camera_fb_return(fb);
          delay(50);
        }
      }
    }
    
    return;
  }

  // начало записи
  if(frame_cnt == 1 && moveStop > 0){
    we_are_already_stopped = 0;

    // установка параметров записи
    if((c1_or_c2 == 2 ) && (c1_or_c2_now != c1_or_c2){
        c1_or_c2_now = c1_or_c2;
        framesize = c2_framesize;
        quality = c2_quality;
        avi_length = c2_avi_length;

        sensor_t * ss = esp_camera_sensor_get();
        ss->set_quality(ss, quality);
        ss->set_framesize(ss, (framesize_t) framesize);

        delay(800);
        for (int j = 0; j < 4; j++) {
          camera_fb_t * fb = esp_camera_fb_get();
          esp_camera_fb_return(fb);
          delay(50);
        }
    } else {
      if((c1_or_c2 == 1 ) && (c1_or_c2_now != c1_or_c2)){
        c1_or_c2_now = c1_or_c2;
        framesize = c1_framesize;
        quality = c1_quality;
        avi_length = c1_avi_length;

        sensor_t * ss = esp_camera_sensor_get();
        ss->set_quality(ss, quality);
        ss->set_framesize(ss, (framesize_t)framesize);

        delay(800);
        for (int j = 0; j < 4; j++) {
          camera_fb_t * fb = esp_camera_fb_get();
          esp_camera_fb_return(fb);
          delay(50);
        }
      }
    }

    avi_start_time = millis();
    Serial.printf("Start the avi ... at %d\n", avi_start_time);
    Serial.printf("Framesize %d, quality %d, length %d seconds\n\n", framesize, quality, avi_length);

    fb_curr = get_good_jpeg();                     // should take zero time
    start_avi();
    fb_next = get_good_jpeg();                    // should take nearly zero time due to time spent writing header
    another_save_avi( fb_curr);                  // put first frame in avi

    digitalWrite(33, frame_cnt % 2);                // blink

    esp_camera_fb_return(fb_curr);               // get rid of first frame
    fb_curr = NULL;

    return;
  }
  
  // оставнока записи
  if((frame_cnt > 1 && moveStop < 1) ||  millis() > (avi_start_time + avi_length * 1000)){
    Serial.println("End the Avi");
    fb_curr = fb_next;
    fb_next = NULL;

    another_save_avi(fb_curr);                 // save final frame of avi
    digitalWrite(33, frame_cnt % 2);
    esp_camera_fb_return(fb_curr);
    fb_curr = NULL;

    end_avi();                                // end the movie

    digitalWrite(33, HIGH);          // light off
    avi_end_time = millis();

    float fps = frame_cnt / ((avi_end_time - avi_start_time) / 1000) ;
    Serial.printf("End the avi at %d.  It was %d frames, %d ms at %.1f fps...\n", millis(), frame_cnt, avi_end_time, avi_end_time - avi_start_time, fps);

    frame_cnt = 0;             // start recording again on the next loop
    frame_mov = 0;
    moveStop  = 0;
    return;
  }
  
  // продолжение записи
  if (frame_cnt > 1 && moveStop > 0) {
    //Serial.println("Another frame");

    fb_curr = fb_next;           // we will write a frame, and get the camera preparing a new one
    fb_next = get_good_jpeg();    // should take near zero, unless the sd is faster than the camera, when we will have to wait for the camera
    another_save_avi(fb_curr);

    digitalWrite(33, frame_cnt % 2);

    esp_camera_fb_return(fb_curr);
    fb_curr = NULL;

    if (frame_cnt % 100 == 10 ) {     // print some status every 100 frames
      if (frame_cnt == 10) {
        bytes_before_last_100_frames = movi_size;
        time_before_last_100_frames = millis();
      } else {
        most_recent_fps = 100.0 / ((millis() - time_before_last_100_frames) / 1000.0) ;
        most_recent_avg_framesize = (movi_size - bytes_before_last_100_frames) / 100;

        Serial.printf("So far: %04d frames, in %6.1f seconds, for last 100 frames: avg frame size %6.1f kb, %.2f fps ...\n", frame_cnt, 0.001 * (millis() - avi_start_time), 1.0 / 1024  * most_recent_avg_framesize, most_recent_fps);
        total_delay = 0;

        bytes_before_last_100_frames = movi_size;
        time_before_last_100_frames = millis();
      }
    }

    return;
  }
  
}
