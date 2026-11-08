#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <errno.h>
#include <lwip/sockets.h>
#include "esp_camera.h"
#include "driver/temperature_sensor.h"
#include "driver/uart.h"
#include "robot_link.h"
#include "web_page.h"

// ====== WIFI ======
const char *WIFI_SSID = "hoilamgi";
const char *WIFI_PASS = "khongbiet";
#define WIFI_TX_POWER     WIFI_POWER_11dBm
#define USE_STATIC_IP     1
static const IPAddress ROUTER_STATIC_IP(192, 168, 1, 200);
static const IPAddress ROUTER_GATEWAY(192, 168, 1, 1);
static const IPAddress ROUTER_SUBNET(255, 255, 255, 0);
static const IPAddress ROUTER_DNS(192, 168, 1, 1);
const char *AP_SSID = "BalanceBot-Web";
const char *AP_PASS = "balance123";
#define AP_ALWAYS_ON      1
#define MDNS_NAME         "balancebot"

// ====== UDP ======
#define UDP_PORT          4210
#define CHUNK_PAYLOAD     1400
#define HEADER_SIZE       8
#define CLIENT_TIMEOUT_MS 5000
#define MAX_FRAME_BYTES   100000
#define PKTS_PER_TICK     3
#define FRAME_SEND_TIMEOUT_MS 150

// ====== CPU ======
#define CPU_MHZ_STREAM    160
#define CPU_MHZ_IDLE      80

// ====== TANG NHIET ======
struct Tier { uint8_t fps; uint8_t xclkMHz; uint8_t quality; float enterC; };
static const Tier TIERS[] = {
  {  7, 10, 15,  0.0f },
  {  6,  9, 18, 55.0f },
  {  5,  8, 24, 62.0f },
  {  4,  8, 29, 70.0f },
};

#define NUM_TIERS         (sizeof(TIERS) / sizeof(TIERS[0]))
#define THERMAL_HYST      3.0f
#define QUALITY_MAX       35
#define QUALITY_OFFSET_MAX 10
#define WARMUP_FRAMES     3

// ====== SO DO CHAN (Freenove ESP32-S3 WROOM CAM) ======
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  15
#define SIOD_GPIO_NUM  4
#define SIOC_GPIO_NUM  5

#define Y2_GPIO_NUM    11
#define Y3_GPIO_NUM    9
#define Y4_GPIO_NUM    8
#define Y5_GPIO_NUM    10
#define Y6_GPIO_NUM    12
#define Y7_GPIO_NUM    18
#define Y8_GPIO_NUM    17
#define Y9_GPIO_NUM    16

#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM  7
#define PCLK_GPIO_NUM  13

// ====== DIEU KHIEN XE (chuyen tu ESP can bang sang) ======
#define DRIVE_TIMEOUT_MS   600
#define SESSION_TIMEOUT_MS 3000

// ====== LIEN KET UART VOI ESP CAN BANG ======
#define LINK_UART          UART_NUM_1
#define PIN_LINK_TX        21
#define PIN_LINK_RX        14
#define LINK_TIMEOUT_MS    300
#define LINK_ACK_MS        40

// ====== TRANG THAI DUNG CHUNG ======
static int udpSock = -1;
static sockaddr_in clientAddr;
static portMUX_TYPE clientMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool hasClient = false;
static volatile uint32_t lastClientMs = 0;
static volatile int clientLossPct = 0;

static volatile uint8_t thermalLevel = 0;
static volatile uint8_t qualityOffset = 0;

static QueueHandle_t frameQ = NULL;
static bool cameraInStandby = false;

static temperature_sensor_handle_t tempSensor = NULL;
static float chipTempC = 0.0f;

static uint16_t frameId = 0;
static volatile uint32_t statCaptured = 0, statQDrop = 0, statSent = 0, statBad = 0, statSendFail = 0, statNoMem = 0;
static volatile uint32_t avgFrameBytes = 0, avgSendMs = 0;

// ====== NHIET DO ======
static void initTempSensor() {
  temperature_sensor_config_t conf = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
  if (temperature_sensor_install(&conf, &tempSensor) == ESP_OK) {
    temperature_sensor_enable(tempSensor);
    Serial.println("Cam bien nhiet do ESP32-S3: OK");
  } else {
    Serial.println("Cam bien nhiet do: khong khoi tao duoc (tang nhiet se dung muc 0)");
    tempSensor = NULL;
  }
}

static float readChipTemp() {
  if (!tempSensor) return 0.0f;
  float t = 0;
  temperature_sensor_get_celsius(tempSensor, &t);
  return t;
}

static void updateThermal() {
  uint8_t lv = thermalLevel;
  while (lv + 1 < NUM_TIERS && chipTempC >= TIERS[lv + 1].enterC) lv++;
  while (lv > 0 && chipTempC < TIERS[lv].enterC - THERMAL_HYST) lv--;
  if (lv != thermalLevel) {
    thermalLevel = lv;
    Serial.printf("[THERMAL] %.1fC -> tang %u (fps %u, xclk %uMHz, q %u)\n", chipTempC, lv,
                  TIERS[lv].fps, TIERS[lv].xclkMHz, TIERS[lv].quality);
  }
}

static void adaptNetwork() {
  static uint32_t prevCap = 0, prevDrop = 0;
  uint32_t cap = statCaptured - prevCap, drop = statQDrop - prevDrop;
  prevCap = statCaptured;
  prevDrop = statQDrop;
  bool congested = (cap > 0 && drop * 100 / cap > 25) || clientLossPct >= 8;
  if (congested) {
    if (qualityOffset + 2 <= QUALITY_OFFSET_MAX) qualityOffset = qualityOffset + 2;
    else qualityOffset = QUALITY_OFFSET_MAX;
  } else if (qualityOffset > 0) {
    qualityOffset = qualityOffset - 1;
  }
}

static int ctlFps() { return TIERS[thermalLevel].fps; }
static int ctlQuality() {
  int q = TIERS[thermalLevel].quality + qualityOffset;
  return q > QUALITY_MAX ? QUALITY_MAX : q;
}

// ====== OV5640 STANDBY / WAKE (chi goi tu captureTask hoac setup) ======
static void cameraSleep() {
  if (cameraInStandby) return;
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_reg(s, 0x3008, 0xFF, 0x42);
    cameraInStandby = true;
    Serial.println("OV5640 -> standby");
  }
}

static void cameraWake() {
  if (!cameraInStandby) return;
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_reg(s, 0x3008, 0xFF, 0x02);
    vTaskDelay(pdMS_TO_TICKS(30));
    cameraInStandby = false;
    Serial.println("OV5640 -> wake");
  }
}

static bool applySensorSettings(bool force) {
  static int lastX = -1, lastQ = -1;
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return false;
  bool xchanged = false;
  int x = TIERS[thermalLevel].xclkMHz;
  int q = ctlQuality();
  if (force || x != lastX) {
    if (s->set_xclk) s->set_xclk(s, LEDC_TIMER_0, x);
    lastX = x;
    xchanged = true;
  }
  if (force || q != lastQ) {
    s->set_quality(s, q);
    lastQ = q;
  }
  return xchanged;
}

// ====== KHOI TAO CAMERA ======
static bool initCamera() {
  camera_config_t cam = {};
  cam.ledc_channel = LEDC_CHANNEL_0;
  cam.ledc_timer = LEDC_TIMER_0;
  cam.pin_d0 = Y2_GPIO_NUM;  cam.pin_d1 = Y3_GPIO_NUM;
  cam.pin_d2 = Y4_GPIO_NUM;  cam.pin_d3 = Y5_GPIO_NUM;
  cam.pin_d4 = Y6_GPIO_NUM;  cam.pin_d5 = Y7_GPIO_NUM;
  cam.pin_d6 = Y8_GPIO_NUM;  cam.pin_d7 = Y9_GPIO_NUM;
  cam.pin_xclk = XCLK_GPIO_NUM;
  cam.pin_pclk = PCLK_GPIO_NUM;
  cam.pin_vsync = VSYNC_GPIO_NUM;
  cam.pin_href = HREF_GPIO_NUM;
  cam.pin_sccb_sda = SIOD_GPIO_NUM;
  cam.pin_sccb_scl = SIOC_GPIO_NUM;
  cam.pin_pwdn = PWDN_GPIO_NUM;
  cam.pin_reset = RESET_GPIO_NUM;

  cam.pixel_format = PIXFORMAT_JPEG;
  cam.grab_mode = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    cam.frame_size = FRAMESIZE_CIF;
    cam.jpeg_quality = TIERS[0].quality;
    cam.fb_count = 4;
    cam.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    Serial.println("Khong tim thay PSRAM!");
    cam.frame_size = FRAMESIZE_QVGA;
    cam.jpeg_quality = 15;
    cam.fb_count = 3;
    cam.fb_location = CAMERA_FB_IN_DRAM;
  }

  cam.xclk_freq_hz = TIERS[0].xclkMHz * 1000000;
  esp_err_t err = esp_camera_init(&cam);
  if (err != ESP_OK) {
    Serial.printf("Khoi tao XCLK %uMHz loi 0x%x, thu lai 20MHz...\n", TIERS[0].xclkMHz, err);
    cam.xclk_freq_hz = 20000000;
    err = esp_camera_init(&cam);
  }
  if (err != ESP_OK) {
    Serial.printf("[-] Khoi tao camera that bai, ma loi 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    Serial.printf("[+] Sensor PID: 0x%04x (OV5640 = 0x5640)\n", s->id.PID);
    if (s->set_gain_ctrl)      s->set_gain_ctrl(s, 1);
    if (s->set_exposure_ctrl)  s->set_exposure_ctrl(s, 1);
    if (s->set_awb_gain)       s->set_awb_gain(s, 1);
    if (s->set_special_effect) s->set_special_effect(s, 0);
    if (s->set_sharpness)      s->set_sharpness(s, 0);
    if (s->set_denoise)        s->set_denoise(s, 0);
    if (s->set_gainceiling)    s->set_gainceiling(s, GAINCEILING_8X);
    if (s->set_vflip)          s->set_vflip(s, 1);
    if (s->set_hmirror)        s->set_hmirror(s, 1);
  }
  cameraInStandby = false;
  return true;
}

static bool isValidJpeg(const camera_fb_t *fb) {
  if (!fb || fb->format != PIXFORMAT_JPEG) return false;
  if (fb->len < 100 || fb->len > MAX_FRAME_BYTES) return false;
  if (fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) return false;
  size_t n = fb->len;
  size_t from = (n > 8) ? n - 8 : 1;
  for (size_t i = n - 1; i >= from && i >= 1; i--) {
    if (fb->buf[i - 1] == 0xFF && fb->buf[i] == 0xD9) return true;
  }
  return false;
}

// ====== UDP (lwip socket, dung chung cho nhan HELLO va gui frame) ======
static void udpOpen() {
  udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  int yes = 1;
  setsockopt(udpSock, SOL_SOCKET, SO_BROADCAST, &yes, sizeof(yes));
  sockaddr_in a = {};
  a.sin_family = AF_INET;
  a.sin_port = htons(UDP_PORT);
  a.sin_addr.s_addr = htonl(INADDR_ANY);
  bind(udpSock, (sockaddr *)&a, sizeof(a));
}

static bool sendPacket(const uint8_t *p, size_t n, const sockaddr_in &dst, uint32_t deadline) {
  for (;;) {
    int r = sendto(udpSock, p, n, 0, (const sockaddr *)&dst, sizeof(dst));
    if (r == (int)n) return true;
    if (errno == ENOMEM || errno == ENOBUFS || errno == EAGAIN) {
      statNoMem = statNoMem + 1;
      if ((int32_t)(millis() - deadline) >= 0) return false;
      vTaskDelay(1);
      continue;
    }
    return false;
  }
}

static bool sendFrame(const camera_fb_t *fb) {
  sockaddr_in dst;
  portENTER_CRITICAL(&clientMux);
  dst = clientAddr;
  portEXIT_CRITICAL(&clientMux);
  if (dst.sin_port == 0) return false;

  static uint8_t pkt[HEADER_SIZE + CHUNK_PAYLOAD];
  uint16_t total = (fb->len + CHUNK_PAYLOAD - 1) / CHUNK_PAYLOAD;
  uint16_t id = frameId++;
  size_t offset = 0;
  uint32_t deadline = millis() + FRAME_SEND_TIMEOUT_MS;

  for (uint16_t i = 0; i < total; i++) {
    uint16_t plen = (fb->len - offset > CHUNK_PAYLOAD) ? CHUNK_PAYLOAD : (fb->len - offset);
    pkt[0] = id & 0xFF;     pkt[1] = id >> 8;
    pkt[2] = i & 0xFF;      pkt[3] = i >> 8;
    pkt[4] = total & 0xFF;  pkt[5] = total >> 8;
    pkt[6] = plen & 0xFF;   pkt[7] = plen >> 8;
    memcpy(pkt + HEADER_SIZE, fb->buf + offset, plen);
    if (!sendPacket(pkt, HEADER_SIZE + plen, dst, deadline)) return false;
    offset += plen;
    if ((i % PKTS_PER_TICK) == PKTS_PER_TICK - 1) vTaskDelay(1);
  }
  return true;
}

// ====== CONG SUAT: CPU + WIFI SLEEP ======
static bool controlActive();
static bool powerHigh = true;
static void applyPower(bool busy) {
  if (busy == powerHigh) return;
  powerHigh = busy;
  if (busy) {
    setCpuFrequencyMhz(CPU_MHZ_STREAM);
    WiFi.setSleep(false);
  } else {
    WiFi.setSleep(true);
    setCpuFrequencyMhz(CPU_MHZ_IDLE);
  }
}

static void setStreaming(bool on) {
  if (on == hasClient) return;
  if (on) {
    applyPower(true);
    hasClient = true;
  } else {
    hasClient = false;
    applyPower(controlActive());
    Serial.println("Client camera im lang -> camera standby (CPU / WiFi ha khi khong ai lai xe)");
  }
}

static void handleHello(const char *buf, const sockaddr_in &from) {
  clientLossPct = atoi(buf + 5);

  bool changed = (!hasClient) || from.sin_addr.s_addr != clientAddr.sin_addr.s_addr ||
                 from.sin_port != clientAddr.sin_port;
  portENTER_CRITICAL(&clientMux);
  clientAddr = from;
  portEXIT_CRITICAL(&clientMux);
  if (changed) {
    Serial.print("Client: ");
    Serial.print(IPAddress(from.sin_addr.s_addr));
    Serial.printf(":%u\n", ntohs(from.sin_port));
  }
  lastClientMs = millis();
  setStreaming(true);
}

static void sendBeacon() {
  sockaddr_in b = {};
  b.sin_family = AF_INET;
  b.sin_port = htons(UDP_PORT);
  b.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  const char *m = "ESP32_CAM_BEACON";
  sendto(udpSock, m, strlen(m), 0, (sockaddr *)&b, sizeof(b));
}

// ====== QUEUE: giu frame MOI NHAT, bo frame cu ======
static void flushQueue() {
  camera_fb_t *old;
  while (xQueueReceive(frameQ, &old, 0) == pdTRUE) esp_camera_fb_return(old);
}

static void pushLatest(camera_fb_t *fb) {
  if (xQueueSend(frameQ, &fb, 0) == pdTRUE) return;
  camera_fb_t *old;
  if (xQueueReceive(frameQ, &old, 0) == pdTRUE) {
    esp_camera_fb_return(old);
    statQDrop = statQDrop + 1;
  }
  if (xQueueSend(frameQ, &fb, 0) != pdTRUE) esp_camera_fb_return(fb);
}

// ====== TASK CAPTURE (core 1): lay frame theo nhip FPS cua tang nhiet ======
static void captureTask(void *) {
  uint32_t nextGrab = 0;
  int warmup = 0;
  for (;;) {
    if (!hasClient) {
      if (!cameraInStandby) {
        flushQueue();
        cameraSleep();
      }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    if (cameraInStandby) {
      cameraWake();
      applySensorSettings(true);
      warmup = WARMUP_FRAMES;
      nextGrab = 0;
    } else if (applySensorSettings(false)) {
      warmup = 2;
      nextGrab = 0;
    }

    uint32_t now = millis();
    if (nextGrab != 0 && (int32_t)(nextGrab - now) > 0) {
      TickType_t t = pdMS_TO_TICKS(nextGrab - now);
      vTaskDelay(t ? t : 1);
      continue;
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      statBad = statBad + 1;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    uint32_t interval = 1000 / ctlFps();
    nextGrab = millis() + interval;

    if (warmup > 0) {
      warmup--;
      esp_camera_fb_return(fb);
      nextGrab = 0;
      continue;
    }
    if (!isValidJpeg(fb)) {
      statBad = statBad + 1;
      esp_camera_fb_return(fb);
      continue;
    }
    statCaptured = statCaptured + 1;
    pushLatest(fb);
  }
}

// ====== TASK GUI (core 0): lay frame tu queue va gui UDP ======
static void sendTask(void *) {
  for (;;) {
    camera_fb_t *fb = NULL;
    if (xQueueReceive(frameQ, &fb, pdMS_TO_TICKS(200)) != pdTRUE) continue;
    if (!hasClient) {
      esp_camera_fb_return(fb);
      continue;
    }
    uint32_t t0 = millis();
    size_t len = fb->len;
    bool ok = sendFrame(fb);
    esp_camera_fb_return(fb);
    uint32_t dt = millis() - t0;
    avgSendMs = (avgSendMs * 7 + dt) / 8;
    avgFrameBytes = (avgFrameBytes * 7 + len) / 8;
    if (ok) statSent = statSent + 1; else statSendFail = statSendFail + 1;
  }
}

// ====================================================================
// ====== LIEN KET UART VOI ESP CAN BANG ======
static rl::Parser linkParser;
static rl::TelemetryMsg tele = {};
static bool haveTele = false;
static uint32_t teleRxMs = 0;
static rl::SettingsMsg balSettings = {};
static bool haveSettings = false;
static char balNotice[rl::MAX_PAYLOAD + 1] = "Chua co lien lac voi mach can bang";
static uint8_t linkNextReq = 1, ackReq = 0, ackResult = 0;
static bool ackGot = false;
static uint32_t camBootId = 0;
static String bootToken;
static uint32_t statLinkTx = 0;

static bool linkBegin() {
  uart_config_t c = {};
  c.baud_rate = (int)rl::BAUD;
  c.data_bits = UART_DATA_8_BITS;
  c.parity = UART_PARITY_DISABLE;
  c.stop_bits = UART_STOP_BITS_1;
  c.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  c.source_clk = UART_SCLK_DEFAULT;
  if (uart_driver_install(LINK_UART, 2048, 1024, 0, NULL, 0) != ESP_OK) return false;
  if (uart_param_config(LINK_UART, &c) != ESP_OK) return false;
  if (uart_set_pin(LINK_UART, PIN_LINK_TX, PIN_LINK_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
  gpio_pullup_en((gpio_num_t)PIN_LINK_RX);
  return true;
}

static void linkSend(uint8_t type, const void *payload, size_t n) {
  uint8_t f[rl::MAX_FRAME];
  size_t len = rl::encode(type, payload, (uint8_t)n, f);
  uart_write_bytes(LINK_UART, (const char *)f, len);
  statLinkTx = statLinkTx + 1;
}

static bool linkFresh() { return haveTele && millis() - teleRxMs < LINK_TIMEOUT_MS; }
static uint32_t balanceNow() { return tele.at + (millis() - teleRxMs); }
static bool teleFlag(uint8_t f) { return (tele.flags & f) != 0; }
static bool teleReady() { return linkFresh() && teleFlag(rl::F_READY) && teleFlag(rl::F_TASK) && millis() - teleRxMs < 100; }

static void updateBootToken() {
  bootToken = String((unsigned long)camBootId) + "-" + String((unsigned long)(haveTele ? tele.boot : 0));
}

static void onLinkFrame() {
  switch (linkParser.type()) {
    case rl::MSG_TELEMETRY: {
      rl::TelemetryMsg t;
      if (!linkParser.read(t)) return;
      bool rebooted = !haveTele || t.boot != tele.boot;
      tele = t;
      haveTele = true;
      teleRxMs = millis();
      if (rebooted) {
        updateBootToken();
        haveSettings = false;
        Serial.printf("[UART] Da lien lac mach can bang (boot %08lx)\n", (unsigned long)t.boot);
      }
      return;
    }
    case rl::MSG_SETTINGS: {
      rl::SettingsMsg m;
      if (!linkParser.read(m)) return;
      balSettings = m;
      haveSettings = true;
      return;
    }
    case rl::MSG_NOTICE: {
      size_t n = linkParser.len();
      memcpy(balNotice, linkParser.data(), n);
      balNotice[n] = 0;
      for (size_t i = 0; i < n; i++)
        if (balNotice[i] == '"' || balNotice[i] == '\\' || (uint8_t)balNotice[i] < 0x20) balNotice[i] = ' ';
      return;
    }
    case rl::MSG_ACK: {
      rl::AckMsg a;
      if (!linkParser.read(a)) return;
      ackReq = a.req;
      ackResult = a.result;
      ackGot = true;
      return;
    }
    default:
      return;
  }
}

static void linkPoll() {
  uint8_t b[128];
  for (;;) {
    size_t avail = 0;
    uart_get_buffered_data_len(LINK_UART, &avail);
    if (avail == 0) return;
    int n = uart_read_bytes(LINK_UART, b, avail > sizeof(b) ? sizeof(b) : avail, 0);
    if (n <= 0) return;
    for (int i = 0; i < n; i++)
      if (linkParser.push(b[i])) onLinkFrame();
  }
}

template <typename T>
static int linkRequest(uint8_t type, T &m, int tries = 1) {
  for (int t = 0; t < tries; t++) {
    m.req = linkNextReq++;
    if (linkNextReq == 0) linkNextReq = 1;
    ackGot = false;
    linkSend(type, &m, sizeof(m));
    uint32_t t0 = millis();
    while (millis() - t0 < LINK_ACK_MS) {
      linkPoll();
      if (ackGot && ackReq == m.req) return ackResult;
      delay(1);
    }
  }
  return -1;
}
template <typename T>
static void linkFire(uint8_t type, T &m) {
  m.req = 0;
  linkSend(type, &m, sizeof(m));
}

// ====================================================================
// ====== WEB (chuyen tu ESP can bang sang): CHI CHUYEN LENH VA DOC TRANG THAI ======
WebServer server(80);
String ownerToken;
uint32_t ownerSeen = 0, ownerSeq = 0;
uint32_t udpOwnerIp = 0;
bool udpHasOwner = false;
uint32_t udpSeen = 0, udpSeq = 0;
bool udpActive() { return udpHasOwner && millis() - udpSeen < SESSION_TIMEOUT_MS; }
static bool controlActive() { return udpActive() || (ownerToken.length() && millis() - ownerSeen < SESSION_TIMEOUT_MS); }

void sendJson(int code, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json; charset=utf-8", body);
}
void rejectRequest(int code, const char *reason) { sendJson(code, String("{\"error\":\"") + reason + "\"}"); }
bool parseUIntArg(const char *name, uint32_t &v) {
  if (!server.hasArg(name)) return false;
  String s = server.arg(name);
  if (s.length() == 0 || s.length() > 10) return false;
  uint64_t n = 0;
  for (size_t i = 0; i < s.length(); i++) {
    if (s[i] < '0' || s[i] > '9') return false;
    n = n * 10 + (s[i] - '0');
    if (n > UINT32_MAX) return false;
  }
  v = (uint32_t)n;
  return true;
}
bool parseFloatArg(const char *name, float &v) {
  if (!server.hasArg(name)) return false;
  String s = server.arg(name);
  if (s.length() == 0 || s.length() > 24) return false;
  char *end = nullptr;
  errno = 0;
  v = strtof(s.c_str(), &end);
  return end != s.c_str() && *end == 0 && errno != ERANGE && isfinite(v);
}
bool validBoot() {
  if (server.arg("boot") != bootToken) { rejectRequest(403, "Xe vừa khởi động lại; tải lại trang"); return false; }
  return true;
}
bool validOwner(bool sequence) {
  if (!validBoot()) return false;
  if (ownerToken.length() == 0 || server.arg("token") != ownerToken || millis() - ownerSeen >= SESSION_TIMEOUT_MS) {
    rejectRequest(403, "Phiên điều khiển đã hết hạn");
    return false;
  }
  if (sequence) {
    uint32_t seq;
    if (!parseUIntArg("seq", seq) || seq <= ownerSeq) { rejectRequest(409, "Gói lệnh cũ đã bị bỏ"); return false; }
    ownerSeq = seq;
  }
  ownerSeen = millis();
  return true;
}
bool validEpoch(uint32_t &e) {
  if (!linkFresh()) { rejectRequest(503, "Mất liên lạc UART với mạch cân bằng"); return false; }
  if (!teleFlag(rl::F_TASK)) { rejectRequest(503, "Tác vụ cân bằng chưa sẵn sàng"); return false; }
  if (!parseUIntArg("epoch", e) || e != tele.epoch) { rejectRequest(409, "Trạng thái xe đã đổi; nhả và nhấn lại nút"); return false; }
  return true;
}
bool freshRequest(uint32_t &stamp, uint32_t ttl) {
  if (!parseUIntArg("issued", stamp) || balanceNow() - stamp >= ttl) { rejectRequest(409, "Dữ liệu điều khiển đã cũ; nhấn lại nút"); return false; }
  return true;
}
void replyLink(int r, const char *busyReason) {
  if (r == rl::ACK_OK) sendJson(202, "{\"queued\":true}");
  else if (r == rl::ACK_BUSY) rejectRequest(409, busyReason);
  else if (r == rl::ACK_REJECT) rejectRequest(400, "Chức năng không hợp lệ");
  else rejectRequest(503, "Mạch cân bằng không trả lời qua UART");
}
void handleSession() {
  if (!validBoot()) return;
  if (udpActive()) { rejectRequest(409, "Client UDP đang điều khiển"); return; }
  if (ownerToken.length() && millis() - ownerSeen < SESSION_TIMEOUT_MS) { rejectRequest(409, "Một trình duyệt khác đang điều khiển"); return; }
  ownerToken = String((unsigned long)esp_random()) + String((unsigned long)esp_random());
  ownerSeen = millis();
  ownerSeq = 0;
  rl::SimpleMsg m = {};
  linkFire(rl::MSG_STOP, m);
  sendJson(200, String("{\"token\":\"") + ownerToken + "\"}");
}
void handleState() {
  char b[1400];
  bool fresh = linkFresh();
  const char *msg = fresh ? balNotice : "Mất liên lạc UART với mạch cân bằng — kiểm tra dây TX/RX/GND và nguồn";
  snprintf(b, sizeof(b),
    "{\"boot\":\"%s\",\"armed\":%s,\"autoArm\":%s,\"ready\":%s,\"moving\":%s,\"saved\":%s,\"epoch\":%lu,\"revision\":%lu,\"at\":%lu,\"headingOn\":%s,\"pitch\":%.3f,\"v\":%.2f,\"vRef\":%.2f,\"x\":%.2f,\"xRef\":%.2f,\"yaw\":%.3f,\"yawRef\":%.3f,\"w\":%.3f,\"wRef\":%.3f,\"u\":%.4f,\"y\":%.4f,\"limited\":%s,\"armHold\":%u,\"link\":%s,\"message\":\"%s\"}",
    bootToken.c_str(), teleFlag(rl::F_ARMED) ? "true" : "false", teleFlag(rl::F_AUTOARM) ? "true" : "false",
    teleReady() ? "true" : "false", teleFlag(rl::F_MOVING) ? "true" : "false", teleFlag(rl::F_SAVED) ? "true" : "false",
    (unsigned long)tele.epoch, (unsigned long)tele.revision, (unsigned long)tele.at,
    (tele.flags2 & rl::F2_HEADING_ON) ? "true" : "false",
    tele.pitch, tele.v, tele.vRef, tele.x, tele.xRef, tele.yaw, tele.yawRef, tele.w, tele.wRef, tele.u, tele.y,
    (tele.flags2 & (rl::F2_XLIMITED | rl::F2_YAWLIMITED)) ? "true" : "false", (unsigned)tele.armHoldMs,
    fresh ? "true" : "false", msg);
  sendJson(200, b);
}
void handleSettingsRead() {
  if (linkFresh() && (!haveSettings || balSettings.revision != tele.revision)) {
    rl::SimpleMsg m = {};
    linkFire(rl::MSG_GET_SETTINGS, m);
    uint32_t t0 = millis();
    while (millis() - t0 < 120 && !(haveSettings && balSettings.revision == tele.revision)) {
      linkPoll();
      delay(2);
    }
  }
  if (!haveSettings) { rejectRequest(503, "Chưa đọc được thông số từ mạch cân bằng"); return; }
  const Settings &st = balSettings.values;
  String body;
  body.reserve(3200);
  body = "{\"revision\":" + String(balSettings.revision) + ",\"values\":{\"maxSpeed\":" + String(st.maxSpeed, 0) +
         ",\"maxYawRate\":" + String(st.maxYawRate, 0) + "},\"fields\":[";
  for (size_t i = 0; i < FIELD_COUNT; i++) {
    if (i) body += ',';
    body += "{\"key\":\"";
    body += FIELDS[i].key;
    body += "\",\"value\":";
    body += String(fieldValue(st, i), 8);
    body += ",\"min\":";
    body += String(FIELDS[i].low, 8);
    body += ",\"max\":";
    body += String(FIELDS[i].high, 8);
    body += '}';
  }
  body += "]}";
  sendJson(200, body);
}
void handleSettingsWrite() {
  if (!validOwner(true)) return;
  rl::ActionMsg c = {};
  c.action = APPLY;
  if (!validEpoch(c.epoch) || !freshRequest(c.stamp, 1000)) return;
  if (teleFlag(rl::F_ARMED) || teleFlag(rl::F_AUTOARM)) { rejectRequest(409, "Bấm DISARM trước khi chỉnh thông số"); return; }
  for (size_t i = 0; i < FIELD_COUNT; i++) {
    float v;
    if (!parseFloatArg(FIELDS[i].key, v)) { rejectRequest(400, "Thiếu thông số hoặc số không hợp lệ"); return; }
    setField(c.settings, i, v);
  }
  if (!validSettings(c.settings)) { rejectRequest(400, "Thông số vượt giới hạn cho phép"); return; }
  replyLink(linkRequest(rl::MSG_ACTION, c), "Xe đang xử lý yêu cầu trước; thử lại");
}
void handleDrive() {
  if (!validOwner(true)) return;
  rl::DriveMsg d = {};
  if (!validEpoch(d.epoch) || !freshRequest(d.stamp, DRIVE_TIMEOUT_MS)) return;
  if (!teleFlag(rl::F_ARMED)) { rejectRequest(409, "Xe chưa ARM"); return; }
  if (!parseFloatArg("v", d.v) || !parseFloatArg("w", d.w) || fabsf(d.v) > tele.maxSpeed || fabsf(d.w) > tele.maxYawRate) {
    rejectRequest(400, "Tốc độ không hợp lệ hoặc vượt giới hạn");
    return;
  }
  if (d.w != 0 && !(tele.flags2 & rl::F2_HEADING_ON)) { rejectRequest(409, "Cần bật giữ hướng trước khi xoay"); return; }
  replyLink(linkRequest(rl::MSG_DRIVE, d), "Đang dừng xe");
}
void handleAction() {
  String name = server.arg("action");
  if (name == "disarm") {
    if (!validBoot()) return;
    rl::SimpleMsg m = {};
    if (linkRequest(rl::MSG_DISARM, m, 3) == rl::ACK_OK) sendJson(202, "{\"queued\":true}");
    else rejectRequest(503, "Không gửi được DISARM tới mạch cân bằng — hãy ngắt công tắc nguồn");
    return;
  }
  if (!validOwner(true)) return;
  uint32_t e;
  if (!validEpoch(e)) return;
  if (name == "stop") {
    rl::SimpleMsg m = {};
    replyLink(linkRequest(rl::MSG_STOP, m, 3), "Đang dừng xe");
    return;
  }
  rl::ActionMsg c = {};
  c.epoch = e;
  if (!freshRequest(c.stamp, 1000)) return;
  if (name == "arm") c.action = ARM;
  else if (name == "zero") c.action = ZERO;
  else if (name == "calibrate") c.action = CALIBRATE;
  else if (name == "save") c.action = SAVE;
  else if (name == "defaults") c.action = DEFAULTS;
  else { rejectRequest(400, "Chức năng không hợp lệ"); return; }
  replyLink(linkRequest(rl::MSG_ACTION, c), "Xe đang xử lý yêu cầu trước; thử lại");
}

void startWeb() {
  server.on("/", HTTP_GET, [] {
    server.sendHeader("Cache-Control", "no-store");
    server.send_P(200, "text/html; charset=utf-8", WEB_PAGE);
  });
  server.on("/api/session", HTTP_POST, handleSession);
  server.on("/api/ping", HTTP_POST, [] { if (validOwner(false)) sendJson(200, "{\"ok\":true}"); });
  server.on("/api/state", HTTP_GET, handleState);
  server.on("/api/settings", HTTP_GET, handleSettingsRead);
  server.on("/api/settings", HTTP_POST, handleSettingsWrite);
  server.on("/api/drive", HTTP_POST, handleDrive);
  server.on("/api/action", HTTP_POST, handleAction);
  server.onNotFound([] { rejectRequest(404, "Không tìm thấy đường dẫn"); });
  server.enableDelay(false);
  server.begin();
}

// ====================================================================
// ====== LENH XE QUA UDP (chuyen tu ESP can bang sang) ======
static void udpReply(const sockaddr_in &to) {
  char b[96];
  int n = snprintf(b, sizeof(b), "T,%d,%d,%d,%d,%.2f,%.0f,%.1f", teleFlag(rl::F_ARMED) ? 1 : 0,
                   teleFlag(rl::F_AUTOARM) ? 1 : 0, teleReady() ? 1 : 0, teleFlag(rl::F_MOVING) ? 1 : 0,
                   tele.pitch, tele.v, tele.yaw - tele.yawRef);
  if (n <= 0) return;
  sendto(udpSock, b, n, 0, (const sockaddr *)&to, sizeof(to));
}
static void handleUdpControl(const char *m, const sockaddr_in &from) {
  const char c = m[0];
  const uint32_t ip = from.sin_addr.s_addr;
  uint32_t now = millis();
  if (c == 'X') {
    rl::SimpleMsg x = {};
    linkFire(rl::MSG_DISARM, x);
    udpReply(from);
    return;
  }
  if (c == 'H') {
    udpOwnerIp = ip;
    udpHasOwner = true;
    udpSeen = now;
    udpSeq = 0;
    ownerToken = "";
    rl::SimpleMsg s = {};
    linkFire(rl::MSG_STOP, s);
    udpReply(from);
    return;
  }
  if (!udpActive() || ip != udpOwnerIp) return;
  udpSeen = now;
  if (c == 'P') { udpReply(from); return; }
  unsigned long sq = 0;
  float v = 0, w = 0;
  if (c == 'D') { if (sscanf(m, "D,%lu,%f,%f", &sq, &v, &w) != 3) return; }
  else if (c == 'S' || c == 'A') { if (sscanf(m, "%*c,%lu", &sq) != 1) return; }
  else return;
  if (sq <= udpSeq) return;
  udpSeq = sq;
  if (!linkFresh() || !teleFlag(rl::F_TASK)) { udpReply(from); return; }
  if (c == 'S') {
    rl::SimpleMsg s = {};
    linkFire(rl::MSG_STOP, s);
  } else if (c == 'A') {
    rl::ActionMsg a = {};
    a.action = ARM;
    a.epoch = tele.epoch;
    a.stamp = balanceNow();
    linkFire(rl::MSG_ACTION, a);
  } else {
    if (teleFlag(rl::F_ARMED) && isfinite(v) && isfinite(w)) {
      v = constrain(v, -tele.maxSpeed, tele.maxSpeed);
      w = constrain(w, -tele.maxYawRate, tele.maxYawRate);
      if (!(w != 0 && !(tele.flags2 & rl::F2_HEADING_ON))) {
        rl::DriveMsg d = {};
        d.v = v;
        d.w = w;
        d.epoch = tele.epoch;
        d.stamp = balanceNow();
        linkFire(rl::MSG_DRIVE, d);
      }
    }
  }
  udpReply(from);
}

static void pollUdp() {
  char buf[64];
  for (int k = 0; k < 16; k++) {
    sockaddr_in from;
    socklen_t fl = sizeof(from);
    int n = recvfrom(udpSock, buf, sizeof(buf) - 1, MSG_DONTWAIT, (sockaddr *)&from, &fl);
    if (n <= 0) break;
    buf[n] = 0;
    if (strncmp(buf, "HELLO", 5) == 0) handleHello(buf, from);
    else if (strncmp(buf, "ESP32_CAM_BEACON", 16) == 0) continue;
    else handleUdpControl(buf, from);
  }
}

static void waitUdp(uint32_t ms) {
  if (udpSock < 0) { delay(ms); return; }
  fd_set rf;
  FD_ZERO(&rf);
  FD_SET(udpSock, &rf);
  timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = ms * 1000;
  select(udpSock + 1, &rf, NULL, NULL, &tv);
}

// ====================================================================
static void startAP() {
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  if (WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4)) {
    WiFi.setTxPower(WIFI_TX_POWER);
    Serial.printf("[+] Phat Wi-Fi '%s' (mat khau '%s'): web http://192.168.4.1\n", AP_SSID, AP_PASS);
  } else {
    Serial.println("[-] Khong phat duoc Wi-Fi AP");
  }
}

static void connectWiFi() {
  if (USE_STATIC_IP) WiFi.config(ROUTER_STATIC_IP, ROUTER_GATEWAY, ROUTER_SUBNET, ROUTER_DNS);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) WiFi.setTxPower(WIFI_TX_POWER);
}

static void serviceWiFi() {
  static bool staLost = false;
  static uint32_t tryMs = 0;
  if (WiFi.status() == WL_CONNECTED) {
    if (staLost) {
      staLost = false;
      WiFi.setTxPower(WIFI_TX_POWER);
      Serial.print("[+] Da ket noi lai WiFi. IP ESP32: ");
      Serial.println(WiFi.localIP());
    }
    return;
  }
  uint32_t now = millis();
  if (!staLost) {
    staLost = true;
    Serial.println("Mat WiFi, dang ket noi lai...");
    setStreaming(false);
  } else if (now - tryMs < 15000) {
    return;
  }
  tryMs = now;
  WiFi.disconnect();
  if (USE_STATIC_IP) WiFi.config(ROUTER_STATIC_IP, ROUTER_GATEWAY, ROUTER_SUBNET, ROUTER_DNS);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==========================================");
  Serial.println("  ESP32-S3 WiFi Camera (UDP) v2 - low heat");
  Serial.println("  + web / lenh UDP / cau noi UART toi ESP can bang");
  Serial.println("==========================================");

  setCpuFrequencyMhz(CPU_MHZ_STREAM);
  do { camBootId = esp_random(); } while (camBootId == 0);
  updateBootToken();
  bool linkOk = linkBegin();
  Serial.printf("[%c] UART toi ESP can bang: UART%d %lu baud, TX=GPIO%d RX=GPIO%d\n", linkOk ? '+' : '-',
                (int)LINK_UART, (unsigned long)rl::BAUD, PIN_LINK_TX, PIN_LINK_RX);

  WiFi.mode(AP_ALWAYS_ON ? WIFI_AP_STA : WIFI_STA);
  if (AP_ALWAYS_ON) startAP();
  Serial.printf("[*] Dang ket noi WiFi: '%s' ...\n", WIFI_SSID);
  connectWiFi();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[+] Da ket noi WiFi. IP ESP32: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.printf("[-] Khong ket noi duoc WiFi! Ma trang thai: %d\n", WiFi.status());
    if (!AP_ALWAYS_ON) {
      WiFi.mode(WIFI_AP_STA);
      startAP();
    }
  }

  startWeb();
  if (MDNS.begin(MDNS_NAME)) MDNS.addService("http", "tcp", 80);

  initTempSensor();

  frameQ = xQueueCreate(1, sizeof(camera_fb_t *));
  udpOpen();

  if (!initCamera()) {
    Serial.println("[-] Loi khoi tao camera! Kiem tra lai cap/chan.");
  } else {
    Serial.println("[+] Camera san sang!");
    cameraSleep();
    xTaskCreatePinnedToCore(captureTask, "capture", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(sendTask, "send", 4096, NULL, 2, NULL, 0);
  }

  powerHigh = true;
  applyPower(false);
  Serial.printf("[*] Cho HELLO / lenh xe tren UDP cong %d | Web: http://%s  hoac  http://192.168.4.1  (http://%s.local)\n",
                UDP_PORT, WiFi.localIP().toString().c_str(), MDNS_NAME);
}

void loop() {
  serviceWiFi();
  waitUdp((hasClient || controlActive()) ? 2 : 10);
  pollUdp();
  linkPoll();
  server.handleClient();

  uint32_t now = millis();
  if (hasClient && now - lastClientMs > CLIENT_TIMEOUT_MS) setStreaming(false);
  applyPower(hasClient || controlActive());

  static uint32_t lastTempMs = 0;
  if (now - lastTempMs > 2000) {
    lastTempMs = now;
    chipTempC = readChipTemp();
    updateThermal();
    if (hasClient) adaptNetwork();
  }

  static uint32_t lastInfo = 0, lastSentSnap = 0;
  if (now - lastInfo > 10000) {
    float secs = (now - lastInfo) / 1000.0f;
    uint32_t sent = statSent;
    float fpsOut = (sent - lastSentSnap) / secs;
    lastSentSnap = sent;
    lastInfo = now;
    Serial.print("IP: ");
    Serial.print(WiFi.localIP());
    Serial.printf(" | gui %lu (%.1f fps) bo-queue %lu hong %lu loi %lu het-bo-dem %lu | frame %lu B, gui %lu ms",
                  (unsigned long)statSent, fpsOut, (unsigned long)statQDrop,
                  (unsigned long)statBad, (unsigned long)statSendFail, (unsigned long)statNoMem,
                  (unsigned long)avgFrameBytes, (unsigned long)avgSendMs);
    if (tempSensor) Serial.printf(" | chip %.1fC tang %u q%d", chipTempC, thermalLevel, ctlQuality());
    Serial.printf(" | UART %s khung %lu loi %lu", linkFresh() ? "OK" : "MAT",
                  (unsigned long)linkParser.frames, (unsigned long)linkParser.errors);
    Serial.println(hasClient ? "" : " [IDLE]");
  }

  if (!hasClient) {
    static uint32_t lastBeaconMs = 0;
    if (now - lastBeaconMs > 2000) {
      lastBeaconMs = now;
      sendBeacon();
    }
  }
}
