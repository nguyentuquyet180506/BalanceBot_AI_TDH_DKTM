#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

// ==================== THÔNG SỐ ĐIỀU KHIỂN (dùng chung) ====================
struct Settings {
  float kx,kv,ka,kg,headingP,headingD,yawLimit,balanceOffset;
  float deadzoneL,deadzoneR,trimL,trimR;
  float maxSpeed,maxYawRate,speedAccel,speedDecel,yawAccel,yawDecel;
  float positionLead,headingLead,positionOn,headingOn,autoArmBoot;
};
struct FieldSpec { const char *key; size_t offset; float low,high; };
#define FIELD(name,lo,hi) {#name,offsetof(Settings,name),lo,hi}
inline constexpr FieldSpec FIELDS[]={
  FIELD(kx,0,0.01f),FIELD(kv,0,0.1f),FIELD(ka,0,1),FIELD(kg,0,0.1f),
  FIELD(headingP,0,0.1f),FIELD(headingD,0,0.1f),FIELD(yawLimit,0,0.3f),FIELD(balanceOffset,-10,10),
  FIELD(deadzoneL,0,0.5f),FIELD(deadzoneR,0,0.5f),FIELD(trimL,0.5f,1.5f),FIELD(trimR,0.5f,1.5f),
  FIELD(maxSpeed,50,3600),FIELD(maxYawRate,5,90),FIELD(speedAccel,20,1000),FIELD(speedDecel,20,1500),
  FIELD(yawAccel,5,180),FIELD(yawDecel,5,270),FIELD(positionLead,10,1000),FIELD(headingLead,1,45),
  FIELD(positionOn,0,1),FIELD(headingOn,0,1),FIELD(autoArmBoot,0,1)
};
#undef FIELD
inline constexpr size_t FIELD_COUNT=sizeof(FIELDS)/sizeof(FIELDS[0]);
inline float fieldValue(const Settings &s,size_t i){return *reinterpret_cast<const float*>(reinterpret_cast<const char*>(&s)+FIELDS[i].offset);}
inline void setField(Settings &s,size_t i,float v){*reinterpret_cast<float*>(reinterpret_cast<char*>(&s)+FIELDS[i].offset)=v;}
inline bool validSettings(const Settings &s){
  for(size_t i=0;i<FIELD_COUNT;i++){
    float v=fieldValue(s,i);
    if(!isfinite(v)||v<FIELDS[i].low||v>FIELDS[i].high)return false;
  }
  return (s.positionOn==0||s.positionOn==1)&&(s.headingOn==0||s.headingOn==1)&&
         (s.autoArmBoot==0||s.autoArmBoot==1);
}

enum Action : uint8_t { NONE,ARM,ZERO,CALIBRATE,APPLY,SAVE,DEFAULTS };

// ==================== GIAO THỨC UART ====================
namespace rl {

constexpr uint32_t BAUD = 460800;
constexpr uint8_t  SYNC0 = 0xA5, SYNC1 = 0x5A;
constexpr uint8_t  MAX_PAYLOAD = 200;
constexpr size_t   MAX_FRAME = MAX_PAYLOAD + 5;

enum MsgType : uint8_t {
  MSG_DRIVE        = 0x01,
  MSG_STOP         = 0x02,
  MSG_DISARM       = 0x03,
  MSG_ACTION       = 0x04,
  MSG_GET_SETTINGS = 0x05,
  MSG_TELEMETRY    = 0x81,
  MSG_SETTINGS     = 0x82,
  MSG_NOTICE       = 0x83,
  MSG_ACK          = 0x84,
};

enum AckResult : uint8_t { ACK_OK = 0, ACK_BUSY = 1, ACK_REJECT = 2 };

enum : uint8_t {
  F_ARMED = 1u<<0, F_AUTOARM = 1u<<1, F_IMU = 1u<<2, F_PWM = 1u<<3,
  F_MOVING = 1u<<4, F_SAVED = 1u<<5, F_TASK = 1u<<6, F_READY = 1u<<7,
};
enum : uint8_t { F2_XLIMITED = 1u<<0, F2_YAWLIMITED = 1u<<1, F2_HEADING_ON = 1u<<2, F2_FALL_REARM = 1u<<3 };

struct SimpleMsg { uint8_t req; uint8_t pad[3]; };
struct DriveMsg  { uint32_t epoch, stamp; float v, w; uint8_t req; uint8_t pad[3]; };
struct ActionMsg { uint32_t epoch, stamp; Settings settings; uint8_t req, action; uint8_t pad[2]; };
struct AckMsg    { uint8_t req, result; uint8_t pad[2]; };
struct SettingsMsg { uint32_t revision; Settings values; };
struct TelemetryMsg {
  uint32_t boot, epoch, revision, at;
  float pitch, rate, x, xRef, v, vCmd, vRef, yaw, yawRef, w, wCmd, wRef, u, y;
  float maxSpeed, maxYawRate;
  uint16_t armHoldMs;
  uint8_t flags, flags2;
};
static_assert(sizeof(DriveMsg) == 20, "DriveMsg");
static_assert(sizeof(ActionMsg) == 104, "ActionMsg");
static_assert(sizeof(SettingsMsg) == 96, "SettingsMsg");
static_assert(sizeof(TelemetryMsg) == 84, "TelemetryMsg");
static_assert(sizeof(TelemetryMsg) <= MAX_PAYLOAD && sizeof(ActionMsg) <= MAX_PAYLOAD, "payload");

inline uint8_t crc8(uint8_t c, uint8_t b) {
  c ^= b;
  for (int k = 0; k < 8; k++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  return c;
}

inline size_t encode(uint8_t type, const void *payload, uint8_t n, uint8_t *out) {
  out[0] = SYNC0; out[1] = SYNC1; out[2] = type; out[3] = n;
  if (n) memcpy(out + 4, payload, n);
  uint8_t c = crc8(crc8(0, type), n);
  for (uint8_t i = 0; i < n; i++) c = crc8(c, out[4 + i]);
  out[4 + n] = c;
  return (size_t)n + 5;
}

class Parser {
 public:
  bool push(uint8_t b) {
    switch (st_) {
      case 0: if (b == SYNC0) st_ = 1; break;
      case 1: st_ = (b == SYNC1) ? 2 : (b == SYNC0 ? 1 : 0); break;
      case 2: type_ = b; crc_ = crc8(0, b); st_ = 3; break;
      case 3:
        if (b > MAX_PAYLOAD) { st_ = 0; errors++; break; }
        len_ = b; i_ = 0; crc_ = crc8(crc_, b); st_ = len_ ? 4 : 5; break;
      case 4: buf_[i_++] = b; crc_ = crc8(crc_, b); if (i_ >= len_) st_ = 5; break;
      case 5:
        st_ = 0;
        if (b == crc_) { frames++; return true; }
        errors++; break;
    }
    return false;
  }
  uint8_t type() const { return type_; }
  uint8_t len() const { return len_; }
  const uint8_t *data() const { return buf_; }
  template <typename T> bool read(T &out) const {
    if (len_ != sizeof(T)) return false;
    memcpy(&out, buf_, sizeof(T));
    return true;
  }
  uint32_t frames = 0, errors = 0;

 private:
  uint8_t st_ = 0, type_ = 0, len_ = 0, i_ = 0, crc_ = 0;
  uint8_t buf_[MAX_PAYLOAD];
};

}
