#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stddef.h>
#include "robot_link.h"

struct Encoder { int pinA,pinB; bool invert; volatile int32_t count; volatile uint8_t prev; int32_t lastCount; };
struct ImuRaw { int16_t ay,az,gx,gz; };
struct Snapshot;struct Command;struct Drive;

// ==================== 01. PHẦN CỨNG VÀ CẤU HÌNH ====================
constexpr int PIN_SDA   = 1;
constexpr int PIN_SCL   = 2;
constexpr int PIN_PWMA  = 12;
constexpr int PIN_AIN1  = 15;
constexpr int PIN_AIN2  = 16;
constexpr int PIN_STBY  = 17;
constexpr int PIN_BIN1  = 18;
constexpr int PIN_BIN2  = 8;
constexpr int PIN_PWMB  = 13;
constexpr int PIN_ENC_L_A = 4;
constexpr int PIN_ENC_L_B = 5;
constexpr int PIN_ENC_R_A = 6;
constexpr int PIN_ENC_R_B = 7;
// ===== CHIỀU MOTOR VÀ CẢM BIẾN (THEO PHẦN CỨNG XE, ĐỘC LẬP VỚI GAIN) =====
constexpr bool  MOTOR_L_INVERT = false;
constexpr bool  MOTOR_R_INVERT = true;
constexpr bool  ENC_L_INVERT   = false;
constexpr bool  ENC_R_INVERT   = true;
constexpr float IMU_PITCH_SIGN = 1.0f;
constexpr float IMU_GYRO_SIGN  = 1.0f;

// ===== THỜI GIAN, NGƯỠNG ARM VÀ BỘ LỌC =====
constexpr uint32_t LOOP_US = 5000;
constexpr uint32_t MAX_LOOP_US = 15000;
constexpr int MOTOR_PWM_FREQ = 20000;
constexpr int MOTOR_PWM_BITS = 10;
constexpr int GYRO_CAL_SAMPLES = 500;
constexpr uint32_t PWM_MAX = 1023;
constexpr float FALL_ANGLE = 42;
constexpr float RESUME_ANGLE = 3;
constexpr uint32_t AUTO_ARM_MS = 700;
constexpr bool     FALL_AUTO_REARM = true;
constexpr uint32_t FALL_REARM_MS = 3000;
constexpr float COMP_ALPHA = 0.98f;
constexpr float SPEED_TAU = 0.05f;


constexpr float IMU_YAW_SIGN = 1.0f;
constexpr uint32_t DRIVE_TIMEOUT_MS = 600;
constexpr uint32_t SETTINGS_SCHEMA = 0x42570202;

Settings defaultSettings(){
  Settings s{};
  s.kx = 0.000061f;
  s.kv = 0.00065f;
  s.ka = 0.056f;
  s.kg = 0.00147f;
  s.headingP = 0.01f;
  s.headingD = 0.0020f;
  s.yawLimit = 0.1f;
  s.balanceOffset = -0.3f;
  s.deadzoneL = 0.0203f;
  s.deadzoneR = 0.021f;
  s.trimL = 1.0f;
  s.trimR = 1.0f;
  s.maxSpeed = 3600;
  s.maxYawRate = 90;
  s.speedAccel = 200;
  s.speedDecel = 300;
  s.yawAccel = 60;
  s.yawDecel = 90;
  s.positionLead = 200;
  s.headingLead = 15;
  s.positionOn = 1;
  s.headingOn = 1;
  s.autoArmBoot = 1;
  return s;
}
Settings cfg=defaultSettings();
struct StoredSettings { uint32_t schema; Settings values; };
bool loadSettings(){
  Preferences p;
  if(!p.begin("bbweb0611",true))return false;
  StoredSettings s{};
  bool ok=p.getBytesLength("settings")==sizeof(s)&&p.getBytes("settings",&s,sizeof(s))==sizeof(s);
  p.end();
  if(!ok||s.schema!=SETTINGS_SCHEMA||!validSettings(s.values))return false;
  cfg=s.values;return true;
}
bool saveSettings(){
  Preferences p;if(!p.begin("bbweb0611",false))return false;
  StoredSettings s{SETTINGS_SCHEMA,cfg};
  bool ok=p.putBytes("settings",&s,sizeof(s))==sizeof(s);p.end();return ok;
}

// ==================== 02. TRẠNG THÁI VÀ HỘP THƯ LỆNH (từ UART) ====================
float imuPitch=0,imuPitchRate=0,gyroBias=0,yawBias=0;
bool imuInitDone=false,imuReady=false,pwmReady=false,balActive=false,autoArm=false;
uint32_t uprightSince=0,sampleTime=0,lastControl=0,lastDrive=0,epoch=1,configRevision=1;
float positionTicks=0,velocityTicks=0,headingDeg=0,headingRate=0;
float positionRef=0,headingRef=0,baseCmd=0,yawCmd=0,terms[4]={};
float speedCommand=0,speedTarget=0,yawRateCommand=0,yawRateTarget=0;
bool motionEngaged=false,driveLive=false,positionLimited=false,headingLimited=false;
bool nvsSaved=false,taskReady=false;
bool fallRearm=false;
char notice[180]="Đang khởi tạo";

struct Command { Action action; uint32_t epoch,stamp; Settings settings; };
struct Drive { float v,w; uint32_t epoch,stamp; };
struct Mailbox {
  bool disarm,stop,actionPending,drivePending;
  Command command;Drive drive;
};
struct Snapshot {
  bool armed,autoArm,imu,pwm,moving,saved,task,ready;
  uint32_t epoch,revision,at;
  float pitch,rate,x,xRef,v,vCmd,vRef,yaw,yawRef,w,wCmd,wRef,u,y;
  bool xLimited,yawLimited,fallRearm;
  Settings settings;char message[180];
};
Mailbox inbox{};Snapshot published{};
portMUX_TYPE exchangeMux=portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE encoderMux=portMUX_INITIALIZER_UNLOCKED;
void setNotice(const char *s){snprintf(notice,sizeof(notice),"%s",s);}
Snapshot readSnapshot(){
  Snapshot s;portENTER_CRITICAL(&exchangeMux);s=published;portEXIT_CRITICAL(&exchangeMux);return s;
}
void publishSnapshot(){
  Snapshot s{};
  s.armed=balActive;s.autoArm=autoArm;s.imu=imuReady;s.pwm=pwmReady;s.moving=motionEngaged;
  s.saved=nvsSaved;s.task=taskReady;s.epoch=epoch;s.revision=configRevision;s.at=millis();
  s.ready=imuReady&&imuInitDone&&pwmReady&&micros()-sampleTime<MAX_LOOP_US&&fabsf(imuPitch-cfg.balanceOffset)<RESUME_ANGLE;
  s.pitch=isfinite(imuPitch)?imuPitch:0;s.rate=isfinite(imuPitchRate)?imuPitchRate:0;s.x=positionTicks;s.xRef=positionRef;
  s.v=velocityTicks;s.vCmd=speedCommand;s.vRef=speedTarget;s.yaw=isfinite(headingDeg)?headingDeg:0;s.yawRef=headingRef;
  s.w=isfinite(headingRate)?headingRate:0;s.wCmd=yawRateCommand;s.wRef=yawRateTarget;s.u=baseCmd;s.y=yawCmd;
  s.xLimited=positionLimited;s.yawLimited=headingLimited;s.fallRearm=fallRearm;s.settings=cfg;
  snprintf(s.message,sizeof(s.message),"%s",notice);
  portENTER_CRITICAL(&exchangeMux);published=s;portEXIT_CRITICAL(&exchangeMux);
}

// ==================== 03. MPU: ĐỌC CẢM BIẾN, HIỆU CHUẨN GYRO VÀ BỘ LỌC GÓC ====================
constexpr uint8_t MPU_ADDR   = 0x68;
constexpr float   GYRO_SCALE = 1.0f / 65.5f;
constexpr float   RAD2DEG    = 57.2957795f;

uint8_t mpuReadReg(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  if (Wire.requestFrom((int)MPU_ADDR, 1) != 1) return 0;
  return Wire.read();
}

bool mpuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuReadRaw(ImuRaw& r) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 14) != 14) return false;
  uint8_t b[14];
  Wire.readBytes(b, 14);
  r.ay = (int16_t)((b[2]  << 8) | b[3]);
  r.az = (int16_t)((b[4]  << 8) | b[5]);
  r.gx = (int16_t)((b[8]  << 8) | b[9]);
  r.gz = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

bool imuBegin() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(10);

  const uint8_t id = mpuReadReg(0x75);
  if (id != 0x70 && id != 0x71 && id != 0x73) {
    return false;
  }
  if (!mpuWriteReg(0x6B, 0x80)) return false;
  delay(100);
  if (!mpuWriteReg(0x6B, 0x01)) return false;
  delay(50);
  if (!mpuWriteReg(0x19, 0x00)) return false;
  if (!mpuWriteReg(0x1A, 0x03)) return false;
  if (!mpuWriteReg(0x1B, 0x08)) return false;
  if (!mpuWriteReg(0x1C, 0x00)) return false;
  if (!mpuWriteReg(0x1D, 0x03)) return false;
  delay(50);
  return true;
}

bool imuCalibrateGyro() {
  float sum = 0.0f, sumZ=0.0f;
  int ok = 0;
  ImuRaw r;
  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    if (mpuReadRaw(r)) {
      sum += r.gx * GYRO_SCALE;
      sumZ += r.gz * GYRO_SCALE;
      ok++;
    }
    delay(2);
  }
  if (ok != GYRO_CAL_SAMPLES) return false;
  gyroBias = sum / ok;
  yawBias=sumZ/ok;
  headingDeg=headingRate=0;
  imuInitDone=false;
  return true;
}

bool imuUpdate(float dt) {
  ImuRaw r;
  if (!mpuReadRaw(r)) return false;

  const float accAngle = IMU_PITCH_SIGN * atan2f((float)r.ay, (float)r.az) * RAD2DEG;
  imuPitchRate = IMU_GYRO_SIGN * (r.gx * GYRO_SCALE - gyroBias);

  headingRate=IMU_YAW_SIGN*(r.gz*GYRO_SCALE-yawBias);
  headingDeg+=headingRate*dt;

  if (!imuInitDone) {
    imuPitch = accAngle;
    imuInitDone = true;
  } else {
    imuPitch = COMP_ALPHA * (imuPitch + imuPitchRate * dt) + (1.0f - COMP_ALPHA) * accAngle;
  }
  return true;
}


// ==================== 04. ENCODER VÀ MOTOR ====================
Encoder encL = {PIN_ENC_L_A, PIN_ENC_L_B, ENC_L_INVERT, 0, 0, 0};
Encoder encR = {PIN_ENC_R_A, PIN_ENC_R_B, ENC_R_INVERT, 0, 0, 0};

static const DRAM_ATTR int8_t ENC_TABLE[16] = {
   0,  1, -1,  0,
  -1,  0,  0,  1,
   1,  0,  0, -1,
   0, -1,  1,  0};

void IRAM_ATTR encoderIsr(void* arg) {
  Encoder* e = static_cast<Encoder*>(arg);
  portENTER_CRITICAL_ISR(&encoderMux);
  const uint8_t curr = (gpio_get_level((gpio_num_t)e->pinA) << 1) | gpio_get_level((gpio_num_t)e->pinB);
  e->count = (int32_t)((uint32_t)e->count + (int32_t)ENC_TABLE[(e->prev << 2) | curr]);
  e->prev = curr;
  portEXIT_CRITICAL_ISR(&encoderMux);
}

void encoderBegin(Encoder& e) {
  pinMode(e.pinA, INPUT_PULLUP);
  pinMode(e.pinB, INPUT_PULLUP);
  e.prev = (digitalRead(e.pinA) << 1) | digitalRead(e.pinB);
  attachInterruptArg(digitalPinToInterrupt(e.pinA), encoderIsr, &e, CHANGE);
  attachInterruptArg(digitalPinToInterrupt(e.pinB), encoderIsr, &e, CHANGE);
}

void motorBegin() {
  pinMode(PIN_STBY, OUTPUT); digitalWrite(PIN_STBY, LOW);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  bool a=ledcAttach(PIN_PWMA, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  bool b=ledcAttach(PIN_PWMB, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  pwmReady=a&&b;
}

void motorEnable(bool on) {
  digitalWrite(PIN_STBY, on ? HIGH : LOW);
}

void motorDrive(int pwmPin, int in1, int in2, bool invert, float deadzone, float trim, float cmd) {
  cmd *= trim;
  if (invert) cmd = -cmd;
  cmd = constrain(cmd, -1.0f, 1.0f);
  const float mag = fabsf(cmd);

  uint32_t duty = 0;
  if (mag > 0.001f) {
    const float scaled = deadzone + mag * (1.0f - deadzone);
    duty = (uint32_t)(scaled * PWM_MAX);
  }

  if (cmd > 0.001f)       { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (cmd < -0.001f) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                    { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
  ledcWrite(pwmPin, duty);
}

void motorSet(float left, float right) {
  motorDrive(PIN_PWMA, PIN_AIN1, PIN_AIN2, MOTOR_L_INVERT, cfg.deadzoneL, cfg.trimL, left);
  motorDrive(PIN_PWMB, PIN_BIN1, PIN_BIN2, MOTOR_R_INVERT, cfg.deadzoneR, cfg.trimR, right);
}

void motorStop() {
  motorSet(0.0f, 0.0f);
  motorEnable(false);
}


void wheelSnapshot(int32_t &l,int32_t &r){
  portENTER_CRITICAL(&encoderMux);l=encL.count;r=encR.count;portEXIT_CRITICAL(&encoderMux);
}
void wheelReset(){
  int32_t l,r;wheelSnapshot(l,r);encL.lastCount=l;encR.lastCount=r;
  positionTicks=velocityTicks=0;
  positionRef=headingRef=0;
}
void wheelUpdate(float dt){
  int32_t l,r;wheelSnapshot(l,r);
  int32_t dl=(int32_t)((uint32_t)l-(uint32_t)encL.lastCount);
  int32_t dr=(int32_t)((uint32_t)r-(uint32_t)encR.lastCount);
  encL.lastCount=l;encR.lastCount=r;
  float left=(encL.invert?-1.0f:1.0f)*dl;
  float right=(encR.invert?-1.0f:1.0f)*dr;
  float travel=0.5f*(left+right);
  positionTicks+=travel;
  float beta=dt/(SPEED_TAU+dt);
  velocityTicks+=beta*(travel/dt-velocityTicks);

}
// ==================== 05. MỤC TIÊU CHUYỂN ĐỘNG VÀ PHẢN HỒI ====================
void clearMotion(){
  speedCommand=speedTarget=yawRateCommand=yawRateTarget=0;
  driveLive=motionEngaged=positionLimited=headingLimited=false;
}
void resetTargets(){positionRef=positionTicks;headingRef=headingDeg;}
void haltBalance(const char *reason){
  balActive=false;autoArm=false;fallRearm=false;uprightSince=0;clearMotion();
  baseCmd=yawCmd=0;motorStop();for(float &t:terms)t=0;
  ++epoch;setNotice(reason);
}
void stopMotion(const char *reason){
  speedCommand=yawRateCommand=0;driveLive=false;++epoch;setNotice(reason);
}
bool canArm(){
  return imuReady&&imuInitDone&&pwmReady&&micros()-sampleTime<MAX_LOOP_US&&
         fabsf(imuPitch-cfg.balanceOffset)<RESUME_ANGLE;
}
void armBalance(){
  clearMotion();baseCmd=yawCmd=0;motorSet(0,0);balActive=true;autoArm=false;
  ++epoch;motorEnable(true);setNotice("Đã ARM — giữ vị trí và hướng mục tiêu");
}
float slewRate(float current,float requested,float accel,float decel,float dt){
  float goal=current*requested<0?0:requested;
  float step=(fabsf(goal)>fabsf(current)?accel:decel)*dt;
  return current<goal?fminf(current+step,goal):fmaxf(current-step,goal);
}
float advanceReference(float ref,float measured,float step,float lead,bool &limited){
  float error=ref-measured,allowed=step;
  if(step>0)allowed=fminf(step,fmaxf(0.0f,lead-error));
  else if(step<0)allowed=fmaxf(step,fminf(0.0f,-lead-error));
  limited=fabsf(allowed-step)>1e-6f;return ref+allowed;
}
void expireDrive(uint32_t now){
  if(driveLive&&now-lastDrive>=DRIVE_TIMEOUT_MS)stopMotion("Mất lệnh giữ nút — giảm tốc và giữ mốc");
}
void acceptDrive(const Drive &d){
  expireDrive(millis());
  if(!balActive||d.epoch!=epoch||millis()-d.stamp>=DRIVE_TIMEOUT_MS)return;
  if(!isfinite(d.v)||!isfinite(d.w)||fabsf(d.v)>cfg.maxSpeed||fabsf(d.w)>cfg.maxYawRate)return;
  if(d.w!=0&&cfg.headingOn==0){setNotice("Bật giữ hướng trước khi rẽ hoặc xoay");return;}
  if(d.v==0&&d.w==0){stopMotion("Đang giảm tốc dừng");return;}
  if(!motionEngaged)resetTargets();
  motionEngaged=driveLive=true;speedCommand=d.v;yawRateCommand=d.w;lastDrive=d.stamp;
  setNotice("Đang chạy — nhả nút để dừng");
}
void updateMotion(float dt){
  expireDrive(millis());
  float oldV=speedTarget,oldW=yawRateTarget;
  speedTarget=slewRate(speedTarget,speedCommand,cfg.speedAccel,cfg.speedDecel,dt);
  yawRateTarget=slewRate(yawRateTarget,yawRateCommand,cfg.yawAccel,cfg.yawDecel,dt);
  positionLimited=headingLimited=false;
  if(motionEngaged){
    positionRef=advanceReference(positionRef,positionTicks,(oldV+speedTarget)*0.5f*dt,cfg.positionLead,positionLimited);
    headingRef=advanceReference(headingRef,headingDeg,(oldW+yawRateTarget)*0.5f*dt,cfg.headingLead,headingLimited);
    if(!driveLive&&speedTarget==0&&yawRateTarget==0){motionEngaged=false;setNotice("Giữ mốc cuối sau chuyển động");}
  }
}
void computeControl(){
  const float K[4]={cfg.kx,cfg.kv,-cfg.ka,-cfg.kg};
  const float state[4]={cfg.positionOn?positionTicks-positionRef:0,
    velocityTicks-speedTarget,imuPitch-cfg.balanceOffset,imuPitchRate};
  baseCmd=0;for(int i=0;i<4;i++){terms[i]=-K[i]*state[i];baseCmd+=terms[i];}
  if(!isfinite(baseCmd)){haltBalance("Lỗi tính toán cân bằng — đã DISARM");return;}
  baseCmd=constrain(baseCmd,-1.0f,1.0f);
  float requested=cfg.headingOn?cfg.headingP*(headingRef-headingDeg)+cfg.headingD*(yawRateTarget-headingRate):0;
  if(!isfinite(requested)){haltBalance("Lỗi tính toán hướng — đã DISARM");return;}
  float room=fminf(cfg.yawLimit,1.0f-fabsf(baseCmd));
  yawCmd=constrain(requested,-room,room);
}

// ==================== 06. TÁC VỤ CÂN BẰNG 200 Hz ====================
bool processCommand(const Command &c){
  if(c.epoch!=epoch||millis()-c.stamp>1000){setNotice("Lệnh đã cũ — hãy thao tác lại");return false;}
  if(c.action==ARM){
    if(balActive)setNotice("Xe đã ARM");
    else if(canArm())armBalance();
    else setNotice("Chưa ARM: dựng thẳng xe và kiểm tra IMU/PWM");
    return false;
  }
  if(c.action==ZERO){
    if(motionEngaged)setNotice("Chờ giảm tốc xong trước khi lấy mốc mới");
    else {resetTargets();++epoch;setNotice("Đã lấy vị trí và hướng hiện tại làm mốc");}
    return false;
  }
  if(balActive||autoArm){setNotice("Bấm DISARM trước khi chỉnh hoặc lưu thông số / calib");return false;}
  if(c.action==APPLY){
    if(!validSettings(c.settings)){setNotice("Thông số không hợp lệ — giữ bộ số hiện tại");return false;}
    cfg=c.settings;nvsSaved=false;++configRevision;++epoch;
    setNotice("Đã áp dụng vào RAM; bấm Lưu khởi động nếu muốn giữ sau reset");
  }else if(c.action==DEFAULTS){
    cfg=defaultSettings();nvsSaved=false;++configRevision;++epoch;
    setNotice("Đã khôi phục bộ số từ file mới nhất vào RAM");
  }else if(c.action==SAVE){
    nvsSaved=saveSettings();++epoch;
    setNotice(nvsSaved?"Đã lưu bộ số đang áp dụng vào NVS":"Lưu NVS thất bại; RAM vẫn giữ bộ số hiện tại");
    return true;
  }else if(c.action==CALIBRATE){
    motorStop();clearMotion();imuInitDone=false;imuReady=false;setNotice("Đang calib — giữ xe bất động");publishSnapshot();
    imuReady=imuBegin()&&imuCalibrateGyro();wheelReset();resetTargets();sampleTime=0;
    uprightSince=0;fallRearm=false;autoArm=imuReady&&pwmReady&&cfg.autoArmBoot!=0;++epoch;
    setNotice(imuReady?"Calib xong — đã chốt mốc vị trí và hướng":"Calib lỗi — kiểm tra cảm biến");
    return true;
  }
  return false;
}
bool serviceMailbox(){
  Mailbox m;
  portENTER_CRITICAL(&exchangeMux);m=inbox;inbox={};portEXIT_CRITICAL(&exchangeMux);
  if(m.disarm){haltBalance("Đã DISARM — motor tắt, tự arm bị khóa");return false;}
  if(m.stop){stopMotion("Đang giảm tốc dừng");return false;}
  if(m.actionPending&&processCommand(m.command))return true;
  if(m.drivePending)acceptDrive(m.drive);
  return false;
}
void controlStep(uint32_t elapsed){
  const uint32_t start=micros();
  if(elapsed>MAX_LOOP_US){haltBalance("Trễ vòng điều khiển — đã DISARM");imuInitDone=false;}
  float dt=elapsed>MAX_LOOP_US?0.005f:elapsed*1e-6f;
  if(!imuReady)return;
  bool valid=imuUpdate(dt);
  if(!valid||!isfinite(imuPitch)||!isfinite(imuPitchRate)||!isfinite(headingDeg)||!isfinite(headingRate)){
    haltBalance("Lỗi IMU — DISARM rồi calib lại");imuReady=false;return;
  }
  if(micros()-start>10000){haltBalance("Đọc IMU quá chậm — đã DISARM");return;}
  sampleTime=micros();wheelUpdate(dt);
  if(autoArm&&!balActive&&pwmReady){
    if(fabsf(imuPitch-cfg.balanceOffset)<RESUME_ANGLE){
      if(uprightSince==0)uprightSince=millis();
      else if(millis()-uprightSince>=(fallRearm?FALL_REARM_MS:AUTO_ARM_MS)){
        if(fallRearm){fallRearm=false;resetTargets();}
        armBalance();
      }
    }else uprightSince=0;
  }
  if(balActive){
    if(fabsf(imuPitch-cfg.balanceOffset)>FALL_ANGLE){
      if(FALL_AUTO_REARM&&pwmReady){
        haltBalance("Xe đổ — motor tắt. Dựng thẳng xe, giữ yên 3 s để tự cân bằng lại");
        autoArm=true;fallRearm=true;
      }else haltBalance("Xe vượt ngưỡng ngã — đã DISARM");
      return;
    }
    updateMotion(dt);computeControl();
    if(balActive)motorSet(baseCmd-yawCmd,baseCmd+yawCmd);
  }
}
void controlTask(void *){
  TickType_t wake=xTaskGetTickCount();lastControl=micros();
  const TickType_t period=pdMS_TO_TICKS(5);
  for(;;){
    vTaskDelayUntil(&wake,period);
    uint32_t now=micros(),elapsed=now-lastControl;lastControl=now;
    bool slow=serviceMailbox();
    if(slow){wake=xTaskGetTickCount();lastControl=micros();publishSnapshot();continue;}
    controlStep(elapsed);publishSnapshot();
    if(elapsed>MAX_LOOP_US||micros()-now>=LOOP_US)wake=xTaskGetTickCount();
  }
}


// ==================== 07. LIÊN KẾT UART VỚI ESP CAM ====================
constexpr uart_port_t LINK_UART = UART_NUM_1;
constexpr int PIN_LINK_TX = 21;
constexpr int PIN_LINK_RX = 42;
constexpr uint32_t TELEMETRY_MS = 20;
constexpr uint32_t NOTICE_REPEAT_MS = 500;
uint32_t bootId=0;
volatile bool settingsWanted=true;

bool linkBegin(){
  uart_config_t c{};
  c.baud_rate=(int)rl::BAUD;c.data_bits=UART_DATA_8_BITS;c.parity=UART_PARITY_DISABLE;
  c.stop_bits=UART_STOP_BITS_1;c.flow_ctrl=UART_HW_FLOWCTRL_DISABLE;c.source_clk=UART_SCLK_DEFAULT;
  if(uart_driver_install(LINK_UART,1024,1024,0,nullptr,0)!=ESP_OK)return false;
  if(uart_param_config(LINK_UART,&c)!=ESP_OK)return false;
  if(uart_set_pin(LINK_UART,PIN_LINK_TX,PIN_LINK_RX,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE)!=ESP_OK)return false;
  gpio_pullup_en((gpio_num_t)PIN_LINK_RX);
  return true;
}
void linkSend(uint8_t type,const void *payload,size_t n){
  uint8_t f[rl::MAX_FRAME];
  size_t len=rl::encode(type,payload,(uint8_t)n,f);
  uart_write_bytes(LINK_UART,(const char*)f,len);
}
void linkAck(uint8_t req,uint8_t result){
  if(req==0)return;
  rl::AckMsg a{};a.req=req;a.result=result;linkSend(rl::MSG_ACK,&a,sizeof(a));
}
void sendTelemetry(const Snapshot &s){
  rl::TelemetryMsg t{};
  t.boot=bootId;t.epoch=s.epoch;t.revision=s.revision;t.at=s.at;
  t.pitch=s.pitch;t.rate=s.rate;t.x=s.x;t.xRef=s.xRef;t.v=s.v;t.vCmd=s.vCmd;t.vRef=s.vRef;
  t.yaw=s.yaw;t.yawRef=s.yawRef;t.w=s.w;t.wCmd=s.wCmd;t.wRef=s.wRef;t.u=s.u;t.y=s.y;
  t.maxSpeed=s.settings.maxSpeed;t.maxYawRate=s.settings.maxYawRate;
  t.armHoldMs=(uint16_t)(s.fallRearm?FALL_REARM_MS:AUTO_ARM_MS);
  t.flags=(s.armed?rl::F_ARMED:0)|(s.autoArm?rl::F_AUTOARM:0)|(s.imu?rl::F_IMU:0)|(s.pwm?rl::F_PWM:0)|
          (s.moving?rl::F_MOVING:0)|(s.saved?rl::F_SAVED:0)|(s.task?rl::F_TASK:0)|(s.ready?rl::F_READY:0);
  t.flags2=(s.xLimited?rl::F2_XLIMITED:0)|(s.yawLimited?rl::F2_YAWLIMITED:0)|
           (s.settings.headingOn!=0?rl::F2_HEADING_ON:0)|(s.fallRearm?rl::F2_FALL_REARM:0);
  linkSend(rl::MSG_TELEMETRY,&t,sizeof(t));
}
void sendSettings(const Snapshot &s){
  rl::SettingsMsg m{};m.revision=s.revision;m.values=s.settings;linkSend(rl::MSG_SETTINGS,&m,sizeof(m));
}
void sendNotice(const char *text){
  size_t n=strnlen(text,rl::MAX_PAYLOAD);linkSend(rl::MSG_NOTICE,text,n);
}
void handleLinkFrame(const rl::Parser &p){
  switch(p.type()){
    case rl::MSG_DRIVE:{
      rl::DriveMsg m;if(!p.read(m))return;
      Drive d{};d.v=m.v;d.w=m.w;d.epoch=m.epoch;d.stamp=m.stamp;
      portENTER_CRITICAL(&exchangeMux);
      bool busy=inbox.disarm||inbox.stop;
      if(!busy){inbox.drive=d;inbox.drivePending=true;}
      portEXIT_CRITICAL(&exchangeMux);
      linkAck(m.req,busy?rl::ACK_BUSY:rl::ACK_OK);return;
    }
    case rl::MSG_STOP:{
      rl::SimpleMsg m;if(!p.read(m))return;
      portENTER_CRITICAL(&exchangeMux);
      inbox.stop=true;inbox.drivePending=false;inbox.actionPending=false;
      portEXIT_CRITICAL(&exchangeMux);
      linkAck(m.req,rl::ACK_OK);return;
    }
    case rl::MSG_DISARM:{
      rl::SimpleMsg m;if(!p.read(m))return;
      portENTER_CRITICAL(&exchangeMux);inbox={};inbox.disarm=true;portEXIT_CRITICAL(&exchangeMux);
      linkAck(m.req,rl::ACK_OK);return;
    }
    case rl::MSG_ACTION:{
      rl::ActionMsg m;if(!p.read(m))return;
      if(m.action<ARM||m.action>DEFAULTS){linkAck(m.req,rl::ACK_REJECT);return;}
      Command c{};c.action=(Action)m.action;c.epoch=m.epoch;c.stamp=m.stamp;c.settings=m.settings;
      portENTER_CRITICAL(&exchangeMux);
      bool busy=inbox.actionPending||inbox.disarm||inbox.stop;
      if(!busy){inbox.command=c;inbox.actionPending=true;}
      portEXIT_CRITICAL(&exchangeMux);
      linkAck(m.req,busy?rl::ACK_BUSY:rl::ACK_OK);return;
    }
    case rl::MSG_GET_SETTINGS:
      settingsWanted=true;return;
    default:return;
  }
}
void linkTask(void *){
  rl::Parser parser;uint8_t rx[128];
  uint32_t lastTele=0,lastNotice=0,sentRevision=0;char sentNotice[sizeof(notice)]="";
  for(;;){
    int n=uart_read_bytes(LINK_UART,rx,1,pdMS_TO_TICKS(TELEMETRY_MS));
    if(n>0){
      size_t more=0;uart_get_buffered_data_len(LINK_UART,&more);
      if(more>sizeof(rx)-1)more=sizeof(rx)-1;
      if(more){int m=uart_read_bytes(LINK_UART,rx+1,more,0);if(m>0)n+=m;}
      for(int i=0;i<n;i++)if(parser.push(rx[i]))handleLinkFrame(parser);
    }
    uint32_t now=millis();
    if(now-lastTele<TELEMETRY_MS)continue;
    lastTele=now;
    Snapshot s=readSnapshot();
    sendTelemetry(s);
    if(settingsWanted||s.revision!=sentRevision){settingsWanted=false;sentRevision=s.revision;sendSettings(s);}
    if(strcmp(s.message,sentNotice)!=0||now-lastNotice>=NOTICE_REPEAT_MS){
      lastNotice=now;snprintf(sentNotice,sizeof(sentNotice),"%s",s.message);sendNotice(sentNotice);
    }
  }
}

// ==================== 08. KHỞI TẠO ====================
constexpr uint32_t CPU_MHZ = 160;
void setup(){
  Serial.begin(115200);
  pinMode(PIN_STBY,OUTPUT);digitalWrite(PIN_STBY,LOW);
  if(getCpuFrequencyMhz()!=CPU_MHZ)setCpuFrequencyMhz(CPU_MHZ);
  nvsSaved=loadSettings();motorBegin();motorStop();encoderBegin(encL);encoderBegin(encR);
  delay(1000);imuReady=imuBegin()&&imuCalibrateGyro();wheelReset();resetTargets();
  do{bootId=esp_random();}while(bootId==0);
  bool linkReady=linkBegin();

  Serial.println("\n==================================================");
  Serial.println("  BalanceBot ESP32-S3 (CAN BANG) da khoi dong");
  Serial.println("  - Khong Wi-Fi. Lenh/phan hoi qua UART voi ESP cam.");
  Serial.printf ("  - UART%d %lu baud: TX=GPIO%d -> RX ESP cam, RX=GPIO%d <- TX ESP cam\n",
                 (int)LINK_UART,(unsigned long)rl::BAUD,PIN_LINK_TX,PIN_LINK_RX);
  Serial.printf ("  - IMU %s | PWM %s | UART %s | CPU %lu MHz\n",imuReady?"OK":"LOI",pwmReady?"OK":"LOI",
                 linkReady?"OK":"LOI",(unsigned long)getCpuFrequencyMhz());
  Serial.println("==================================================\n");

  autoArm=imuReady&&pwmReady&&linkReady&&cfg.autoArmBoot!=0;
  setNotice(!linkReady?"Lỗi khởi tạo UART liên kết — tự Arm bị khóa":(!imuReady||!pwmReady)?"Khởi tạo cảm biến/PWM lỗi — motor tắt":
            autoArm?"Calib xong — chờ dựng thẳng 0.7 s để tự Arm":"Calib xong — bấm ARM khi dựng thẳng xe");
  taskReady=true;publishSnapshot();
  BaseType_t ok=xTaskCreatePinnedToCore(controlTask,"balance200Hz",6144,nullptr,4,nullptr,1);
  if(ok!=pdPASS){taskReady=false;haltBalance("Không tạo được tác vụ cân bằng — motor tắt");publishSnapshot();}
  if(linkReady)xTaskCreatePinnedToCore(linkTask,"uartLink",4096,nullptr,3,nullptr,0);
}
void loop(){
  vTaskDelay(portMAX_DELAY);
}
