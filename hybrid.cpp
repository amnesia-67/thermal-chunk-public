/*

Authors     : Anish Subash, Caden Wate
Date        : 20 October, 2025 (end date)
Description : Base of Peltiers equation and power control     
Usage       : Flashed on ESP32

==========================

ACCURACY    : within 0.16 of setpoint for all temps 0-80C (can probably go higher, did not test)
LIMITATION  : Stabilization overshoot initally high (for about 6-10 seconds)
PROS        : Excellent at stabilization post spike, constant power draw and less overall spiking modelled around peltiers equation

==========================

------------- Authors note -------------

Caden Wate (Computer Engineering)

email   : cwate3@gmail.com

----
Anish Subash (Cybersecurity)

email   : anishsubash1@gmail.com
website : https://anishsubash.xyz
*/

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

// ========================= PIN CONFIG =========================
#define BUS_A_PIN 4      // OneWire bus A
#define BUS_B_PIN 5      // OneWire bus B
#define PIN_RPWM   9     // BTS7960 RPWM (HEAT)
#define PIN_LPWM  10     // BTS7960 LPWM (COOL)

// ========================= PWM (ESP32 Arduino-core v3 API) =========================
const uint32_t PWM_FREQ = 3000;   // 2–5 kHz OK for a TEC
const uint8_t  PWM_RES  = 10;     // 10-bit (0..1023)
inline uint32_t dutyFromPct(int pct){
  pct = constrain(pct, 0, 100);
  return (uint32_t)((((1UL << PWM_RES) - 1) * (uint32_t)pct) / 100UL);
}

// ========================= LIMITS / BANDS =========================
#define TEC_FLIPPED 0
float DEAD_BAND_C = 0.20f;   // UI display deadband
int   MIN_DUTY    = 12;      // safe min duty when active
int   MAX_DUTY    = 88;      // cap to protect H-bridge/TEC

// Hybrid handoff thresholds
float HYBRID_TIGHT_BAND_C = 1.0f;   // ≤1C => constant-power model (Algorithm 1)
float HYBRID_MID_BAND_C   = 5.0f;   // (1..5]C => lag-aware hybrid PID (Algorithm 2)

// ========================= LOOP & SENSOR SPEED =========================
const unsigned LOOP_MS   = 100;   // 10 Hz control tick
uint8_t  DS_RES          = 11;    // DS18B20: 11-bit (~375 ms conversion)
unsigned CONV_MS         = 375;

// ========================= Algorithm 1: Exponential constant-power model =========================
// p = (SP - room) * exp(K_NUM / tau);   +p = heat, -p = cool
float K_NUM   = 1.6f;
float T_COOL  = 1.13f;  // use when SP < room
float T_HEAT  = 10.2f;  // use when SP > room

const float TIGHT_BAND_C = 0.2f;  // inside this, use exact model
int   NUDGE = 2;                  // ±2% nudges for 0.2..1.0C region

// ========================= Algorithm 2: Lag-aware hybrid (g–h + 2-DOF PI-D + predictive brake) =========================
float Kp = 6.0f, Ki = 0.10f, Kd = 2.5f;
float beta = 0.20f;        // P on weighted SP (less setpoint kick)
float dFiltHz = 1.8f;      // derivative low-pass cutoff (Hz)
float Kaw = 0.8f;          // anti-windup back-calculation

// Feed-forward & scheduling
float Kff_pct_per_C = 2.2f;   // % duty per °C vs ambient/sink
float NEAR_BAND_C   = 2.0f;   // where gains/caps tighten
float HOLD_BAND_C   = 0.30f;  // learn/hold band
int   CAP_NEAR_MIN  = 32;     // min cap (%) near SP
bool  learnEnabled  = true;

// Lag compensation (g–h observer + lead + soft brake)
float GH_g = 0.35f;           // position gain (0..1)
float GH_h = 0.05f;           // velocity gain (0..1)
float LEAD_SEC = 1.2f;        // sensor/plant lag compensation
float BRAKE_LOOKAHEAD_S = 5.0f;// predictive clamp horizon
float VDOT_MAX = 1.2f;        // cap |dT/dt| (°C/s)
int   CAP_FLIP = 40;          // cap after polarity flip (%)
float FLIP_SOFTSTART_S = 4.0f;// seconds to ramp post-flip

// ========================= STATE =========================
float setpointC = 24.0f;     // user target (default 24C)
float spCmdC    = 24.0f;     // ramped/internal setpoint
float SP_RATE_CPS = 1.0f;    // °C/s setpoint slew

bool  ctrlEnabled = true;    // p1/p0
char  ctrlBus = 'A';         // default control sensor bus
int   ctrlIdx = 0;           // default sensor index

float Iterm = 0.0f;
float dFilt = 0.0f;
float lastPV = NAN;
int   lastDutyPct = 0;       // 0..100
char  lastMode = 'S';        // 'H'/'C'/'S'

// g–h observer states
float That = NAN;            // estimated true temperature
float Vhat = 0.0f;           // estimated dT/dt (°C/s)

// polarity flip softstart
unsigned long flipStartMs = 0;

// ========================= DS18B20 =========================
OneWire owA(BUS_A_PIN), owB(BUS_B_PIN);
DallasTemperature dA(&owA), dB(&owB);
typedef uint8_t DeviceAddress[8];
const int MAX_DEV = 16;
DeviceAddress addrA[MAX_DEV], addrB[MAX_DEV];
int cntA=0, cntB=0;
float tempsA[MAX_DEV], tempsB[MAX_DEV];

// ========================= TIMING =========================
unsigned long lastMs = 0;
unsigned long lastConvMs = 0;

// ========================= HOLD DUTY LEARNING (per rounded SP) =========================
float holdDutyTable[121];
inline void initHoldTable(){ for(int i=0;i<=120;i++) holdDutyTable[i] = NAN; }
inline int spIdx(){ int i=(int)lroundf(setpointC); return constrain(i,0,120); }

// ----------------- helpers -----------------
void printAddr(const DeviceAddress a){
  for (int i=0;i<8;i++){ if(a[i]<16) Serial.print('0'); Serial.print(a[i],HEX); if(i<7) Serial.print(':'); }
}
int discover(DallasTemperature& d, DeviceAddress arr[], int maxDev){
  d.begin(); d.setResolution(DS_RES); d.setWaitForConversion(false);
  d.requestTemperatures(); delay(10);
  int found = d.getDeviceCount(); if (found>maxDev) found=maxDev;
  int ok=0; for (int i=0;i<found;i++) if (d.getAddress(arr[ok], i)) ok++;
  return ok;
}
float readByAddress(DallasTemperature& d, const DeviceAddress a){ return d.getTempC((uint8_t*)a); }
void requestAll(){ dA.requestTemperatures(); dB.requestTemperatures(); }
void readAll(){ for(int i=0;i<cntA;i++) tempsA[i]=readByAddress(dA,addrA[i]);
                for(int i=0;i<cntB;i++) tempsB[i]=readByAddress(dB,addrB[i]); }
float pickPV(){
  if (ctrlBus=='A' && ctrlIdx<cntA) return tempsA[ctrlIdx];
  if (ctrlBus=='B' && ctrlIdx<cntB) return tempsB[ctrlIdx];
  return NAN;
}
inline bool tempsReady(){ return (millis()-lastConvMs)>=CONV_MS; }
inline int slewLimitDuty(int targetPct){
  const int dUmax = 10; // %/tick
  int delta = targetPct - lastDutyPct;
  if (delta >  dUmax) targetPct = lastDutyPct + dUmax;
  if (delta < -dUmax) targetPct = lastDutyPct - dUmax;
  return constrain(targetPct, 0, 100);
}

// Signed constant-power model (+ = heat, - = cool)
inline float modelPower(float sp, float room){
  float tau = (sp < room) ? T_COOL : T_HEAT;
  return (sp - room) * expf(K_NUM / tau); // signed
}

// -------- H-bridge control --------
void heatPercent(int pct){
  pct = constrain(pct, 0, 100);
  uint32_t d = dutyFromPct(pct);
#if TEC_FLIPPED
  ledcWrite(PIN_RPWM, 0);  ledcWrite(PIN_LPWM, d);
#else
  ledcWrite(PIN_LPWM, 0);  ledcWrite(PIN_RPWM, d);
#endif
  lastDutyPct = pct; lastMode = (pct>0)?'H':'S';
}
void coolPercent(int pct){
  pct = constrain(pct, 0, 100);
  uint32_t d = dutyFromPct(pct);
#if TEC_FLIPPED
  ledcWrite(PIN_LPWM, 0);  ledcWrite(PIN_RPWM, d);
#else
  ledcWrite(PIN_RPWM, 0);  ledcWrite(PIN_LPWM, d);
#endif
  lastDutyPct = pct; lastMode = (pct>0)?'C':'S';
}
void allOff(){ ledcWrite(PIN_RPWM,0); ledcWrite(PIN_LPWM,0); lastDutyPct=0; lastMode='S'; }

// ========================= MENU =========================
void printMenu(){
  Serial.println(F("\nCommands:"));
  Serial.println(F("  tNN       -> setpoint NN°C"));
  Serial.println(F("  p1/p0     -> control on/off"));
  Serial.println(F("  sel A0|B0 -> pick control sensor"));
  Serial.println(F("  list      -> list sensor addresses"));
  Serial.println(F("  hNN / cNN -> manual heat/cool NN% (control OFF)"));
  Serial.println(F("  s         -> stop outputs"));
  Serial.println(F("  db X / min X / max X       -> deadband & duty caps"));
  Serial.println(F("  res 10|11 -> DS18B20 bits"));
  // Alg2 tuners
  Serial.println(F("  kp/ki/kd/b/df/kaw X -> PID & filters"));
  Serial.println(F("  kff X  | nb X | hb X | capmin X | learn 0|1"));
  Serial.println(F("  spd X  -> SP ramp °C/s"));
  Serial.println(F("  ghg X  | ghh X | lead X | brake X | vmax X"));
  Serial.println(F("  capflip X | flipsec X"));
  // Hybrid / Alg1 tuners
  Serial.println(F("  mid X    -> mid band (Alg2) width °C (default 5.0)"));
  Serial.println(F("  tight X  -> tight band (Alg1) width °C (default 1.0)"));
  Serial.println(F("  knum X   -> Alg1 exp numerator K_NUM"));
  Serial.println(F("  tauc X   -> Alg1 cool tau"));
  Serial.println(F("  tauh X   -> Alg1 heat tau"));
  Serial.println(F("  nudge X  -> Alg1 ±nudge % (default 2)"));
}

void handleSerial(){
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n'); line.trim(); if(!line.length()) return;

  if (line=="m"){ printMenu(); return; }
  if (line=="s"){ ctrlEnabled=false; allOff(); Serial.println("STOP"); return; }
  if (line=="p1"){ ctrlEnabled=true;  Serial.println("CTRL: ON"); return; }
  if (line=="p0"){ ctrlEnabled=false; Serial.println("CTRL: OFF"); return; }

  if (line=="list"){
    Serial.print("Bus A (GPIO "); Serial.print(BUS_A_PIN); Serial.print(") = "); Serial.print(cntA); Serial.println(" device(s)");
    for (int i=0;i<cntA;i++){ Serial.print("  A[");Serial.print(i);Serial.print("] "); printAddr(addrA[i]); Serial.println(); }
    Serial.print("Bus B (GPIO "); Serial.print(BUS_B_PIN); Serial.print(") = "); Serial.print(cntB); Serial.println(" device(s)");
    for (int i=0;i<cntB;i++){ Serial.print("  B[");Serial.print(i);Serial.print("] "); printAddr(addrB[i]); Serial.println(); }
    return;
  }

  if (line[0]=='t'){ float sp=line.substring(1).toFloat();
    if (sp>-40&&sp<120){ setpointC=sp; Serial.printf("Setpoint = %.1f °C\n", setpointC);} return; }

  if (line.startsWith("sel")){
    if (line.length()>=5){
      char b = toupper(line.charAt(4)); int idx = line.substring(5).toInt();
      if ((b=='A' && idx<cntA) || (b=='B' && idx<cntB)){ ctrlBus=b; ctrlIdx=idx;
        Serial.print("Control PV = "); Serial.print(ctrlBus); Serial.println(ctrlIdx); }
      else Serial.println("Invalid selection.");
    } else Serial.println("Use: sel A0  or  sel B0");
    return;
  }

  // manual overrides
  if (line[0]=='h'){ int pct=line.substring(1).toInt(); ctrlEnabled=false; heatPercent(constrain(pct,0,100)); Serial.printf("HEAT %d%% (OFF)\n", constrain(pct,0,100)); return; }
  if (line[0]=='c'){ int pct=line.substring(1).toInt(); ctrlEnabled=false; coolPercent(constrain(pct,0,100)); Serial.printf("COOL %d%% (OFF)\n", constrain(pct,0,100)); return; }

  // Common tuners
  if (line.startsWith("db ")){ DEAD_BAND_C = max(0.05f, line.substring(3).toFloat()); Serial.printf("deadband=%.3f C\n", DEAD_BAND_C); return; }
  if (line.startsWith("min ")){ MIN_DUTY = constrain(line.substring(4).toInt(), 0, 100); Serial.printf("MIN_DUTY=%d%%\n", MIN_DUTY); return; }
  if (line.startsWith("max ")){ MAX_DUTY = constrain(line.substring(4).toInt(), 0, 100); Serial.printf("MAX_DUTY=%d%%\n", MAX_DUTY); return; }
  if (line.startsWith("res ")){
    int r = line.substring(4).toInt();
    if (r==10 || r==11){ DS_RES=r; CONV_MS = (r==10)?188:375; dA.setResolution(DS_RES); dB.setResolution(DS_RES);
      Serial.printf("DS18B20 res=%d-bit, CONV_MS=%u\n", DS_RES, CONV_MS); }
    else Serial.println("Use: res 10  or  res 11");
    return;
  }

  // Alg2 (hybrid) tuners
  if (line.startsWith("kp ")){ Kp = line.substring(3).toFloat(); Serial.printf("Kp=%.3f\n", Kp); return; }
  if (line.startsWith("ki ")){ Ki = line.substring(3).toFloat(); Serial.printf("Ki=%.3f 1/s\n", Ki); return; }
  if (line.startsWith("kd ")){ Kd = line.substring(3).toFloat(); Serial.printf("Kd=%.3f s\n", Kd); return; }
  if (line.startsWith("b ")){  beta = constrain(line.substring(2).toFloat(),0.0f,1.0f); Serial.printf("beta=%.3f\n", beta); return; }
  if (line.startsWith("df ")){ dFiltHz = max(0.1f, line.substring(3).toFloat()); Serial.printf("dFiltHz=%.2f Hz\n", dFiltHz); return; }
  if (line.startsWith("kaw ")){ Kaw = line.substring(4).toFloat(); Serial.printf("Kaw=%.3f\n", Kaw); return; }
  if (line.startsWith("kff ")){ Kff_pct_per_C = max(0.0f, line.substring(4).toFloat()); Serial.printf("Kff=%.3f %%/C\n", Kff_pct_per_C); return; }
  if (line.startsWith("nb ")){  NEAR_BAND_C = max(0.2f, line.substring(3).toFloat()); Serial.printf("NEAR_BAND=%.3f C\n", NEAR_BAND_C); return; }
  if (line.startsWith("hb ")){  HOLD_BAND_C = max(0.05f, line.substring(3).toFloat()); Serial.printf("HOLD_BAND=%.3f C\n", HOLD_BAND_C); return; }
  if (line.startsWith("capmin ")){ CAP_NEAR_MIN = constrain(line.substring(7).toInt(), 0, 100); Serial.printf("CAP_NEAR_MIN=%d%%\n", CAP_NEAR_MIN); return; }
  if (line.startsWith("learn ")){ learnEnabled = (line.substring(6).toInt()!=0); Serial.printf("learn=%d\n", (int)learnEnabled); return; }
  if (line.startsWith("spd ")){ SP_RATE_CPS = max(0.05f, line.substring(4).toFloat()); Serial.printf("SP_RATE=%.2f C/s\n", SP_RATE_CPS); return; }
  if (line.startsWith("ghg ")){ GH_g = constrain(line.substring(4).toFloat(),0.0f,1.0f); Serial.printf("GH_g=%.3f\n", GH_g); return; }
  if (line.startsWith("ghh ")){ GH_h = constrain(line.substring(4).toFloat(),0.0f,1.0f); Serial.printf("GH_h=%.3f\n", GH_h); return; }
  if (line.startsWith("lead ")){ LEAD_SEC = constrain(line.substring(5).toFloat(),0.0f,3.0f); Serial.printf("LEAD=%.2f s\n", LEAD_SEC); return; }
  if (line.startsWith("brake ")){ BRAKE_LOOKAHEAD_S = constrain(line.substring(6).toFloat(),0.0f,10.0f); Serial.printf("BRAKE_LOOKAHEAD=%.2f s\n", BRAKE_LOOKAHEAD_S); return; }
  if (line.startsWith("capflip ")){ CAP_FLIP = constrain(line.substring(8).toInt(), 0, 100); Serial.printf("CAP_FLIP=%d%%\n", CAP_FLIP); return; }
  if (line.startsWith("flipsec ")){ FLIP_SOFTSTART_S = max(0.0f, line.substring(8).toFloat()); Serial.printf("FLIP_SOFT=%.2f s\n", FLIP_SOFTSTART_S); return; }
  if (line.startsWith("vmax ")){ VDOT_MAX = max(0.2f, line.substring(5).toFloat()); Serial.printf("VDOT_MAX=%.2f C/s\n", VDOT_MAX); return; }

  // Hybrid & Alg1 tuners
  if (line.startsWith("mid ")){   HYBRID_MID_BAND_C   = max(1.0f, line.substring(4).toFloat()); Serial.printf("MID_BAND=%.2f C\n", HYBRID_MID_BAND_C); return; }
  if (line.startsWith("tight ")){ HYBRID_TIGHT_BAND_C = constrain(line.substring(6).toFloat(), 0.1f, 2.0f); Serial.printf("TIGHT_BAND=%.2f C\n", HYBRID_TIGHT_BAND_C); return; }
  if (line.startsWith("knum ")){  K_NUM = line.substring(5).toFloat(); Serial.printf("K_NUM=%.3f\n", K_NUM); return; }
  if (line.startsWith("tauc ")){  T_COOL = max(0.2f, line.substring(5).toFloat()); Serial.printf("T_COOL=%.3f\n", T_COOL); return; }
  if (line.startsWith("tauh ")){  T_HEAT = max(0.2f, line.substring(5).toFloat()); Serial.printf("T_HEAT=%.3f\n", T_HEAT); return; }
  if (line.startsWith("nudge ")){ NUDGE = constrain(line.substring(6).toInt(), 0, 10); Serial.printf("NUDGE=%d%%\n", NUDGE); return; }

  Serial.println("Unknown cmd. Type 'm' for menu.");
}

// ========================= SETUP =========================
void setup(){
  Serial.begin(115200); delay(300);
  Serial.println("\nBOOT: Hybrid controller (Alg2 outside 1C, Alg1 inside 1C; full power outside 5C)");

  bool ok1 = ledcAttach(PIN_RPWM, PWM_FREQ, PWM_RES);
  bool ok2 = ledcAttach(PIN_LPWM, PWM_FREQ, PWM_RES);
  if (!ok1 || !ok2) Serial.println("LEDC attach failed.");
  allOff();

  dA.begin(); dB.begin();
  dA.setResolution(DS_RES); dB.setResolution(DS_RES);
  dA.setWaitForConversion(false); dB.setWaitForConversion(false);

  cntA = discover(dA, addrA, MAX_DEV);
  cntB = discover(dB, addrB, MAX_DEV);

  printMenu();
  Serial.println("CSV header: time_s,PV,SP,SPcmd,A[0..],B[0..],mode,duty%");
  requestAll(); lastConvMs = millis(); lastMs = millis();
  spCmdC = setpointC;
  initHoldTable();
}

// ========================= MAIN LOOP =========================
void loop(){
  // 10 Hz tick
  if (millis() - lastMs < LOOP_MS) { handleSerial(); return; }
  float dt = (millis() - lastMs) / 1000.0f;
  lastMs = millis();
  handleSerial();

  // Setpoint ramp
  float step = SP_RATE_CPS * dt;
  if (spCmdC < setpointC) spCmdC = min(spCmdC + step, setpointC);
  else if (spCmdC > setpointC) spCmdC = max(spCmdC - step, setpointC);

  // Pipelined conversion
  static bool first = true;
  if (first || (millis() - lastConvMs) >= CONV_MS){
    if (!first) readAll();
    requestAll(); lastConvMs = millis(); first = false;

    // --------- Measurements ---------
    float pv = pickPV();
    if (!(ctrlEnabled && !isnan(pv))) {
      allOff();
      // CSV
      Serial.print(millis()/1000.0f,1); Serial.print(",");
      if (!isnan(pv)) Serial.print(pv,2); else Serial.print("NaN"); Serial.print(",");
      Serial.print(setpointC,2); Serial.print(",");
      Serial.print(spCmdC,2); Serial.print(",");
      for (int i=0;i<cntA;i++){ if (i) Serial.print("|"); Serial.print(tempsA[i],2); }
      Serial.print(",");
      for (int i=0;i<cntB;i++){ if (i) Serial.print("|"); Serial.print(tempsB[i],2); }
      Serial.print(","); Serial.print(lastMode); Serial.print(","); Serial.println(lastDutyPct);
      return;
    }

    // --------- g–h observer (lag compensation) ---------
    // Predict
    if (isnan(That)) { That = pv; Vhat = 0; }
    else { That += Vhat * dt; }
    // Update with measurement
    float resid = pv - That;
    That += GH_g * resid;
    Vhat += (GH_h * resid) / max(0.001f, dt);
    // Clamp velocity to realistic bounds
    if (Vhat > VDOT_MAX) Vhat = VDOT_MAX;
    if (Vhat < -VDOT_MAX) Vhat = -VDOT_MAX;

    // Lead compensation (predictive PV used by control)
    float pvCtrl = That + LEAD_SEC * Vhat;

    // Use Vhat as derivative signal (already filtered); blend for smoothness
    dFilt = 0.8f * dFilt + 0.2f * Vhat;

    // --------- Region selection (Hybrid) ----------
    float errToTarget = setpointC - pvCtrl;   // for mode/limits/learning
    float abserr      = fabsf(errToTarget);

    // Ambient (room) estimate for Alg1/FF
    float ambient = (!isnan(tempsA[2])) ? tempsA[2] : pv;

    // ===== Region 3: far from SP (>5C): full power toward SP =====
    if (abserr > HYBRID_MID_BAND_C){
      int duty = slewLimitDuty(MAX_DUTY);
      if (errToTarget > 0) heatPercent(duty); else coolPercent(duty);

      // Gentle I-term bleed to avoid windup when saturated
      Iterm *= 0.95f;

    // ===== Region 2: mid band (1..5]C: use Algorithm 2 (lag-aware hybrid) =====
    } else if (abserr > HYBRID_TIGHT_BAND_C){
      // Gain scheduling near SP
      float KpEff = Kp, KiEff = Ki, KdEff = Kd;
      if (fabsf(errToTarget) < NEAR_BAND_C){ KpEff *= 0.6f; KdEff *= 1.6f; }
      if (fabsf(errToTarget) > 1.0f) { KiEff = 0.0f; Iterm *= 0.95f; }
      else { KiEff *= 1.4f; }

      // Feed-forward baseline from ambient/sink
      float u_ff_mag = Kff_pct_per_C * fabsf(setpointC - ambient);
      u_ff_mag = constrain(u_ff_mag, (float)MIN_DUTY, (float)(MAX_DUTY-4));

      // Blend learned hold near SP
      int idx = spIdx();
      if (learnEnabled && !isnan(holdDutyTable[idx])) {
        float w = constrain(1.0f - (abserr/NEAR_BAND_C), 0.0f, 1.0f);
        u_ff_mag = (1.0f - w) * u_ff_mag + w * holdDutyTable[idx];
      }

      // 2-DOF PI-D feedback (on pvCtrl)
      float P = KpEff * (beta*spCmdC - pvCtrl);
      float I = Iterm;
      float D = -KdEff * dFilt;
      float u_fb = P + I + D;

      float signFF = (errToTarget >= 0.0f) ? 1.0f : -1.0f;
      float u_unsat = signFF * u_ff_mag + u_fb;

      // Polarity & hysteresis using spCmdC
      const float POL_HYST_C = 0.30f;
      char desiredMode;
      if      (pvCtrl < spCmdC - DEAD_BAND_C - POL_HYST_C) desiredMode = 'H';
      else if (pvCtrl > spCmdC + DEAD_BAND_C + POL_HYST_C) desiredMode = 'C';
      else desiredMode = lastMode;

      // Predictive brake
      float cap = (float)MAX_DUTY;
      if (abserr < NEAR_BAND_C) {
        float t = abserr / NEAR_BAND_C; // 1 far -> 0 at SP
        float capNear = CAP_NEAR_MIN + (MAX_DUTY - CAP_NEAR_MIN) * t;
        cap = min(cap, capNear);
      }
      float t_cross = INFINITY;
      if (fabsf(Vhat) > 1e-3f) {
        float tc = (spCmdC - pvCtrl) / Vhat; // use ramped SP error for crossing time
        if (tc > 0) t_cross = tc;
      }
      if (t_cross < BRAKE_LOOKAHEAD_S) {
        float scale = constrain(t_cross / BRAKE_LOOKAHEAD_S, 0.20f, 1.0f);
        cap = min(cap, CAP_NEAR_MIN + (MAX_DUTY - CAP_NEAR_MIN) * scale);
        u_ff_mag *= scale;
        u_unsat = signFF * u_ff_mag + u_fb;
      }

      // Soft-start after polarity flip
      static char prevDesired = 'S';
      if (desiredMode != prevDesired && desiredMode != 'S') {
        flipStartMs = millis();
        prevDesired = desiredMode;
      }
      if (flipStartMs) {
        float sec = (millis() - flipStartMs)/1000.0f;
        if (sec < FLIP_SOFTSTART_S) {
          float ramp = CAP_FLIP + (MAX_DUTY - CAP_FLIP) * (sec / FLIP_SOFTSTART_S);
          cap = min(cap, ramp);
        } else {
          flipStartMs = 0;
        }
      }

      // Duty mapping + slew
      int dutyAbsCmd = constrain((int)fabsf(u_unsat), 0, (int)cap);
      int duty = (dutyAbsCmd==0) ? 0 : constrain(max(dutyAbsCmd, MIN_DUTY), MIN_DUTY, (int)cap);
      duty = slewLimitDuty(duty);

      if (desiredMode == 'H') heatPercent(duty);
      else if (desiredMode == 'C') coolPercent(duty);
      else { allOff(); duty = 0; }

      // Anti-windup + conditional integration
      float u_sat = (lastMode=='H') ? (float)lastDutyPct : (lastMode=='C' ? -(float)lastDutyPct : 0.0f);
      float aw = Kaw * (u_sat - u_unsat);
      bool holdI = ((lastMode=='H' && u_unsat<0) || (lastMode=='C' && u_unsat>0));
      if (!holdI) Iterm += (KiEff * (spCmdC - pvCtrl) + aw) * dt;
      else Iterm *= 0.95f;

      // Learn steady hold
      if (learnEnabled) {
        const float dStable = 0.015f; // °C/s
        if (abserr <= HOLD_BAND_C && fabsf(Vhat) <= dStable && lastDutyPct > 0) {
          float prev = holdDutyTable[idx];
          float newd = isnan(prev) ? (float)lastDutyPct : (0.80f*prev + 0.20f*(float)lastDutyPct);
          holdDutyTable[idx] = constrain(newd, (float)MIN_DUTY, (float)MAX_DUTY);
        }
      }

    // ===== Region 1: tight band (≤1C): use Algorithm 1 (constant-power model) =====
    } else {
      // Compute signed model power from ambient (A2) toward SP
      float p_model = modelPower(setpointC, ambient);        // signed (+ heat, - cool)
      int   baseMag = (int)roundf(fabsf(p_model));
      baseMag = constrain(baseMag, MIN_DUTY, MAX_DUTY);

      // Keep polarity equal to the model’s sign inside ±1C (do NOT flip)
      int signMode = (p_model >= 0.0f) ? +1 : -1;

      int mag = baseMag;
      if (abserr > TIGHT_BAND_C){
        // outside ±0.2C but within ±1C → apply ±NUDGE% without changing sign
        if (errToTarget > 0){
          // too cold -> want warmer
          mag += (signMode > 0) ? +NUDGE : -NUDGE;  // more heat OR less cool
        } else {
          // too hot -> want colder
          mag += (signMode > 0) ? -NUDGE : +NUDGE;  // less heat OR more cool
        }
      }
      mag = constrain(mag, MIN_DUTY, MAX_DUTY);
      mag = slewLimitDuty(mag);

      if (signMode > 0) heatPercent(mag);
      else              coolPercent(mag);

      // Bleed I-term so Alg2 re-engages cleanly when exiting tight band
      Iterm *= 0.95f;
    }

    // CSV
    Serial.print(millis()/1000.0f,1); Serial.print(",");
    Serial.print(pv,2);                Serial.print(",");
    Serial.print(setpointC,2);         Serial.print(",");
    Serial.print(spCmdC,2);            Serial.print(",");
    for (int i=0;i<cntA;i++){ if (i) Serial.print("|"); Serial.print(tempsA[i],2); }
    Serial.print(",");
    for (int i=0;i<cntB;i++){ if (i) Serial.print("|"); Serial.print(tempsB[i],2); }
    Serial.print(",");
    Serial.print(lastMode); Serial.print(",");
    Serial.println(lastDutyPct);
  }
}
