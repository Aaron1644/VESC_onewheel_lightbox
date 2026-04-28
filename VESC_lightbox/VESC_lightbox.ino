#include "./src/Adafruit_NeoPixel/Adafruit_NeoPixel.h"
#include "./src/VescUart/src/VescUart.h"
#include <math.h>

// ============================================================
//  TEST MODE — uncomment to enable simulation without VESC
// ============================================================
// #define TEST_MODE

// Uncomment for Serial Monitor output (USB connected)
// #define SERIAL_DEBUG

// ============================================================
//  PINS AND STRIPS
// ============================================================
#define PIN_FRONT           11
#define PIN_REAR             7
#define NUM_LEDS_FRONT      10
#define NUM_LEDS_REAR       10

// ============================================================
//  BATTERY — 16S Samsung 50S
// ============================================================
#define BAT_CELLS           16

// ============================================================
//  RPM THRESHOLDS (eRPM = mechanical RPM x 30)
// ============================================================
#define RPM_DEAD_ZONE        300
#define RPM_FULL_BRIGHTNESS 8800
#define RPM_FILTER          0.09f

// ============================================================
//  FOOTPAD / PITCH
// ============================================================
#define PITCH_LEVEL_DEG     15.0f
#define PITCH_UPRIGHT_DEG   40.0f
#define FOOTPAD_BRIGHTNESS   50
#define ADC_THRESHOLD        0.5f

// ============================================================
//  TIMERS
// ============================================================
#define BRAKE_HOLD_MS        700UL

// ============================================================
//  BRIGHTNESS [0-255]
// ============================================================
#define BRIGHT_RIDE_MIN      70
#define BRIGHT_REAR_NORMAL  128
#define BRIGHT_IDLE          60

// ============================================================
//  COLOR TRANSITIONS (lerp) ~33 Hz
// ============================================================
#define LERP_NORMAL         0.110f

// ============================================================
//  DUTY CYCLE WARNING
// ============================================================
#define DUTY_WARN_THRESHOLD  0.90f
#define DUTY_BLINK_MS         80UL

// ============================================================
//  BRAKE DETECTION via RPM delta
// ============================================================
#define BRAKE_RPM_THRESHOLD  1000
#define BRAKE_WINDOW_MS       350UL
#define BRAKE_HIST            5

// ============================================================
//  BATTERY OPTIMIZATION
// ============================================================
#define VOLTAGE_UPDATE_MS   5000UL

// ============================================================

Adafruit_NeoPixel stripFront(NUM_LEDS_FRONT, PIN_FRONT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripRear (NUM_LEDS_REAR,  PIN_REAR,  NEO_GRB + NEO_KHZ800);

VescUart vesc;

// --- Current smoothed colors (float) ---
float cFR = 0, cFG = 0, cFB = 0;
float cRR = 0, cRG = 0, cRB = 0;

// --- Global state ---
float         smoothRPM      = 0;
float         lastDirBlend   = 1.0f;
unsigned long brakingUntil   = 0;
bool          everMoved      = false;
float         cachedVoltage  = 60.5f;
unsigned long lastVoltageMs  = 0;

// --- Cached VESC values ---
long    lastRPM        = 0;
float   lastDuty       = 0.0f;
float   lastPitch      = 0.0f;
float   lastAdc1       = 0.0f;
float   lastAdc2       = 0.0f;
uint8_t lastBeepReason = 0;

// --- Brake detection ring buffer ---
long          rpmHist[BRAKE_HIST]   = {0};
unsigned long rpmHistMs[BRAKE_HIST] = {0};
int           rpmHistIdx            = 0;

// ============================================================
//  HELPERS
// ============================================================

inline float lerpF(float a, float b, float t) { return a + (b - a) * t; }
inline float clamp01(float x) { return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x); }

inline void applyColors() {
    uint8_t fr = (uint8_t)cFR, fg = (uint8_t)cFG, fb = (uint8_t)cFB;
    uint8_t rr = (uint8_t)cRR, rg = (uint8_t)cRG, rb = (uint8_t)cRB;
    for (int i = 0; i < NUM_LEDS_FRONT; i++)
        stripFront.setPixelColor(i, stripFront.Color(fr, fg, fb));
    for (int i = 0; i < NUM_LEDS_REAR; i++)
        stripRear.setPixelColor(i, stripRear.Color(rr, rg, rb));
}

inline void lerpColors(float tFR, float tFG, float tFB,
                       float tRR, float tRG, float tRB, float t) {
    cFR = lerpF(cFR, tFR, t); cFG = lerpF(cFG, tFG, t); cFB = lerpF(cFB, tFB, t);
    cRR = lerpF(cRR, tRR, t); cRG = lerpF(cRG, tRG, t); cRB = lerpF(cRB, tRB, t);
}

// Li-ion 21700 Samsung 50S discharge curve (voltage per cell [V])
static const float CELL_V[]   = { 4.20f, 4.05f, 3.95f, 3.85f, 3.75f,
                                   3.68f, 3.60f, 3.52f, 3.43f, 3.30f, 3.10f };
static const float CELL_PCT[] = { 100.0f, 90.0f, 80.0f, 70.0f, 60.0f,
                                    50.0f, 40.0f, 30.0f, 20.0f, 10.0f,  0.0f };
#define VOLT_TBL_SIZE 11

float batteryPercent(float packV) {
    float v = packV / BAT_CELLS;
    if (v >= CELL_V[0])                  return 100.0f;
    if (v <= CELL_V[VOLT_TBL_SIZE - 1]) return   0.0f;
    for (int i = 0; i < VOLT_TBL_SIZE - 1; i++) {
        if (v >= CELL_V[i + 1]) {
            float t = (v - CELL_V[i + 1]) / (CELL_V[i] - CELL_V[i + 1]);
            return CELL_PCT[i + 1] + t * (CELL_PCT[i] - CELL_PCT[i + 1]);
        }
    }
    return 0.0f;
}

void showBattery(float voltage) {
    float   pct = batteryPercent(voltage);
    int     lit = (int)((pct / 100.0f) * NUM_LEDS_FRONT + 0.5f);
    uint8_t r   = (pct > 20.0f) ? 0 : (uint8_t)(BRIGHT_IDLE);
    uint8_t g   = (pct > 20.0f) ?
                  (uint8_t)((uint16_t)200 * BRIGHT_IDLE / 255) : 0;
    for (int i = 0; i < NUM_LEDS_FRONT; i++)
        stripFront.setPixelColor(i, (i < lit) ?
            stripFront.Color(r, g, 0) : 0);
    for (int i = 0; i < NUM_LEDS_REAR; i++)
        stripRear.setPixelColor(i, 0);
}

void pushRPM(long rpm, unsigned long now) {
    rpmHist[rpmHistIdx]   = rpm;
    rpmHistMs[rpmHistIdx] = now;
    rpmHistIdx = (rpmHistIdx + 1) % BRAKE_HIST;
}

bool detectBraking(unsigned long now) {
    long newestRPM = rpmHist[(rpmHistIdx + BRAKE_HIST - 1) % BRAKE_HIST];
    long oldestRPM = newestRPM;
    bool foundOld  = false;
    for (int i = 0; i < BRAKE_HIST; i++) {
        int idx = (rpmHistIdx + i) % BRAKE_HIST;
        if ((now - rpmHistMs[idx]) > BRAKE_WINDOW_MS) continue;
        if (!foundOld) { oldestRPM = rpmHist[idx]; foundOld = true; }
    }
    if (!foundOld) return false;
    long delta = newestRPM - oldestRPM;
    return (delta < -BRAKE_RPM_THRESHOLD || delta > BRAKE_RPM_THRESHOLD);
}

// ============================================================
//  STARTUP ANIMATION — lightsaber effect
// ============================================================

void startupAnimation() {
    const unsigned long SWEEP_MS = 200UL;
    const unsigned long FLASH_MS = 1000UL;
    const unsigned long FADE_MS  = 600UL;
    unsigned long t = 0;

    stripFront.clear();
    stripRear.clear();
    stripFront.show();
    stripRear.show();

    // Phase 1: fast sweep from center outward
    unsigned long phaseStart = millis();
    while ((t = millis() - phaseStart) < SWEEP_MS) {
        float progress = (float)t / (float)SWEEP_MS;
        float reach    = progress * 5.5f;
        for (int i = 0; i < NUM_LEDS_FRONT; i++) {
            float   dist = (i <= 4) ? (4.0f - i) : (i - 5.0f);
            uint8_t b    = (uint8_t)(clamp01(reach - dist) * 255.0f);
            stripFront.setPixelColor(i, stripFront.Color(b, b, b));
            stripRear.setPixelColor(i,  stripRear.Color(b, 0, 0));
        }
        stripFront.show();
        stripRear.show();
        delay(10);
    }

    // Phase 2: ramp to full brightness
    phaseStart = millis();
    while ((t = millis() - phaseStart) < FLASH_MS) {
        float   p = (float)t / (float)FLASH_MS;
        uint8_t w = (uint8_t)(220.0f + p * 35.0f);
        for (int i = 0; i < NUM_LEDS_FRONT; i++)
            stripFront.setPixelColor(i, stripFront.Color(w, w, w));
        for (int i = 0; i < NUM_LEDS_REAR; i++)
            stripRear.setPixelColor(i, stripRear.Color(w, 0, 0));
        stripFront.show();
        stripRear.show();
        delay(10);
    }

    // Phase 3: fade to normal riding brightness
    phaseStart = millis();
    while ((t = millis() - phaseStart) < FADE_MS) {
        float   p = (float)t / (float)FADE_MS;
        uint8_t w = (uint8_t)(255.0f - p * (255.0f - BRIGHT_RIDE_MIN));
        uint8_t r = (uint8_t)(255.0f - p * (255.0f - BRIGHT_REAR_NORMAL));
        for (int i = 0; i < NUM_LEDS_FRONT; i++)
            stripFront.setPixelColor(i, stripFront.Color(w, w, w));
        for (int i = 0; i < NUM_LEDS_REAR; i++)
            stripRear.setPixelColor(i, stripRear.Color(r, 0, 0));
        stripFront.show();
        stripRear.show();
        delay(10);
    }

    cFR = BRIGHT_RIDE_MIN; cFG = BRIGHT_RIDE_MIN; cFB = BRIGHT_RIDE_MIN;
    cRR = BRIGHT_REAR_NORMAL; cRG = 0; cRB = 0;
    applyColors();
    stripFront.show();
    stripRear.show();

    everMoved    = true;
    lastDirBlend = 1.0f;
}

// ============================================================
//  MAIN LIGHTING LOGIC
// ============================================================

void updateLights(long rawRPM, float dutyCycle, float voltage,
                  float pitch, float adc1, float adc2,
                  uint8_t beepReason, unsigned long now) {

    // 0 — DUTY CYCLE WARNING: highest priority
    bool dutyWarning = (beepReason != 0) ||
                       (dutyCycle >  DUTY_WARN_THRESHOLD) ||
                       (dutyCycle < -DUTY_WARN_THRESHOLD);
    if (dutyWarning) {
        bool    on = ((now / DUTY_BLINK_MS) % 2) == 0;
        uint8_t v  = on ? 255 : 0;
        cFR = v; cFG = 0; cFB = 0;
        cRR = v; cRG = 0; cRB = 0;
        applyColors();
        return;
    }

    // 1 — BOARD UPRIGHT: second priority — immediate battery display
    bool boardUpright = (pitch > PITCH_UPRIGHT_DEG || pitch < -PITCH_UPRIGHT_DEG);
    if (boardUpright) {
        showBattery(voltage);
        return;
    }

    // 2 — FOOTPAD: rider stepping on, board not yet level
    {
        bool sensorL   = (adc1 > ADC_THRESHOLD);
        bool sensorR   = (adc2 > ADC_THRESHOLD);
        bool anySensor = sensorL || sensorR;
        bool isLevel   = (pitch > -PITCH_LEVEL_DEG && pitch < PITCH_LEVEL_DEG);

        if (anySensor && !isLevel) {
            for (int i = 0; i < NUM_LEDS_FRONT; i++) {
                bool lit = (i < 5 && sensorL) || (i >= 5 && sensorR);
                stripFront.setPixelColor(i, lit ?
                    stripFront.Color(FOOTPAD_BRIGHTNESS,
                                     FOOTPAD_BRIGHTNESS,
                                     FOOTPAD_BRIGHTNESS) : 0);
            }
            for (int i = 0; i < NUM_LEDS_REAR; i++)
                stripRear.setPixelColor(i, 0);
            lerpColors(0, 0, 0, 0, 0, 0, 0.06f);
            return;
        }
    }

    // 3 — Low-pass filter on RPM
    smoothRPM = smoothRPM * (1.0f - RPM_FILTER) + (float)rawRPM * RPM_FILTER;

    // 4 — Brake detection via RPM delta
    pushRPM(rawRPM, now);
    bool newBrake = detectBraking(now);
    if (newBrake) brakingUntil = now + BRAKE_HOLD_MS;
    bool braking = (now < brakingUntil);

    // 5 — Direction blend
    if (smoothRPM > RPM_DEAD_ZONE) {
        lastDirBlend = 1.0f;
    } else if (smoothRPM < -RPM_DEAD_ZONE) {
        lastDirBlend = -1.0f;
    }
    float dirBlend = lastDirBlend;

    // 6 — Speed brightness
    float speedT   = clamp01(fabsf(smoothRPM) / (float)RPM_FULL_BRIGHTNESS);
    float brtWhite = (BRIGHT_RIDE_MIN + speedT * (255.0f - BRIGHT_RIDE_MIN))
                     / 255.0f;

    // 7 — Riding colors
    float fwdFR = 255.0f*brtWhite, fwdFG = 255.0f*brtWhite, fwdFB = 255.0f*brtWhite;
    float fwdRR = BRIGHT_REAR_NORMAL, fwdRG = 0, fwdRB = 0;
    float bwdFR = BRIGHT_REAR_NORMAL, bwdFG = 0, bwdFB = 0;
    float bwdRR = 255.0f*brtWhite, bwdRG = 255.0f*brtWhite, bwdRB = 255.0f*brtWhite;

    // 8 — Target colors
    float tFR, tFG, tFB, tRR, tRG, tRB;
    if (dirBlend >= 0.0f) {
        tFR = fwdFR * dirBlend; tFG = fwdFG * dirBlend; tFB = fwdFB * dirBlend;
        tRR = fwdRR * dirBlend; tRG = fwdRG * dirBlend; tRB = fwdRB * dirBlend;
    } else {
        float d = -dirBlend;
        tFR = bwdFR * d; tFG = bwdFG * d; tFB = bwdFB * d;
        tRR = bwdRR * d; tRG = bwdRG * d; tRB = bwdRB * d;
    }

    // 9 — Braking: immediate red, no lerp on first frame
    if (braking) {
        if (newBrake) {
            if (smoothRPM >= 0) { cRR = 255.0f; cRG = 0; cRB = 0; }
            else                { cFR = 255.0f; cFG = 0; cFB = 0; }
        }
        if (smoothRPM >= 0) { tRR = 255.0f; tRG = 0; tRB = 0; }
        else                { tFR = 255.0f; tFG = 0; tFB = 0; }
    }

    // 10 — Lerp and apply
    lerpColors(tFR, tFG, tFB, tRR, tRG, tRB, LERP_NORMAL);
    applyColors();
}

// ============================================================
//  SETUP / LOOP
// ============================================================

void setup() {
    stripFront.begin();
    stripFront.setBrightness(255);
    stripFront.show();

    stripRear.begin();
    stripRear.setBrightness(255);
    stripRear.show();

    Serial.begin(115200);
    vesc.setSerialPort(&Serial);

    delay(500);
    startupAnimation();
}

void loop() {
    unsigned long now = millis();

    if (vesc.getVescValues()) {
        if (now - lastVoltageMs >= VOLTAGE_UPDATE_MS) {
            cachedVoltage = vesc.data.inpVoltage;
            lastVoltageMs = now;
        }
        lastRPM  = vesc.data.rpm;
        lastDuty = vesc.data.dutyCycleNow;

        // Always get float values — pitch needed for upright detection
        if (vesc.getFloatValues()) {
            lastPitch      = vesc.floatData.truePitch;
            lastAdc1       = vesc.floatData.adc1;
            lastAdc2       = vesc.floatData.adc2;
            lastBeepReason = vesc.floatData.beepReason;
        }
    }

    updateLights(lastRPM, lastDuty, cachedVoltage,
                 lastPitch, lastAdc1, lastAdc2,
                 lastBeepReason, now);

    stripFront.show();
    stripRear.show();
    delay(30);
}