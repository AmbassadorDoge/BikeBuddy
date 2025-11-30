// BikeBuddy - Arduino sketch for Pico 2W (RP2040)
// Features: GPS (NEO-6M), OLED 128x64, 4 capacitive buttons, trip distance/time/speed
// Libraries required: TinyGPSPlus, U8g2


#include <TinyGPS++.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Arduino.h>

// ----------------- PIN CONFIG -----------------
const uint8_t GPS_RX_PIN = 16;   // GPS TX -> Pico RX (GPIO16)
const uint8_t GPS_TX_PIN = 17;   // Pico TX -> GPS RX (GPIO17) (optional)
const uint8_t OLED_SDA_PIN = 11; // GPIO11 (SDA)
const uint8_t OLED_SCL_PIN = 10; // GPIO10 (SCL)

const uint8_t CAP_PIN_1 = 2;  // Start/Stop (GP2)
const uint8_t CAP_PIN_2 = 3;  // Lap/Reset (GP3)
const uint8_t CAP_PIN_3 = 4;  // Mode (GP4)
const uint8_t CAP_PIN_4 = 5;  // Backlight toggle (GP5)

// ----------------- DISPLAY -----------------
U8G2_SSD1309_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

// ----------------- GPS -----------------
TinyGPSPlus gps;

HardwareSerial gpsSerial(1); // UART1

// ----------------- TRIP STATS -----------------
bool tripRunning = false;
unsigned long tripStartMillis = 0;
unsigned long tripAccumulatedMillis = 0; // accumulated when paused

double lastLat = NAN, lastLng = NAN;
bool haveLastFix = false;
double tripDistanceMeters = 0.0;
double currentSpeedKmh = 0.0;
int displayMode = 0; // 0 = main stats, 1 = map-like raw coords/time, 2 = debug

// Debounce / simple edge-detect for capacitive buttons
bool lastBtnState1 = false;
bool lastBtnState2 = false;
bool lastBtnState3 = false;
bool lastBtnState4 = false;
unsigned long lastDebounceMillis = 0;
const unsigned long debounceDelay = 120; // ms

// Backlight variable (if display supports it; here we simulate by inverting)
bool backlightOn = true;

// ----------------- UTILITY (Haversine distance) -----------------
const double R_earth = 6371000.0; // meters

double deg2rad(double deg) {
  return deg * (M_PI / 180.0);
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
  // If any is NaN, return 0
  if (!isfinite(lat1) || !isfinite(lon1) || !isfinite(lat2) || !isfinite(lon2)) return 0.0;

  double dlat = deg2rad(lat2 - lat1);
  double dlon = deg2rad(lon2 - lon1);
  double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
             cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
             sin(dlon / 2.0) * sin(dlon / 2.0);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  double d = R_earth * c;
  return d;
}

// ----------------- SETUP -----------------
void setupPins() {
  pinMode(CAP_PIN_1, INPUT);
  pinMode(CAP_PIN_2, INPUT);
  pinMode(CAP_PIN_3, INPUT);
  pinMode(CAP_PIN_4, INPUT);
}

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println(F("BikeBuddy starting..."));

  // Initialize GPS serial (UART1) with RX/TX pins
  // On some Arduino cores the begin takes (baud, config, rxPin, txPin)
  // The RP2040 Arduino core supports this signature.
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Configure Wire for custom I2C pins before display init
  Wire.setSDA(OLED_SDA_PIN);
  Wire.setSCL(OLED_SCL_PIN);
  Wire.begin();

  // Initialize display
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);

  setupPins();

  // initial screen
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub30_tr);
  u8g2.drawStr(0, 40, "BikeBuddy");
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 58, "waiting for GPS...");
  u8g2.sendBuffer();
}

// ----------------- BUTTONS -----------------
bool readBtn(uint8_t pin) {
  // TTP223 typically returns HIGH when touched. We treat HIGH as pressed.
  return digitalRead(pin) == HIGH;
}

void handleButtons() {
  bool s1 = readBtn(CAP_PIN_1);
  bool s2 = readBtn(CAP_PIN_2);
  bool s3 = readBtn(CAP_PIN_3);
  bool s4 = readBtn(CAP_PIN_4);

  unsigned long now = millis();
  if (now - lastDebounceMillis < debounceDelay) return; // simple debounce

  // Start/Stop toggle
  if (s1 && !lastBtnState1) {
    // pressed down edge
    if (!tripRunning) {
      // start or resume
      tripRunning = true;
      tripStartMillis = millis();
    } else {
      // pause: accumulate time
      tripRunning = false;
      tripAccumulatedMillis += (millis() - tripStartMillis);
    }
    lastDebounceMillis = now;
  }

  // Lap/Reset
  if (s2 && !lastBtnState2) {
    if (tripRunning) {
      // lap: we could store lap (not implemented), for now just mark a short beep or log
      Serial.println(F("Lap (not stored)"));
    } else {
      // reset trip
      tripDistanceMeters = 0.0;
      tripAccumulatedMillis = 0;
      haveLastFix = false;
      lastLat = lastLng = NAN;
      Serial.println(F("Trip reset"));
    }
    lastDebounceMillis = now;
  }

  // Mode
  if (s3 && !lastBtnState3) {
    displayMode = (displayMode + 1) % 3;
    lastDebounceMillis = now;
  }

  // Backlight toggle
  if (s4 && !lastBtnState4) {
    backlightOn = !backlightOn;
    lastDebounceMillis = now;
  }

  lastBtnState1 = s1;
  lastBtnState2 = s2;
  lastBtnState3 = s3;
  lastBtnState4 = s4;
}

// ----------------- GPS PROCESSING -----------------
void processGPS() {
  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();
    gps.encode(c);
  }

  if (gps.location.isUpdated()) {
    double lat = gps.location.lat();
    double lng = gps.location.lng();
    if (isfinite(lat) && isfinite(lng)) {
      if (haveLastFix) {
        double d = haversine(lastLat, lastLng, lat, lng);
        // ignore implausible spikes: e.g., > 200 m between successive fixes
        if (d > 0 && d < 200.0) {
          // Only add distance while GPS sees movement; count it even if trip not running?
          // We'll accumulate distance only when tripRunning to match expectation.
          if (tripRunning) {
            // Add distance
            tripDistanceMeters += d;
          }
        }
      }
      lastLat = lat;
      lastLng = lng;
      haveLastFix = true;
    }
  }

  if (gps.speed.isUpdated()) {
    // speed in km/h via gps.speed.kmph()
    currentSpeedKmh = gps.speed.kmph();
  }
}

// ----------------- UI DRAW -----------------
void drawMainScreen() {
  u8g2.clearBuffer();

  // Title
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, "BikeBuddy");

  // Trip time
  unsigned long runningMillis = tripAccumulatedMillis;
  if (tripRunning) runningMillis += (millis() - tripStartMillis);
  unsigned int secs = (runningMillis / 1000UL) % 60UL;
  unsigned int mins = (runningMillis / 60000UL) % 60UL;
  unsigned int hrs  = (runningMillis / 3600000UL);

  char timebuf[20];
  snprintf(timebuf, sizeof(timebuf), "%02u:%02u:%02u", hrs, mins, secs);
  u8g2.setFont(u8g2_font_fub14_tr);
  u8g2.drawUTF8(0, 28, timebuf);

  // Speed
  char spdbuf[16];
  snprintf(spdbuf, sizeof(spdbuf), "%.1f km/h", currentSpeedKmh);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(85, 20, spdbuf);

  // Distance
  double km = tripDistanceMeters / 1000.0;
  char distbuf[24];
  snprintf(distbuf, sizeof(distbuf), "%.3f km", km);
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(85, 40, distbuf);

  // GPS fix status & coords
  u8g2.setFont(u8g2_font_6x10_tr);
  if (gps.location.isValid()) {
    char coordbuf[40];
    snprintf(coordbuf, sizeof(coordbuf), "%02.6f,%03.6f", gps.location.lat(), gps.location.lng());
    u8g2.drawStr(0, 54, coordbuf);
  } else {
    u8g2.drawStr(0, 54, "GPS: no fix");
  }

  // backlight indicator
  if (!backlightOn) {
    u8g2.setContrast(0); // attempt to simulate backlight off if supported
  } else {
    u8g2.setContrast(255);
  }

  u8g2.sendBuffer();
}

void drawRawScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 10, "RAW / Debug");

  u8g2.setFont(u8g2_font_6x12_tr);
  char line[48];

  if (gps.location.isValid()) {
    snprintf(line, sizeof(line), "Lat: %.6f", gps.location.lat());
    u8g2.drawStr(0, 25, line);
    snprintf(line, sizeof(line), "Lon: %.6f", gps.location.lng());
    u8g2.drawStr(0, 37, line);
  } else {
    u8g2.drawStr(0, 25, "Lat: ---");
    u8g2.drawStr(0, 37, "Lon: ---");
  }
  if (gps.time.isValid()) {
    snprintf(line, sizeof(line), "UTC %02u:%02u:%02u", gps.time.hour(), gps.time.minute(), gps.time.second());
    u8g2.drawStr(0, 49, line);
  } else {
    u8g2.drawStr(0, 49, "UTC: ---");
  }
  u8g2.sendBuffer();
}

void drawDebugScreen() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_tr);
  char buf[64];
  snprintf(buf, sizeof(buf), "Mode: %d  Running: %d", displayMode, tripRunning ? 1 : 0);
  u8g2.drawStr(0, 12, buf);
  snprintf(buf, sizeof(buf), "Dist: %.1fm", tripDistanceMeters);
  u8g2.drawStr(0, 26, buf);
  snprintf(buf, sizeof(buf), "Speed: %.2f km/h", currentSpeedKmh);
  u8g2.drawStr(0, 40, buf);
  u8g2.sendBuffer();
}

// ----------------- MAIN LOOP -----------------
unsigned long lastUIDraw = 0;
const unsigned long uiInterval = 500; // ms

void loop() {
  handleButtons();
  processGPS();

  // Update UI periodically
  unsigned long now = millis();
  if (now - lastUIDraw >= uiInterval) {
    lastUIDraw = now;
    if (displayMode == 0) drawMainScreen();
    else if (displayMode == 1) drawRawScreen();
    else drawDebugScreen();
  }

  // Optional: log GPS and trip info to Serial occasionally
  static unsigned long lastLog = 0;
  if (now - lastLog > 2000) {
    lastLog = now;
    if (gps.location.isValid()) {
      Serial.print(F("Pos: "));
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.print(gps.location.lng(), 6);
      Serial.print(F("  S: "));
      Serial.print(currentSpeedKmh);
      Serial.print(F(" km/h  D: "));
      Serial.print(tripDistanceMeters, 3);
      Serial.println(F(" m"));
    } else {
      Serial.println(F("GPS waiting for fix..."));
    }
  }
}
