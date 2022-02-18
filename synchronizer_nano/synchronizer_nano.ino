// Synchronizer_nano
//
// Get time from RTC chip and GPS.
// Display relative time alignment of each (relative to millis())
// Allow synchronization to each
// Emit time sync messages over serial on demand.

// This version runs on an Arduino Nano without display, control/display via USB serial.

// Wiring:
// Arduino <> DS3231
//     GND -> GND
//     5V  -> VCC
//     A5  -> SCK
//     A4  -> SDA
//     D3  <- SQWV

// Arduino <> UGPSM
//     GND -> GND
//     5V  -> VCC
//     D8  <- TX   (for AltSoftSerial)
//     D2  <- 1PPS


#include <Wire.h>

#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html

// =============================================================
// PPS change time recording.
// =============================================================

const int sqwPin = 3; // The number of the pin for monitor alarm status on DS3231
const int ppsPin = 2; // PPS output from GPS board

const int ledPin = 13; // On-board LED

volatile unsigned long rtc_micros = 0;
volatile unsigned long gps_micros = 0;

void rtc_mark_isr(void)
{
  rtc_micros = micros();
}
void gps_mark_isr(void)
{
  gps_micros = micros();
}

void setup_interrupts() {
  // DS3231 is on falling edge.
  attachInterrupt(digitalPinToInterrupt(sqwPin), rtc_mark_isr, FALLING);
  // GPS mark is on rising edge.
  attachInterrupt(digitalPinToInterrupt(ppsPin), gps_mark_isr, RISING);
} 

// ======================================================
// Base Clock class tracks unixtime rel to millis/micros.
// ======================================================

// Ignore the first few syncs for RTC and GPS, to avoid noisy initial pulses.
#define MIN_SYNC_COUNT 4
// How many secs back to let the oldest sync go.  
// We have one actual tick per 4 us, so 40 sec is 10,000,000 actual ticks.
#define OLDEST_CLOCK_SYNC_SECS 40

class Clock {
 public:
  const char *name_ = 0;
  // base_unixtime_ corresponds to base_micros_.
  time_t base_unixtime_ = 0;
  unsigned long base_micros_ = 0;
  bool synced_ = false;
  // Calibrated system ticks per Clock's second.  
  // This is stored in tenths of microseconds to allow fractional microseconds.
  long decimicros_per_sec_ = 10000000;   // 10^7
  // for time since oldest sync.
  time_t oldest_sync_unixtime_ = 0;
  unsigned long oldest_sync_micros_ = 0;
  // Store one intermediate value so we can hop.
  time_t next_oldest_sync_unixtime_ = 0;
  unsigned long next_oldest_sync_micros_ = 0;
  // Count how many syncs we've received.
  int sync_count_ = 0;
  int min_sync_count_ = 0;

  Clock(const char *name, int min_sync_count=MIN_SYNC_COUNT) 
    : name_(name), min_sync_count_(min_sync_count) {}

  long skew_micros(Clock& ref) {
    // Report the delay in *system micros* between ref clock reaching a time and this clock reaching the same time.
    // A positive skew means this clock is behind the ref clock.
    // This only compares the last sync times of each clock, not actual "current" time.
    // If this clock is slow relative to ref clock, skew_micros will increas with time.
    // Strictly, value depends on whether we're evaluating it at this.base_unixtime_, or ref.base_unixtime_.  
    // Use ref, because ref.micros_per_sec is less likely to be wildly off.
    long micros_per_sec = (ref.decimicros_per_sec_ + 5) / 10;  // Apply rounding.
    return (base_micros_ - ref.base_micros_) - micros_per_sec * (base_unixtime_ - ref.base_unixtime_);
  }

  long unixtime(void) {
    // Return current time in seconds + milliseconds.
    unsigned long micros_since_base = micros() - base_micros_;
    long seconds_since_base = (10 * micros_since_base) / decimicros_per_sec_;
    return base_unixtime_ + seconds_since_base;
  }

  void sync(time_t sync_unixtime, unsigned long sync_micros=0) {
    // Sync the clock to sync_unixtime exactly now (or at sync_micros).
    ++sync_count_;
    if (sync_micros == 0) {
      sync_micros = micros();
    }
    if (sync_count_ > min_sync_count_) {
      if (oldest_sync_micros_ == 0) {
        oldest_sync_micros_ = sync_micros;
        oldest_sync_unixtime_ = sync_unixtime;
        next_oldest_sync_micros_ = sync_micros;
        next_oldest_sync_unixtime_ = sync_unixtime;
      } else {
        if ((sync_unixtime - next_oldest_sync_unixtime_) > OLDEST_CLOCK_SYNC_SECS) {
          // Push a (time, micros) sync pair onto the (short) queue every 2 minutes.
          oldest_sync_micros_ = next_oldest_sync_micros_;
          oldest_sync_unixtime_ = next_oldest_sync_unixtime_;
          next_oldest_sync_micros_ = sync_micros;
          next_oldest_sync_unixtime_ = sync_unixtime;
        }
        // This is going to wrap at 2**32 / 1e7 = 420 secs, or about 7 minutes.
        // Hopefully the queue will ensure it's never more than ~4 minutes.
        int delta_seconds = sync_unixtime - oldest_sync_unixtime_;
        if (delta_seconds < 400) {
          decimicros_per_sec_ = (10 * (sync_micros - oldest_sync_micros_)) / 
                                delta_seconds;
        }
      }
    }
    base_micros_ = sync_micros;
    base_unixtime_ = sync_unixtime;
    synced_ = true;
  }

  void clear_sync_history(void) {
    // If we think there's a discontinuity in sync history.
    oldest_sync_unixtime_ = 0;
    oldest_sync_micros_ = 0;
    next_oldest_sync_unixtime_ = 0;
    next_oldest_sync_micros_ = 0;
    sync_count_ = 0;
    synced_ = false;
  }

};

Clock rtc_clock("RTC");
Clock gps_clock("GPS");
Clock *sys_clock = &rtc_clock;

// ======================================================
// Track RTC times.
// ======================================================

#include "RTClib.h"
RTC_DS3231 rtc;

// from DS3231.cpp
#define CLOCK_ADDRESS 0x68

void ds3231_readAllRegisters(byte *registers, int num_bytes /* =19 */) {
  // Read all 19 bytes from the clock status.
  if (num_bytes > 19) num_bytes = 19;
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(0x0);
    Wire.endTransmission();
    Wire.requestFrom(CLOCK_ADDRESS, num_bytes);
  for(int i = 0; i < num_bytes; ++i) {
     registers[i] = Wire.read();
  }
}

int ds3231_getAgingOffset(void) {
  // Read aging offset register
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.requestFrom(CLOCK_ADDRESS, 1);
  int regVal = Wire.read();
  if (regVal > 127) regVal -= 256;
  return regVal;
}

void ds3231_setAgingOffset(int offset) {
  // Write the aging register. 
  // It's a signed 8 bit value, -128..127
  // Crystal *slows* by approx 0.1 ppm per unit.
  Wire.beginTransmission(CLOCK_ADDRESS);
  Wire.write(0x10);
  Wire.write(offset);
  Wire.endTransmission();
}

bool pending_RTC_interrupt = false;

unsigned long last_rtc_micros = 0;

void sync_time_from_RTC(void) {
  // This is called soon after an A0 transition is detected, so RTC unixtime is still current.
  rtc_clock.sync(rtc.now().unixtime(), rtc_micros);
  last_rtc_micros = rtc_micros;
}

void setup_RTC(void) {
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  rtc.writeSqwPinMode(DS3231_SquareWave1Hz); // Place SQW pin into 1 Hz mode
  pinMode(sqwPin, INPUT_PULLUP); // Set alarm pin as pullup
  pinMode(ledPin, OUTPUT);

  // Sync even without interrupts
  rtc_micros = micros();
  sync_time_from_RTC();
}

int last_min = 0;
int rtc_aging = 0;
int rtc_temperature_centidegs = 0.0;

// Updated in main loop.
long sys_secs = 0;

bool have_rtc = false;

// Give up on RTC if no sync in this long.
#define MIN_RTC_SYNC_GAP 10

void update_RTC(void) {
  if (pending_RTC_interrupt) {
    sync_time_from_RTC();
    pending_RTC_interrupt = false;
  }
  have_rtc = (sys_secs - ((long)rtc_clock.base_unixtime_)) < MIN_RTC_SYNC_GAP;
  if (have_rtc) {
    int sys_mins = (sys_secs / 60) % 60;
    if (sys_mins != last_min) {
      last_min = sys_mins;
      // Update temp 1/min.
      rtc_temperature_centidegs = (int)(100 * rtc.getTemperature());
      rtc_aging = ds3231_getAgingOffset();
    }
  }
}

// ======================================================
// Track GPS times.
// ======================================================

#include <TinyGPS.h>       // http://arduiniana.org/libraries/TinyGPS/

// AltSoftSerial - Only RX on Pin 8, so can't work with ES100 devboard
#include <AltSoftSerial.h> // https://github.com/PaulStoffregen/AltSoftSerial
AltSoftSerial SerialGPS;   // Teensy: rx pin 20, tx pin 21.  Nano: rx 8, tx 9

TinyGPS gps;

bool pending_GPS_interrupt = false;

time_t gps_now(class TinyGPS& gps) {
  tmElements_t tm;
  unsigned long date, time, age_millis;
  gps.get_datetime(&date, &time, &age_millis);
  // We *could* add age_millis to get current time, but we really want the sync'd time from the last second.
  // date and time are stored as digit pairs in decimal i.e. ddmmyy and hhmmsscc.
  int centis = time % 100;
  //Serial.print("gps centis=");
  //Serial.println(centis);
  // it's always 0.
  time /= 100;
  tm.Second = time % 100;
  time /= 100;
  tm.Minute = time % 100;
  time /= 100;
  tm.Hour = time % 100;
  tm.Year = (date % 100) + 2000 - 1970;
  date /= 100;
  tm.Month = date % 100;
  date /= 100;
  tm.Day = date % 100;
  return makeTime(tm);
}

bool gps_time_valid(class TinyGPS& gps) {
  // It's only valid if it was updated within the past second.
  unsigned long time, age_millis;
  gps.get_datetime(0, &time, &age_millis);
  return (time != gps.GPS_INVALID_TIME) && (age_millis < 1000);
}

unsigned long last_gps_micros = 0;

void sync_time_from_GPS(void) {
  // This is called soon after an A1 transition is detected, so GPS unixtime is still current.
  if (gps_time_valid(gps)) {
    // We can assume that the last-stored time from the GPS messages is the second *preceeding* this mark,
    // so we add 1 second to get the actual time corresponding to the mark.
    // (potential race condition).
    gps_clock.sync(gps_now(gps) + 1, gps_micros);
    last_gps_micros = gps_micros;
  }
}

void setup_GPS_serial(void) {
  SerialGPS.begin(9600);
}

void update_GPS_serial(void) {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }
}

bool request_RTC_sync = false;
bool request_sync_output = false;

void setup_GPS(void) {
    pinMode(ppsPin, INPUT_PULLUP); // Set alarm pin as pullup
}

void update_GPS(void) {
  if (pending_GPS_interrupt) {
    sync_time_from_GPS();
    pending_GPS_interrupt = false;
    // Request for sync e.g. from button press.
    if (request_RTC_sync) {
      Serial.println("Setting DS3231 from GPS");
      rtc.adjust(DateTime(gps_clock.unixtime()));
      rtc_clock.clear_sync_history();  // Old sync records are irrelevant now.
      request_RTC_sync = false;
    }
    if (request_sync_output) {
      emit_sync_command(gps_clock.unixtime());
      request_sync_output = false;
    }
  }
}

// ======================================================
// Time display
// ======================================================

char *sprint_int(char *s, int n, int decimal_place=0)
{ // Print a signed int as may places as needed. 
  // returns next char* to write to string.
  // If decimal place > 0, assumed passed int is value * 10**decimal_place
  // and print a decimal place.
  if (n < 0) {
    *s++ = '-';
    n = -n;
  }
  if (n > 9 || decimal_place > 0) {
     s = sprint_int(s, n / 10, decimal_place - 1);
  }
  if (decimal_place == 1) {
    *s++ = '.';
  }
  *s++ = '0' + (n % 10);
  return s;
}

char *sprint_int2(char *s, int n)
{  // Always 2 digits, assume n nonnegative, < 99.
  *s++ = '0' + (n / 10);
  *s++ = '0' + (n % 10);
  return s;
}

char *sprint_unixtime(char *s, time_t t, bool show_date=false)
{  // Returns full terminated string
  char *entry_s = s;
  // s must provide 20 bytes, or 9 if just time.
  tmElements_t tm;
  breakTime(t, tm);
  if (show_date) {
    s = sprint_int(s, tm.Year + 1970);
    *s++ = '-';
    s = sprint_int2(s, tm.Month);
    *s++ = '-';
    s = sprint_int2(s, tm.Day);
    *s++ = ' ';
  }
  s = sprint_int2(s, tm.Hour);
  *s++ = ':';
  s = sprint_int2(s, tm.Minute);
  *s++ = ':';
  s = sprint_int2(s, tm.Second);
  *s++ = 0;
  return entry_s;
}

char *sprint_sync_command(char *s, time_t t) {
  // Generate a "ZYYYYMMDDHHMMSS" string that can be sent over serial to resync a flipclock.
  char *entry_s = s;
  // s must provide 16 bytes.
  tmElements_t tm;
  breakTime(t, tm);
  *s++ = 'Z';
  s = sprint_int(s, tm.Year + 1970);
  s = sprint_int2(s, tm.Month);
  s = sprint_int2(s, tm.Day);
  s = sprint_int2(s, tm.Hour);
  s = sprint_int2(s, tm.Minute);
  s = sprint_int2(s, tm.Second);
  *s++ = 0;
  return entry_s;
}

void emit_sync_command(time_t t) {
  char s[16];
  Serial.println(sprint_sync_command(s, t));
}

void serial_print_unixtime(time_t t) {
  char s[20];
  sprint_unixtime(s, t);
  Serial.print(s);
}

char *sprint_clock_comparison(char *s, class Clock& clock, class Clock& ref_clock)
{  // Returns full terminated string
  char *entry_s = s;
  // Report skew in ms with 1 dp.
  s = sprint_int(s, clock.skew_micros(ref_clock) / 100, /* dp */ 1);
  strcpy(s, "ms ");
  s += strlen(s);
  // Approximate (1 + a)/(1 + b) - 1 as (a - b) (i.e. 1/(1 + b) = 1 - b, and a/(1 + b) = a), for a, b << 1.
  int decimicro_deviation = ref_clock.decimicros_per_sec_ - clock.decimicros_per_sec_;
  s = sprint_int(s, decimicro_deviation, /* dp */ 1);
  strcpy(s, "ppm");
  return entry_s;
}

char *sprint_rtc_info(char *s) {
  // Read RTC info from globals.
  char *entry_s = s;
  //strcpy(s, "RTC: ");
  //s += strlen(s);
  if (!have_rtc) {
    strcpy(s, "no RTC");
    return entry_s;
  }
  strcpy(s, "t=");
  s += strlen(s);
  s = sprint_int(s, rtc_temperature_centidegs, 2);
  strcpy(s, " a=");
  s += strlen(s);
  s = sprint_int(s, rtc_aging);
  *s++ = 0;
  return entry_s;
}

void serial_print_clock_comparison(class Clock& clock, class Clock& ref_clock) {
  char s[64];
  sprint_clock_comparison(s, clock, ref_clock);
  Serial.print(s);
}

void serial_clock_debug(class Clock& clock) {
    char s[9];
  Serial.print(" ");
  Serial.print(clock.name_);
  Serial.print(": ");
  serial_print_unixtime(clock.unixtime());
  Serial.print(" dmspt=");
  Serial.print(clock.decimicros_per_sec_);
  Serial.print(" syncs=");
  Serial.print(clock.sync_count_);
//  Serial.print(" sync=");
//  sprint_unixtime(s, clock.base_unixtime_, false);
//  Serial.print(s);
//  Serial.print(" osut=");
//  sprint_unixtime(s, clock.oldest_sync_unixtime_, false);
//  Serial.print(s);
}


#define NUM_REGISTERS 19

void serial_print_registers(void) {
  // Read all 19 DS3231 registers and print out in hex.
  Serial.print("DS3231 Regs: ");
  byte registers[NUM_REGISTERS];
  ds3231_readAllRegisters(registers, NUM_REGISTERS);
  for (int i = 0; i < NUM_REGISTERS; ++i) {
    if (registers[i] < 16)  Serial.print("0");
    Serial.print(registers[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

// -------------------------------------------------------------------
// Input commands over serial line

void cmd_setup(void) {
  // Nothing to do?
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

void cmd_update(void) {
  if (Serial.available() > 0) {
    // read the incoming byte:
    char new_char = Serial.read();
    if (new_char == '\n') {
      // handle command.
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        byte cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a' && cmd0 <= 'z')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case '?':
            Serial.println("S    - Sync DS3231 to GPS");
            Serial.println("R    - Print DS3231 registers");
            Serial.println("C    - Initiate temperature conversion");
            Serial.println("T    - Read DS3231 temperature");
            Serial.println("Axx  - Set Aging Offset -128 to 127");
            break;
          case 'S':
            // Sync DS3231 to GPS
            request_RTC_sync = true;
            break;
          case 'R':
            // Read DS3231 registers
            serial_print_registers();
            break;
          case 'C':
            // Initiate temperature conversion.
            //ds3231_startConvertTemperature();
            //Serial.println("Temp conversion initiated");
            break;
          case 'T':
            // Read DS3231 temperature
            Serial.print("DS3231 temperature=");
            Serial.println(rtc.getTemperature());
            break;
          case 'A':
            // Set DS3231 aging register.
            int aging_offset = atoi(cmd_buffer + 1);
            ds3231_setAgingOffset(aging_offset);
            Serial.print("Aging offset set to ");
            Serial.println(aging_offset);
            break;
        }
      }
      cmd_len = 0;
    } else {
      if (cmd_len < CMD_BUF_LEN) {
        cmd_buffer[cmd_len++] = new_char;
      }
    }
  }
}

void adjust_rtc_trim(int val) {
  // Adjust the DS3231 "aging register" trim by specified amount relative.
  rtc_aging = ds3231_getAgingOffset();
  rtc_aging += val;
  ds3231_setAgingOffset(rtc_aging);
  Serial.print("rtc_aging=");
  Serial.println(rtc_aging);
}

// ======================================================
// =========== Button management ==============
// ======================================================

#ifdef HAVE_BUTTONS

#define NUM_BUTTONS 3   // on D9, D6, D5 on feather OLED wing are GPIO 9, 8, 7 on RP2040 Feather
int button_pins[NUM_BUTTONS] = {9, 8, 7};
int button_state[NUM_BUTTONS] = {0, 0, 0};
unsigned long button_last_change_time[NUM_BUTTONS] = {0, 0, 0};
#define DEBOUNCE_MILLIS 50

void buttons_setup(void) {
  for (int button = 0; button < NUM_BUTTONS; ++button) {
    pinMode(button_pins[button], INPUT_PULLUP);
  }
}

void buttons_update(void) {
  for (int button = 0; button < NUM_BUTTONS; ++button) {
    // Buttons are pulled up, so read as 1 when open, 0 when pressed.
    int new_state = 1 - digitalRead(button_pins[button]);
    if (button_state[button] == new_state) {
      continue;
    }
    // State has changed.
    unsigned long millis_now = millis();
    unsigned long millis_since_last_change = millis_now - button_last_change_time[button];
    button_last_change_time[button] = millis_now;
    if (millis_since_last_change <= DEBOUNCE_MILLIS) {
      continue;
    }
    // Debounced state change
    button_state[button] = new_state;
    if (!new_state) {
      continue;
    }
    // State change was to "pressed".
    switch(button) {
      case 0:
        // Write GPS time to RTC at next good opportunity.
        request_RTC_sync = true;
        break;
      case 1:
        // Increase aging register
        adjust_rtc_trim(1);
        //// Emit a sync command.
        //request_sync_output = true;
        break;
      case 2:
        // Decrease aging register
        adjust_rtc_trim(-1);
        break;
    }
  }
}

#endif // HAVE_BUTTONS

// ======================================================
// =========== OLED Display ==============
// ======================================================

#ifdef HAVE_OLED

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

void setup_display(void) {
  display.begin(0x3C, true); // Address 0x3C default

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);

  // text display tests
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.print("DS3231 Feather");
  display.display(); // actually display all of the above
}

#define TZ_HOURS -5

void update_display(void) {
  char s[64];  // We can only use up to 21, but sprint_clock_comparison could get large for very large skews.
  // Line 1: Current time, sync source.
  display.fillRect(0, 0, 128, 64, SH110X_BLACK);
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print(sprint_unixtime(s, (*sys_clock).unixtime() + 3600 * TZ_HOURS, false));
  display.setCursor(110, 0);
  display.setTextSize(1);
  display.print(sys_clock->name_);
  // Line 2: RTC vs GPS.
  display.setCursor(0, 32);
  display.print("RTC:");
  display.setCursor(0, 41);
  display.print(sprint_clock_comparison(s, rtc_clock, *sys_clock));
  // Line 3: RTC status
  display.setCursor(0, 50);
  display.print(sprint_rtc_info(s));

  display.display(); // actually display all of the above
}

#endif // HAVE_OLED

// ======================================================
// Main setup() and loop()
// ======================================================

void setup() {
  // Configure Pico RP2040 I2C
  //Wire.setSDA(sda_pin);
  //Wire.setSCL(scl_pin);
  // I2C interface is started by display driver.
  
  // initialize serial communication at 9600 bits per second.
  Serial.begin(9600);
  Serial.println("Post-reset settling...");
  delay(2000);

#ifdef HAVE_OLED
  setup_display();  // includes i2c init.
#endif // HAVE_OLED
  setup_RTC();
  setup_GPS_serial();
  setup_GPS();
#ifdef HAVE_BUTTONS
  buttons_setup();
#endif // HAVE_BUTTONS
  cmd_setup();

  setup_interrupts();
};

// How long can we miss syncs before we give up on GPS clock?
#define GPS_TIMEOUT_SECS 5

boolean gps_has_been_live = false;

long secs_last_change = 0;

void loop() {
  digitalWrite(ledPin, digitalRead(sqwPin));

#ifdef HAVE_BUTTONS
  buttons_update();
#endif // HAVE_BUTTONS
  cmd_update();

  if (rtc_micros != last_rtc_micros)  pending_RTC_interrupt = true;
  if (gps_micros != last_gps_micros)  pending_GPS_interrupt = true;
  update_RTC();
  update_GPS();
  sys_secs = sys_clock->unixtime();
  if(gps_clock.synced_ && 
     (!gps_has_been_live || (((long)gps_clock.base_unixtime_) - sys_secs) < GPS_TIMEOUT_SECS)) {
    // GPS is sync'ing, and it has been sync'd recently, or it's the first time we've seen it sync'd
    // (in which case the difference from sys_secs may be immaterial).
    sys_clock = &gps_clock;
    gps_has_been_live = true;
  } else {
    sys_clock = &rtc_clock;
  }
  update_GPS_serial();
  if ((secs_last_change != sys_secs)) {
    secs_last_change = sys_secs;
    serial_clock_debug(rtc_clock);
    serial_clock_debug(gps_clock);
    char s[64];
    Serial.print(" RTCinfo: ");
    Serial.print(sprint_rtc_info(s));
    Serial.print(" VsGPS: ");
    Serial.println(sprint_clock_comparison(s, rtc_clock, *sys_clock));
    // And on display:
#ifdef HAVE_OLED
    update_display();
#endif // HAVE_OLED
  }
}
