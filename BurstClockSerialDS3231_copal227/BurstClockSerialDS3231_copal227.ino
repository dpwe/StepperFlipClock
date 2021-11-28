// BurstClockSerialDS3231.ino
// 
// Flip-clock-by-stepper-motor retrofit.
//
// Rather than emulating the continuous, slow rotation of the original Copal 60 Hz 
// synchronous motor, this version rotates by 1/12 rev as fast as possible once per
// minute.  This should allow us to control exactly when the flips fall, instead of 
// the +/- 15 sec of the original mechanism.  
//
// The code also aims to keep track of what the clock is currently displaying, and
// supports commanding to display arbitrary numbers.
//
// This version accepts control commands over the serial line.
// It uses the DS3231 instead of the MCP79412.  We use the DS3231's alarm register 
// as the battery-backed RAM to persist the current believed display state.

// Connections:
// ULN2003A board: (note, sequencing is backwards)
// IN1 -> D12
// IN2 -> D11
// IN3 -> D10
// IN4 -> D9
// DS3231 board:
// SDA -> A4
// SCL -> A5
// SQW -> D3 (supports interrupt)
// Clock Advance Button: -> D2 (pulls down to ground)
// Mechanical Alarm microswitch -> D4 (pulls down to ground when alarm should sound)
// Backlight LED -> D5 (supports Timer0 PWM, possibly interferes with delay() timer)
// Buzzer: Tone pattern on D6 (using Timer1, also D6 is OC0A, so might support PWM output)
// Time Wrong LED -> D13 (built-in LED)

// GE 7-4305
//#define STEPS_PER_CYCLE 2048
//#define MINUTES_PER_CYCLE 15
// Copal 227
#define STEPS_PER_CYCLE 1280
#define MINUTES_PER_CYCLE 9

#include <Tone.h>  // https://github.com/bhagman/Tone
Tone tone_out[2];  // Allocate two timers so we can *not* use tone_out[0],
                   // which tries to use Timer2 (but we have other plans).

#include <TimeLib.h>        // https://www.pjrc.com/teensy/td_libs_DS1307RTC.html
#include <Wire.h>           // https://www.arduino.cc/en/Reference/Wire

#include <Timezone.h>       // https://github.com/JChristensen/Timezone
// US Eastern Time Zone (New York, Detroit)
//TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
//TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours
// US Pacific Time Zone (Las Vegas, Los Angeles)
TimeChangeRule myDST = {"PDT", Second, Sun, Mar, 2, -420};
TimeChangeRule mySTD = {"PST", First, Sun, Nov, 2, -480};
Timezone myTZ(myDST, mySTD);

#include <DS3231.h>         // https://github.com/NorthernWidget/DS3231
DS3231 ds3231;
RTClib RTC;

// -------------------------------------------------------------------
// Timer 2:
// Each time the stepper is advanced, Timer2 is set to generate an interrupt
// 3 ms later.  This interrupt either advances to the next step (if there are
// remaining steps to do) or powers down the coils.

// What to call on interrupt.
void (*_timeout_callback)(void);

// Timer2 Overflow Interrupt Vector
ISR(TIMER2_OVF_vect) {
  TIFR2 = 0x00;         // Timer2 INT Flag Reg: Clear Timer Overflow Interrupt
  TCCR2B = 0x00;        // Stop Timer2 until next timeout_restart().
  (*_timeout_callback)();
}

void timeout_setup(void (*callback_func)(void)) {
  // Set up Timer2 for the post-step timeout.
  TCCR2B = 0x00;        // Disable Timer2 while we set it up
  _timeout_callback = callback_func;
  TIFR2  = 0x00;        // Timer2 INT Flag Reg: Clear Timer Overflow Flag
  TIMSK2 = 0x01;        // Timer2 INT Reg: Timer2 Overflow Interrupt Enable
  TCCR2A = 0x00;        // Timer2 Control Reg A: Wave Gen Mode norm
}

void timeout_restart(unsigned int ms) {
  // Reset timeout timer (timer2). It counts up to 255 at 16 kHz, so 0 gives ~16ms, 128 is 8ms, 192 is 4ms.
  int init_val = 255 - (ms * 16);
  if (init_val < 0)  init_val = 0;  // max delay is 16 ms.
  TCNT2 = init_val;
  TCCR2B = 0x07;        // Timer2 Control Reg B: Timer Prescaler set to 1024 (starts the clock).
}

// -------------------------------------------------------------------
// Stepper motor control

// Pins to drive motor coils.
#define PIN_A 12
#define PIN_B 11
#define PIN_C 10
#define PIN_D 9

void stepper_setup(void)
{
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);
  pinMode(PIN_D, OUTPUT);

  timeout_setup(next_step_callback);
}

void set_pins(int a, int b, int c, int d) {
  digitalWrite(PIN_A, a);
  digitalWrite(PIN_B, b);
  digitalWrite(PIN_C, c);
  digitalWrite(PIN_D, d);
}

void step_to_state(int state) {
  switch (state % 4) {
    // The "backwards" motion of the motor is encoded directly here.
    case 0:
      set_pins(0, 0, 1, 1);
      break;
    case 1:
      set_pins(0, 1, 1, 0);
      break;
    case 2:
      set_pins(1, 1, 0, 0);
      break;
    case 3:
      set_pins(1, 0, 0, 1);
      break;
  }
}

// Current steps within cycle of stepper
int _stepper_state = 0;
// We can peek at steps_to_do as a more informative alternative to stepper_active.
volatile long steps_to_do = 0;
// Semaphore that client writes to request more steps.  Gets cleared when accepted.
volatile long steps_to_do_semaphore = 0;
// Status that can be inspected by client.
volatile bool stepper_active = 0;

// When active, we advance the stepper every 3 ms.
#define MS_BETWEEN_STEPS 3

void next_step_callback(void)
{
  // This is usually executed in interrupt state, so be careful not to change variables used
  // outside, except the steps_to_do semaphore.
  if (steps_to_do_semaphore) {
    steps_to_do += steps_to_do_semaphore;
    steps_to_do_semaphore = 0;
  }
  if (steps_to_do) {
    stepper_active = 1;
    timeout_restart(MS_BETWEEN_STEPS);
    _stepper_state = (_stepper_state + 1) % STEPS_PER_CYCLE;
    step_to_state(_stepper_state);
    --steps_to_do;
  } else {
    // We are idling the stepper.
    stepper_active = 0;
    set_pins(0, 0, 0, 0);
    // Notify clients.
    //inc_display_minute();
    // Somehow this was being called during initialization and adding an extra minute.
    // Now inc_display_minute() is called by flip_update when it notices stepper_active 
    // has gone low.
  }
}

// Called by client to start new steps.
void add_steps(int steps) {
  // Drop the value into the semaphore; it will be added on to the total by the interrupt routine.
  Serial.print("add_steps=");
  Serial.println(steps);
  steps_to_do_semaphore = steps;
  if (!stepper_active) {
    // Calling next_step_callback will accept the semaphore, take a step immediately, then restart timer.
    next_step_callback();
  }
}

// ==========  persistent read/write =============

void save_display_state_to_sram(int mins_state) {
  // Write hrs/mins to DS3231 Alarm 2.  See DS3231.h
  byte disp_hour = (byte)(mins_state / 60);
  byte disp_min = (byte)(mins_state % 60);
  byte A2Day = 1, AlarmBits = 0;  // AlarmBits=0 means alarm when all of min, hr, day, month match.
  bool A2Dy = 0, A2h12 = 0, A2PM = 0;
  ds3231.setA2Time(A2Day, disp_hour, disp_min, AlarmBits, A2Dy, A2h12, A2PM);
}

int load_display_state_from_sram(void) {
  byte disp_hour, disp_min, A2Day, AlarmBits;
  bool A2Dy, A2h12, A2PM;
  ds3231.getA2Time(A2Day, disp_hour, disp_min, AlarmBits, A2Dy, A2h12, A2PM);
  if (A2h12 != 0) {
    Serial.println("DS3231 Alarm is in 12h mode - uh-oh!");
  }
  return 60 * (int)disp_hour + (int)disp_min;  
}

// -------------------------------------------------------------------
// Core flip-dispay driver.

// Keep track of where we are within the MINUTES_PER_CYCLE.
int current_minute = 0;
// Keep track of where we are within the STEPS_PER_CYCLE.
unsigned int current_step = 0;

#define MINS_PER_DAY 1440
// Believed current state of display, in minutes since midnight
// 0 = 00:00, 1439 = 23:59, -1 = unknown.
volatile int display_mins_shown = -1;

void inc_display_minute(void) {
  // Intended to be called by stepper interrupt routine when stepper goes idle because 
  // it has completed stepping.  However, it was being called "extra" during setup for 
  // an unexplained reason, so now it's called by the flip_update poll when it notices
  // a stepper command has finished.
  display_mins_shown = (display_mins_shown + 1) % MINS_PER_DAY;
}

void advance_one_minute(void) {
  ++current_minute;
  // Integer arithmetic - will round down.
  unsigned int new_step = (((long)current_minute * STEPS_PER_CYCLE) / MINUTES_PER_CYCLE);
  // Command the new steps to happen.
  add_steps(new_step - current_step);
  //Serial.print("steps: ");
  //Serial.println(new_step - current_step);
  // Update state.
  current_minute = current_minute % MINUTES_PER_CYCLE;
  current_step = new_step % STEPS_PER_CYCLE;
}

#define TIME_WRONG_LED LED_BUILTIN

void flip_setup(int initial_display_mins) {
  display_mins_shown = initial_display_mins;
  Serial.print("Setup ");
  flip_report_display(display_mins_shown);

  stepper_setup();
  // We use TIME_WRONG_LED to indicate when clock is "waiting" to catch up.
  pinMode(TIME_WRONG_LED, OUTPUT);
}

void set_time_error_led(bool state) {
  if (state) {
    digitalWrite(TIME_WRONG_LED, HIGH);
  } else {
    digitalWrite(TIME_WRONG_LED, LOW);    
  }
}

void printDigits(int digits)
{
  // utility function for digital clock display: prints preceding colon and leading 0
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

// The time that we are trying to drive the display towards.
// -1 is a special value meaning don't update.
// Otherwise a value 0.1439 corresponding to 00:00 to 23:59.
int requested_time = -1;

void flip_report_display(int mins) {
  // Echo current belief about display.
  Serial.print("Display: ");
  printDigits(mins / 60);
  Serial.print(":");
  printDigits(mins % 60);
  // Also let us know what we're aiming for.
  Serial.print(" / Goal: ");
  printDigits(requested_time / 60);
  Serial.print(":");
  printDigits(requested_time % 60);  
  Serial.println();
}

void flip_set_state_mins(int mins) {
  // Somehow we got info about what the display currently says, during init.
  // Ignore -1
  if (mins < 0)
    return;
  display_mins_shown = mins;
  Serial.print("Warp ");
  flip_report_display(display_mins_shown);
  // Save display state in persistent RAM.
  save_display_state_to_sram(display_mins_shown);
}

// Time changes of up to two hours backwards will result in the 
// clock stopping until "real time" catches up.
#define MAX_PAUSE 120

bool minute_in_progress = false;

void flip_update(void) 
{
  // Called periodically in update loop.  Checks to see if we have stuff to do.
  if (stepper_active) {
    // An update is not yet complete.
    return;
  }
  if (minute_in_progress) {
      // We were advancing one minute, but now that's done.
      minute_in_progress = false;
      // Increment the minutes count (not done by stepper handler at present).
      inc_display_minute();
      flip_report_display(display_mins_shown);
      // Save display state in persistent RAM.
      save_display_state_to_sram(display_mins_shown);
  }
  // If requested_time is -1, don't do anything.
  if (requested_time >= 0 && (display_mins_shown != requested_time)) {
    set_time_error_led(true);
    // Maybe *don't* move the clock if the requested time is close to catching up.
    if ( ((requested_time - display_mins_shown + MINS_PER_DAY) % MINS_PER_DAY) 
         < (MINS_PER_DAY - MAX_PAUSE)) {
      // When we advance, we advance one minute at a time.
      minute_in_progress = 1;
      advance_one_minute();
    }
  } else {
    // We good.
    set_time_error_led(false);
  }
}

int move_to_time(int new_mins)
{
  requested_time = new_mins;
}

// -------------------------------------------------------------------
// Input commands over serial line

void cmd_setup(void) {
  // Nothing to do?
}

byte atoi2(char *s) {
  // Convert two ascii digits to a uint8.
  return (s[1] - '0') + 10 * (s[0] - '0');
}

int parse_string_to_mins(char *mins_string) {
  // Convert a string like "2359" into minutes since midnight (1439 in this case)
  if (strlen(mins_string) != 4) {
    Serial.println("Error: cmd arg string is not 4 chrs.");
    return -1;
  }
  int hours = atoi2(mins_string);
  int minutes = atoi2(mins_string + 2);
  return 60 * hours + minutes;
}

tmElements_t parse_time_string(char *time_string) {
  // time_string must point to exactly 14 chars in format YYYYMMDDHHMMSS.
  tmElements_t tm;
  if (time_string[0] != '2' or time_string[1] != '0') {
    Serial.println("Warn: Year does not start with 20...");
  }
  tm.Year = y2kYearToTm(atoi2(time_string + 2));
  tm.Month = atoi2(time_string + 4);
  tm.Day = atoi2(time_string + 6);
  tm.Hour = atoi2(time_string + 8);
  tm.Minute = atoi2(time_string + 10);
  tm.Second = atoi2(time_string + 12);
  return tm;
}

#define CMD_BUF_LEN 32
char cmd_buffer[CMD_BUF_LEN];
int cmd_len = 0;

bool enable_rtc_updates = true;

byte backlight_led = 5;
byte backlight_brightness = 64;

void cmd_update(void) {
  if (Serial.available() > 0) {
    // read the incoming byte:
    char new_char = Serial.read();
    if (new_char == '\n') {
      // handle command.
      cmd_buffer[cmd_len] = '\0';
      if (cmd_len > 0) {
        byte cmd0 = cmd_buffer[0];
        if (cmd0 >= 'a')  cmd0 -= ('a' - 'A');
        switch (cmd0) {
          case 'D':
            // Specify display request: D2359
            // Any invalid time string (eg. D-1) will stop the system trying to reach the time.
            move_to_time(parse_string_to_mins(cmd_buffer + 1));
            break;
          case 'W':
            // Warp display state (to match actual display): W2359
            flip_set_state_mins(parse_string_to_mins(cmd_buffer + 1));
            break;
          case 'T':
            // Enable (T1) / disable (T0) RTC time updates, or toggle: T
            enable_rtc_updates = (cmd_len==1) ? !enable_rtc_updates : (cmd_buffer[1] == '1');
            Serial.print("enable_rtc_updates: ");
            Serial.println(enable_rtc_updates);
            break;
          case 'S':
            // Execute some individual steps, bypassing the flip manager.
            // Typically, one minute is 170 or 171 steps (142 for Copal227).
            // We can use this to trim the stepper to be "mid cycle": 
            //  - slowly wind on by ~10 steps until flap falls
            //  - step on 85 (71) steps to be mid-cycle
            //  - Warp display to current reading, you are now good.
            add_steps(atoi(cmd_buffer + 1));
            break;
          case 'B':
            // Set backlight brightness 0..255.
            backlight_brightness = atoi(cmd_buffer + 1);
            analogWrite(backlight_led, backlight_brightness);
            break;
          case 'Z':
            // Set date/time: Z20211118094000 - 2021-11-18 09:40:00.
            if (strlen(cmd_buffer) != 15) {
              Serial.println("Bad format - Zyyyymmddhhmmss");
            } else {
              RTC_set_time(parse_time_string(cmd_buffer + 1));
            }
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

// -------------------------------------------------------------------
// Pin3 interrupt service
const byte INTERRUPT_PIN = 3;
volatile long tick_micros = 0;

void tick_ISR(void) {
  // Record the micros when tick occurs.
  tick_micros = micros();
}

void int_setup(void) {
  // Set interrupt input pin
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  // Attach the tick ISR
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), tick_ISR, FALLING);
}

long last_tick_micros = 0;

void int_update(void) {
  // Did the ISR update tick_micros?
  if (tick_micros != last_tick_micros) {
    Serial.print("Micros since last tick=");
    Serial.println(tick_micros - last_tick_micros);
    // Clear the interrupt.
    ds3231.checkIfAlarm(1);
    // Make sure the alarm is on.
    ds3231.turnOnAlarm(1);
    last_tick_micros = tick_micros;
  }
}

// -------------------------------------------------------------------
// Real-time clock.

//const int tz_offset = -4;  // Eastern Daylight Time (USA)

//time_t RTC_tzlocal_get(void) {
//  time_t utc_time = RTC.now().unixtime();
//  time_t local_time = utc_time + tz_offset * SECS_PER_HOUR;
//  return local_time;
//}

time_t RTC_utc_get(void) {
  return RTC.now().unixtime();
}

time_t now_local(void) {
  // Like now(), but includes timezone modification.
  return myTZ.toLocal(now());
}

void RTC_setup(void) {
  // Have the now() function return UTC.
  setSyncProvider(RTC_utc_get);   // the function to get the time from the RTC
  if(timeStatus() != timeSet)
     Serial.println("Unable to sync with the RTC");
  else {
     Serial.println("RTC has set the system time");
     tmElements_t tm;
     breakTime(now_local(), tm);
     serial_print_tm(tm);
  }
  // Set up the SQWV output. 0 = 1 Hz, 1 = 1024 Hz, 2 = 4096 Hz, 8 = 8192 Hz.
  byte frequency = 0;
  ds3231.enableOscillator(true, /* battery= */ false, frequency);
  // Actually, let's have Alarm1 interrupt us every minute.  AlarmBits is value from 
  // p12 table2 of https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
  // (I guess AlarmBits 0..3 are used for Alarm1 control, in setA1Time().)
  // 0xF = every second, 0xE = when seconds match; 0xC = min & sec match; 
  // 0x8 = hour, min, sec match; 0x0 = day/date, hour, min, sec match.
  ds3231.setA1Time(/* Day */ 0, /* Hour */ 0, /* Minute */ 0, /* Second */ 0,
                   /* AlarmBits= */ 0x0E, false, false, false);
  // Clear the interrupt.
  ds3231.checkIfAlarm(1);
  // Set SQWV output to Alarm interrupts, and enable Alarm1.
  ds3231.turnOnAlarm(1);
}

void RTC_set_time(const tmElements_t& tm) {
  // Set the DS3231 time.
  Serial.print("Set RTC: ");
  serial_print_tm(tm);
  ds3231.setYear(tmYearToY2k(tm.Year));
  ds3231.setMonth(tm.Month);
  ds3231.setDate(tm.Day);
  ds3231.setHour(tm.Hour);
  ds3231.setMinute(tm.Minute);
  ds3231.setSecond(tm.Second);
  // Resync TimeLib
  //setTime(tm.Hour, tm.Minute, tm.Second, tm.Day, tm.Month, tm.Year);
  setTime(RTC_utc_get());
}

int last_time_mins = -1;

void RTC_update(void) {
  // See if the minutes have changed.
  tmElements_t tm;
  breakTime(now_local(), tm);
  int time_mins = tm.Hour * 60 + tm.Minute;
  if (time_mins != last_time_mins) {
    Serial.print("Ard: ");
    serial_print_tm(tm);
    // Let's see how we're doing relative to the RTC.
    Serial.print("RTC: ");
    breakTime(RTC_utc_get(), tm);
    serial_print_tm(tm);    
    // Advance the flip display.
    if (enable_rtc_updates) {
      move_to_time(time_mins);
    }
    last_time_mins = time_mins;
  }
}

void serial_print_tm(const tmElements_t &tm)
{
  Serial.print(tmYearToCalendar(tm.Year));
  Serial.print("-");
  printDigits(tm.Month);
  Serial.print("-");
  printDigits(tm.Day);
  Serial.print(" ");
  printDigits(tm.Hour);
  Serial.print(":");
  printDigits(tm.Minute);
  Serial.print(":");
  printDigits(tm.Second);
  Serial.println();
}

// ------------------------
// Other misc system (tone generator, LEDs, wind-on button).

#define WIND_ON_BTN_PIN 2
#define BUZZER_PIN 6
#define ALARM_IN_PIN 4

void sys_setup(void) {
    tone_out[0].begin(2);  // This is a dummy, to stop Timer2 being used.
    tone_out[1].begin(BUZZER_PIN);  // This tone is assigned to Timer1, we'll use it.
    
    pinMode(backlight_led, OUTPUT);
    
    analogWrite(backlight_led, backlight_brightness);

    // Override tone_out[0] to use D2 as button input.
    pinMode(WIND_ON_BTN_PIN, INPUT_PULLUP);
    // Pulled down by the alarm microswitch.
    pinMode(ALARM_IN_PIN, INPUT_PULLUP);
}

int loop_count = 0;

void sys_update(void) {
  // Expect calls ~ every 10 ms.
  ++loop_count;
  if ((loop_count % 15) == 0) {
    // Once every 150 ms.
    if (loop_count <= 60) {
      // If the alarm switch is down, play a 50 ms 4kHz pip on the first 4 of 6 steps.
      if (digitalRead(ALARM_IN_PIN) == LOW) {
        tone_out[1].play(4000, 50);
      }
    }
  }
  loop_count %= 90;  // 90 x 10ms cycle

  // Check the wind-on button.
  byte button_down = 1 - digitalRead(WIND_ON_BTN_PIN);
  if (button_down) {
    // If button is down and stepper is about to stop stepping, add more steps.
    // We expect this to be updated every 10ms, so add if we have less than 10ms of steps left.
    if (steps_to_do < 5) {
      add_steps(10);
    }
  }
}

// ----------------------------------------------------------
// Arduino routines.

void setup() {
    // Start the I2C interface
    Wire.begin();

    // initialize serial communication at 9600 bits per second.
    Serial.begin(9600);
    
    Serial.println("Post-reset settling...");
    delay(2000);

    // Restore saved display state (from previous run or set_time_in_rtc_sram).
    long saved_display_time = load_display_state_from_sram();
    flip_setup(saved_display_time);
    RTC_setup();
    cmd_setup();
    int_setup();
    sys_setup();
}

void loop() {
  RTC_update();
  flip_update();
  cmd_update();
  int_update();
  sys_update();
  delay(10); // ~100 Hz looping.
}
