// Config.h - pins, constants and shared types for the Nano firmware.
// Everything you are likely to tune or re-wire lives in this file.
#pragma once
#include <Arduino.h>

#define FW_VERSION  "0.6.0"
#define SERIAL_BAUD 9600        // must match NANO_BAUD in Screen/ui/ui.ino

// ---------------------------------------------------------------- Pins
// Linear actuators (dual H-bridge). NEVER drive UP and DOWN of the same
// actuator together - the driver will short itself. Only actuatorDrive()
// in Level.ino is allowed to touch these pins.
#define PIN_ACT1_UP    9        // Actuator 1 (at 150 deg), Timer1 PWM
#define PIN_ACT1_DOWN  10
#define PIN_ACT2_UP    5        // Actuator 2 (at 30 deg), Timer0 PWM
#define PIN_ACT2_DOWN  6

// HX711 load cell amplifiers: A (210 deg), B (90 deg), C (330 deg)
const uint8_t PIN_HX_DOUT[3] = {2, 7, 11};
const uint8_t PIN_HX_SCK[3]  = {3, 8, 12};

// Laser pointer servo + the two back-to-back lasers
#define PIN_SERVO      4        // PD4 - the Timer2 interrupt writes PORTD bit 4 directly
#define SERVO_PORT     PORTD
#define SERVO_BIT      4
#define PIN_LASER1     A2       // covers 0..180 deg (after LaserOffset)
#define PIN_LASER2     A3       // covers 180..360 deg

// I2C (A4 = SDA, A5 = SCL)
#define MPU_TOP_ADDR     0x68   // on the platform
#define MPU_BOTTOM_ADDR  0x69   // on the frame

// ---------------------------------------------------------------- Leveling
#define LEVEL_PERIOD_MS   20        // control loop period (50 Hz)
#define ACCEL_COUNTS_1G   16384.0f  // MPU6050 at +-2 g
#define MAX_BASE_ANGLE    13.0f     // deg - frame tilt the actuators cannot correct
#define BASE_CHECK_TICKS  25        // read the frame MPU every 25 ticks (0.5 s)
#define ARM_TICKS         5         // error must stay outside the band this long before moving (0.1 s)
#define STABLE_TICKS      25        // both actuators at rest this long = level (0.5 s)
#define STALE_TICKS       25        // identical MPU readings this long = sensor fault
#define LEVEL_TIMEOUT_MS  120000UL  // give up (Auto Stop mode only)
#define DEAD_TIME_MS      150       // both H-bridge inputs off this long before any restart
#define MASS_COMP_FULL_KG 20.0f     // load at which Mass Comp doubles the drive

// Actuator mixing. Platform top view, X to the right (0 deg), Y away from the
// screen (90 deg). Fixed pivot at 270 deg, actuators at 150 and 30 deg.
// For a small tilt the height change needed at each actuator is
//   act1 = pitchErr - MIX_ROLL * rollErr      act2 = pitchErr + MIX_ROLL * rollErr
// (roll tilts about the Y axis, pitch about the X axis; MIX_ROLL = cos30 / 1.5)
#define MIX_ROLL          0.5774f

// ---------------------------------------------------------------- Scales
// Unit vectors of the three load cell arms
#define LC_A_COS  (-0.8660254f)     // 210 deg
#define LC_A_SIN  (-0.5f)
#define LC_B_COS  (0.0f)            // 90 deg
#define LC_B_SIN  (1.0f)
#define LC_C_COS  (0.8660254f)      // 330 deg
#define LC_C_SIN  (-0.5f)

#define JOB_SAMPLES       10        // readings averaged for Tare / Balance / Cal (10 SPS -> 1 s)
#define SCALE_TIMEOUT_MS  1500      // a load cell that stays silent this long is reported
#define MIN_TOTAL_KG      1.0f      // below this there is no tire on the machine
#define MIN_CAL_COUNTS    1000      // smallest raw change accepted by CalA/B/C

// ---------------------------------------------------------------- Laser servo
#define SERVO_US_0        550       // pulse width at 0 deg
#define SERVO_US_180      2260      // pulse width at 180 deg

// ---------------------------------------------------------------- Serial events
#define LINE_MAX          48        // longest command line accepted
#define EV_RETRY_MS       400       // resend an event until the screen answers "Ack"
#define EV_MAX_TRIES      5

// ---------------------------------------------------------------- Types
enum : uint8_t { ST_IDLE, ST_LEVELING, ST_SAMPLING };

enum : uint8_t { JOB_NONE, JOB_TARE, JOB_BALANCE, JOB_CAL_A, JOB_CAL_B, JOB_CAL_C };

enum : uint8_t { EV_NONE, EV_LEVEL_DONE, EV_TARE_DONE, EV_BALANCE_DONE, EV_ERROR };

enum : uint8_t {
  ERR_BUSY, ERR_BASE_TILT, ERR_MPU_TOP, ERR_MPU_BOTTOM, ERR_LEVEL_TIMEOUT,
  ERR_SCALE, ERR_TIRE_DATA, ERR_NO_LOAD, ERR_NO_TARE, ERR_CAL
};

// Everything in here survives power-off ("Save" command, EEPROM).
#define SETTINGS_MAGIC 0x7B06
struct Settings {
  uint16_t magic;
  float cal[3];        // load cell counts per kg (sign included) - set with CalA/CalB/CalC
  float pitchOffset;   // deg - platform MPU reading when the platform is truly level
  float rollOffset;    // deg
  float minSpeed;      // PWM 0..255 - below this the actuators do not move
  float maxSpeed;      // PWM 0..255
  float kp;            // PWM per degree of error
  float ki;            // PWM per degree-second
  float kd;            // PWM per degree/second
  float alpha;         // angle smoothing 0..0.99 (higher = smoother, slower)
  float levelTol;      // deg - "level" means pitch and roll are both inside this
  float laserOffset;   // deg - servo zero vs. platform zero
  float testMass;      // kg added to every load cell while TestMass is on
};
