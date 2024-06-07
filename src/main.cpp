#include <TM1637Display.h>
#include <SimpleFOC.h>
#include <EEPROM.h>

/* DOKUMENTASI PROJECT
#################################################################################################
#   SPINDLE ORIENTATION FEELER
#   VBUS        : 24V @5A
#   SUPPLY IC   : 12V
#   BUILD       : FRIDAY, 17 NOVEMBER 2023
#################################################################################################

#PIN KONFIGURASI
MOTOR OUTPUT PWM
B13
B14
B15
A8
A9
A10

ENCODER >> INTERRUPT CHANGE, QUADRARTURE ON | CPR = 4*PPR
FEELER  >> 1000PPR >> 4000CPR
A4    A
A3    B
A5    Z

BUTTON INPUT >> LOW TRIGGER INPUT
A6    OK
B9    B++
B8    B--

ORIENTATION IO
B0    UMBRELLA FD     >> LOW TRIGGER INPUT
A7    UMBRELLA BD     >> LOW TRIGGER INPUT
A2    FEEDBACK OK     >> ACTIVE LOW OUTPUT
A1    REQUEST         >> LOW TRIGGER INPUT
A0    KONTAKTOR VFD   >> ACTIVE LOW OUTPUT
B11   KONTAKTOR ORI   >> ACTIVE LOW OUTPUT

DISPLAY               >> TM1637 7SEGMENT 4DIGIT
C15   CLK
C14   DIO
*/

// GLOBAL VARIABLE
// KONFOGURASI

#define LED_PIN PC13
// Encoder pin
#define A_ENCODER PA4
#define B_ENCODER PA3
#define Z_ENCODER PA5

#define buttonOK PA6
#define buttonUP PB9
#define buttonDN PB8

#define VFD PA0         // Kontaktor VFD
#define KONTAKTOR2 PB11 // Kontaktor board orientasi

#define REQUEST PA1
#define FEEDBACK PA2
#define UMBRELLA_H PB0
#define UMBRELLA_L PA7

#define DIO PC15
#define CLK PC14

#define vbusPin PB1
#define brakeMosfet PB10

// motor pin
#define PWM_HA PA8
#define PWM_HB PA9
#define PWM_HC PA10
#define PWM_LA PB13
#define PWM_LB PB14
#define PWM_LC PB15

// display
const uint8_t SEG_SET[] = {
    SEG_A | SEG_F | SEG_G | SEG_C | SEG_D, // S
    SEG_A | SEG_D | SEG_E | SEG_F | SEG_G, // E
    SEG_G | SEG_D | SEG_E | SEG_F          // t
};

int vbusOK = 0;
int vbusLow = 10;
int vbusHigh = 35;
int vbus = 0;

int save;

uint32_t RPM = 0;
int C = 0;                      // DISTANCE POSITION
int target = 0;                 // SETPOINT
int aligmenPos = 40000;         // HOMING DISTANCE
int pulse_counter = aligmenPos; // ENCODER VALUE
bool homed = 0;                 // HOMING CYCLE STATUS

int allowOrientation = 0;
int umbrellaON = 0;

int dtarget = 0;  // DISPLAY TARGET
int dpc = 0;      // DISPLAY ENCODER
int dpm = 0;      // DISPLAY RPM
int failSafe = 0; // TIMER ORIENTATION ERROR

int waktu;
int allowSetting;
int allowButton;

int quadrature = 0; // 1 OFF , 0 ON. CPR = 4*PPR

volatile long button_interval = 100;   //!< last impulse timestamp in us
volatile long setting_interval = 3000; //!< last impulse timestamp in us
volatile long index_timestamp = 0;     //!< last impulse timestamp in us
volatile long button_timestamp;        //!< last impulse timestamp in us
volatile int A_active;                 //!< current active states of A channel
volatile int B_active;                 //!< current active states of B channel
volatile int I_active;                 //!< current active states of Index channel

TM1637Display display(CLK, DIO);

BLDCMotor motor = BLDCMotor(2);
BLDCDriver6PWM driver = BLDCDriver6PWM(PWM_HA, PWM_LA, PWM_HB, PWM_LB, PWM_HC, PWM_LC);
// BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PB13, PA9, PB14, PA10, PB15);
Encoder sensor = Encoder(A_ENCODER, B_ENCODER, 1000, Z_ENCODER);

// ENCODER FUNCTION
// A channel
void doA()
{
  bool A = digitalRead(A_ENCODER);
  switch (quadrature)
  {
  case Quadrature::ON:
    if (A != A_active)
    {
      pulse_counter += (A_active == B_active) ? 1 : -1;
      A_active = A;
    }
    break;
  case Quadrature::OFF:
    if (A && !digitalRead(B_ENCODER))
    {
      pulse_counter++;
    }
    break;
  }
}
// B channel
void doB()
{
  bool B = digitalRead(B_ENCODER);
  switch (quadrature)
  {
  case Quadrature::ON:
    if (B != B_active)
    {
      pulse_counter += (A_active != B_active) ? 1 : -1;
      B_active = B;
    }
    break;
  case Quadrature::OFF:
    // CPR = PPR
    if (B && !digitalRead(A_ENCODER))
    {
      pulse_counter--;
      // pulse_timestamp = _micros();
    }
    break;
  }
}

// Z channel
void doIndex()
{
  int b = 0;
  if (digitalRead(Z_ENCODER))
  {
    pulse_counter = 0;
    uint32_t usRotation = micros() - index_timestamp;
    if (usRotation < 60000000)
    {
      RPM = 60000000 / usRotation;
      RPM * 2;
    }
    else
      RPM = 0;
    index_timestamp = micros();
  }
} // encoder sensor index, counter = 0

void setup()
{
  // FOC CONFIGURATION
  sensor.init();
  sensor.enableInterrupts(doA, doB, doIndex);

  motor.linkDriver(&driver);
  driver.pwm_frequency = 8000;
  driver.voltage_power_supply = 30;
  driver.dead_zone = 0.1;
  driver.init();

  motor.phase_resistance = 3; // 2.7;
  motor.voltage_limit = 100;
  motor.pole_pairs = 1;
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::velocity_openloop;
  motor.torque_controller = TorqueControlType::voltage;
  motor.init();
  motor.enable();

  // IO
  pinMode(REQUEST, INPUT_PULLUP);
  pinMode(UMBRELLA_L, INPUT_PULLUP);
  pinMode(UMBRELLA_H, INPUT_PULLUP);
  pinMode(buttonOK, INPUT_PULLUP);
  pinMode(buttonUP, INPUT_PULLUP);
  pinMode(buttonDN, INPUT_PULLUP);

  pinMode(vbusPin, INPUT_ANALOG);
  analogWriteFrequency(10000);
  analogWriteResolution(10);

  digitalWrite(VFD, 0);
  digitalWrite(KONTAKTOR2, 1);
  digitalWrite(FEEDBACK, 1);

  pinMode(LED_PIN, OUTPUT);
  pinMode(brakeMosfet, OUTPUT);
  pinMode(VFD, OUTPUT_OPEN_DRAIN);
  pinMode(KONTAKTOR2, OUTPUT_OPEN_DRAIN);
  pinMode(FEEDBACK, OUTPUT_OPEN_DRAIN);

  // EEPROM
  EEPROM.begin();
  EEPROM.get(0, target); // EEPROM ADDRESS 0 >> READ >> SET TARGET POSITION

  // DISPLAY
  display.setBrightness(3);
} // end setup

void poolButton()
{
  if ((waktu - button_timestamp) > button_interval)
  {
    allowButton = 1;
  }

  if (!digitalRead(buttonUP) && allowButton)
  {
    pulse_counter++;
    Serial.print("+");
    button_timestamp = millis();
    allowButton = 0;
  }
  else if (!digitalRead(buttonDN) && allowButton)
  {
    pulse_counter--;
    Serial.print("-");
    button_timestamp = millis();
    allowButton = 0;
  }

  waktu = millis();
  while (!digitalRead(buttonOK))
  {
    if (millis() - waktu > 2000)
    {
      EEPROM.put(0, pulse_counter);
      display.clear();
      display.setSegments(SEG_SET);
      digitalWrite(LED_PIN, 0);
      EEPROM.get(0, target);
    }
  }
} // END POOL BUTTON

void vbusSensing()
{
  vbus = analogRead(vbusPin) / 27; // konversi ke volt
  if (vbus < vbusLow)
  {
    vbusOK = 0;
  }
  else if (vbus > vbusHigh)
  {
    analogWrite(brakeMosfet, map(20, 0, 100, 0, 1024));
    vbusOK = 0;
  }
  else
  {
    analogWrite(brakeMosfet, 0);
    vbusOK = 1;
  }
} //  END VBUS SENSING

void report()
{
  vbusSensing();
  if (homed)
  {
    Serial.print("O ");
  }
  else
  {
    Serial.print("I ");
  }
  Serial.print(vbus);
  Serial.print(" ");
  Serial.print(pulse_counter);
  Serial.print(" ");
  Serial.print(target);
  Serial.print(" ");
  Serial.print(C);
  Serial.print(" ");
  Serial.println(failSafe);
} // END REPORT

void orientation()
{
  allowOrientation = 0;
  motor.disable();
  homed = 0;
  digitalWrite(FEEDBACK, 1);

  if (!digitalRead(KONTAKTOR2))
  {
    digitalWrite(KONTAKTOR2, 1);
    delay(300);
    digitalWrite(VFD, 0);
  }

  unsigned long RequOn = millis();
  while (!digitalRead(REQUEST) && digitalRead(UMBRELLA_H) && !digitalRead(UMBRELLA_L) && !digitalRead(VFD) && !allowOrientation)
  {
    if (millis() - RequOn > 1000)
    {
      digitalWrite(VFD, 1);
      delay(300);
      allowOrientation = 1;
      digitalWrite(KONTAKTOR2, 0);
      pulse_counter = aligmenPos;
    }
  }

  while (allowOrientation)
  {
    int oldtimer;
    // report();

    motor.loopFOC();
    motor.move();

    switch (homed)
    {
    case 0:
      motor.enable();
      homed = 1;
      break;
    case 1:
      C = pulse_counter - target; // CALCULATE ENCODER POSITION
      // ORIENTATION POSITION MODE STARTING
      if (abs(C) > 4000)
      {
        // motor.current_limit = 1.4;
        motor.target = -80 * _RPM_TO_RADS;
      }
      else if (C > 200)
      {
        motor.target = 60 * _RPM_TO_RADS;
      }
      else if (C < -200)
      {
        motor.target = -60 * _RPM_TO_RADS;
      }
      else if (C > 100)
      {
        motor.target = 8 * _RPM_TO_RADS;
      }
      else if (C < -100)
      {
        motor.target = -8 * _RPM_TO_RADS;
      }
      else if (C > 0)
      {
        motor.target = 2 * _RPM_TO_RADS;
      }
      else if (C < 0)
      {
        motor.target = -2 * _RPM_TO_RADS;
      }
      else
      {
        motor.target = 0;
        // motor.current_limit = 1;
      }

      if (abs(C) < 5)
      {
        motor.current_limit = 0.9;
      }
      else
      {
        motor.current_limit = 1.4;
      }

      if (!digitalRead(UMBRELLA_H))
      {
        if (millis() - umbrellaON > 100)
        {
          allowOrientation = 0;
        }
      }
      else
      {
        umbrellaON = millis();
      }

      if (abs(C) < 7)
      {
        if (millis() - oldtimer > 1000)
        {
          digitalWrite(FEEDBACK, 0);
          failSafe = 0;
        }
      }
      else
      {
        digitalWriteFast(digitalPinToPinName(FEEDBACK), 1);
        oldtimer = millis();
      }
      break; // ORIENTATION FINISHED
    }        // END SWITCH
  }          // end while
} // END ORIENTATION

void tesMotor()
{
  motor.move();
  motor.loopFOC();
  motor.target = 10;
}

void loop()
{
  // tesMotor();
  vbusSensing();
  if (RPM < 1000)
  {
    failSafe = 0;
    if (pulse_counter != dpc)
      display.showNumberDec(pulse_counter);
    dpc = pulse_counter; // UPDATE DISPLAY >> ENCODER
    if (target != dtarget)
      display.showNumberDec(target);
    dtarget = target; // UPDATE DISPLAY >> SETPOINT
    orientation();
    poolButton();
    // report();
  }
  else
  {
    if (RPM > 9999)
      RPM / 10;
    if (RPM != dpm)
      display.showNumberDec(RPM);
    dpm = RPM; // UPDATE DISPLAY >> RPM
  }
} // END LOOP