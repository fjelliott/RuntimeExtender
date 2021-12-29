#include <SevSeg.h>
#include <ButtonDebounce.h>
#include <EEPROM.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

// Configuration
const byte displayDigitPins[] = {1, 1, 1};
const byte displaySegmentPins[] = {1, 1, 1, 1, 1, 1, 1, 1};

const byte increaseDelayPin = 1;
const byte decreaseDelayPin = 1;
const byte sensorPin = 1;
const byte relayPin = 1;

#define BUTTON_DEBOUNCE_DELAY_MS 10
#define SENSOR_DEBOUNCE_DELAY_MS 500

#define TEST_DURATION_MS 1000
#define SLEEP_DELAY_MS 30000
#define ADJUST_WAIT_TIME_MS 10000

#define DEFAULT_RUN_DELAY_TIME_MS 10000
#define MIN_RUN_DELAY_TIME_MS 0
#define MAX_RUN_DELAY_TIME_MS 99500
#define RUN_DELAY_ADJUST_INCREMENT_MS 500

//#define SENSOR_USE_INTERNAL_PULLUP

// End Configuration

const byte displayNumDigits = sizeof(displayDigitPins);

enum State {
  Test,
  Idle,
  AdjustDelay,
  Run,
  RunDelay,
}
#if TEST_DURATION_MS > 0
state = State::Test;
#else
state = State::Idle;
#endif

SevSeg display;
ButtonDebounce increaseButton(increaseDelayPin, BUTTON_DEBOUNCE_DELAY_MS);
ButtonDebounce decreaseButton(decreaseDelayPin, BUTTON_DEBOUNCE_DELAY_MS);
ButtonDebounce sensor(sensorPin, SENSOR_DEBOUNCE_DELAY_MS);

EMPTY_INTERRUPT(PCINT0_vect);
EMPTY_INTERRUPT(PCINT1_vect);
EMPTY_INTERRUPT(PCINT2_vect);

unsigned long lastEvent = 0;

unsigned long runDelayTime_ms = 0;

void EnterSleep();
void IncreaseButtonHandler(int buttonState);
void DecreaseButtonHandler(int buttonState);
void EnablePCMSKForPin(byte pin);

void setup()
{
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  display.begin(COMMON_CATHODE, displayNumDigits, displayDigitPins, displaySegmentPins, false, false, false, false);
  display.setBrightness(100);

  increaseButton.setCallback(IncreaseButtonHandler);
  decreaseButton.setCallback(DecreaseButtonHandler);

  EEPROM.get(0, runDelayTime_ms);
  if (runDelayTime_ms > MAX_RUN_DELAY_TIME_MS)
  {
    // Corrupted or uninitialized. Reset stored value to default.
    runDelayTime_ms = DEFAULT_RUN_DELAY_TIME_MS;
    EEPROM.put(0, runDelayTime_ms);
  }

  #ifndef SENSOR_USE_INTERNAL_PULLUP
  pinMode(sensorPin, INPUT);
  #endif

  // Configure PC interrupts, which are used to wake up the arduino from sleep mode.
  PCICR = 0;
  PCMSK0 = 0b00000000;
  PCMSK1 = 0b00000000;
  PCMSK2 = 0b00000000;
  EnablePCMSKForPin(increaseDelayPin);
  EnablePCMSKForPin(decreaseDelayPin);
  EnablePCMSKForPin(sensorPin);
}

void loop()
{
  if (state != Test && sensor.state() == LOW)
  {
    lastEvent = millis();
    digitalWrite(relayPin, HIGH);
    state = Run;
  }
  
  switch (state)
  {
    case Test:
    {
      display.setChars("tst");
      if (millis() - lastEvent > 2000 + TEST_DURATION_MS)
      {
        state = Idle;
        lastEvent = millis();
      }
      else if (millis() - lastEvent > 1000 + TEST_DURATION_MS)
      {
        digitalWrite(relayPin, LOW);
      }
      else if (millis() - lastEvent > 1000)
      {
        digitalWrite(relayPin, HIGH);
      }
    }
    break;
    case Idle:
    {
      display.setChars("off");
      if (millis() - lastEvent > SLEEP_DELAY_MS)
      {
        EnterSleep();
        lastEvent = millis();
      }
    }
    break;
    case AdjustDelay:
    {
      display.setNumberF((float)runDelayTime_ms / 1000.0f, 1);
      if (millis() - lastEvent > ADJUST_WAIT_TIME_MS)
      {
        lastEvent = millis();
        state = Idle;
      }
    }
    break;
    case Run:
    {
      display.setChars("run");
      if (sensor.state() == HIGH)
      {
        lastEvent = millis();
        state = RunDelay;
      }
    }
    break;
    case RunDelay:
    {
      unsigned long timeSinceSensorOff_ms = millis() - lastEvent;
      display.setNumberF(((float)(runDelayTime_ms - timeSinceSensorOff_ms)) / 1000.0f, 1);
      if (sensor.state() == LOW)
      {
        // Sensor came back on.
        lastEvent = millis();
        state = Run;
      }
      else if (timeSinceSensorOff_ms > runDelayTime_ms)
      {
        lastEvent = millis();
        digitalWrite(relayPin, LOW);
        state = Idle;
      }
    }
    break;
  }

  display.refreshDisplay();
  sensor.update();
  increaseButton.update();
  decreaseButton.update();
}

void IncreaseButtonHandler(int buttonState)
{
  if (buttonState != LOW || (state != Idle && state != AdjustDelay))
  {
    return;
  }

  lastEvent = millis();
  
  if (state == Idle)
  {
    state == AdjustDelay;
    return;
  }

  if (runDelayTime_ms + RUN_DELAY_ADJUST_INCREMENT_MS > MAX_RUN_DELAY_TIME_MS)
  {
    runDelayTime_ms = MAX_RUN_DELAY_TIME_MS;
  }
  else
  {
    runDelayTime_ms += RUN_DELAY_ADJUST_INCREMENT_MS;
  }
  EEPROM.put(0, runDelayTime_ms);
}

void DecreaseButtonHandler(int buttonState)
{
  if (buttonState != LOW || (state != Idle && state != AdjustDelay))
  {
    return;
  }

  lastEvent = millis();
  
  if (state == Idle)
  {
    state == AdjustDelay;
    return;
  }

  if (runDelayTime_ms - MIN_RUN_DELAY_TIME_MS < RUN_DELAY_ADJUST_INCREMENT_MS)
  {
    runDelayTime_ms = MIN_RUN_DELAY_TIME_MS;
  }
  else
  {
    runDelayTime_ms -= RUN_DELAY_ADJUST_INCREMENT_MS;
  }
  EEPROM.put(0, runDelayTime_ms);
}

void EnterSleep()
{
  if (sensor.state() == LOW || digitalRead(sensorPin) == LOW)
  {
    // Sensor is on. Bail.
    return;
  }
  // Prepare to enter sleep mode...
  display.blank();
  // Save ADC register:
  byte adcsra = ADCSRA;
  ADCSRA = 0;
  // Set sleep mode:
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // Disable interrupts:
  cli();
  // We use the pin change interrupt to wake up.
  // Which pins we want interrupts to trigger for should already be configured in setup().
  // Enable pin change interrupts:
  PCICR = 0b00000111;
  // Enable sleeping:
  sleep_enable();
  // Disable brown-out detector:
  sleep_bod_disable();
  sei();
  // ZZZZZZZZZzzzzzzzzzzzzzzz.........
  sleep_cpu();
  // Clean up after sleeping:
  sleep_disable();
  ADCSRA = adcsra;
  // Disable pin change interrupts:
  PCICR = 0;
}

void EnablePCMSKForPin(uint8_t pin)
{
  if (pin <= 7)
  {
    bitSet(PCMSK2, pin);
  }
  else if (pin <= 13)
  {
    bitSet(PCMSK0, pin - 8);
  }
  else if (pin <= 19)
  {
    bitSet(PCMSK1, pin - 14);
  }
}
