#include "Debounce/Debounce.h"
#include "SparkIntervalTimer.h"
#include "blynk.h"
//Blynk token kept in file listed in .gitignore
#include "AUTH_TOKEN.h"

const String event_prefix = "garage/door/";
// door sensor and control
const int SENSOR_CLOSED_INPUT_PIN = D1;
const int SENSOR_OPEN_INPUT_PIN = D2;
//Actual door pin for controlling relay to operate door
const int DOOR_OUTPUT_PIN = D3;
//Pin only initialized, never used
const int DOOR_OUTPUT_PIN2 = D4;
//The amount of time in milliseconds the GPIO output is kept HIGH
const int DOOR_OUTPUT_PULSE_TIME = 400;
//The amount of time in milliseconds the open/closed sensors
//take to react to the garage door opening/closing
const int DOOR_SENSOR_REACT_TIME = 3000;
//The maximum amount of time in milliseconds the garage door should take to open or close
const int DOOR_MAX_OPEN_CLOSE_TIME = 25000;
//The amount of time in milliseconds to wait before the garage door
//operator will respond to a second triggering of the GPIO output
const int DOOR_OUTPUT_PULSE_DELAY_TIME = 1250;
const int BLYNK_TERMINAL = V2;

// TYPE DEFINITIONS
enum DoorState
{
  DOORSTATE_OPEN = 0,
  DOORSTATE_CLOSED = 1,
  DOORSTATE_OPENING = 2,
  DOORSTATE_CLOSING = 3,
  DOORSTATE_STOPPED_OPENING = 4,
  DOORSTATE_STOPPED_CLOSING = 5,
  DOORSTATE_UNKNOWN = -1
};

static inline char *stringFromDoorState(enum DoorState doorState)
{
  switch (doorState)
  {
  case DOORSTATE_OPEN:
    return (char *)"open";
  case DOORSTATE_CLOSED:
    return (char *)"closed";
  case DOORSTATE_OPENING:
    return (char *)"opening";
  case DOORSTATE_CLOSING:
    return (char *)"closing";
  case DOORSTATE_STOPPED_OPENING:
    return (char *)"stopped-opening";
  case DOORSTATE_STOPPED_CLOSING:
    return (char *)"stopped-closing";
  case DOORSTATE_UNKNOWN:
    return (char *)"unknown";
  default:
    return (char *)"unknown";
  }
}

// door status, asume initial state is open
bool sensorOpen;
bool sensorClosed;
enum DoorState doorState = DOORSTATE_UNKNOWN;
enum DoorState doorStateLastKnown = DOORSTATE_UNKNOWN;
enum DoorState doorStatePublished = DOORSTATE_UNKNOWN;
String doorStatePublishedString = stringFromDoorState(DOORSTATE_UNKNOWN);
int doorTimeLastOperated = 0;

Debounce sensorOpenDebouncer = Debounce();
Debounce sensorClosedDebouncer = Debounce();
IntervalTimer monitorInputTimer;
BlynkTimer blynkTimer;
WidgetTerminal terminal(BLYNK_TERMINAL);

void setup()
{
  Particle.publish(event_prefix + "setup", "starting", PRIVATE);
  Particle.function("openClose", openClose);
  Particle.variable("state", doorStatePublishedString);

  delay(5000);   // Allow board to settle
  Time.zone(+1);
  Blynk.begin(BLYNK_AUTH_TOKEN);

  //Magnetic switch
  pinMode(SENSOR_CLOSED_INPUT_PIN, INPUT_PULLUP);
  sensorClosedDebouncer.attach(SENSOR_CLOSED_INPUT_PIN);
  sensorClosedDebouncer.interval(70);

  pinMode(SENSOR_OPEN_INPUT_PIN, INPUT_PULLUP);
  sensorOpenDebouncer.attach(SENSOR_OPEN_INPUT_PIN);
  sensorOpenDebouncer.interval(70);

  //Dual realy, but we will only use DOOR_OUTPUT_PIN. The setup is intended to reverse
  //the problem with pins set to high during startup and thus triggering the door unintentional
  pinMode(DOOR_OUTPUT_PIN, OUTPUT);
  pinMode(DOOR_OUTPUT_PIN2, OUTPUT);
  //Get current door state, every 2s (hmSec timescale (2000 * 0.5ms period))
  monitorInputTimer.begin(monitorInputs, 4000, hmSec);
  blynkTimer.setInterval(1000L, publishInputs);
}

// To open door if V1 button is pressed or reinitiate lock
BLYNK_WRITE(V1)
{
  String action = param.asStr();
  if (action == "1")
  {
    openClose("open");
    Blynk.setProperty(V1, "onLabel", "OPENING...");
  }
  else
  {
    openClose("close");
    Blynk.setProperty(V1, "offLabel", "CLOSING...");
  }
}

void updateBouncers(void)
{
  // Handle debouncing
  sensorOpenDebouncer.update();
  sensorClosedDebouncer.update();
}

void publishInputs(void)
{

  if (doorStatePublished != doorState)
  {
    //Lets publish new state
    doorStatePublished = doorState;
    doorStatePublishedString = stringFromDoorState(doorStatePublished);
    Particle.publish("door-state", doorStatePublishedString, PRIVATE);

    Blynk.virtualWrite(V0, doorStatePublishedString);

    if (doorStatePublished == DOORSTATE_OPEN)
    {
      Blynk.virtualWrite(V1, HIGH);
      Blynk.setProperty(V1, "onLabel", "CLOSE");
    }
    else if (doorStatePublished == DOORSTATE_CLOSED)
    {
      Blynk.virtualWrite(V1, LOW);
      Blynk.setProperty(V1, "offLabel", "OPEN");
    }

    if (doorStatePublished == DOORSTATE_OPEN || doorStatePublished == DOORSTATE_CLOSED ||
        doorStatePublished == DOORSTATE_STOPPED_OPENING || doorStatePublished == DOORSTATE_STOPPED_CLOSING ||
        doorStatePublished == DOORSTATE_UNKNOWN)
    {

      terminal.println(Time.format(Time.now(), "%F %R GarageDoor " + doorStatePublishedString));
      terminal.flush();
    }
  }
}

void monitorInputs(void)
{
  // Check door sensor inputs
  sensorOpen = !(bool)sensorOpenDebouncer.read();
  sensorClosed = !(bool)sensorClosedDebouncer.read();

  unsigned long timeNow = millis();

  if ((doorTimeLastOperated == 0) || ((timeNow - doorTimeLastOperated) >= DOOR_SENSOR_REACT_TIME))
  {
    if (sensorOpen && !sensorClosed)
    {
      doorState = DOORSTATE_OPEN;
    }
    else if (!sensorOpen && sensorClosed)
    {
      doorState = DOORSTATE_CLOSED;
    }
    else if (sensorOpen && sensorClosed)
    {
      doorState = DOORSTATE_UNKNOWN;
    }
    else if (!sensorOpen && !sensorClosed)
    {
      if ((doorState != DOORSTATE_OPENING) && (doorState != DOORSTATE_CLOSING) && (doorState != DOORSTATE_STOPPED_OPENING) && (doorState != DOORSTATE_STOPPED_CLOSING))
      {
        if (doorStateLastKnown == DOORSTATE_CLOSED)
        {
          doorState = DOORSTATE_OPENING;
        }
        else if (doorStateLastKnown == DOORSTATE_OPEN)
        {
          doorState = DOORSTATE_CLOSING;
        }
        else
        {
          doorState = DOORSTATE_UNKNOWN;
        }

        if ((doorState == DOORSTATE_OPENING) || (doorState == DOORSTATE_CLOSING))
        {
          // Door must have been manually operated
          doorTimeLastOperated = timeNow;
        }
      }
    }
  }

  if ((doorState == DOORSTATE_OPEN) || (doorState == DOORSTATE_CLOSED))
  {
    doorTimeLastOperated = 0;
    doorStateLastKnown = doorState;
  }
  else if ((doorState == DOORSTATE_OPENING) || (doorState == DOORSTATE_CLOSING))
  {
    if ((timeNow - doorTimeLastOperated) >= DOOR_MAX_OPEN_CLOSE_TIME)
    {
      // Door was opening/closing, but has now exceeded max open/close time
      doorTimeLastOperated = 0;

      if (doorState == DOORSTATE_OPENING)
      {
        doorState = DOORSTATE_STOPPED_OPENING;
      }
      else if (doorState == DOORSTATE_CLOSING)
      {
        doorState = DOORSTATE_STOPPED_CLOSING;
      }
    }
  }
}

void loop()
{
  updateBouncers();
  Blynk.run();
  blynkTimer.run();
}

//Particle function
int openClose(String action)
{
  int cycles = 0;

  if ((action == "0" || action == "open") && doorState != DOORSTATE_OPEN)
  {
    if ((doorState == DOORSTATE_CLOSED) || (doorState == DOORSTATE_STOPPED_CLOSING) || (doorState == DOORSTATE_UNKNOWN))
    {
      cycles = 1;
    }
    else if (doorState == DOORSTATE_CLOSING)
    {
      cycles = 2;
    }
    else if (doorState == DOORSTATE_STOPPED_OPENING)
    {
      cycles = 3;
    }

    doorState = DOORSTATE_OPENING;
  }
  else if ((action == "1" || action == "close") && doorState != DOORSTATE_CLOSED)
  {
    if ((doorState == DOORSTATE_OPEN) || (doorState == DOORSTATE_STOPPED_OPENING) || (doorState == DOORSTATE_UNKNOWN))
    {
      cycles = 1;
    }
    else if (doorState == DOORSTATE_OPENING)
    {
      cycles = 2;
    }
    else if (doorState == DOORSTATE_STOPPED_CLOSING)
    {
      cycles = 3;
    }

    doorState = DOORSTATE_CLOSING;
  }

  if (cycles > 0)
  {
    char line[50];
    sprintf(line, "Door moving to state: %s, using cycles: %d", stringFromDoorState(doorState), cycles);
    Particle.publish(event_prefix + "debug", line, PRIVATE);

    operateDoor(cycles);
    doorTimeLastOperated = millis();
  }

  return 1;
}

//Move the actual door, cycle relay enough times to 
//move door as per desired state
void operateDoor(int cycles)
{
  for (int i = 0; i < cycles; i++)
  {
    if (i != 0)
    {
      delay(DOOR_OUTPUT_PULSE_DELAY_TIME);
    }

    digitalWrite(DOOR_OUTPUT_PIN, HIGH);
    delay(DOOR_OUTPUT_PULSE_TIME);
    digitalWrite(DOOR_OUTPUT_PIN, LOW);
  }
}
