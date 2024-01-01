#include <OneWire.h>
#include <DallasTemperature.h>
#include <Servo.h>
#include <AccelStepper.h>

//Temperature
#define ONE_WIRE_BUS 30
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float tempC = 0;

//RGB light
#define Rpin 13
#define Gpin 12
#define Bpin 11
#define delayLEDS 3
#define sensorPin A2
float sensorValue = 0, filteredSignal = 0, filteredSignalValues[] = {3.4, 3.1, 2.7, 2.4, 2.1, 1.7, 1.3, 0.9, 0.4};

//Bluetooth light
int relay = 28;
char sms;

//Stepper motors
#define STEPPER1_STEP_PIN 2
#define STEPPER1_DIR_PIN 22
#define STEPPER2_STEP_PIN 3
#define STEPPER2_DIR_PIN 24

#define JOYSTICK_X_PIN A0
#define JOYSTICK_Y_PIN A1

#define MOTOR_STEPS_PER_REV 50

AccelStepper stepper1(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2(AccelStepper::DRIVER, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

//buttons for servo bracket movement

const int bButton = 4;     //left  CCW (pan)
const int wButton = 5;     //right CW (pan)
const int rButton = 6;    //up (tilt)
const int lButton = 7;    //down (tilt)

const int tiltPin = 10; //tilt servo on pin 10
const int panPin = 9; //pan servo on pin 9

//servos
Servo tilt;
Servo pan;

//helpers
int z, y, posT, posP; //pan, tilt pos (bits) & mapped positions (deg)
int deg = 180; //servo rotation in degrees

bool viewPrints = false; //true: prints out servo positions to serial monitor
//false: disables prints to serial monitor

bool initServos = true; //true: moves servos to "0" at startup
//false: servos startup at current position

//button states
int bState = 0;
int wState = 0;
int rState = 0;
int lState = 0;

//debouncing buttons
int bLBS = LOW; //blue button last state (init)
int wLBS = LOW; //white button last state (init)
int rLBS = LOW; //red button last state (init)
int lLBS = LOW; //black button last state (init)
unsigned long lastDebounceTime = 0;
unsigned long lastMoveTime = 0;
unsigned long debounceDelay = 50;

//speed & timing for Jog
int multiplier = 5; //steps between positions
int smoothTime = 10; //time to complete the move

unsigned long previousTime = 0;
unsigned long interval = 5000; // Interval for temperature check

void setup()
{
  sensors.begin();
  Serial.begin(9600);
  pinMode(26, OUTPUT);

  pinMode(relay, OUTPUT);
  digitalWrite(relay, HIGH);

  //stepper controlling
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(1000);
  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(1000);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);


  // Initialization for pan-tilt servos
  pinMode(bButton, INPUT);
  pinMode(wButton, INPUT);
  pinMode(rButton, INPUT);
  pinMode(lButton, INPUT);

  tilt.attach(tiltPin);
  pan.attach(panPin);

  if (initServos)
  {
    tilt.write(0);
    pan.write(0);
  }

  Serial.println("Initializing...");

}


void loop()
{
  temp();
  btl();
  stepper() ;
  buttonJogControl(); // Run jog control method
}

//RGB Light blinking
void FilterSignal(float sensorSignal) {
  filteredSignal = (0.945 * filteredSignal) + (0.0549 * sensorSignal);
}

void CompareSignalFiltered(float filteredSignal) {
  if (filteredSignal > filteredSignalValues[0])
  {
    RGBColor(0, 0, 255);
    Serial.println("Blue");
  }
  else if (filteredSignal <= filteredSignalValues[0] && filteredSignal > filteredSignalValues[1])
  {
    Serial.println("Azure");
    RGBColor(0, 255, 255);
  }
  else if (filteredSignal <= filteredSignalValues[1] && filteredSignal > filteredSignalValues[2])
  {
    RGBColor(0, 127, 255);
    Serial.println("Cyan");
  }
  else if (filteredSignal <= filteredSignalValues[2] && filteredSignal > filteredSignalValues[3])
  {
    RGBColor(0, 255, 127);
    Serial.println("Aqua marine");
  }
  else if (filteredSignal <= filteredSignalValues[3] && filteredSignal > filteredSignalValues[4])
  {
    RGBColor(0, 255, 0);
    Serial.println("Green");
  }
  else if (filteredSignal <= filteredSignalValues[5] && filteredSignal > filteredSignalValues[6])
  {
    RGBColor(255, 0, 255);
    Serial.println("Magenta");
  }
  else if (filteredSignal <= filteredSignalValues[6] && filteredSignal > filteredSignalValues[7])
  {
    RGBColor(255, 0, 127);
    Serial.println("Rose");
  }
  else if (filteredSignal <= filteredSignalValues[7] && filteredSignal > filteredSignalValues[8])
  {
    RGBColor(255, 0, 0);
    Serial.println("Red");
  }
  else if (filteredSignal <= filteredSignalValues[8])
  {
    RGBColor(255, 255, 255);
    Serial.println("White");
  }
}

void RGBColor(int red, int green, int blue) {
  analogWrite(Rpin, red);
  analogWrite(Gpin, green);
  analogWrite(Bpin, blue);
}


//servo bracket
void buttonJogControl() {
  bState = digitalRead(bButton);
  wState = digitalRead(wButton);
  rState = digitalRead(rButton);
  lState = digitalRead(lButton);
  unsigned long current = millis();
  if ((current- lastDebounceTime) > debounceDelay)
  {
    Serial.print(lastDebounceTime);
    Serial.print(" ");
    Serial.print(debounceDelay);
    Serial.print(" ");

    if (bState == HIGH && posP < deg)
    {
      z = z + 1;         //bits
      posP = z * multiplier; //deg
    }
    else
    {
      if (wState == HIGH && posP > 0)
      {
        z = z - 1;         //bits
        posP = z * multiplier; //deg
      }
    }
    if (lState == HIGH && posT < deg)
    {
      y = y + 1;         //bits
      posT = y * multiplier; //deg
    }
    else
    {
      if (rState == HIGH && posT > 0)
      {
        y = y - 1;         //bits
        posT = y * multiplier; //deg
      }
    }
    if (viewPrints)
    {
      Serial.print("Pan Current Position = ");
      Serial.print(posP);
      Serial.print(" deg ");
      Serial.print(" Tilt Current Position = ");
      Serial.print(posT);
      Serial.println(" deg ");
    }
    lastDebounceTime = millis();

    if((current - lastMoveTime) > 10){
      pan.write(posP);
      tilt.write(posT);
      lastMoveTime = millis();
    }
  }

}

void stepper() {
  //stepper
  int joystickX = analogRead(JOYSTICK_X_PIN);
  int joystickY = analogRead(JOYSTICK_Y_PIN);

  if (joystickX > 600)
  {
    stepper1.move(MOTOR_STEPS_PER_REV);
  }
  else if (joystickX < 400)
  {
    stepper1.move(-MOTOR_STEPS_PER_REV);
  }

  if (joystickY > 600)
  {
    stepper2.move(MOTOR_STEPS_PER_REV);
  }
  else if (joystickY < 400)
  {
    stepper2.move(-MOTOR_STEPS_PER_REV);
  }

  stepper1.runSpeedToPosition();
  stepper2.runSpeedToPosition();


  sensorValue = (float)analogRead(sensorPin) * (5.0 / 1024.0);
  FilterSignal(sensorValue);
  Serial.print(sensorValue);
  Serial.print(" ");
  Serial.println(filteredSignal);
  CompareSignalFiltered(filteredSignal);
}

void temp() {
  //power on fan using temperature sensor
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= interval)
  {
    previousTime = currentTime;

    sensors.requestTemperatures();
    tempC = sensors.getTempCByIndex(0);
    Serial.print(tempC);
    Serial.println(" C");

    if (tempC > 32)
    {
      digitalWrite(26, LOW);
      Serial.println("FAN ON");
    }
    else
    {
      digitalWrite(26, HIGH);
      Serial.println("FAN OFF");
    }
  }
}


void btl() {
  //Bluetooth light
  if (Serial.available() != 0)
  {
    sms = Serial.read();
  }
  if (sms == 'a')
  {
    digitalWrite(relay, LOW);
  }
  if (sms == 'b')
  {
    digitalWrite(relay, HIGH);
  }

}
