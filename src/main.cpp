// CODE FOR TEENSY 4.0 ; TMC2209 stepper driver; NEMA11 stepper motor; 24V external power supply
// Control Steppe Motor via Serial Monitor with commands:
// 's' - start motor (move to a large target position)
// 'p' - stop motor (decelerate to stop)
// Note: This code uses the AccelStepper library for motor control.
// Wiring:
// TMC2209_AXIS   Teensy 4.0
// VIO            3.3V  Red
// GND            GND   Black
// VM             24V (externes Netzteil)   White
// GND            GND (externes Netzteil)   Black
// EN             GND (LOW to enable)       Purple
// STEP           Pin 10                     Green
// DIR            Pin 11                     Blue                      
// TMC2209        NEMA11
// A1             Black
// A2             Green
// B1             Red
// B2             Blue


#include "QuadEncoder.h"
#include <AccelStepper.h>

// Define stepper pins
#define STEP_PIN_AXIS 5      // Step pin
#define DIR_PIN_AXIS 6       // Direction pin
#define STEP_PIN_FILTER 10   // Filter stepper step pin
#define DIR_PIN_FILTER 11    // Filter stepper direction pin

// Homesensor / Solenoid pins
#define HOMESENSOR_IN1_PIN 13
#define HOMESENSOR_IN2_PIN 14
#define HOMESENSOR_PULSE_MS 100  // Pulsdauer in Millisekunden

// Mechanische Umrechnung
const long ENCODER_PULSES_PER_REV = 4000;   // Encoder-Quadraturauflösung
const long STEPPER_STEPS_PER_REV = 1600;    // Stepmotor-Schritte pro Umdrehung
const float SPINDLE_LEAD_MM_PER_REV = 1.0;  // Spindel-Steigung in mm pro Umdrehung

// AccelStepper Instanzen im Driver-Modus
AccelStepper STEPPER_AXIS(AccelStepper::DRIVER, STEP_PIN_AXIS, DIR_PIN_AXIS);
AccelStepper STEPPER_FILTER(AccelStepper::DRIVER, STEP_PIN_FILTER, DIR_PIN_FILTER);

QuadEncoder ENCODER_AXIS(2, 2, 3, 0, 4); // Hardware-Modul 1: Phase A an Pin 0, Phase B an Pin 1
QuadEncoder ENCODER_FILTER(4, 7, 8, 0); // Hardware-Modul 4: Phase A an Pin 7, Phase B an Pin 8, kein Index

//Zeitsteuerung für die serielle Ausgabe (alle 100 Millisekunden)
unsigned long letzteAusgabeZeit = 0;
const unsigned long ausgabeIntervall = 1000; 
  
// Laufzeitzustand für get/set Befehle
bool outputsEnabled = true;
float currentAcceleration = 500.0; // spiegelt zuletzt gesetzte Acceleration wider

void deviationAxis ();
void homingAxis();
void homesensor(int state);

void setup() {
  Serial.begin(9600); 

  ENCODER_AXIS.setInitConfig();

  ENCODER_AXIS.EncConfig.enableModuloCountMode = 0;
  ENCODER_AXIS.EncConfig.revolutionCountCondition = DISABLE;
  ENCODER_AXIS.EncConfig.INDEXTriggerMode = 0;      // standardmäßig deaktiviert, wird nur beim Homing aktiv
  ENCODER_AXIS.EncConfig.IndexTrigger = DISABLE;

  ENCODER_AXIS.init();

  ENCODER_FILTER.setInitConfig();

  ENCODER_FILTER.EncConfig.enableModuloCountMode = 0;
  ENCODER_FILTER.EncConfig.revolutionCountCondition = DISABLE;
  ENCODER_FILTER.EncConfig.INDEXTriggerMode = 0;    // kein Index für ENCODER_FILTER
  ENCODER_FILTER.EncConfig.IndexTrigger = DISABLE;

  ENCODER_FILTER.init();

  // Dem Stepper mitteilen, welcher Pin für Enable zuständig ist
  // Das 'true' am Ende ist entscheidend: Es invertiert das Signal! (LOW = Aktiv)
  STEPPER_AXIS.setPinsInverted(true, false, true); // (directionInvert, stepInvert, enableInvert)
  STEPPER_FILTER.setPinsInverted(true, false, true);
  
  // Jetzt die Ausgänge aktivieren -> Schaltet EN_PIN sauber auf LOW
  STEPPER_AXIS.enableOutputs();
  STEPPER_FILTER.enableOutputs();

  // Parameter setzen
  STEPPER_AXIS.setMaxSpeed(10000);     // max Geschwindigkeit
  STEPPER_AXIS.setSpeed(6000);
  STEPPER_AXIS.setAcceleration(6000);  // Beschleunigung

  STEPPER_FILTER.setMaxSpeed(10000);
  STEPPER_FILTER.setSpeed(6000);
  STEPPER_FILTER.setAcceleration(6000);

  STEPPER_AXIS.setCurrentPosition(0); // Startposition kalibrieren
  STEPPER_FILTER.setCurrentPosition(0); // Filter-Startposition kalibrieren

  pinMode(HOMESENSOR_IN1_PIN, OUTPUT);
  pinMode(HOMESENSOR_IN2_PIN, OUTPUT);
  digitalWrite(HOMESENSOR_IN1_PIN, LOW);
  digitalWrite(HOMESENSOR_IN2_PIN, LOW);

  Serial.println("System bereit. Warte auf Befehle (t, a, h, r, p, g, homesensor)...");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      int sp = line.indexOf(' ');
      String cmd = (sp == -1) ? line : line.substring(0, sp);
      String args = (sp == -1) ? "" : line.substring(sp + 1);
      cmd.toLowerCase();
      args.trim();

      if (cmd == "t") { // Test
        Serial.println("24.07.2026 #1");
      }
      else if (cmd == "help" || cmd == "?") {
        Serial.println("Verfügbare Befehle:");
        Serial.println(" t            - Test");
        Serial.println(" a            - goto absolute position 10000 (demo)");
        Serial.println(" h            - go home (position 0)");
        Serial.println(" r            - move relative (predefined 800)");
        Serial.println(" g<steps>     - move relative by steps (z.B. g1000 oder g 1000)");
        Serial.println(" set <param> <value> - set parameters: maxspeed, accel, speed, position, target, enable, minpulse");
        Serial.println(" get <param>  - get parameters: maxspeed, accel, speed, position, target, distance, running, enabled");
        Serial.println(" p            - stop (decelerate)");
      }
      else if (cmd == "a") { // absolute Position 10 000
        Serial.println("Fahre zu absoluter Position: 16000");
        STEPPER_AXIS.moveTo(16000); 
      }
      else if (cmd == "home") { // home position
        Serial.println("Fahre zu Home-Position: 0");
        STEPPER_AXIS.moveTo(0); 
      }
      else if (cmd == "resetencoder") { // home position
        Serial.println("Reset Enocer-Pulcounter to zero!");
        ENCODER_AXIS.write(0);                               // Setzt den aktuellen Pulscounter zurück auf 0; 
      }
      else if (cmd == "calibration") { // home position
        Serial.println("calibrate Encoder-Counter with Stepper-Counter");
        ENCODER_AXIS.write(0);                               // Setzt den aktuellen Pulscounter zurück auf 0; 
      }
      else if (cmd == "homingAxis") { // home position
      Serial.println("Start axis homing ... ");
      homingAxis (); 
      }
      else if (cmd == "1") {
        Serial.println("Fahre relativ: +100 Steps");
        STEPPER_AXIS.move(100);
      }
      else if (cmd == "2") {
        Serial.println("Fahre relativ: +200 Steps");
        STEPPER_AXIS.move(200);
      }
      else if (cmd == "3") {
        Serial.println("Fahre relativ: +300 Steps");
        STEPPER_AXIS.move(300);
      }
      else if (cmd == "4") {
        Serial.println("Fahre relativ: +400 Steps");
        STEPPER_AXIS.move(400);
      }
      else if (cmd == "6") {
        Serial.println("Fahre relativ: -100 Steps");
        STEPPER_AXIS.move(-100);
      }
      else if (cmd == "7") {
        Serial.println("Fahre relativ: -200 Steps");
        STEPPER_AXIS.move(-200);
      }
      else if (cmd == "8") {
        Serial.println("Fahre relativ: -300 Steps");
        STEPPER_AXIS.move(-300);
      }
      else if (cmd == "9") {
        Serial.println("Fahre relativ: -400 Steps");
        STEPPER_AXIS.move(-400);
      }      
      else if (cmd == "g") { // move relative with parameter
        long steps = 0;
        if (args.length() > 0) steps = args.toInt();
        // support g1000 without space
        if (steps == 0 && cmd.length() > 1) steps = cmd.substring(1).toInt();
        if (steps == 0 && args.indexOf('0') == -1) {
          Serial.println("Fehler: Keine gültige Schrittzahl nach 'g' gefunden.");
        } else {
          Serial.print("Fahre relativ: ");
          Serial.print(steps);
          Serial.println(" Steps");
          STEPPER_AXIS.move(steps);
        }
      }
      else if (cmd == "homesensor") {
        if (args == "1") {
          Serial.println("Homesensor: Solenoid ausfahren");
          homesensor(1);
        } else if (args == "0") {
          Serial.println("Homesensor: Solenoid einfahren");
          homesensor(0);
        } else {
          Serial.println("Fehler: homesensor erwartet 0 oder 1");
        }
      }
      else if (cmd == "set") {
        int sp2 = args.indexOf(' ');
        String param = (sp2 == -1) ? args : args.substring(0, sp2);
        String val = (sp2 == -1) ? "" : args.substring(sp2 + 1);
        param.toLowerCase();
        val.trim();

        if (param == "maxspeed") {
          float v = val.toFloat();
          if (v > 0) { STEPPER_AXIS.setMaxSpeed(v); Serial.print("maxSpeed gesetzt: "); Serial.println(v, 2); }
          else Serial.println("Ungültiger Wert für maxspeed");
        }
        else if (param == "accel" || param == "acceleration") {
          float v = val.toFloat();
          if (v > 0) { STEPPER_AXIS.setAcceleration(v); currentAcceleration = v; Serial.print("Acceleration gesetzt: "); Serial.println(v, 2); }
          else Serial.println("Ungültiger Wert für acceleration");
        }
        else if (param == "speed") {
          float v = val.toFloat(); STEPPER_AXIS.setSpeed(v); Serial.print("Speed gesetzt: "); Serial.println(v, 2);
        }
        else if (param == "position" || param == "pos") {
          long v = val.toInt(); STEPPER_AXIS.setCurrentPosition(v); Serial.print("Position gesetzt: "); Serial.println(v);
        }
        else if (param == "target") {
          long v = val.toInt(); STEPPER_AXIS.moveTo(v); Serial.print("Target gesetzt (moveTo): "); Serial.println(v);
        }
        else if (param == "enable") {
          String t = val; t.toLowerCase();
          if (t == "on" || t == "1" || t == "true") { STEPPER_AXIS.enableOutputs(); outputsEnabled = true; Serial.println("Outputs enabled"); }
          else { STEPPER_AXIS.disableOutputs(); outputsEnabled = false; Serial.println("Outputs disabled"); }
        }
        else if (param == "minpulse" || param == "minpulsewidth") {
          unsigned int v = (unsigned int)val.toInt(); STEPPER_AXIS.setMinPulseWidth(v); Serial.print("minPulseWidth gesetzt: "); Serial.println(v);
        }
        else {
          Serial.print("Unbekannter Parameter: "); Serial.println(param);
        }
      }
      else if (cmd == "get") {
        String param = args;
        param.toLowerCase();
        if (param == "maxspeed") { Serial.print("maxSpeed: "); Serial.println(STEPPER_AXIS.maxSpeed(), 2); }
        else if (param == "accel" || param == "acceleration") { Serial.print("acceleration: "); Serial.println(currentAcceleration, 2); }
        else if (param == "speed") { Serial.print("speed: "); Serial.println(STEPPER_AXIS.speed(), 2); }
        else if (param == "position" || param == "pos") { Serial.print("currentPosition: "); Serial.println(STEPPER_AXIS.currentPosition()); }
        else if (param == "target") { Serial.print("targetPosition: "); Serial.println(STEPPER_AXIS.targetPosition()); }
        else if (param == "distance") { Serial.print("distanceToGo: "); Serial.println(STEPPER_AXIS.distanceToGo()); }
        else if (param == "running") { Serial.print("isRunning: "); Serial.println(STEPPER_AXIS.isRunning() ? 1 : 0); }
        else if (param == "enabled") { Serial.print("outputsEnabled: "); Serial.println(outputsEnabled ? "on" : "off"); }
        else {
          Serial.print("Unbekannter Parameter: "); Serial.println(param);
        }
      }
      else if (cmd == "p") { // Stop
        Serial.println("Stop-Befehl empfangen");
        STEPPER_AXIS.stop();
      }
      else {
        Serial.print("Unknown command: ");
        Serial.println(cmd);
      }
    }
  }
  // Wichtig: Muss völlig frei laufen können
  STEPPER_AXIS.run();
  STEPPER_FILTER.run();


// --------------------------------------

  // Simultane und blockierfreie Ausgabe im Seriellen Monitor
  unsigned long aktuelleZeit = millis();
  if (aktuelleZeit - letzteAusgabeZeit >= ausgabeIntervall) {
    letzteAusgabeZeit = aktuelleZeit;

    // Die Rohwerte aus der Hardware
    long encoderPulsesAxis = ENCODER_AXIS.read();
    long encoderPulsesFilter = ENCODER_FILTER.read();
    long indexPulsesAxis = (short)ENCODER_AXIS.getRevolution(); 
    
    // Berechnungen Motor
    long absolutStepsAxis = STEPPER_AXIS.currentPosition();
    long absolutStepsFilter = STEPPER_FILTER.currentPosition();

    // Ausgabe
    Serial.print("encoderPulsesAxis: ");
    Serial.print(encoderPulsesAxis);
    
    Serial.print(" | indexPulsesAxis: ");
    Serial.print(indexPulsesAxis); 
    
    Serial.print(" | absolutStepsAxis: ");
    Serial.print(absolutStepsAxis);
    
    Serial.print(" | encoderPulsesFilter: ");
    Serial.print(encoderPulsesFilter);

    Serial.print(" | absolutStepsFilter: ");
    Serial.print(absolutStepsFilter);

    //if (!stepper.isRunning()) {
      deviationAxis();
    //}
  }
}

void deviationAxis () {

  // Encoder-Werte
  long encoderPulsesAxis = ENCODER_AXIS.read();
  // long indexPulsesAxis = (short)ENCODER_AXIS.getRevolution();

  // Schrittmotor-Werte
  long absolutStepsAxis = STEPPER_AXIS.currentPosition(); 

  // Hardware-Konfigurationen Stepper and Encoder
  const long ENCODER_PULSES_PER_REV = 4000;   // Encoder-Quadraturauflösung
  const long STEPPER_STEPS_PER_REV = 1600;    // Stepmotor-Schritte pro Umdrehung

  // Faktor/Verhältnis Encoder-Schritte zu Stepper-Schritte je volle Umdrehung - Hardwareabhängig 
  float deviationFactor = (float)STEPPER_STEPS_PER_REV / (float)ENCODER_PULSES_PER_REV; // current 13.07.2026 -> 1600/4000 Steps/Rev = 0,4

  // Tatsächliche und erwartete Encoder-Position
  long currentAbsolutEncoderPosition = encoderPulsesAxis ;
  long targetAbsolutEncoderPosition = absolutStepsAxis / deviationFactor ;

  // Erlaubte Abweichung vom Soll
  // float deviationThreshold = 4 ; // 16 stepperSteps = 0.01 mm -> minimum accuracy = 0.005mm -> +/- 0.0025mm = 4 Steps
  short currentDeviation = currentAbsolutEncoderPosition - targetAbsolutEncoderPosition ;

  // Berechnung der Abweichung
  // if (abs(currentAbsolutEncoderPosition - targetAbsolutEncoderPosition) > deviationThreshold) {
  //   Serial.println("deviation to high. homingAxis necessary!");
  // }

  // Ausgabe
  Serial.print(" | Encoder-Achse-Soll : ");
  Serial.print(targetAbsolutEncoderPosition);
  Serial.print(" | Encoder-Achse-: ");
  Serial.println(currentDeviation);
}

void homingAxis() {
  // Index-Trigger nur für das Homing aktivieren
  ENCODER_AXIS.setInitConfig();                  // activate Index-Trigger-Mode
  ENCODER_AXIS.EncConfig.INDEXTriggerMode = RISING_EDGE;
  ENCODER_AXIS.EncConfig.IndexTrigger = true;
  ENCODER_AXIS.init();

  STEPPER_AXIS.setSpeed(100);                                      // slow speed to find Index-Trigger
  long startingRevolution = ENCODER_AXIS.getRevolution();      // take reference value from index-trigger ...
  while (ENCODER_AXIS.getRevolution() == startingRevolution) { // ... and rotate stepper as long as value is changing
    STEPPER_AXIS.runSpeed();
  }

  STEPPER_AXIS.stop(); 
  STEPPER_AXIS.setCurrentPosition(0); 
  ENCODER_AXIS.write(0);
  delay(1000);
  STEPPER_AXIS.setSpeed(1000);       // reset normal speed
  STEPPER_AXIS.setMaxSpeed(2000);

  ENCODER_AXIS.setInitConfig();                  // deactivate Index-Trigger-Mode
  ENCODER_AXIS.EncConfig.INDEXTriggerMode = 0;   
  ENCODER_AXIS.EncConfig.IndexTrigger = false;
  ENCODER_AXIS.init();
}

void homesensor(int state) {
  if (state == 1) {
    // Solenoid ausfahren: IN1 HIGH, IN2 LOW
    digitalWrite(HOMESENSOR_IN1_PIN, HIGH);
    digitalWrite(HOMESENSOR_IN2_PIN, LOW);
    delay(HOMESENSOR_PULSE_MS);
    digitalWrite(HOMESENSOR_IN1_PIN, LOW);
  } else if (state == 0) {
    // Solenoid einfahren: IN1 LOW, IN2 HIGH
    digitalWrite(HOMESENSOR_IN1_PIN, LOW);
    digitalWrite(HOMESENSOR_IN2_PIN, HIGH);
    delay(HOMESENSOR_PULSE_MS);
    digitalWrite(HOMESENSOR_IN2_PIN, LOW);
  } else {
    Serial.println("Fehler: homesensor erwartet 0 oder 1");
  }
}