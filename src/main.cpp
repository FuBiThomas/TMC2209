// CODE FOR TEENSY 4.0 ; TMC2209 stepper driver; NEMA11 stepper motor; 24V external power supply
// Control Steppe Motor via Serial Monitor with commands:
// 's' - start motor (move to a large target position)
// 'p' - stop motor (decelerate to stop)
// Note: This code uses the AccelStepper library for motor control.
// Wiring:
// TMC2209    Teensy 4.0
// VIO        3.3V  Red
// GND        GND   Black
// VM         24V (externes Netzteil)   White
// GND        GND (externes Netzteil)   Black
// EN         Pin 9 (LOW to enable)     Purple
// STEP       Pin 10                     Green
// DIR        Pin 11                     Blue                      
// TMC2209    NEMA11
// A1         Black
// A2         Green
// B1         Red
// B2         Blue

#include "QuadEncoder.h"
#include <AccelStepper.h>

// Define stepper pins
#define EN_PIN 9         // LOW = Aktiv, HIGH = Deaktiviert
#define STEP_PIN 10      // Step pin
#define DIR_PIN 11       // Direction pin
// Encoder index input
#define INDEX_PIN 2      // Index signal (Z) des Encoders

// Mechanische Umrechnung
const long ENCODER_PULSES_PER_REV = 4000;   // Encoder-Quadraturauflösung
const long STEPPER_STEPS_PER_REV = 1600;    // Stepmotor-Schritte pro Umdrehung
const float SPINDLE_LEAD_MM_PER_REV = 1.0;  // Spindel-Steigung in mm pro Umdrehung

// AccelStepper Instanz im Driver-Modus
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

QuadEncoder meinEncoder(1, 0, 1, 0, 4); // Hardware-Modul 1: Phase A an Pin 0, Phase B an Pin 1

//Zeitsteuerung für die serielle Ausgabe (alle 100 Millisekunden)
unsigned long letzteAusgabeZeit = 0;
const unsigned long ausgabeIntervall = 1000; 
  
// Laufzeitzustand für get/set Befehle
bool outputsEnabled = true;
float currentAcceleration = 500.0; // spiegelt zuletzt gesetzte Acceleration wider

void deviation ();

void setup() {
  Serial.begin(9600); 

  meinEncoder.setInitConfig();

  meinEncoder.EncConfig.INDEXTriggerMode = RISING_EDGE;

  meinEncoder.init();

  // Index-Pin konfigurieren (Pullup für sauberen Zustand, je nach Sensor anpassen)
  pinMode(INDEX_PIN, INPUT_PULLDOWN);

  // Dem Stepper mitteilen, welcher Pin für Enable zuständig ist
  // Das 'true' am Ende ist entscheidend: Es invertiert das Signal! (LOW = Aktiv)
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(true, false, true); // (directionInvert, stepInvert, enableInvert)
  
  // Jetzt die Ausgänge aktivieren -> Schaltet EN_PIN sauber auf LOW
  stepper.enableOutputs();

  // Parameter setzen
  stepper.setMaxSpeed(10000);     // max Geschwindigkeit
  stepper.setSpeed(6000);
  stepper.setAcceleration(6000);  // Beschleunigung

  stepper.setCurrentPosition(0); // Startposition kalibrieren

  Serial.println("System bereit. Warte auf Befehle (t, a, h, r, p, g)...");
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
        Serial.println("09:52Uhr");
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
        stepper.moveTo(16000); 
      }
      else if (cmd == "home") { // home position
        Serial.println("Fahre zu Home-Position: 0");
        stepper.moveTo(0); 
      }
      else if (cmd == "axis homing") { // home position
        Serial.println("Achse wird gehomed");
        stepper.moveTo(0); 
      }
      else if (cmd == "r") { // relativ +800 Steps
        Serial.println("Fahre relativ: +1600 Steps (ganze Umdrehung)");
        stepper.move(1600); 
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
          stepper.move(steps);
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
          if (v > 0) { stepper.setMaxSpeed(v); Serial.print("maxSpeed gesetzt: "); Serial.println(v, 2); }
          else Serial.println("Ungültiger Wert für maxspeed");
        }
        else if (param == "accel" || param == "acceleration") {
          float v = val.toFloat();
          if (v > 0) { stepper.setAcceleration(v); currentAcceleration = v; Serial.print("Acceleration gesetzt: "); Serial.println(v, 2); }
          else Serial.println("Ungültiger Wert für acceleration");
        }
        else if (param == "speed") {
          float v = val.toFloat(); stepper.setSpeed(v); Serial.print("Speed gesetzt: "); Serial.println(v, 2);
        }
        else if (param == "position" || param == "pos") {
          long v = val.toInt(); stepper.setCurrentPosition(v); Serial.print("Position gesetzt: "); Serial.println(v);
        }
        else if (param == "target") {
          long v = val.toInt(); stepper.moveTo(v); Serial.print("Target gesetzt (moveTo): "); Serial.println(v);
        }
        else if (param == "enable") {
          String t = val; t.toLowerCase();
          if (t == "on" || t == "1" || t == "true") { stepper.enableOutputs(); outputsEnabled = true; Serial.println("Outputs enabled"); }
          else { stepper.disableOutputs(); outputsEnabled = false; Serial.println("Outputs disabled"); }
        }
        else if (param == "minpulse" || param == "minpulsewidth") {
          unsigned int v = (unsigned int)val.toInt(); stepper.setMinPulseWidth(v); Serial.print("minPulseWidth gesetzt: "); Serial.println(v);
        }
        else {
          Serial.print("Unbekannter Parameter: "); Serial.println(param);
        }
      }
      else if (cmd == "get") {
        String param = args;
        param.toLowerCase();
        if (param == "maxspeed") { Serial.print("maxSpeed: "); Serial.println(stepper.maxSpeed(), 2); }
        else if (param == "accel" || param == "acceleration") { Serial.print("acceleration: "); Serial.println(currentAcceleration, 2); }
        else if (param == "speed") { Serial.print("speed: "); Serial.println(stepper.speed(), 2); }
        else if (param == "position" || param == "pos") { Serial.print("currentPosition: "); Serial.println(stepper.currentPosition()); }
        else if (param == "target") { Serial.print("targetPosition: "); Serial.println(stepper.targetPosition()); }
        else if (param == "distance") { Serial.print("distanceToGo: "); Serial.println(stepper.distanceToGo()); }
        else if (param == "running") { Serial.print("isRunning: "); Serial.println(stepper.isRunning() ? 1 : 0); }
        else if (param == "enabled") { Serial.print("outputsEnabled: "); Serial.println(outputsEnabled ? "on" : "off"); }
        else {
          Serial.print("Unbekannter Parameter: "); Serial.println(param);
        }
      }
      else if (cmd == "p") { // Stop
        Serial.println("Stop-Befehl empfangen");
        stepper.stop();
      }
      else {
        Serial.print("Unknown command: ");
        Serial.println(cmd);
      }
    }
  }
  // Wichtig: Muss völlig frei laufen können
  stepper.run();


// --------------------------------------

  // Simultane und blockierfreie Ausgabe im Seriellen Monitor
  unsigned long aktuelleZeit = millis();
  if (aktuelleZeit - letzteAusgabeZeit >= ausgabeIntervall) {
  letzteAusgabeZeit = aktuelleZeit;

  // Die Rohwerte aus der Hardware
  long encoderPulses = meinEncoder.read();
  long indexPulses = (short)meinEncoder.getRevolution();
 
  // Berechnungen Motor
  long absolutMotorSteps = stepper.currentPosition(); 

  // Ausgabe
  Serial.print("EncoderPulses: ");
  Serial.print(encoderPulses);
  
  Serial.print(" | indexPulses: ");
  Serial.print(indexPulses); 
  
  Serial.print(" | Motor-Schritte: ");
  Serial.println(absolutMotorSteps);
}
}


void deviation () {

// Encoder-Werte
long encoderPulses = meinEncoder.read();
long indexPulses = (short)meinEncoder.getRevolution();

// Schrittmotor-Werte
long absolutMotorSteps = stepper.currentPosition(); 

// Hardware-Konfigurationen Stepper and Encoder
const long ENCODER_PULSES_PER_REV = 4000;   // Encoder-Quadraturauflösung
const long STEPPER_STEPS_PER_REV = 1600;    // Stepmotor-Schritte pro Umdrehung

float deviationFactor = STEPPER_STEPS_PER_REV / ENCODER_PULSES_PER_REV ; // current 13.07.2026 -> 1600/4000 Steps/Rev = 0,4

// float deviationThreshold = 4 ; // 16 stepperSteps = 0.01 mm -> minimum accuracy = 0.005mm -> +/- 0.0025mm = 4 Steps
//float realDeviationFactor = absolutMotorSteps / (ENCODER_PULSES_PER_REV * indexPulses + encoderPulses);

long currentAbsolutEncoderPosition;

long targetAbsolutEncoderPosition;

if ()

}