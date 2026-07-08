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

// Mechanische Umrechnung
const long ENCODER_PULSES_PER_REV = 4000;   // Encoder-Quadraturauflösung
const long STEPPER_STEPS_PER_REV = 1600;    // Stepmotor-Schritte pro Umdrehung
const float SPINDLE_LEAD_MM_PER_REV = 1.0;  // Spindel-Steigung in mm pro Umdrehung

// AccelStepper Instanz im Driver-Modus
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

  // Hardware-Modul 1: Phase A an Pin 0, Phase B an Pin 1
  QuadEncoder meinEncoder(1, 0, 1); 
  //Zeitsteuerung für die serielle Ausgabe (alle 100 Millisekunden)
  unsigned long letzteAusgabeZeit = 0;
  const unsigned long ausgabeIntervall = 1000; 
  
  // Laufzeitzustand für get/set Befehle
  bool outputsEnabled = true;
  float currentAcceleration = 500.0; // spiegelt zuletzt gesetzte Acceleration wider

void setup() {
  Serial.begin(9600); 

  meinEncoder.setInitConfig();
  meinEncoder.init();

  // Dem Stepper mitteilen, welcher Pin für Enable zuständig ist
  // Das 'true' am Ende ist entscheidend: Es invertiert das Signal! (LOW = Aktiv)
  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true); // (directionInvert, stepInvert, enableInvert)
  
  // Jetzt die Ausgänge aktivieren -> Schaltet EN_PIN sauber auf LOW
  stepper.enableOutputs();

  // Parameter setzen
  stepper.setMaxSpeed(4000);     // max Geschwindigkeit
  stepper.setAcceleration(100);  // Beschleunigung

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
        Serial.println("Test 12:19Uhr");
      }
      else if (cmd == "a") { // absolute Position 10 000
        Serial.println("Fahre zu absoluter Position: 10000");
        stepper.moveTo(10000); 
      }
      else if (cmd == "h") { // home position
        Serial.println("Fahre zu Home-Position: 0");
        stepper.moveTo(0); 
      }
      else if (cmd == "r") { // relativ +800 Steps
        Serial.println("Fahre relativ: +800 Steps (halbe Umdrehung)");
        stepper.move(800); 
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

    // Aktuellen Zählerstand direkt aus dem Hardware-Register lesen
    long encoderPulses = meinEncoder.read();
    float revolutions = (float)encoderPulses / ENCODER_PULSES_PER_REV;
    float motorSteps = revolutions * STEPPER_STEPS_PER_REV;
    float axisPositionMm = revolutions * SPINDLE_LEAD_MM_PER_REV;

    // Ausgabe
    Serial.print("Encoder-Pulse: ");
    Serial.print(encoderPulses);
    Serial.print(" | Motor-Schritte (äquivalent): ");
    Serial.print(motorSteps, 2);
    Serial.print(" | Achse: ");
    Serial.print(axisPositionMm, 2);
    Serial.println(" mm");
  }


}
