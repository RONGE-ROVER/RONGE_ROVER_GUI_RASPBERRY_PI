#include <Servo.h>

Servo myServo;  // Create a Servo object

int IN1pin = 9;
int IN2pin = 12;
int ENApin = 11;
int servoPin = 10;  // Pin connected to the control wire of the servo
int angle = 0;     // Variable to store the servo position (in degrees)
int vitesse = 0;
String inputString = "";         // Variable pour stocker la chaîne de caractères reçue
bool stringComplete = false;     // Drapeau pour indiquer que la chaîne est complète

void setup() {
  pinMode(IN1pin, OUTPUT);
  pinMode(IN2pin, OUTPUT);
  pinMode(ENApin, OUTPUT);
  myServo.attach(servoPin);  // Attach the servo on pin 10
  Serial.begin(9600);
  inputString.reserve(50);   // Réserve de l'espace pour optimiser les performances
}

void loop() {
  if (stringComplete) {
    // Sépare l'angle et la vitesse en utilisant la virgule
    int separatorIndex = inputString.indexOf(',');
    if (separatorIndex != -1) {
      angle = inputString.substring(0, separatorIndex).toInt();
      vitesse = inputString.substring(separatorIndex + 1).toInt();

      // Contrôle du servo
      myServo.write(angle);           // Set the servo position
      // Afficher les valeurs reçues
      Serial.print("angle: ");
      Serial.println(angle);
      Serial.print("vitesse: ");
      Serial.println(vitesse);

      // Contrôle du moteur en fonction de la vitesse
      if (vitesse > 0) {
        digitalWrite(IN1pin, HIGH);
        digitalWrite(IN2pin, LOW);
        analogWrite(ENApin, vitesse);
      } else if (vitesse < 0) {
        digitalWrite(IN1pin, LOW);
        digitalWrite(IN2pin, HIGH);
        analogWrite(ENApin, -vitesse);
      } else {
        digitalWrite(IN1pin, LOW);
        digitalWrite(IN2pin, LOW);
      }
    }
    // Réinitialise la chaîne pour recevoir de nouvelles données
    inputString = "";
    stringComplete = false;
  }
}

// Fonction pour recevoir les données caractère par caractère
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;

    // Vérifie si le caractère de fin de ligne est reçu ('\n')
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
