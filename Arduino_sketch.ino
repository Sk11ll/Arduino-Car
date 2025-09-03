#include <SoftwareSerial.h> // Libreria per la comunicazione seriale con HC-05

// --- CONFIGURAZIONE PIN ARDUINO ---

// Modulo HC-05 Bluetooth
// COLLEGARE:
//   HC-05 TX -> Arduino Digital Pin 2 (Questo è il pin RX per SoftwareSerial)
//   HC-05 RX -> Arduino Digital Pin 3 (Questo è il pin TX per SoftwareSerial)
//   *** IMPORTANTE: USARE UN DIVISORE DI TENSIONE (es. 1k Ohm e 2k Ohm) SUL PIN RX DELL'HC-05! ***
//   *** Il pin TX dell'Arduino (D3) emette 5V, mentre l'HC-05 RX si aspetta 3.3V. ***
#define HC05_RX_PIN 10
#define HC05_TX_PIN 11
SoftwareSerial bluetoothSerial(HC05_RX_PIN, HC05_TX_PIN); // Crea un oggetto SoftwareSerial per l'HC-05

// Driver Motori L293D (Basato sul tuo schema)
// MOTORE 1 (es. motore sinistro)
#define MOTOR1_IN1_PIN 8     // Collegato a IN1 del L293D
#define MOTOR1_IN2_PIN 7     // Collegato a IN2 del L293D
#define MOTOR1_ENABLE_PIN 9  // Collegato a EN1/2 del L293D (Deve essere un pin PWM)

// MOTORE 2 (es. motore destro)
#define MOTOR2_IN1_PIN 5     // Collegato a IN3 del L293D
#define MOTOR2_IN2_PIN 4     // Collegato a IN4 del L293D
#define MOTOR2_ENABLE_PIN 3  // Collegato a EN3/4 del L293D (Deve essere un pin PWM)

// --- VARIABILI GLOBALI ---

char lastMotorCommand = 'S'; // Memorizza l'ultimo comando di movimento ('F','B','L','R','S')
int motorSpeed = 150;        // Velocità iniziale dei motori (valore da 0 a 255)
int motorSpeed2 = 150;

// --- FUNZIONI PER IL CONTROLLO DEI MOTORI ---

/**
 * Imposta la velocità di un motore tramite il pin ENABLE (PWM).
 * @param enablePin Il pin ENABLE del driver motore (es. MOTOR1_ENABLE_PIN).
 * @param speed La velocità desiderata (da 0 a 255). Il valore viene limitato automaticamente.
 */
void setMotorSpeed(int enablePin, int speed) {
  speed = constrain(speed, 0, 255); // Assicura che la velocità sia tra 0 e 255
  //analogWrite(enablePin, speed);    // Applica la velocità tramite PWM
  analogWrite(3, speed);
  analogWrite(9, speed);
}

/**
 * Controlla la direzione di un singolo motore.
 * @param in1 Pin IN1 del driver L293D per questo motore.
 * @param in2 Pin IN2 del driver L293D per questo motore.
 * @param direction Direzione: 1 = Avanti, -1 = Indietro, 0 = Stop/Freno.
 */
void controlMotorDirection(int in1, int in2, int direction) {
  if (direction == 1) {      // Movimento in avanti
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (direction == -1) { // Movimento all'indietro
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {                   // Stop (freno, o coasting a seconda del driver)
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);  // Per un freno più forte con L293D, puoi anche usare (HIGH, HIGH)
  }
}

/**
 * Fa avanzare la macchinina.
 * Entrambi i motori si muovono in avanti alla velocità corrente.
 */
void moveForward() {
  controlMotorDirection(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);
  controlMotorDirection(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, 1);
  setMotorSpeed(MOTOR1_ENABLE_PIN, motorSpeed);
  setMotorSpeed(MOTOR2_ENABLE_PIN, motorSpeed);
  lastMotorCommand = 'F';
}

/**
 * Fa indietreggiare la macchinina.
 * Entrambi i motori si muovono all'indietro alla velocità corrente.
 */
void moveBackward() {
  controlMotorDirection(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, -1);
  controlMotorDirection(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, -1);
  setMotorSpeed(MOTOR1_ENABLE_PIN, motorSpeed);
  setMotorSpeed(MOTOR2_ENABLE_PIN, motorSpeed2);
  lastMotorCommand = 'B';
}

/**
 * Ferma entrambi i motori.
 * La velocità di entrambi i motori viene impostata a 0.
 */
void stopAllMotors() {
  controlMotorDirection(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 0);
  controlMotorDirection(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, 0);
  setMotorSpeed(MOTOR1_ENABLE_PIN, 0);
  setMotorSpeed(MOTOR2_ENABLE_PIN, 0);
  lastMotorCommand = 'S';
}

/**
 * Fa girare la macchinina a destra (sterzo differenziale).
 * Il motore sinistro va avanti, il motore destro va indietro (per una rotazione sul posto).
 * Puoi modificare la logica di sterzo se preferisci (es. un motore fermo, l'altro avanti).
 */
void turnRight() {
  controlMotorDirection(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, 1);  // Motore sinistro avanti
  controlMotorDirection(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, -1); // Motore destro indietro
  setMotorSpeed(MOTOR1_ENABLE_PIN, motorSpeed);
  setMotorSpeed(MOTOR2_ENABLE_PIN, motorSpeed2);
  lastMotorCommand = 'R';
}

/**
 * Fa girare la macchinina a sinistra (sterzo differenziale).
 * Il motore sinistro va indietro, il motore destro va avanti (per una rotazione sul posto).
 */
void turnLeft() {
  controlMotorDirection(MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, -1); // Motore sinistro indietro
  controlMotorDirection(MOTOR2_IN1_PIN, MOTOR2_IN2_PIN, 1);  // Motore destro avanti
  setMotorSpeed(MOTOR1_ENABLE_PIN, motorSpeed);
  setMotorSpeed(MOTOR2_ENABLE_PIN, motorSpeed2);
  lastMotorCommand = 'L';
}

/**
 * Applica l'ultima velocità conosciuta ai motori.
 * Utile quando la velocità cambia tramite '+' o '-'.
 */
void applyCurrentMotorCommand() {
  switch (lastMotorCommand) {
    case 'F': moveForward(); break;
    case 'B': moveBackward(); break;
    case 'L': turnLeft(); break;
    case 'R': turnRight(); break;
    case 'S': stopAllMotors(); break;
  }
}

// --- FUNZIONE SETUP (viene eseguita una volta all'avvio di Arduino) ---

void setup() {
  // Inizializza la comunicazione seriale con il PC (per il Serial Monitor)
  Serial.begin(9600);
  Serial.println("--- Sistema di Controllo Macchinina Avviato ---");

  // Inizializza la comunicazione seriale con il modulo HC-05
  // *** QUESTO BAUD RATE DEVE CORRISPONDERE ESATTAMENTE AL TUO HC-05! ***
  // *** Il default è 9600. Se hai simboli strani, prova 38400, 57600, o 115200. ***
  bluetoothSerial.begin(9600);
  Serial.println("Comunicazione HC-05 iniziata a 9600 baud. In attesa di connessione Bluetooth...");

  // Configura tutti i pin dei motori come OUTPUT
  pinMode(MOTOR1_IN1_PIN, OUTPUT);
  pinMode(MOTOR1_IN2_PIN, OUTPUT);
  pinMode(MOTOR1_ENABLE_PIN, OUTPUT);
  pinMode(MOTOR2_IN1_PIN, OUTPUT);
  pinMode(MOTOR2_IN2_PIN, OUTPUT);
  pinMode(MOTOR2_ENABLE_PIN, OUTPUT);

  // Assicurati che i motori siano fermi all'avvio
  stopAllMotors();
  Serial.println("Motori inizializzati e fermi. Pronto a ricevere comandi.");
}

// --- FUNZIONE LOOP (viene eseguita ripetutamente all'infinito) ---

void loop() {
  // Controlla se ci sono dati disponibili dal modulo Bluetooth
  if (bluetoothSerial.available()) {
    char receivedCommand = bluetoothSerial.read(); // Legge il carattere ricevuto
    Serial.print("Comando Bluetooth ricevuto: '");
    Serial.print(receivedCommand);
    Serial.println("'");

    // Esegue l'azione corrispondente al comando ricevuto
    switch (receivedCommand) {
      case 'F': // Avanti
        moveForward();
        break;
      case 'B': // Indietro
        moveBackward();
        break;
      case 'L': // Gira a sinistra
        turnLeft();
        break;
      case 'R': // Gira a destra
        turnRight();
        break;
      case 'S': // Stop
        stopAllMotors();
        break;
      case '+': // Aumenta la velocità
        motorSpeed += 30; // Aumenta la velocità di 20 unità
        motorSpeed2 += 30;
        motorSpeed = constrain(motorSpeed, 0, 255); // Limita la velocità tra 0 e 255

        Serial.print("Velocita' attuale: ");
        Serial.println(motorSpeed);
        applyCurrentMotorCommand(); // Riapplica l'ultimo comando con la nuova velocità
        break;
      case '-': // Diminuisci la velocità
        motorSpeed -= 20; // Diminuisci la velocità di 20 unità
        motorSpeed = constrain(motorSpeed, 0, 255); // Limita la velocità tra 0 e 255
        Serial.print("Velocita' attuale: ");
        Serial.println(motorSpeed);
        applyCurrentMotorCommand(); // Riapplica l'ultimo comando con la nuova velocità
        break;
      default:
        Serial.println("Comando Bluetooth sconosciuto o non valido.");
        break;
    }
  }

  // Puoi aggiungere qui qualsiasi altro codice che deve essere eseguito continuamente
  // (es. lettura di altri sensori, ecc., se presenti)
  // delay(10); // Un piccolo ritardo può aiutare la stabilità in alcuni casi
}