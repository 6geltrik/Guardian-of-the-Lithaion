#include <Wire.h>
#include <MPU6050.h>
#include <DHT.h> 

MPU6050 mpu;

// Μεταβλητές για το καλιμπράρισμα (μόνο Z άξονας)
float gyroZ_offset = 0;

// Μεταβλητές για γωνίες (μόνο yaw)
float yaw = 0;
unsigned long lastTime = 0;

// Μεταβλητές για HC-SR04
const int TRIG_PIN = 22;    // Trigger pin για HC-SR04
const int ECHO_PIN = 23;    // Echo pin για HC-SR04
float distance_cm = 0;      // Απόσταση σε εκατοστά

// Μεταβλητές για τα 4 relays
const int RELAY1_PIN = 24;   // Relay 1
const int RELAY2_PIN = 25;   // Relay 2
const int RELAY3_PIN = 26;   // Relay 3
const int RELAY4_PIN = 27;   // Relay 4

// Pin για TDS sensor
#define TdsSensorPin A0     // Χρησιμοποιείται για TDS sensor
#define VREF 5.0            // Τάση αναφοράς για TDS
#define SCOUNT 30           // Αριθμός δειγμάτων για TDS

// Pin για μέτρηση από το voltage sensor (μετακινήθηκε σε A1)
const int VOLTAGE_SENSOR_PIN = A1;  // Νέα θέση για voltage sensor

// Pin για push button (εκκίνηση ακολουθίας)
const int BUTTON_PIN = 30;

// Αντιστοιχία τάσης διαιρέτη για voltage sensor
const float scaleFactor = 5.0;
float voltageIn = 0.0;      // Τάση εισόδου

// Μεταβλητές για TDS sensor
int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;
unsigned long lastTDSUpdate = 0;
const unsigned long TDS_UPDATE_INTERVAL = 800;

// Καταστάσεις των relays
bool relay1_state = false;  // OFF
bool relay2_state = false;  // OFF
bool relay3_state = false;  // OFF
bool relay4_state = false;  // OFF

// Μεταβλητές για το button
int lastButtonState = HIGH;  // Θεωρούμε ότι το button είναι pull-up
int buttonState = HIGH;
bool sequenceRequested = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Παράμετροι για διαδοχικές καταστάσεις relays
const int SEQUENCE_STEPS = 6;      // 6 διαδοχικές καταστάσεις
const int SEQUENCE_REPETITIONS = 2; // Εκτέλεση 2 φορές
int currentRepetition = 1;          // Τρέχουσα επανάληψη (1 ή 2)
int currentSequenceStep = 0;        // Τρέχουσα κατάσταση (0 = αρχική, 1-6 = καταστάσεις)
bool sequenceActive = false;        // Είναι ενεργή η ακολουθία;
bool stepCompleted = false;         // Έχει ολοκληρωθεί η τρέχουσα κατάσταση;

// Παράμετροι για την πρώτη κατάσταση (απόσταση)
const float STEP1_DISTANCE_THRESHOLD = 20.0;  // 20cm όριο

// Παράμετροι για τη δεύτερη κατάσταση (yaw)
const float STEP2_YAW_THRESHOLD = 88.0;  // 88° όριο (απόλυτη τιμή)
const unsigned long STEP2_SPECIAL_DURATION = 10000;  // 10 δευτερόλεπτα για relays 1&2
bool step2SpecialActive = false;    // Είναι ενεργή η ειδική κατάσταση (relays 1&2 ON);
unsigned long step2SpecialStartTime = 0;  // Χρόνος έναρξης ειδικής κατάστασης

// Παράμετροι για την τρίτη κατάσταση
const float STEP3_YAW_THRESHOLD = 178.0;  // 178° όριο (απόλυτη τιμή)
bool step3Active = false;  // Είναι ενεργή η τρίτη κατάσταση;
bool step3YawConditionMet = false;  // Έχει πληρωθεί η συνθήκη yaw < 178°;

// Παράμετροι για την τέταρτη κατάσταση
const float STEP4_YAW_THRESHOLD = 88.0;  // 88° όριο (απόλυτη τιμή)
const unsigned long STEP4_SPECIAL_DURATION = 10000;  // 10 δευτερόλεπτα για relays 1&2
bool step4Active = false;  // Είναι ενεργή η τέταρτη κατάσταση;
bool step4YawConditionMet = false;  // Έχει πληρωθεί η συνθήκη yaw > 88°;
bool step4SpecialActive = false;    // Είναι ενεργή η ειδική κατάσταση (relays 1&2 ON);
unsigned long step4SpecialStartTime = 0;  // Χρόνος έναρξης ειδικής κατάστασης

// Παράμετροι για την πέμπτη κατάσταση
const float STEP5_YAW_THRESHOLD = 2.0;  // 2° όριο (απόλυτη τιμή)
bool step5Active = false;  // Είναι ενεργή η πέμπτη κατάσταση;
bool step5YawConditionMet = false;  // Έχει πληρωθεί η συνθήκη yaw > 2°;

// Παράμετροι για την έκτη κατάσταση
const unsigned long STEP6_DURATION = 5000;  // 5 δευτερόλεπτα
unsigned long step6StartTime = 0;  // Χρόνος έναρξης κατάστασης 6

// Παράμετροι καλιμπραρίσματος
const int CALIBRATION_SAMPLES = 500;

// Χρονική περίοδος ανανέωσης (ms)
const unsigned long UPDATE_INTERVAL = 50;

// Μεταβλητές για αισθητήρα υγρασίας HW-103
int analogPin = A2;  // Αναλογική έξοδος από HW-103 (ΧΡΗΣΙΜΟΠΟΙΕΙΤΑΙ A2 ΕΠΕΙΔΗ ΤΟ A0 ΕΙΝΑΙ ΓΙΑ TDS)
int digitalPin = 2;  // Ψηφιακή έξοδος D0 του HW-103
int humidityAnalogValue = 0;  // Αναλογική τιμή υγρασίας
int humidityDigitalValue = 0; // Ψηφιακή τιμή υγρασίας
unsigned long lastHumidityRead = 0; // Χρόνος τελευταίας μέτρησης υγρασίας
const unsigned long HUMIDITY_UPDATE_INTERVAL = 500; // Ανάγνωση υγρασίας κάθε 500ms

// ===== ΝΕΑ: Μεταβλητές για DHT11 και επικοινωνία με ESP8266 =====
#define DHTPIN 4              // Pin για DHT11
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Μεταβλητές για δεδομένα DHT11
float humidity = 0;
float temperature = 0;
unsigned long lastDHTRead = 0;
const unsigned long DHT_UPDATE_INTERVAL = 2000; // DHT11 κάθε 2 δευτερόλεπτα

// Serial για επικοινωνία με ESP8266 (χρήση Hardware Serial3 για Mega)
// Mega έχει 4 Hardware Serial: Serial, Serial1, Serial2, Serial3
// Χρησιμοποιούμε Serial3 (TX3 = 14, RX3 = 15)
#define ESP_SERIAL Serial3

// Χρονόμετρο για αποστολή δεδομένων στο ESP8266
unsigned long lastESPSend = 0;
const unsigned long ESP_SEND_INTERVAL = 2000; // Αποστολή κάθε 2 δευτερόλεπτα
// =============================================================

void setup() {
  // Αλλαγή baud rate για επικοινωνία με ESP8266 στα 115200
  Serial.begin(115200);      // Για debugging
  ESP_SERIAL.begin(115200);  // Επικοινωνία με ESP8266 στα 9600
  
  Wire.begin();
  
  // Αρχικοποίηση DHT11
  dht.begin();
  
  // Αρχικοποίηση ακροφυσίων HC-SR04
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  // Αρχικοποίηση ακροφυσίων Relays
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  
  // Αρχικοποίηση pins για sensors
  pinMode(TdsSensorPin, INPUT);     // TDS sensor
  pinMode(VOLTAGE_SENSOR_PIN, INPUT); // Voltage sensor
  
  // Αρχικοποίηση pin για button (με pull-up resistor)
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // Αρχικοποίηση pin για αισθητήρα υγρασίας HW-103
  pinMode(digitalPin, INPUT);
  
  // Αρχικοποίηση όλων των relays σε OFF κατάσταση
  turnAllRelaysOff();
  
  Serial.println("Αρχικοποίηση MPU6050...");
  
  // Αρχικοποίηση MPU6050
  mpu.initialize();
  
  // Έλεγχος σύνδεσης
  if (!mpu.testConnection()) {
    Serial.println("Σφάλμα: MPU6050 δεν βρέθηκε!");
    while (1);
  }
  
  Serial.println("Καλιμπράρισμα... ΜΗΝ ΚΙΝΕΙΤΕ τον αισθητήρα!");
  delay(2000);
  
  // Καλιμπράρισμα (μόνο Z άξονας)
  calibrateMPU();
  
  Serial.println("Καλιμπράρισμα ολοκληρώθηκε!");
  Serial.println("==============================");
  lastTime = micros();
  
  // Εμφάνιση οδηγιών
  Serial.println("\nΟδηγίες χειρισμού Relays:");
  Serial.println("-------------------------");
  Serial.println("Εισάγετε μια από τις παρακάτω εντολές:");
  Serial.println("  1-4 - Αλλαγή Relay 1-4");
  Serial.println("  A  - ON όλα τα relays");
  Serial.println("  X  - OFF όλα τα relays");
  Serial.println("  S  - Εμφάνιση κατάστασης relays");
  Serial.println("  ?  - Εμφάνιση οδηγιών");
  Serial.println("  C  - Διακοπή ακολουθίας και reset");
  Serial.println("\nΕκκίνηση ακολουθίας: ΠΙΕΣΤΕ ΤΟ ΚΟΥΜΠΙ (Pin 30)");
  Serial.println("\nΠεριγραφή Ακολουθίας:");
  Serial.println("----------------------");
  Serial.println("Η ακολουθία θα εκτελεστεί 2 ΦΟΡΕΣ ΔΙΑΔΟΧΙΚΑ");
  Serial.println("Κατάσταση 1: Relays 1 & 2 ON όταν απόσταση > 20cm");
  Serial.println("             Μετάβαση σε κατάσταση 2 όταν απόσταση ≤ 20cm");
  Serial.println("Κατάσταση 2: Relay 3 ON όταν |yaw| < 88°, 1,2,4 OFF");
  Serial.println("             Όταν |yaw| ≥ 88°: Relays 1&2 ON για 10s");
  Serial.println("             Μετά τα 10s: Όλα OFF και μετάβαση σε κατάσταση 3");
  Serial.println("Κατάσταση 3: Relay 3 ON όταν |yaw| < 178°, 1,2,4 OFF");
  Serial.println("             Αν |yaw| ≥ 178°: Relays 3,4 OFF και Relays 1,2 ON");
  Serial.println("             όσο απόσταση > 20cm, μετά Όλα OFF και μετάβαση σε κατάσταση 4");
  Serial.println("Κατάσταση 4: Relay 4 ON όταν |yaw| > 88°, 1,2,3 OFF");
  Serial.println("             Αν |yaw| ≤ 88°: Relays 3,4 OFF και Relays 1,2 ON για 10s");
  Serial.println("             Μετά τα 10s: Όλα OFF και μετάβαση σε κατάσταση 5");
  Serial.println("Κατάσταση 5: Relay 4 ON όταν |yaw| > 2°, 1,2,3 OFF");
  Serial.println("             Αν |yaw| ≤ 2°: Relays 3,4 OFF και μετάβαση σε κατάσταση 6");
  Serial.println("Κατάσταση 6: Όλα τα relays OFF για 5 δευτερόλεπτα");
  Serial.println("             Εμφάνιση τάσης, TDS και υγρασίας από sensors");
  Serial.println("\nΕπικοινωνία με ESP8266 στα 115200 baud (Serial3)");
  Serial.println();
}

void loop() {
  static unsigned long lastUpdate = 0;
  static unsigned long lastVoltageRead = 0;
  static unsigned long analogSampleTimepoint = 0;
  unsigned long currentMillis = millis();
  
  // Χρόνος που πέρασε από την τελευταία μέτρηση
  unsigned long currentTime = micros();
  float dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  
  // Υπολογισμός yaw (μόνο Z άξονας)
  int16_t gz_raw = mpu.getRotationZ();
  float gyroZ = (gz_raw / 131.0) - gyroZ_offset;
  yaw = yaw + gyroZ * dt;
  
  // Περιορισμός yaw μεταξύ -180 και 180 μοιρών
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
  
  // Μέτρηση απόστασης με HC-SR04
  measureDistance();
  
  // Μέτρηση τάσης κάθε 500ms
  if (currentMillis - lastVoltageRead >= 500) {
    lastVoltageRead = currentMillis;
    measureVoltage();
  }
  
  // Μέτρηση υγρασίας κάθε 500ms
  if (currentMillis - lastHumidityRead >= HUMIDITY_UPDATE_INTERVAL) {
    lastHumidityRead = currentMillis;
    measureHumidity();
  }
  
  // ===== ΝΕΟ: Μέτρηση DHT11 κάθε 2 δευτερόλεπτα =====
  if (currentMillis - lastDHTRead >= DHT_UPDATE_INTERVAL) {
    lastDHTRead = currentMillis;
    readDHT11();
  }
  // =================================================
  
  // Δειγματοληψία TDS sensor κάθε 40ms
  if (currentMillis - analogSampleTimepoint > 40) {
    analogSampleTimepoint = currentMillis;
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  
  // Υπολογισμός και εμφάνιση TDS κάθε 800ms
  if (currentMillis - lastTDSUpdate >= TDS_UPDATE_INTERVAL) {
    lastTDSUpdate = currentMillis;
    calculateTDS();
  }
  
  // ===== ΝΕΟ: Αποστολή δεδομένων στο ESP8266 κάθε 2 δευτερόλεπτα =====
  if (currentMillis - lastESPSend >= ESP_SEND_INTERVAL) {
    lastESPSend = currentMillis;
    sendDataToESP();
  }
  // =================================================================
  
  // Έλεγχος για πίεση του button
  checkButton();
  
  // Έλεγχος για Serial input (από USB για debugging)
  checkSerialInput();
  
  // Έλεγχος ακολουθίας καταστάσεων
  checkSequence();
  
  // Εμφάνιση αποτελεσμάτων κάθε UPDATE_INTERVAL ms
  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentMillis;
    
    // Γωνία Yaw
    Serial.print("Yaw:");
    printSignedFixed(yaw, 1, 6);
    Serial.print("° | ");
    
    // Απόσταση
    Serial.print("Dist:");
    printFixed(distance_cm, 1, 5);
    Serial.print("cm | ");
    
    // Τάση
    Serial.print("Volt:");
    printFixed(voltageIn, 2, 5);
    Serial.print("V | ");
    
    // TDS
    Serial.print("TDS:");
    Serial.print((int)tdsValue);
    Serial.print("ppm | ");
    
    // Υγρασία HW-103
    Serial.print("Hum:");
    Serial.print(humidityAnalogValue);
    Serial.print(" (");
    Serial.print(humidityDigitalValue == 0 ? "WET" : "DRY");
    Serial.print(") | ");
    
    // ===== ΝΕΟ: Εμφάνιση DHT11 δεδομένων =====
    Serial.print("DHT11 H:");
    printFixed(humidity, 1, 4);
    Serial.print("% T:");
    printFixed(temperature, 1, 4);
    Serial.print("C | ");
    // =========================================
    
    // Καταστάσεις Relays
    Serial.print("Rel:");
    Serial.print(relay1_state ? "1" : "0");
    Serial.print(relay2_state ? "1" : "0");
    Serial.print(relay3_state ? "1" : "0");
    Serial.print(relay4_state ? "1" : "0");
    
    // Εμφάνιση κατάστασης ακολουθίας
    Serial.print(" | Step:");
    if (sequenceActive) {
      Serial.print(currentSequenceStep);
      Serial.print("/6");
      
      // Προσθήκη επανάληψης στην εμφάνιση
      Serial.print(" (");
      Serial.print(currentRepetition);
      Serial.print("/");
      Serial.print(SEQUENCE_REPETITIONS);
      Serial.print(")");
      
      if (currentSequenceStep == 1) {
        Serial.print(" D:");
        Serial.print(distance_cm, 0);
        Serial.print("cm");
      } else if (currentSequenceStep == 2) {
        Serial.print(" Y:");
        Serial.print((int)fabs(yaw));
        Serial.print("°");
        if (step2SpecialActive) {
          Serial.print(" T:");
          Serial.print((currentMillis - step2SpecialStartTime) / 1000);
          Serial.print("s");
        }
      } else if (currentSequenceStep == 3) {
        Serial.print(" Y:");
        Serial.print(fabs(yaw), 1);
        Serial.print("°");
        if (step3Active) {
          if (fabs(yaw) < STEP3_YAW_THRESHOLD) {
            Serial.print("<178°");
          } else {
            Serial.print("≥178°");
          }
        }
      } else if (currentSequenceStep == 4) {
        Serial.print(" Y:");
        Serial.print(fabs(yaw), 1);
        Serial.print("°");
        if (step4Active) {
          if (fabs(yaw) > STEP4_YAW_THRESHOLD) {
            Serial.print(">88°");
          } else {
            Serial.print("≤88°");
          }
          if (step4SpecialActive) {
            Serial.print(" T:");
            Serial.print((currentMillis - step4SpecialStartTime) / 1000);
            Serial.print("s");
          }
        }
      } else if (currentSequenceStep == 5) {
        Serial.print(" Y:");
        Serial.print(fabs(yaw), 1);
        Serial.print("°");
        if (step5Active) {
          if (fabs(yaw) > STEP5_YAW_THRESHOLD) {
            Serial.print(">2°");
          } else {
            Serial.print("≤2°");
          }
        }
      } else if (currentSequenceStep == 6) {
        Serial.print(" Volt:");
        Serial.print(voltageIn, 2);
        Serial.print("V");
        Serial.print(" TDS:");
        Serial.print((int)tdsValue);
        Serial.print("ppm");
        Serial.print(" Hum:");
        Serial.print(humidityAnalogValue);
        Serial.print(" T:");
        Serial.print((currentMillis - step6StartTime) / 1000);
        Serial.print("s");
      }
    } else {
      Serial.print("--");
    }
    
    Serial.println();
  }
  
  delay(1);
}

// ===== ΝΕΕΣ ΣΥΝΑΡΤΗΣΕΙΣ ΓΙΑ DHT11 ΚΑΙ ΕΠΙΚΟΙΝΩΝΙΑ ΜΕ ESP8266 =====

// Συνάρτηση ανάγνωσης DHT11
void readDHT11() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Αποτυχία ανάγνωσης από τον DHT11!");
    humidity = 0;
    temperature = 0;
  }
}

// Συνάρτηση αποστολής δεδομένων στο ESP8266
void sendDataToESP() {
  // Μορφή: απόσταση,υγρασίαDHT,θερμοκρασίαDHT,υγρασίαHW103,τάση,TDS,yaw\n
  // Χρησιμοποιούμε την απόσταση από τον αισθητήρα HC-SR04
  if (distance_cm > 0 && distance_cm < 450) {
    ESP_SERIAL.print((int)distance_cm);  // Απόσταση
    ESP_SERIAL.print(",");
    ESP_SERIAL.print(humidity, 1);        // Υγρασία DHT11
    ESP_SERIAL.print(",");
    ESP_SERIAL.print(temperature, 1);      // Θερμοκρασία DHT11
    ESP_SERIAL.print(",");
    ESP_SERIAL.print(humidityAnalogValue); // Υγρασία HW-103 (αναλογική τιμή)
    ESP_SERIAL.print(",");
    ESP_SERIAL.print(voltageIn, 2);        // Τάση
    ESP_SERIAL.print(",");
    ESP_SERIAL.print((int)tdsValue);       // TDS
    ESP_SERIAL.print(",");
    ESP_SERIAL.println(yaw, 1);             // Γωνία Yaw
    
    // Debug στο Serial monitor
    Serial.print("[ESP] Αποστολή: ");
    Serial.print((int)distance_cm);
    Serial.print("cm, ");
    Serial.print(humidity, 1);
    Serial.print("%, ");
    Serial.print(temperature, 1);
    Serial.print("°C, ");
    Serial.print("HW103:");
    Serial.print(humidityAnalogValue);
    Serial.print(", ");
    Serial.print(voltageIn, 2);
    Serial.print("V, ");
    Serial.print((int)tdsValue);
    Serial.print("ppm, ");
    Serial.print(yaw, 1);
    Serial.println("°");
  }
}

// =================================================================

// Συνάρτηση μέτρησης απόστασης με HC-SR04 (τροποποιημένη για αποφυγή μηδενικών)
void measureDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration > 0) {
    float newDistance = duration * 0.017;
    
    // Έλεγχος αν η μέτρηση είναι εντός αξιόπιστων ορίων (2cm - 400cm)
    if (newDistance >= 2.0 && newDistance <= 400.0) {
      distance_cm = newDistance;
    }
    // Αν η μέτρηση είναι εκτός ορίων, κρατάμε την προηγούμενη τιμή
  }
  // Αν η μέτρηση απέτυχε (duration = 0), κρατάμε την προηγούμενη τιμή
}

// Συνάρτηση μέτρησης τάσης
void measureVoltage() {
  // Διαβάζουμε την τιμή ADC (0-1023)
  int rawValue = analogRead(VOLTAGE_SENSOR_PIN);

  // Μετατροπή σε τάση Arduino (0-5V)
  float voltageADC = (rawValue * 5.0) / 1023.0;

  // Πραγματική τάση εισόδου (με τον διαιρέτη)
  voltageIn = voltageADC * scaleFactor;
}

// Συνάρτηση μέτρησης υγρασίας
void measureHumidity() {
  // Διαβάζουμε την αναλογική τιμή από τον αισθητήρα
  humidityAnalogValue = analogRead(analogPin);
  
  // Διαβάζουμε την ψηφιακή τιμή από τον αισθητήρα
  humidityDigitalValue = digitalRead(digitalPin);
}

// Συνάρτηση υπολογισμού TDS
void calculateTDS() {
  // Αντιγραφή του buffer για επεξεργασία
  for (int i = 0; i < SCOUNT; i++)
    analogBufferTemp[i] = analogBuffer[i];

  // Υπολογισμός μέσης τιμής ADC (χρησιμοποιώντας median filter)
  float averageADC = getMedianNum(analogBufferTemp, SCOUNT);

  // Μετατροπή σε τάση
  averageVoltage = averageADC * (VREF / 1024.0);

  // Υπολογισμός TDS (ppm)
  tdsValue = (133.42 * averageVoltage * averageVoltage * averageVoltage
             - 255.86 * averageVoltage * averageVoltage
             + 857.39 * averageVoltage) * 0.5;
             
  // Περιορισμός σε θετικές τιμές
  if (tdsValue < 0) tdsValue = 0;
}

// Συνάρτηση για median filter
int getMedianNum(int bArray[], int iFilterLen) {
  for (int i = 0; i < iFilterLen - 1; i++) {
    for (int j = i + 1; j < iFilterLen; j++) {
      if (bArray[i] > bArray[j]) {
        int temp = bArray[i];
        bArray[i] = bArray[j];
        bArray[j] = temp;
      }
    }
  }
  return bArray[(iFilterLen - 1) / 2];
}

// Συνάρτηση ελέγχου button
void checkButton() {
  // Διαβάζουμε την κατάσταση του button
  int reading = digitalRead(BUTTON_PIN);
  
  // Αν η κατάσταση έχει αλλάξει (π.χ. λόγω θορύβου ή πραγματικής πίεσης)
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  // Αν έχει περάσει αρκετός χρόνος από την τελευταία αλλαγή
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Αν η κατάσταση έχει αλλάξει
    if (reading != buttonState) {
      buttonState = reading;
      
      // Αν το button είναι πατημένο (LOW για pull-up)
      if (buttonState == LOW) {
        sequenceRequested = true;
        Serial.println("\n[ΚΟΥΜΠΙ] Αιτήθηκε εκκίνηση ακολουθίας!");
      }
    }
  }
  
  // Ελέγχουμε αν έχει αιτηθεί εκκίνηση ακολουθίας
  if (sequenceRequested && !sequenceActive) {
    sequenceRequested = false;
    startSequence();
  }
  
  lastButtonState = reading;
}

// Βοηθητικές συναρτήσεις για relays
void turnAllRelaysOff() {
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
  relay1_state = false;
  relay2_state = false;
  relay3_state = false;
  relay4_state = false;
}

void turnAllRelaysOn() {
  digitalWrite(RELAY1_PIN, HIGH);
  digitalWrite(RELAY2_PIN, HIGH);
  digitalWrite(RELAY3_PIN, HIGH);
  digitalWrite(RELAY4_PIN, HIGH);
  relay1_state = true;
  relay2_state = true;
  relay3_state = true;
  relay4_state = true;
}

void toggleRelay(int relayNumber) {
  switch(relayNumber) {
    case 1:
      relay1_state = !relay1_state;
      digitalWrite(RELAY1_PIN, relay1_state ? HIGH : LOW);
      break;
    case 2:
      relay2_state = !relay2_state;
      digitalWrite(RELAY2_PIN, relay2_state ? HIGH : LOW);
      break;
    case 3:
      relay3_state = !relay3_state;
      digitalWrite(RELAY3_PIN, relay3_state ? HIGH : LOW);
      break;
    case 4:
      relay4_state = !relay4_state;
      digitalWrite(RELAY4_PIN, relay4_state ? HIGH : LOW);
      break;
  }
}

void showRelayStatus() {
  Serial.println("\nΚατάσταση Relays:");
  Serial.print("R1:"); Serial.print(relay1_state ? "ON " : "OFF ");
  Serial.print("R2:"); Serial.print(relay2_state ? "ON " : "OFF ");
  Serial.print("R3:"); Serial.print(relay3_state ? "ON " : "OFF ");
  Serial.print("R4:"); Serial.println(relay4_state ? "ON" : "OFF");
}

// Συνάρτηση ελέγχου Serial input
void checkSerialInput() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    
    switch(command) {
      case '1': toggleRelay(1); break;
      case '2': toggleRelay(2); break;
      case '3': toggleRelay(3); break;
      case '4': toggleRelay(4); break;
      case 'A': case 'a':
        turnAllRelaysOn();
        Serial.println("All ON");
        break;
      case 'X': case 'x':
        turnAllRelaysOff();
        Serial.println("All OFF");
        break;
      case 'S': case 's':
        showRelayStatus();
        break;
      case 'C': case 'c':
        stopSequence();
        break;
      case '?':
        Serial.println("\n1-4:Relay, A:All ON, X:All OFF");
        Serial.println("S:Status, C:Stop (ακολουθίας)");
        Serial.println("Εκκίνηση ακολουθίας: ΠΙΕΣΤΕ ΤΟ ΚΟΥΜΠΙ στο Pin 30");
        break;
      case 'B': case 'b':  // Προσθήκη πάλι για εναλλακτική μέθοδο
        Serial.println("\nΧρήση button για εκκίνηση ακολουθίας!");
        Serial.println("Πατήστε το button στο pin 30");
        break;
    }
    
    while(Serial.available() > 0) {
      Serial.read();
    }
  }
}

// Συναρτήσεις για διαδοχικές καταστάσεις
void startSequence() {
  if (!sequenceActive) {
    sequenceActive = true;
    currentSequenceStep = 1;
    currentRepetition = 1; // Αρχικοποίηση πρώτης επανάληψης
    stepCompleted = false;
    step2SpecialActive = false;
    step3Active = false;
    step3YawConditionMet = false;
    step4Active = false;
    step4YawConditionMet = false;
    step4SpecialActive = false;
    step5Active = false;
    step5YawConditionMet = false;
    
    Serial.println("\n=== ΕΚΚΙΝΗΣΗ ΑΚΟΛΟΥΘΙΑΣ ===");
    Serial.print("Επανάληψη ");
    Serial.print(currentRepetition);
    Serial.print(" από ");
    Serial.println(SEQUENCE_REPETITIONS);
    applySequenceStep(1);
  } else {
    Serial.println("Ακολουθία είναι ήδη ενεργή! Πατήστε 'C' για διακοπή.");
  }
}

void stopSequence() {
  if (sequenceActive) {
    sequenceActive = false;
    currentSequenceStep = 0;
    currentRepetition = 1; // Reset επανάληψης
    step2SpecialActive = false;
    step3Active = false;
    step3YawConditionMet = false;
    step4Active = false;
    step4YawConditionMet = false;
    step4SpecialActive = false;
    step5Active = false;
    step5YawConditionMet = false;
    turnAllRelaysOff();
    Serial.println("\n=== ΔΙΑΚΟΠΗ ΑΚΟΛΟΥΘΙΑΣ ===");
  } else {
    Serial.println("Δεν υπάρχει ενεργή ακολουθία!");
  }
}

void checkSequence() {
  static unsigned long stepStartTime = 0;
  
  if (sequenceActive && !stepCompleted) {
    switch(currentSequenceStep) {
      case 1:
        // ΚΑΤΑΣΤΑΣΗ 1: Ενεργοποίηση με βάση απόσταση (>20cm)
        if (distance_cm > 0) {
          if (distance_cm > STEP1_DISTANCE_THRESHOLD) {
            if (!relay1_state || !relay2_state) {
              digitalWrite(RELAY1_PIN, HIGH);
              digitalWrite(RELAY2_PIN, HIGH);
              relay1_state = true;
              relay2_state = true;
              digitalWrite(RELAY3_PIN, LOW);
              digitalWrite(RELAY4_PIN, LOW);
              relay3_state = false;
              relay4_state = false;
            }
          } else {
            stepCompleted = true;
            turnAllRelaysOff();
            moveToNextStep();
          }
        }
        break;
        
      case 2:
        // ΚΑΤΑΣΤΑΣΗ 2: Ενεργοποίηση με βάση yaw (<88°)
        if (!step2SpecialActive) {
          float absYaw = fabs(yaw);
          
          if (absYaw < STEP2_YAW_THRESHOLD) {  // <88°
            // |yaw| < 88°: Relay 3 ON, τα υπόλοιπα OFF
            if (!relay3_state || relay1_state || relay2_state || relay4_state) {
              digitalWrite(RELAY1_PIN, LOW);
              digitalWrite(RELAY2_PIN, LOW);
              digitalWrite(RELAY3_PIN, HIGH);
              digitalWrite(RELAY4_PIN, LOW);
              relay1_state = false;
              relay2_state = false;
              relay3_state = true;
              relay4_state = false;
            }
          } else {
            // |yaw| ≥ 88°: Ενεργοποίηση ειδικής κατάστασης
            step2SpecialActive = true;
            step2SpecialStartTime = millis();
            
            // Απενεργοποίηση relays 3 & 4, ενεργοποίηση relays 1 & 2
            digitalWrite(RELAY1_PIN, HIGH);
            digitalWrite(RELAY2_PIN, HIGH);
            digitalWrite(RELAY3_PIN, LOW);
            digitalWrite(RELAY4_PIN, LOW);
            relay1_state = true;
            relay2_state = true;
            relay3_state = false;
            relay4_state = false;
          }
        } else {
          // Ειδική κατάσταση ενεργή: relays 1&2 ON για 10 δευτερόλεπτα
          if (millis() - step2SpecialStartTime >= STEP2_SPECIAL_DURATION) {
            // Πέρασαν 10 δευτερόλεπτα
            stepCompleted = true;
            turnAllRelaysOff();
            moveToNextStep();
          }
        }
        break;
        
      case 3:
        // ΚΑΤΑΣΤΑΣΗ 3: Ενεργοποίηση με βάση yaw (<178°)
        if (!step3Active) {
          step3Active = true;
          stepStartTime = millis();
          step3YawConditionMet = false;
        }
        
        {
          float absYaw = fabs(yaw);
          
          if (absYaw < STEP3_YAW_THRESHOLD) {  // <178°
            // |yaw| < 178°: Relay 3 ON, τα υπόλοιπα OFF
            if (!step3YawConditionMet) {
              step3YawConditionMet = true;
            }
            
            if (!relay3_state || relay1_state || relay2_state || relay4_state) {
              digitalWrite(RELAY1_PIN, LOW);
              digitalWrite(RELAY2_PIN, LOW);
              digitalWrite(RELAY3_PIN, HIGH);
              digitalWrite(RELAY4_PIN, LOW);
              relay1_state = false;
              relay2_state = false;
              relay3_state = true;
              relay4_state = false;
            }
          } else {
            // |yaw| ≥ 178°: Relays 3,4 OFF και Relays 1,2 ON όσο απόσταση > 20cm
            stepCompleted = true;
            
            // Απενεργοποίηση relays 3 & 4
            digitalWrite(RELAY3_PIN, LOW);
            digitalWrite(RELAY4_PIN, LOW);
            relay3_state = false;
            relay4_state = false;
            
            // Έλεγχος απόστασης για ενεργοποίηση relays 1 & 2
            if (distance_cm > 0 && distance_cm > STEP1_DISTANCE_THRESHOLD) {
              digitalWrite(RELAY1_PIN, HIGH);
              digitalWrite(RELAY2_PIN, HIGH);
              relay1_state = true;
              relay2_state = true;
              
              // Περιμένουμε μέχρι η απόσταση να γίνει ≤ 20cm
              while (distance_cm > 0 && distance_cm > STEP1_DISTANCE_THRESHOLD) {
                // Ανανέωση μέτρησης απόστασης
                measureDistance();
                delay(10);
              }
            }
            
            // Απενεργοποίηση όλων των relays
            turnAllRelaysOff();
            moveToNextStep();
          }
        }
        break;
        
      case 4:
        // ΚΑΤΑΣΤΑΣΗ 4: Ενεργοποίηση με βάση yaw (>88°)
        if (!step4Active) {
          step4Active = true;
          stepStartTime = millis();
          step4YawConditionMet = false;
          step4SpecialActive = false;
        }
        
        {
          float absYaw = fabs(yaw);
          
          if (!step4SpecialActive) {
            if (absYaw > STEP4_YAW_THRESHOLD) {  // > 88°
              // |yaw| > 88°: Relay 4 ON, τα υπόλοιπα OFF
              if (!step4YawConditionMet) {
                step4YawConditionMet = true;
              }
              
              if (!relay4_state || relay1_state || relay2_state || relay3_state) {
                digitalWrite(RELAY1_PIN, LOW);
                digitalWrite(RELAY2_PIN, LOW);
                digitalWrite(RELAY3_PIN, LOW);
                digitalWrite(RELAY4_PIN, HIGH);
                relay1_state = false;
                relay2_state = false;
                relay3_state = false;
                relay4_state = true;
              }
            } else {
              // |yaw| ≤ 88°: Ενεργοποίηση ειδικής κατάστασης
              step4SpecialActive = true;
              step4SpecialStartTime = millis();
              
              // Απενεργοποίηση relays 3 & 4, ενεργοποίηση relays 1 & 2
              digitalWrite(RELAY1_PIN, HIGH);
              digitalWrite(RELAY2_PIN, HIGH);
              digitalWrite(RELAY3_PIN, LOW);
              digitalWrite(RELAY4_PIN, LOW);
              relay1_state = true;
              relay2_state = true;
              relay3_state = false;
              relay4_state = false;
            }
          } else {
            // Ειδική κατάσταση ενεργή: relays 1&2 ON για 10 δευτερόλεπτα
            if (millis() - step4SpecialStartTime >= STEP4_SPECIAL_DURATION) {
              // Πέρασαν 10 δευτερόλεπτα
              stepCompleted = true;
              turnAllRelaysOff();
              moveToNextStep();
            }
          }
        }
        break;
        
      case 5:
        // ΚΑΤΑΣΤΑΣΗ 5: Ενεργοποίηση με βάση yaw (>2°)
        if (!step5Active) {
          step5Active = true;
          stepStartTime = millis();
          step5YawConditionMet = false;
        }
        
        {
          float absYaw = fabs(yaw);
          
          if (absYaw > STEP5_YAW_THRESHOLD) {  // > 2°
            // |yaw| > 2°: Relay 4 ON, τα υπόλοιπα OFF
            if (!step5YawConditionMet) {
              step5YawConditionMet = true;
            }
            
            if (!relay4_state || relay1_state || relay2_state || relay3_state) {
              digitalWrite(RELAY1_PIN, LOW);
              digitalWrite(RELAY2_PIN, LOW);
              digitalWrite(RELAY3_PIN, LOW);
              digitalWrite(RELAY4_PIN, HIGH);
              relay1_state = false;
              relay2_state = false;
              relay3_state = false;
              relay4_state = true;
            }
          } else {
            // |yaw| ≤ 2°: ΜΟΝΟ Relays 3,4 OFF (ΧΩΡΙΣ relays 1 & 2) και μετάβαση σε κατάσταση 6
            stepCompleted = true;
            
            // Απενεργοποίηση ΜΟΝΟ relays 3 & 4
            digitalWrite(RELAY3_PIN, LOW);
            digitalWrite(RELAY4_PIN, LOW);
            relay3_state = false;
            relay4_state = false;
            
            // Τα relays 1 & 2 παραμένουν OFF
            digitalWrite(RELAY1_PIN, LOW);
            digitalWrite(RELAY2_PIN, LOW);
            relay1_state = false;
            relay2_state = false;
            
            // Μετάβαση στην επόμενη κατάσταση
            moveToNextStep();
          }
        }
        break;
        
      case 6:
        // ΚΑΤΑΣΤΑΣΗ 6: Όλα τα relays OFF για 5 δευτερόλεπτα και εμφάνιση τάσης, TDS & υγρασίας
        if (step6StartTime == 0) {
          step6StartTime = millis();
          turnAllRelaysOff();
          
          // Εμφάνιση τάσης, TDS και υγρασίας στην έναρξη της κατάστασης 6
          Serial.print("\n[ΚΑΤΑΣΤΑΣΗ 6] Τάση: ");
          Serial.print(voltageIn, 2);
          Serial.print("V | TDS: ");
          Serial.print((int)tdsValue);
          Serial.print(" ppm | Υγρασία: ");
          Serial.print(humidityAnalogValue);
          Serial.print(" (");
          Serial.print(humidityDigitalValue == 0 ? "ΒΡΟΧΗ/ΥΓΡΟ" : "ΞΗΡΟ");
          Serial.println(")");
        }
        
        if (millis() - step6StartTime >= STEP6_DURATION) {
          stepCompleted = true;
          step6StartTime = 0; // Reset για την επόμενη επανάληψη
          moveToNextStep();
        }
        break;
    }
  }
}

void moveToNextStep() {
  if (currentSequenceStep < SEQUENCE_STEPS) {
    currentSequenceStep++;
    stepCompleted = false;
    step2SpecialActive = false;
    step3Active = false;
    step3YawConditionMet = false;
    step4Active = false;
    step4YawConditionMet = false;
    step4SpecialActive = false;
    step5Active = false;
    step5YawConditionMet = false;
    
    applySequenceStep(currentSequenceStep);
  } else {
    // Έλεγχος για επόμενη επανάληψη
    if (currentRepetition < SEQUENCE_REPETITIONS) {
      currentRepetition++;
      Serial.println("\n==================================");
      Serial.print("ΕΠΑΝΑΛΗΨΗ ");
      Serial.print(currentRepetition);
      Serial.print(" ΑΠΟ ");
      Serial.println(SEQUENCE_REPETITIONS);
      Serial.println("==================================");
      
      currentSequenceStep = 1;
      stepCompleted = false;
      step2SpecialActive = false;
      step3Active = false;
      step3YawConditionMet = false;
      step4Active = false;
      step4YawConditionMet = false;
      step4SpecialActive = false;
      step5Active = false;
      step5YawConditionMet = false;
      
      applySequenceStep(currentSequenceStep);
    } else {
      Serial.println("\n=== ΟΛΟΚΛΗΡΩΣΗ ΑΚΟΛΟΥΘΙΑΣ ===");
      Serial.print("Ολοκληρώθηκαν ");
      Serial.print(SEQUENCE_REPETITIONS);
      Serial.println(" επαναλήψεις της ακολουθίας!");
      
      sequenceActive = false;
      currentSequenceStep = 0;
      currentRepetition = 1; // Reset για την επόμενη εκτέλεση
      step2SpecialActive = false;
      step3Active = false;
      step3YawConditionMet = false;
      step4Active = false;
      step4YawConditionMet = false;
      step4SpecialActive = false;
      step5Active = false;
      step5YawConditionMet = false;
      
      turnAllRelaysOff();
    }
  }
}

void applySequenceStep(int step) {
  turnAllRelaysOff();
  
  switch(step) {
    case 1:
      Serial.print("\n[ΕΠΑΝΑΛΗΨΗ ");
      Serial.print(currentRepetition);
      Serial.print("/");
      Serial.print(SEQUENCE_REPETITIONS);
      Serial.println("] Κατάσταση 1: Ενεργοποίηση με βάση απόσταση");
      Serial.println("  - Relays 1 & 2 ON όταν απόσταση > 20cm");
      Serial.println("  - Μετάβαση σε κατάσταση 2 όταν απόσταση ≤ 20cm");
      break;
      
    case 2:
      Serial.print("\n[ΕΠΑΝΑΛΗΨΗ ");
      Serial.print(currentRepetition);
      Serial.print("/");
      Serial.print(SEQUENCE_REPETITIONS);
      Serial.println("] Κατάσταση 2: Ενεργοποίηση με βάση γωνία Yaw");
      Serial.println("  - Relay 3 ON όταν |yaw| < 88°");
      Serial.println("  - Όταν |yaw| ≥ 88°: Relays 1&2 ON για 10s");
      Serial.println("  - Μετά τα 10s: Όλα OFF και μετάβαση σε κατάσταση 3");
      break;
      
    case 3:
      Serial.print("\n[ΕΠΑΝΑΛΗΨΗ ");
      Serial.print(currentRepetition);
      Serial.print("/");
      Serial.print(SEQUENCE_REPETITIONS);
      Serial.println("] Κατάσταση 3: Ενεργοποίηση με βάση γωνία Yaw");
      Serial.println("  - Relay 3 ON όταν |yaw| < 178°");
      Serial.println("  - Αν |yaw| ≥ 178°: Relays 1,2 ON όσο απόσταση > 20cm");
      Serial.println("  - Μετά: Όλα OFF και μετάβαση σε κατάσταση 4");
      break;
      
    case 4:
      Serial.print("\n[ΕΠΑΝΑΛΗΨΗ ");
      Serial.print(currentRepetition);
      Serial.print("/");
      Serial.print(SEQUENCE_REPETITIONS);
      Serial.println("] Κατάσταση 4: Ενεργοποίηση με βάση γωνία Yaw");
      Serial.println("  - Relay 4 ON όταν |yaw| > 88°");
      Serial.println("  - Αν |yaw| ≤ 88°: Relays 1,2 ON για 10s");
      Serial.println("  - Μετά τα 10s: Όλα OFF και μετάβαση σε κατάσταση 5");
      break;
      
    case 5:
      Serial.print("\n[ΕΠΑΝΑΛΗΨΗ ");
      Serial.print(currentRepetition);
      Serial.print("/");
      Serial.print(SEQUENCE_REPETITIONS);
      Serial.println("] Κατάσταση 5: Ενεργοποίηση με βάση γωνία Yaw");
      Serial.println("  - Relay 4 ON όταν |yaw| > 2°");
      Serial.println("  - Αν |yaw| ≤ 2°: Relays 3,4 OFF και μετάβαση σε κατάσταση 6");
      break;
      
    case 6:
      Serial.print("\n[ΕΠΑΝΑΛΗΨΗ ");
      Serial.print(currentRepetition);
      Serial.print("/");
      Serial.print(SEQUENCE_REPETITIONS);
      Serial.println("] Κατάσταση 6: Όλα τα relays OFF");
      Serial.println("Διάρκεια: 5 δευτερόλεπτα");
      Serial.println("Εμφάνιση τάσης, TDS και υγρασίας από sensors");
      break;
  }
}

// Βοηθητικές συναρτήσεις για μορφοποίηση
void printSignedFixed(float value, int decimals, int totalWidth) {
  char buffer[12];
  dtostrf(value, totalWidth, decimals, buffer);
  Serial.print(buffer);
}

void printFixed(float value, int decimals, int totalWidth) {
  char buffer[12];
  dtostrf(value, totalWidth, decimals, buffer);
  Serial.print(buffer);
}

void calibrateMPU() {
  float gyroZ_sum = 0;
  
  Serial.print("Calibrating");
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t gz = mpu.getRotationZ();
    gyroZ_sum += gz / 131.0;
    
    delay(2);
    
    if (i % 50 == 0) {
      Serial.print(".");
    }
  }
  Serial.println();
  
  gyroZ_offset = gyroZ_sum / CALIBRATION_SAMPLES;
  Serial.print("Offset Z: ");
  Serial.println(gyroZ_offset, 4);
}