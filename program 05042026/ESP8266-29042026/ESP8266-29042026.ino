#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <SoftwareSerial.h>  // Χρησιμοποιούμε SoftwareSerial για καλύτερο έλεγχο

const char* ssid = "AndroidAP_8830";
const char* password = "8054fe61d330";

// Χρήση SoftwareSerial για επικοινωνία με Arduino Mega
// Επέλεξε pins που δεν χρησιμοποιούνται για άλλο σκοπό
// D1 (GPIO5) ως RX, D2 (GPIO4) ως TX
#define SOFT_RX D1  // GPIO5 - Συνδέεται στο TX του Arduino (Pin 14)
#define SOFT_TX D2  // GPIO4 - Συνδέεται στο RX του Arduino (Pin 15)

SoftwareSerial arduinoSerial(SOFT_RX, SOFT_TX);

ESP8266WebServer server(80);

// Μεταβλητές για αποθήκευση των δεδομένων από Arduino
String dist = "0";      // Απόσταση
String hum = "0";       // Υγρασία DHT11
String temp = "0";      // Θερμοκρασία DHT11
String humHW = "0";     // Υγρασία HW-103 (αναλογική τιμή)
String voltage = "0";   // Τάση
String tds = "0";       // TDS
String yaw = "0";       // Γωνία Yaw

unsigned long lastDataTime = 0;
const unsigned long DATA_TIMEOUT = 5000; // 5 δευτερόλεπτα timeout
unsigned long lastPrintTime = 0;

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta http-equiv='refresh' content='2'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body{font-family:sans-serif; text-align:center; background:#f4f4f4; margin:0; padding:20px;}";
  html += "h1{color:#333; margin-bottom:30px;}";
  html += ".container{display:flex; flex-wrap:wrap; justify-content:center; gap:20px;}";
  html += ".card{background:white; display:inline-block; padding:20px; margin:0; border-radius:15px; ";
  html += "box-shadow:0 4px 8px rgba(0,0,0,0.1); width:200px; transition:transform 0.3s;}";
  html += ".card:hover{transform:scale(1.05);}";
  html += "h2{font-size:18px; color:#555; margin-top:0;}";
  html += ".val{font-size:32px; font-weight:bold; margin:10px 0;}";
  html += ".unit{font-size:14px; color:#777;}";
  html += ".dist{color:#007bff;}";
  html += ".hum{color:#28a745;}";
  html += ".temp{color:#e74c3c;}";
  html += ".humhw{color:#17a2b8;}";
  html += ".volt{color:#fd7e14;}";
  html += ".tds{color:#6f42c1;}";
  html += ".yaw{color:#dc3545;}";
  html += ".status{background:#e9ecef; border-radius:10px; padding:10px; margin-top:20px; font-size:14px;}";
  html += ".warning{color:#856404; background:#fff3cd; padding:5px; border-radius:5px;}";
  html += "</style></head><body>";
  
  html += "<h1>📊 Σταθμός Μετρήσεων</h1>";
  
  // Έλεγχος αν τα δεδομένα είναι ενημερωμένα
  if (millis() - lastDataTime < DATA_TIMEOUT) {
    html += "<div class='container'>";
    
    // Απόσταση
    html += "<div class='card'><h2>📏 Απόσταση</h2>";
    html += "<div class='val dist'>" + dist + "</div>";
    html += "<span class='unit'>εκατοστά</span></div>";
    
    // Υγρασία DHT11
    html += "<div class='card'><h2>💧 Υγρασία (DHT11)</h2>";
    html += "<div class='val hum'>" + hum + "</div>";
    html += "<span class='unit'>%</span></div>";
    
    // Θερμοκρασία DHT11
    html += "<div class='card'><h2>🌡️ Θερμοκρασία</h2>";
    html += "<div class='val temp'>" + temp + "</div>";
    html += "<span class='unit'>°C</span></div>";
    
    // Υγρασία HW-103
    html += "<div class='card'><h2>💧 Υγρασία (HW-103)</h2>";
    html += "<div class='val humhw'>" + humHW + "</div>";
    html += "<span class='unit'>αναλογική τιμή</span></div>";
    
    // Τάση
    html += "<div class='card'><h2>⚡ Τάση</h2>";
    html += "<div class='val volt'>" + voltage + "</div>";
    html += "<span class='unit'>V</span></div>";
    
    // TDS
    html += "<div class='card'><h2>🧪 TDS</h2>";
    html += "<div class='val tds'>" + tds + "</div>";
    html += "<span class='unit'>ppm</span></div>";
    
    // Γωνία Yaw
    html += "<div class='card'><h2>🔄 Γωνία Yaw</h2>";
    html += "<div class='val yaw'>" + yaw + "</div>";
    html += "<span class='unit'>μοίρες</span></div>";
    
    html += "</div>";
    
    html += "<div class='status'>";
    html += "🕒 Τελευταία ενημέρωση: " + String((millis() - lastDataTime) / 1000) + " δευτ. πριν";
    html += "</div>";
  } else {
    html += "<div class='warning'>⚠️ Δεν λαμβάνονται δεδομένα από το Arduino Mega!</div>";
    html += "<div class='status'>Τελευταία λήψη: " + String((millis() - lastDataTime) / 1000) + " δευτ. πριν</div>";
  }
  
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  // Endpoint για AJAX requests
  String json = "{";
  json += "\"dist\":\"" + dist + "\",";
  json += "\"hum\":\"" + hum + "\",";
  json += "\"temp\":\"" + temp + "\",";
  json += "\"humHW\":\"" + humHW + "\",";
  json += "\"voltage\":\"" + voltage + "\",";
  json += "\"tds\":\"" + tds + "\",";
  json += "\"yaw\":\"" + yaw + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void setup() {
  // Serial για debugging (USB)
  Serial.begin(115200);
  delay(100);
  
  Serial.println();
  Serial.println("ESP8266 - Σταθμός Μετρήσεων");
  Serial.println("Σύνδεση στο WiFi...");
  
  // Serial για επικοινωνία με Arduino Mega στα 9600 (πιο σταθερό για SoftwareSerial)
  arduinoSerial.begin(115200);  // Χαμηλότερο baud rate για SoftwareSerial
  
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) { 
    delay(500); 
    Serial.print("."); 
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("✅ Συνδέθηκε στο WiFi!");
    Serial.print("🌐 IP: ");
    Serial.println(WiFi.localIP().toString());
    
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.begin();
    
    Serial.println("🌍 Web server started!");
  } else {
    Serial.println();
    Serial.println("❌ Αποτυχία σύνδεσης στο WiFi!");
    Serial.println("Το ESP θα λειτουργήσει σε λειτουργία AP");
    // Μπορείς να προσθέσεις κώδικα για Access Point αν θέλεις
  }
  
  Serial.println("📡 Αναμονή για δεδομένα από Arduino Mega...");
  Serial.println("Συνδεσμολογία: ESP8266 RX(D1) -> Arduino TX3(Pin14)");
  Serial.println("              ESP8266 TX(D2) -> Arduino RX3(Pin15)");
  Serial.println("              GND -> GND (ΚΟΙΝΗ ΓΕΙΩΣΗ)");
  Serial.println("              ΧΡΗΣΗ ΔΙΑΙΡΕΤΗ ΤΑΣΗΣ 5V->3.3V!");
}

void loop() {
  // Έλεγχος για δεδομένα από Arduino Mega
  if (arduinoSerial.available() > 0) {
    String data = arduinoSerial.readStringUntil('\n');
    data.trim();
    
    if (data.length() > 0) {
      // Μορφή: απόσταση,υγρασίαDHT,θερμοκρασίαDHT,υγρασίαHW103,τάση,TDS,yaw
      int comma1 = data.indexOf(',');
      int comma2 = data.indexOf(',', comma1 + 1);
      int comma3 = data.indexOf(',', comma2 + 1);
      int comma4 = data.indexOf(',', comma3 + 1);
      int comma5 = data.indexOf(',', comma4 + 1);
      int comma6 = data.indexOf(',', comma5 + 1);
      
      if (comma1 > 0 && comma2 > comma1 && comma3 > comma2 && 
          comma4 > comma3 && comma5 > comma4 && comma6 > comma5) {
        
        dist = data.substring(0, comma1);
        hum = data.substring(comma1 + 1, comma2);
        temp = data.substring(comma2 + 1, comma3);
        humHW = data.substring(comma3 + 1, comma4);
        voltage = data.substring(comma4 + 1, comma5);
        tds = data.substring(comma5 + 1, comma6);
        yaw = data.substring(comma6 + 1);
        
        lastDataTime = millis();
        
        // Debug - εκτύπωση κάθε φορά που λαμβάνουμε δεδομένα
        Serial.print("📥 Λήψη: ");
        Serial.print(dist); Serial.print("cm, ");
        Serial.print(hum); Serial.print("%, ");
        Serial.print(temp); Serial.print("°C, ");
        Serial.print("HW:"); Serial.print(humHW); Serial.print(", ");
        Serial.print(voltage); Serial.print("V, ");
        Serial.print(tds); Serial.print("ppm, ");
        Serial.print(yaw); Serial.println("°");
      } else {
        Serial.print("⚠️ Λανθασμένη μορφή δεδομένων: ");
        Serial.println(data);
      }
    }
  }
  
  // Εκτύπωση κατάστασης κάθε 10 δευτερόλεπτα αν δεν λαμβάνουμε δεδομένα
  if (millis() - lastPrintTime > 10000) {
    lastPrintTime = millis();
    if (millis() - lastDataTime > 5000) {
      Serial.println("⏳ Αναμονή για δεδομένα... (έλεγξε συνδέσεις και τροφοδοσία)");
    }
  }
  
  server.handleClient();
}