/*
 * COULD DO
 * programm für computer schreiben
 * raceconditions, stack overflows, jedes mal int neu erstellen oder einmal, möglichkeit cpu auslastung zu sehen? verdammt viele package misses
 * warm up (jedes mal, wenn man anfängt packete zu senden 5)
 * evtl nochmal anfragen von packeten?
 * chatgtp sagt udp stack problem?
 * irgendwo ist ein array out of bounds und deswegen 
 * Guru Meditation Error: Core  1 panic'ed (StoreProhibited). Exception was unhandled.
 * Guru Meditation Error: Core  0 panic'ed (LoadProhibited). Exception was unhandled.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

#define BUZZER 12
#define BUTTON 14
#define LED 13
#define TX_PIN 17
#define RX_PIN 16
#define NORMAL_MODE_PIN 27
#define NO_BUZZER_MODE_PIN 26
#define NO_PRINTER_MODE_PIN 25
#define SELF_CHECK_MODE_PIN 33
#define SERVER_CHECK_MODE_PIN 32
#define OPT_MODE_PIN 35

//bool BUTTON_PRESSED = false;  // muss öfter gecheckt werdem, darum lieber im sample send task
bool NO_BUZZER_MODE = false;
bool NO_PRINTER_MODE = false;
bool SELF_CHECK_MODE = false;
bool SERVER_CHECK_MODE = false;

// WLAN-Zugangsdaten
//const char *ssid = "NichtDeins";
//const char *password = "oxygen2025";
const char *ssid = "WGlan";
const char *password = "51565735623896715310";
const char *ssid2 = "Leibniz' Hotspot";
const char *password2 = "";

// UDP-Konfiguration
//const char *udpAddress = "192.168.178.186";   // IP des Servers
const char *udpAddress = "morse.hopto.org";   // IP des Servers
const int udpPort = 6969;   // Port zum Senden und Empfangen

// Buffer-Konfiguration
const int BUFFER_SIZE = 32;   // Anzahl Pakete im Jitter-Buffer/*
const int PACKET_SIZE = 8;   // Max Größe eines Pakets (PLAYBACK_INTERVAL / SAMPLE_RATE)

struct __attribute__((packed)) UdpPacket {  // __attribute__((packed)) sorgt dafür, dass es genauso im speiche liegt, ohne lücken etc. wichtig bei der packet aufnahme vom wifi modul
  uint8_t session;
  uint8_t seq;
  uint8_t data;
};

// Ringpuffer für den Ton
uint8_t jitterBuffer[BUFFER_SIZE];
int bufferedPackets = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

QueueHandle_t printQueue;     // eine Kommunikationsschnittstelle zwischen dem receiveTask und dem printTask
QueueHandle_t udpSendQueue;   // eine Kommunikationsschnittstelle zwischen dem receiveTask und dem udptask
QueueHandle_t udpRecvQueue;   // eine Kommunikationsschnittstelle zwischen dem sendTask und dem udpTask

// Intervall zur Ausgabe und Aufnahme
//const int PLAYBACK_INTERVAL = 40; // in ms
const int SAMPLE_RATE = 10;  // in ms
const int TURN_OFF_MAX = (BUFFER_SIZE / 2) * PACKET_SIZE; // turns off transmission after x empty samplings, halbe bufferlänge, damit keine töne im buffer stecken bleiben
const int PING_INTERVAL = 100; // sends ping after x not sent packages NOT SAMPLINGS

const int PRINT_TIMEOUT = 5000; // wenn etwas empfangen wurde, was keine ganze zeile ausfüllt, soll es ausgedruckt werden

const int BUZZER_FREQ = 200;

WiFiUDP udp;
HardwareSerial printer(1);

UBaseType_t updHighwater;
UBaseType_t pinHighwater;
UBaseType_t receiveHighwater;
UBaseType_t sampleSendHighwater;
UBaseType_t playBackHighwater;
UBaseType_t WiFiMonitorHighwater;
UBaseType_t printHighwater;

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(NORMAL_MODE_PIN, INPUT);
  pinMode(NO_BUZZER_MODE_PIN, INPUT);
  pinMode(NO_PRINTER_MODE_PIN, INPUT);
  pinMode(SELF_CHECK_MODE_PIN, INPUT);
  pinMode(SERVER_CHECK_MODE_PIN, INPUT);
  pinMode(OPT_MODE_PIN, INPUT);
  
  printer.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(115200);

  vTaskDelay(1000 / portTICK_PERIOD_MS); //wartezeit für serielle dings, aber wlan dauert schon lange genug
  
  if (digitalRead(NO_BUZZER_MODE_PIN == LOW))
    NO_BUZZER_MODE = true;
  checkWiFi();

  printer.write(27); printer.write('@'); // ESC @ means reset
  printer.write(27); printer.write(55); // ESC 7 configure heating parameters
  printer.write(1);   // n1 = 5   → heizpunkte, die gleichzeitig laufen dürfen
  printer.write(255); // n2 = 240 → lange Heizzeit (langsam, aber dunkler)
  printer.write(255); // n3 = 200 → große Pause zwischen Heizungen, Strom sinkt stark
  // maximaler energiesparmodus, um auch mit 500mA USB auszukommen

  if (digitalRead(NO_PRINTER_MODE_PIN) != LOW){  // nur wenn der drehschalter nicht auf NO_PRINTING steht
    //printer.println("(debug) esp booted");
  }

  printQueue = xQueueCreate(96, sizeof(char)); // 96 sind zwei zeilen des druckers
  udpSendQueue = xQueueCreate(32, sizeof(UdpPacket));
  udpRecvQueue = xQueueCreate(32, sizeof(UdpPacket));

  // Tasks erstellen
  xTaskCreate(receiveTask, "Receive Task", 2048, NULL, 2, NULL);
  xTaskCreate(sampleSendTask, "Sample Task", 2048, NULL, 2, NULL);
  xTaskCreate(playbackBufferTask, "Play Task", 2048, NULL, 2, NULL);
  xTaskCreate(printTask, "Print Task", 4096, NULL, 1, NULL);
  xTaskCreate(udpWorkerTask, "UDP Worker Task", 4096, NULL, 3, NULL);
  xTaskCreate(pinReadTask, "Pin Read Task", 2048, NULL, 1, NULL);
  xTaskCreate(WiFiMonitorTask, "WiFi Monitor Task", 4096, NULL, 1, NULL);

  vTaskDelete(NULL);  // Beendet den Arduino-Loop-Task
}

void loop() {}

void playTone() {
  if (!NO_BUZZER_MODE) // nur wenn der drehschalter nicht "ohne Ton" sagt (LOW = angeschaltet)
     tone(BUZZER, BUZZER_FREQ);   // tone() blockiert auf dem esp32 den thread NICHT
}

void beepOnce() {
  playTone();
  digitalWrite(LED, HIGH);
  vTaskDelay(150 / portTICK_PERIOD_MS);
  noTone(BUZZER);
  digitalWrite(LED, LOW);
  vTaskDelay(850 / portTICK_PERIOD_MS);
}

void beepTwice() {
  playTone();
  digitalWrite(LED, HIGH);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  noTone(BUZZER);
  digitalWrite(LED, LOW);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  playTone();
  digitalWrite(LED, HIGH);
  vTaskDelay(100 / portTICK_PERIOD_MS);
  noTone(BUZZER);
  digitalWrite(LED, LOW);
}

// loopt solange, bis eins von zwei wlans verbunden ist
void checkWiFi() {
  while (WiFi.status() != WL_CONNECTED) {
    connectWiFi(ssid, password);
    if (WiFi.status() != WL_CONNECTED)
      connectWiFi(ssid2, password2);
  }
}

// versucht sich mit einem WLAN zu verbinden
void connectWiFi(const char *ssid, const char *password) {
  int connect_attempt = 0;  // verbindungs timeout
  Serial.print("Verbinde mit ");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED && connect_attempt <= 5) {
    beepOnce();
    Serial.print(".");
    connect_attempt++;
  }
  Serial.println("");
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("Couldn't connect!");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF); //sonst kann keine neue ssid & passwd vergeben werden
    delay(500);
    WiFi.mode(WIFI_STA);
  } 
  else {
    Serial.println("\nWLAN Verbunden!");
    Serial.printf("IP-Adresse: %d.%d.%d.%d Empfang: %ddb\n", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3], WiFi.RSSI());
    beepTwice();
    WiFi.setSleep(false); //um hoffentlich package loss zu verhindern
  }
}

/*
 * dieser Thread setzt global flags, welches Modus gerade ist
 */
void pinReadTask(void *pvParameters) {
  while (true) {
    Serial.printf("debug pin 1\n");
    NO_BUZZER_MODE = (digitalRead(NO_BUZZER_MODE_PIN) == LOW);
    NO_PRINTER_MODE = (digitalRead(NO_PRINTER_MODE_PIN) == LOW);
    SELF_CHECK_MODE = (digitalRead(SELF_CHECK_MODE_PIN) == LOW);
    SERVER_CHECK_MODE = (digitalRead(SERVER_CHECK_MODE_PIN) == LOW);

    pinHighwater = uxTaskGetStackHighWaterMark(NULL);

    Serial.printf("debug pin 2\n");
    vTaskDelay(5);
  }
}

/*
 * Dieser Thread arbeitet udp Aufgaben der anderen Tasks seriall ab, da es sonst zu race conditions kommt. übergabe über queues
 */
void WiFiMonitorTask(void *pvParameters) {
  while(true) {
    Serial.printf("debug wifi 1\n");
    checkWiFi();
    Serial.printf("debug wifi 2\n");
    char buffer[500];
    Serial.printf("debug wifi 3\n");
    vTaskGetRunTimeStats(buffer);
    Serial.printf("debug wifi 4\n");

    WiFiMonitorHighwater = uxTaskGetStackHighWaterMark(NULL);

    Serial.printf("Stacks udp: %d pin: %d receive: %d send: %d play: %d wifi: %d print: %d\n", updHighwater, pinHighwater, receiveHighwater, sampleSendHighwater, playBackHighwater, WiFiMonitorHighwater, printHighwater);

    Serial.printf("debug wifi 5\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);  //alle 5 sek das wlan checken
  }

}

/*
 * Dieser Thread arbeitet udp Aufgaben der anderen Tasks seriall ab, da es sonst zu race conditions kommt. übergabe über queues
 */
void udpWorkerTask(void *pvParameters) {
    UdpPacket outgoing;
    UdpPacket incoming;

    udp.begin(udpPort);
    Serial.printf("Lausche und sende vermutlich nicht auf Port: %d haha\n", udpPort);

    while (true) {
        Serial.printf("debug udp 1\n");
        // Empfang prüfen
        int packetSize = udp.parsePacket();
        if (packetSize >= sizeof(UdpPacket)) {
            udp.read((uint8_t*)&incoming, sizeof(UdpPacket));
            //Serial.printf("recieved sess:%u seq:%u data:%u time:%d\n", incoming.session, incoming.seq, incoming.data, millis());

            if(xQueueSend(udpRecvQueue, &incoming, 0) != pdPASS) { // wartet 0 ticks, falls voll
              Serial.println("udp incoming Queue pass!!!");
              printer.println("udp incoming Queue pass!");
            }
        }
        // Senden falls gewünscht
        if (xQueueReceive(udpSendQueue, &outgoing, 0) == pdPASS) { //wartet 0 ticks, falls nichts drin
            if (SERVER_CHECK_MODE || SELF_CHECK_MODE)
              outgoing.session = 0; // zeichen für den server, dass an selben zurücksenden

            if (SELF_CHECK_MODE)
              udp.beginPacket("127.0.0.1", udpPort);    // 127.0.0.1 
            else
              udp.beginPacket(udpAddress, udpPort);
            udp.write((uint8_t*)&outgoing, sizeof(UdpPacket));
            udp.endPacket();

            if (SERVER_CHECK_MODE || SELF_CHECK_MODE)
              outgoing.session = esp_random() & 0xFF;
            //Serial.printf("sent     sess:%u seq:%u data:%u time:%d\n", outgoing.session, outgoing.seq, outgoing.data, millis());
        }
        updHighwater = uxTaskGetStackHighWaterMark(NULL);
        Serial.printf("debug udp 2\n");
        vTaskDelay(2 / portTICK_PERIOD_MS); // Entspannt CPU-Last und lässt dem netzwerk zeit
    }
}

/*
 * dieser Thread fragt konstant nach neuen packages ab und speichert sie im ringpuffer
 * wenn der ringpuffer voll ist, werden neue packete verworfen
 */
void receiveTask(void *pvParameters) {
  uint8_t session_num = esp_random() & 0xFF;
  uint8_t last_session_num = 0;
  uint8_t last_seq = 0;
  signed char seq_diff;
  int writeIndex = 0;
  UdpPacket incoming;
  
  while (true) {
    Serial.printf("debug recv 1\n");
    //Serial.println("debug 1:1");
    // gucken, ob neues Packet angekommen ist
    if (xQueueReceive(udpRecvQueue, &incoming, 0) == pdPASS) { //wartet 0 ticks, falls nichts drin

      // neue session -> seq_num reset
      if (incoming.session != last_session_num) {
        Serial.println("New Session");
        last_session_num = incoming.session;
        last_seq = incoming.seq -1;
      }

      seq_diff = incoming.seq - last_seq; // pos = new, neg = old

      // verspätetes packet
      if(seq_diff < 0) {
        if(-seq_diff < bufferedPackets) {   //wenn das packet noch neu genug ist (der consumer noch nicht darüber hinweg ist) wird umsortiert
          Serial.printf("old package sorted in %d\n", seq_diff);
          printer.printf("old package sorted in %d\n", seq_diff);
          for(int i = 0; i > seq_diff; i--) {                       // platz für das neue packet wird geschaffen
            jitterBuffer[writeIndex + i] = jitterBuffer[writeIndex + (i-1)];
          }
          jitterBuffer[writeIndex + seq_diff] = incoming.data;
          if(xQueueSend(printQueue, &incoming.data, 1) != pdPASS) {
            Serial.println("Queue pass!!!");
            printer.println("Queue pass!");
          }
          writeIndex = (writeIndex + 1) % BUFFER_SIZE;
          portENTER_CRITICAL(&mux);
          bufferedPackets++;
          portEXIT_CRITICAL(&mux);
        } else {
          Serial.printf("too old package %d - thrown\n", seq_diff);
          printer.printf("too old package %d - thrown\n", seq_diff);
        }
        continue;
      }

      // doppeltes packet
      if(seq_diff == 0) {
          Serial.println("double package - thrown");
          printer.println("double package - thrown");
          continue;
      }

      // erwartetes oder frühes packet
      if(seq_diff > 0) {
        if(seq_diff > 1) { // frühes packet
          //Serial.printf("package missing %d\n", seq_diff);
          //printer.printf("package missing %d\n", seq_diff);
        }
        if(bufferedPackets < BUFFER_SIZE) { // wenn noch platz im jitterbuffer ist
          jitterBuffer[writeIndex] = incoming.data;
          if(xQueueSend(printQueue, &incoming.data, 0) != pdPASS) {
            Serial.println("Print Queue pass!!!");
            printer.println("Print Queue pass!");
          }
          writeIndex = (writeIndex + 1) % BUFFER_SIZE;
          portENTER_CRITICAL(&mux);
          bufferedPackets++;
          portEXIT_CRITICAL(&mux);
        } 
        else { // überlauf
          Serial.println("paket verworfen (Buffer voll)");
          printer.println("paket verworfen buffer voll");
        }
        last_seq = incoming.seq;
      }
    }
    //Serial.println("debug 1:2");
    // Netwerkverbinding überprüfen/erneuern

    receiveHighwater = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("debug recv 2\n");
    vTaskDelay(5 / portTICK_PERIOD_MS); // wartet 5ms
  }
}

/*
 * Dieser Thread speichert Samples in einem Packet '1' für an und '0' für aus
 * Wenn das Packet voll ist, wird das packet abgeschickt
 * evtl ping und abstellen, wenn Stille
 * tatsächlich ist der Schalter bei LOW betätigt
 */
void sampleSendTask(void *pvParameters) {
  UdpPacket packet;
  packet.session = esp_random() & 0xFF;
  packet.seq = 0;
  int zeros = 0;    // counting zeros to turn off while silence
  int pingCounter = 0;

  while (true) {
    Serial.printf("debug send 1\n");
    //Serial.println("debug 2:1");
    packet.data = 0b00000000;
    for(uint8_t read_mask = 0b00000001; read_mask > 0; read_mask = read_mask << 1) {
      //Serial.println("debug 2:2");
      if(digitalRead(BUTTON) == LOW) {
        packet.data = packet.data | read_mask;
        zeros = 0;
        pingCounter = 0;
      }
      else {
        zeros++;
      }
      Serial.printf("debug send 2\n");
      vTaskDelay(SAMPLE_RATE / portTICK_PERIOD_MS); // wartet so viele ticks (zeitscheiben), wie eine sample_rate lang ist
    }
    Serial.printf("debug send 3\n");
    if (zeros <= TURN_OFF_MAX || pingCounter >= PING_INTERVAL) {
      //Serial.println("debug 2:3");
      packet.seq++;
      // übergeben an udp task
      if(xQueueSend(udpSendQueue, &packet, 0) != pdPASS) { // wartet 0 ticks, falls voll
        Serial.println("udp outgoing Queue pass!!!");
        printer.println("udp outgoing Queue pass!");
      }
      pingCounter = 0;
    } 
    else {
      pingCounter++;
    }

    sampleSendHighwater = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("debug send 4\n");
  }
}

/*
 * Dieser Thread guckt, ob packete da sind
 * wenn mehr als 5 packete da sind, beginnt er sie abzuspielen und setzt tonePlaying auf TRUE
 * jetzt werden auch packete abgespielt, wenn weniger als 5 da sind
 * wenn die packets leer gehen, wird wieder nichts gespielt, bis wieder 5 packets da sind
 * evtl delay, um cpu zeit freizugeben
 */
void playbackBufferTask(void *pvParameters) { // sample entpacken
  int readIndex = 0;
  bool samplePlaying = false;
  bool tonePlaying = false;
  while(true) {
    Serial.printf("debug play 1\n");
    //Serial.println(bufferedPackets);
    if (bufferedPackets > BUFFER_SIZE / 2 || (samplePlaying && bufferedPackets > 0)) {
      //Serial.println("debug 3:2");
      samplePlaying = true;
      for(uint8_t read_mask = 0b00000001; read_mask > 0; read_mask = read_mask << 1) { // die 1 wird solange geschiftet, bis sie weg ist und nur noch 0b00000000 da ist
        if(jitterBuffer[readIndex] & read_mask) {
          if(!tonePlaying) {
            playTone(); // tone() blockiert auf dem esp32 den thread NICHT
            digitalWrite(LED, HIGH);
            tonePlaying = true;
          }
        }
        else {
          noTone(BUZZER);
          digitalWrite(LED, LOW);
          tonePlaying = false;
        }
        Serial.printf("debug play 2\n");
        vTaskDelay(SAMPLE_RATE / portTICK_PERIOD_MS); // wartet so viele ticks (zeitscheiben), wie eine sample_rate lang ist
      }
      readIndex = (readIndex + 1) % BUFFER_SIZE;
      portENTER_CRITICAL(&mux);
      bufferedPackets--;
      portEXIT_CRITICAL(&mux);
    } 
    else {
      //Serial.println("debug 3:3");
      samplePlaying = false;
      tonePlaying = false;
      noTone(BUZZER);
      digitalWrite(LED, LOW);
      
      playBackHighwater = uxTaskGetStackHighWaterMark(NULL);
      Serial.printf("debug play 3\n");
      vTaskDelay(SAMPLE_RATE / portTICK_PERIOD_MS);
    }
  }
}

/*
 * Dieser Thread bekommt die empfangenen samples geschickt und baut mit ihnen eine bzw zwei zeilen
 * wenn genug samples für eine ganze zeile gesammelt wurde, dann wird diese gedruckt
 */

void printTask(void *pvParameters) {
  uint8_t packet;
  char print_line[384];  // wird zwei linien übereinander enthalten, 384 spalten breit
  int print_index = 0;  // wo die zeile gerade erstellt wird
  bool something_in_it = false;   // ob die zeile schon etwas in sich hat
  unsigned long last_added = millis();

  while(true) {
    Serial.printf("debug print 1\n");
    //Serial.printf("debug 4:1 %u\n", print_line[0]);
    if(xQueueReceive(printQueue, &packet, portMAX_DELAY) == pdPASS) { // wartet bis neues Element kommt. blockiert die cpu nicht
    Serial.printf("debug print 2\n");
      //Serial.printf("debug 4:2 %u\n", print_line[0]);
      for(uint8_t read_mask = 0b00000001; read_mask > 0; read_mask = read_mask << 1) { // die 1 wird solange geschiftet, bis sie weg ist und nur noch 0b00000000 da ist
        //Serial.printf("debug 4:3 %u\n", print_line[0]);
        if(packet & read_mask) {
          if(print_index < 384)
            print_line[print_index] = 0b01100000;
          else
            print_line[print_index - 384] = (print_line[print_index - 384]) | 0b00000110; //fügt untere linie hinzu
          something_in_it = true;
          print_index++;
          last_added = millis();
        }
        else if (something_in_it) { // wenn null geschrieben wird, aber schon mindestens eine 1 im buffer ist
          if(print_index < 384) 
            print_line[print_index] = 0b00000000;
          print_index++;
        }
        //Serial.printf("debug 4:4 %u\n", print_line[0]);

        // abfrage, ob wenn was im speicher ist, aber lange nichts mehr gekommen ist, dann wird der rest mit 0 aufgefüllt und ausgedruckt
        if(something_in_it && (millis() - last_added > PRINT_TIMEOUT)) {
          for(print_index; print_index < 384*2; print_index++) {
            if(print_index < 384) 
              print_line[print_index] = 0b00000000;
          }
        }

        //Serial.printf("debug 4:5 %u\n", print_line[0]);

        // genug material für eine Zeile
        if(print_index == 384*2) {
          // ESC * m nL nH
          if(!NO_PRINTER_MODE) {
            printer.write(27); printer.write('@'); // ESC @ means reset
            printer.write(27); printer.write(55); // ESC 7 configure heating parameters
            printer.write(1);   // n1 = 5   → heizpunkte, die gleichzeitig laufen dürfen
            printer.write(255); // n2 = 240 → lange Heizzeit (langsam, aber dunkler)
            printer.write(255); // n3 = 200 → große Pause zwischen Heizungen, Strom sinkt stark

            printer.write(27); printer.write('*');
            printer.write((uint8_t)1); // Mode 1 = 8-dot double density (384 pixel)
            printer.write((uint8_t)128); // nL = 128 Bits
            printer.write((uint8_t)1);   // nH = 1 (1*256)    Zeilenlänge = nL + nH * 256 = 384
          
            // Daten senden
            for (int i = 0; i < 384; i++) {
              printer.write(print_line[i]);
              //Serial.print((uint8_t)print_line[i]);
            }
            printer.write(10); // Zeilenvorschub
            printer.flush(); // wartet, bis alles gesendet wurde
          }
          print_index = 0;
          something_in_it = false;
        }
      }
    }
    printHighwater = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("debug print 3\n");
  }
}
