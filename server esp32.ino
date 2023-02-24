#include <WiFi.h>
#include <WebServer.h>
#include <DHT.h>

#define DHTPIN 15
#define DHTTYPE DHT11

const char* ssid = "fusiongames";
const char* password = "octubre32";

WebServer server(80);

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  dht.begin();

  Serial.begin(115200);
  delay(1000);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.print("Conectado a la red WiFi: ");
  Serial.println(ssid);
  Serial.print("Dirección IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", [](){
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    String html = "<html><head><title>ESP32 Web Server</title></head>";
    html += "<body>";
    html += "<h1>ESP32 Web Server</h1>";
    html += "<p>Temperatura: ";
    html += String(t);
    html += " &#8451;</p>";
    html += "<p>Humedad: ";
    html += String(h);
    html += " %</p>";
    html += "<p>LED 1: <a href=\"/on1\"><button>ON</button></a>&nbsp;<a href=\"/off1\"><button>OFF</button></a></p>";
    html += "<p>LED 2: <a href=\"/on2\"><button>ON</button></a>&nbsp;<a href=\"/off2\"><button>OFF</button></a></p>";
    html += "</body></html>";

    server.send(200, "text/html", html);
  });

  server.on("/on1", [](){
    digitalWrite(26, HIGH);
    server.send(200, "text/html", "LED 1 encendido");
  });

  server.on("/off1", [](){
    digitalWrite(26, LOW);
    server.send(200, "text/html", "LED 1 apagado");
  });

  server.on("/on2", [](){
    digitalWrite(27, HIGH);
    server.send(200, "text/html", "LED 2 encendido");
  });

  server.on("/off2", [](){
    digitalWrite(27, LOW);
    server.send(200, "text/html", "LED 2 apagado");
  });

  server.begin();
}

void loop() {
  server.handleClient();
  
  // Enciende el LED en el pin 27 cuando la temperatura es mayor a 25 grados
  float t = dht.readTemperature();
  if (t > 25) {
    digitalWrite(27, HIGH);
  } else {
    digitalWrite(27, LOW);
  }
}