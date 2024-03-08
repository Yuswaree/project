#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "ModernFarm";
const char* password = "Smart@Farm2565";
const char* mqtt_server = "192.168.1.108";

WiFiClient espClient;
PubSubClient client(espClient);

int fan = 4;
int light = 16;
int pump = 17;

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // Check the topic and control the corresponding relay
  if (strcmp(topic, "esp32/fan_control") == 0) {
    digitalWrite(fan, payload[0] == '1' ? HIGH : LOW);
  } else if (strcmp(topic, "esp32/light_control") == 0) {
    digitalWrite(light, payload[0] == '1' ? HIGH : LOW);
  } else if (strcmp(topic, "esp32/pump_control") == 0) {
    digitalWrite(pump, payload[0] == '1' ? HIGH : LOW);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe("esp32/fan_control");
      client.subscribe("esp32/light_control");
      client.subscribe("esp32/pump_control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(fan, OUTPUT);
  pinMode(light, OUTPUT);
  pinMode(pump, OUTPUT);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}