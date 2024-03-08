var mqtt;
var reconnectTimeout = 2000;
var host = "192.168.1.108";  // IP address ของ MQTT broker
var port = 1883;  // พอร์ตของ MQTT broker
var fan_topic = "esp32/fan_control";  // ชื่อของ topic ที่ใช้ควบคุมรีเลย์พัดลม
var light_topic = "esp32/light_control";  // ชื่อของ topic ที่ใช้ควบคุมรีเลย์ไฟ
var pump_topic = "esp32/pump_control";  // ชื่อของ topic ที่ใช้ควบคุมรีเลย์ปั๊ม

function onFailure(message) {
  console.log("Connection Attempt to Host " + host + "Failed");
  setTimeout(MQTTconnect, reconnectTimeout);
}

function onMessageArrived(msg) {
  console.log(msg.destinationName + '  ' + msg.payloadString);
  // Add code here to handle incoming messages if needed
}

function onConnect() {
  console.log("Connected");
  // Subscribe to the topics
  mqtt.subscribe(fan_topic);
  mqtt.subscribe(light_topic);
  mqtt.subscribe(pump_topic);
}

function MQTTconnect() {
  console.log("connecting to " + host + " " + port);
  mqtt = new Paho.MQTT.Client(host, port, "clientjs");
  var options = {
    timeout: 3,
    onSuccess: onConnect,
    onFailure: onFailure,
  };
  mqtt.onMessageArrived = onMessageArrived;
  mqtt.connect(options); //connect
}

function sendCommand(topic, command) {
  // Create a message and publish it
  message = new Paho.MQTT.Message(command);
  message.destinationName = topic;
  mqtt.send(message);
}