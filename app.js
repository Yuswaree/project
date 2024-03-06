
const WebSocket = require('ws');
const express = require('express');
const path = require('path');
const app = express();
const port = 3000;
const mqtt = require('mqtt');
const mqttBroker = 'mqtt://localhost';
const mqttClient = mqtt.connect(mqttBroker);

app.use(express.static(path.join(__dirname,"/public/")));

app.listen(port,() =>{
    console.log("Server is running on port %d",port);
})