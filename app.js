
const WebSocket = require('ws');
const express = require('express');
const path = require('path');
const app = express();
const port = 3000;
const mqtt = require('mqtt');

const client = mqtt.connect('mqtt://192.168.1.108:1883');

app.use(express.static(path.join(__dirname,"/public/")));

client.on('connect', function () {
    console.log('Connected to MQTT Broker');
    // ส่งข้อความเปิด/ปิดพัดลม (fan)
    client.publish('esp32/fan_control', '1'); // เปิดพัดลม
    // ส่งข้อความเปิด/ปิดไฟ (light)
    client.publish('esp32/light_control', '0'); // ปิดไฟ
    // ส่งข้อความเปิด/ปิดปั๊ม (pump)
    client.publish('esp32/pump_control', '1'); // เปิดปั๊ม
});

// หรือสร้างฟังก์ชันเพื่อควบคุมแต่ละอุปกรณ์เป็นตัวแปร
function toggleFan(status) {
    client.publish('esp32/fan_control', status ? '1' : '0'); // 1 เปิด, 0 ปิด
}

function toggleLight(status) {
    client.publish('esp32/light_control', status ? '1' : '0'); // 1 เปิด, 0 ปิด
}

function togglePump(status) {
    client.publish('esp32/pump_control', status ? '1' : '0'); // 1 เปิด, 0 ปิด
}

app.listen(port,() =>{
    console.log("Server is running on port %d",port);
})