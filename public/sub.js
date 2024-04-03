var client;
var mqttData = {};

function mqttConnect() {
  
  var mqttServer = '192.168.199.80';
  var mqttPort = 9001; 
  var clientId = 'web_' + parseInt(Math.random() * 100, 10);

  // สร้าง client instance
  client = new Paho.MQTT.Client(mqttServer, mqttPort, clientId);

  // ตั้งค่า callback handlers
  client.onConnectionLost = onConnectionLost;
  client.onMessageArrived = onMessageArrived;

  // เชื่อมต่อกับ MQTT broker
  client.connect({onSuccess:onConnect});
}

// ทำงานเมื่อเชื่อมต่อ MQTT สำเร็จ
function onConnect() {
  console.log("onConnect");
  // Subscribe to topics ที่คุณต้องการ
  client.subscribe("esp32/temperature");
  client.subscribe("esp32/humidity");
  client.subscribe("esp32/flow_rate");
  client.subscribe("esp32/temperaturewather");
  client.subscribe("esp32/ldr");
  client.subscribe("esp32/tds");
  // ต่อไปนี้สำหรับ topics อื่นๆ ...
}

// ทำงานเมื่อเชื่อมต่อหลุด
function onConnectionLost(responseObject) {
  if (responseObject.errorCode !== 0) {
    console.log("onConnectionLost:"+responseObject.errorMessage);
  }
}

// ทำงานเมื่อมีข้อความมาถึง
function onMessageArrived(message) {
    console.log("onMessageArrived:"+message.payloadString);
    mqttData[message.destinationName] = message.payloadString;
  
    // อัปเดตข้อมูลใน HTML
    if(message.destinationName === "esp32/temperature") {
      document.getElementById('temperatureValue').innerText = message.payloadString;
    } else if(message.destinationName === "esp32/flow_rate") {
      document.getElementById('flowRateValue').innerText = message.payloadString;
    }
     else if(message.destinationName === "esp32/temperaturewather") {
      document.getElementById('temperaturewather').innerText = message.payloadString;
    }
    else if(message.destinationName === "esp32/tds") {
        document.getElementById('tds').innerText = message.payloadString;
    }
    else if(message.destinationName === "esp32/ldr") {
        document.getElementById('ldr').innerText = message.payloadString;
    }
    else if(message.destinationName === "esp32/humidity") {
        document.getElementById('humidity').innerText = message.payloadString;
    }
    // ทำการอัปเดต elements อื่นๆ ตามที่ต้องการ
  }

// เรียกใช้ function เพื่อเชื่อมต่อ MQTT
mqttConnect();