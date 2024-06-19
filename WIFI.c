#include WiFi.h

const char ssid = ESP32_Hotspot;    Hotspot名称
const char password = 12345678;     Hotspot密码

WiFiServer server(2333);  创建一个TCP服务器，端口号为23（Telnet默认端口）
WiFiClient client;

void setup() {
   初始化串口通信，波特率为115200
  Serial.begin(9600);

   连接到Wi-Fi网络
  
   while (WiFi.status() != WL_CONNECTED) {
     WiFi.begin(ssid, password);
     delay(500);
     Serial.print(.);
   }

  WiFi.softAP(ssid, password);
  
  Serial.println();
  Serial.println(WiFi connected.);

   打印设备的IP地址
  Serial.println(IP address );
  Serial.println(WiFi.localIP());

   启动TCP服务器
  server.begin();
  server.setNoDelay(true);
}

void loop() {
   检查是否有新的客户端连接
  if (server.hasClient()) {
    if (!client  !client.connected()) {
      if (client) client.stop();
      client = server.available();
      Serial.println(New client connected.);
    }
  }

   如果有客户端连接，检查是否有数据到达
  if (client && client.connected()) {
    while (client.available()) {
      char c = client.read();
      Serial.write(c);  将接收到的TCP数据通过串口发送出去
    }
  }

   检查是否有串口数据到达，并转发给TCP客户端
  while (Serial.available()) {
    char c = Serial.read();
    if (client && client.connected()) {
      client.write(c);
    }
  }
}