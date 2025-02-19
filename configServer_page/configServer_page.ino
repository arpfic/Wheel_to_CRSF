#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>

// Access Point credentials
const char* ssid = "ESP32_Config";
const char* password = "12345678";

// Create WebServer object on port 80
WebServer server(80);

// Structure to hold configuration
struct Config {
  int sliderValue1;
  int sliderValue2;
  bool checkbox1;
  bool checkbox2;
} config;

// EEPROM size
#define EEPROM_SIZE 512

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>ESP32 Configuration</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin: 0px auto; padding: 15px; }
    .slider { width: 300px; }
    .config-item { margin: 20px; }
  </style>
</head>
<body>
  <h2>ESP32 Configuration</h2>
  
  <form action="/save" method="get">
    <div class="config-item">
      <label>Slider 1: <span id="s1val">%SLIDER1%</span></label><br>
      <input type="range" name="slider1" min="0" max="100" value="%SLIDER1%" class="slider" oninput="s1val.innerHTML=this.value">
    </div>
    
    <div class="config-item">
      <label>Slider 2: <span id="s2val">%SLIDER2%</span></label><br>
      <input type="range" name="slider2" min="0" max="100" value="%SLIDER2%" class="slider" oninput="s2val.innerHTML=this.value">
    </div>
    
    <div class="config-item">
      <input type="checkbox" name="checkbox1" %CHECKBOX1%>
      <label>Checkbox 1</label>
    </div>
    
    <div class="config-item">
      <input type="checkbox" name="checkbox2" %CHECKBOX2%>
      <label>Checkbox 2</label>
    </div>

    <input type="submit" value="Save Configuration">
  </form>
</body>
</html>
)rawliteral";

void loadConfig() {
  EEPROM.get(0, config);
}

void saveConfig() {
  EEPROM.put(0, config);
  EEPROM.commit();
}

String processor(const String& var) {
  if(var == "SLIDER1") return String(config.sliderValue1);
  if(var == "SLIDER2") return String(config.sliderValue2);
  if(var == "CHECKBOX1") return config.checkbox1 ? "checked" : "";
  if(var == "CHECKBOX2") return config.checkbox2 ? "checked" : "";
  return String();
}

void handleRoot() {
  String html = index_html;
  html.replace("%SLIDER1%", String(config.sliderValue1));
  html.replace("%SLIDER2%", String(config.sliderValue2));
  html.replace("%CHECKBOX1%", config.checkbox1 ? "checked" : "");
  html.replace("%CHECKBOX2%", config.checkbox2 ? "checked" : "");
  server.send(200, "text/html", html);
}

void handleSave() {
  if (server.hasArg("slider1")) {
    config.sliderValue1 = server.arg("slider1").toInt();
  }
  if (server.hasArg("slider2")) {
    config.sliderValue2 = server.arg("slider2").toInt();
  }
  config.checkbox1 = server.hasArg("checkbox1");
  config.checkbox2 = server.hasArg("checkbox2");
  
  saveConfig();
  
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Updated");
}

void setup() {
  Serial.begin(115200);
  
  // Initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);
  
  // Load saved configuration
  loadConfig();
  
  // Set up Access Point
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  // Setup server routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_GET, handleSave);
  
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  Serial.println(config.sliderValue1);
  // Your main loop code here
  // You can use config.sliderValue1, config.sliderValue2, 
  // config.checkbox1, and config.checkbox2 values
  
  delay(10);
}
