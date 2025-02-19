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
    :root {
      --primary-color: #007bff;
      --primary-hover: #0056b3;
      --border-color: #dee2e6;
      --background-gray: #f8f9fa;
    }
    
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }
    
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial, sans-serif;
      line-height: 1.5;
      color: #212529;
      background-color: #fff;
      padding: 20px;
    }
    
    .container {
      max-width: 600px;
      margin: 0 auto;
      background-color: white;
      border-radius: 8px;
      box-shadow: 0 2px 4px rgba(0,0,0,0.1);
      padding: 20px;
    }
    
    h2 {
      color: #212529;
      text-align: center;
      margin-bottom: 30px;
      font-weight: 500;
    }
    
    .form-group {
      margin-bottom: 25px;
      background-color: var(--background-gray);
      padding: 20px;
      border-radius: 6px;
    }
    
    label {
      display: block;
      margin-bottom: 8px;
      font-weight: 500;
      color: #495057;
    }
    
    .slider {
      -webkit-appearance: none;
      width: 100%;
      height: 8px;
      border-radius: 4px;
      background: #dee2e6;
      outline: none;
      margin: 10px 0;
    }
    
    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 20px;
      height: 20px;
      border-radius: 50%;
      background: var(--primary-color);
      cursor: pointer;
      transition: background-color 0.2s;
    }
    
    .slider::-webkit-slider-thumb:hover {
      background: var(--primary-hover);
    }
    
    .checkbox-wrapper {
      display: flex;
      align-items: center;
      gap: 8px;
    }
    
    input[type="checkbox"] {
      width: 18px;
      height: 18px;
      cursor: pointer;
    }
    
    .value-display {
      display: inline-block;
      min-width: 40px;
      padding: 2px 8px;
      background-color: var(--primary-color);
      color: white;
      border-radius: 4px;
      text-align: center;
      margin-left: 10px;
    }
    
    .btn-submit {
      background-color: var(--primary-color);
      color: white;
      border: none;
      padding: 12px 24px;
      border-radius: 6px;
      cursor: pointer;
      font-size: 16px;
      font-weight: 500;
      width: 100%;
      transition: background-color 0.2s;
    }
    
    .btn-submit:hover {
      background-color: var(--primary-hover);
    }
    
    @media (max-width: 480px) {
      .container {
        padding: 15px;
      }
      
      .form-group {
        padding: 15px;
      }
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>ESP32 Configuration</h2>
    
    <form action="/save" method="get">
      <div class="form-group">
        <label>Slider 1: <span class="value-display" id="s1val">%SLIDER1%</span></label>
        <input type="range" name="slider1" min="0" max="100" value="%SLIDER1%" class="slider" oninput="s1val.innerHTML=this.value">
      </div>
      
      <div class="form-group">
        <label>Slider 2: <span class="value-display" id="s2val">%SLIDER2%</span></label>
        <input type="range" name="slider2" min="0" max="100" value="%SLIDER2%" class="slider" oninput="s2val.innerHTML=this.value">
      </div>
      
      <div class="form-group">
        <div class="checkbox-wrapper">
          <input type="checkbox" name="checkbox1" id="checkbox1" %CHECKBOX1%>
          <label for="checkbox1">Checkbox 1</label>
        </div>
      </div>
      
      <div class="form-group">
        <div class="checkbox-wrapper">
          <input type="checkbox" name="checkbox2" id="checkbox2" %CHECKBOX2%>
          <label for="checkbox2">Checkbox 2</label>
        </div>
      </div>

      <button type="submit" class="btn-submit">Save Configuration</button>
    </form>
  </div>
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
  
  // Your main loop code here
  // You can use config.sliderValue1, config.sliderValue2, 
  // config.checkbox1, and config.checkbox2 values
  
  delay(10);
}
