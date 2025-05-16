#define CAMERA_MODEL_AI_THINKER 

#include "esp_camera.h" 
#include <WiFi.h>
#include <ESPmDNS.h>     
#include <WebServer.h>   
#include <WiFiUdp.h>     
#include <AESLib.h>      
#include "mbedtls/base64.h" 

//WiFi Configuration
const char* WIFI_SSID = "1";         
const char* WIFI_PASSWORD = "1"; 

//Encryption Configuration (MUST MATCH PYTHON SCRIPT)
//In secrets.h

//UDP Video Stream Configuration
WiFiUDP videoUdp; 
IPAddress pythonClientIP; 
bool pythonClientIPLearned = false;
const int videoUdpPort = 12345;                         
const size_t UDP_PACKET_SIZE = 1400; 
#define START_OF_FRAME_MARKER "<SOF>" 
#define END_OF_FRAME_MARKER   "<EOF>" 
bool pythonClientReadyForVideo = false; 

//UDP Log Forwarding Configuration
WiFiUDP logUdp; 
const int logUdpPort = 12346; 
// #define DISABLE_UDP_LOG_FORWARDING 

//Web Server for Pan/Tilt Commands (TCP/HTTP)
WebServer commandServer(8081); 

//Camera Pin Definitions
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1 
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26 
  #define SIOC_GPIO_NUM     27 
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19 
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected or pins defined!"
#endif


HardwareSerial& ArduinoCommsSerial = Serial; 

String arduinoLogBuffer = ""; 

// --- Forward Declarations for WebServer Handlers ---
void handleRoot();
void handleCommand();
void handlePing(); 
void handleSignalVideoReady();
void handleRegisterPythonClient();

//Function to decrypt data (AES-128 CBC with PKCS7 unpadding)
String decryptCommand(const String& base64Ciphertext) {
  if (base64Ciphertext.length() > 88) { 
    ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): Base64 input too long for decryption."); 
    return "";
  }
  unsigned char ciphertext_bytes[64]; size_t ciphertext_len = 0;
  int ret = mbedtls_base64_decode(ciphertext_bytes, sizeof(ciphertext_bytes), &ciphertext_len, 
                                  (const unsigned char*)base64Ciphertext.c_str(), base64Ciphertext.length());
  if (ret != 0) { ArduinoCommsSerial.printf("ESP32-CAM (SharedSerial): Base64 decode failed! Error: %d\n", ret); return "";} 
  if (ciphertext_len == 0 || ciphertext_len % AES_EFFECTIVE_BLOCK_SIZE != 0) { 
    ArduinoCommsSerial.printf("ESP32-CAM (SharedSerial): Decoded ciphertext length (%d) is invalid.\n", ciphertext_len); return "";} 
  unsigned char decrypted_padded_bytes[64]; memset(decrypted_padded_bytes, 0, sizeof(decrypted_padded_bytes));
  
  uint8_t currentIV[16]; 
  memcpy(currentIV, original_aesIV, 16);

  uint16_t decrypted_padded_len = aesLib.decrypt(ciphertext_bytes, (uint16_t)ciphertext_len, 
                                                 decrypted_padded_bytes, aesKey, sizeof(aesKey) * 8, currentIV); 

  if (decrypted_padded_len == 0) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): AES Decryption failed!"); return "";} 
  if (decrypted_padded_len > ciphertext_len) { ArduinoCommsSerial.printf("ESP32-CAM (SharedSerial): AES Decryption error: out_len %d > in_len %d\n", decrypted_padded_len, ciphertext_len); return "";} 
  int pad_val = (int)decrypted_padded_bytes[decrypted_padded_len - 1];
  if (pad_val <= 0 || pad_val > AES_EFFECTIVE_BLOCK_SIZE) { ArduinoCommsSerial.printf("ESP32-CAM (SharedSerial): Invalid PKCS7 padding value: %d\n", pad_val); return ""; } 
  for (int i = 0; i < pad_val; i++) { if ((int)decrypted_padded_bytes[decrypted_padded_len - 1 - i] != pad_val) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): PKCS7 padding verification failed."); return ""; }} 
  int actual_data_len = decrypted_padded_len - pad_val;
  if (actual_data_len < 0) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): Error in unpadding."); return "";} 
  String result = ""; for(int i=0; i<actual_data_len; i++){ result += (char)decrypted_padded_bytes[i]; }
  ArduinoCommsSerial.print("ESP32-CAM (SharedSerial): Decrypted command: '"); ArduinoCommsSerial.print(result); ArduinoCommsSerial.println("'"); 
  return result;
}


void setup() {
  ArduinoCommsSerial.begin(115200); 
  arduinoLogBuffer.reserve(64); 
  ArduinoCommsSerial.println("\nESP32-CAM Booting (Primary Serial, Quieter Debug)...");
  delay(1000); 

  //WiFi Connection
  ArduinoCommsSerial.println("\n--- WiFi Connection ---"); 
  WiFi.mode(WIFI_STA); delay(100);
  ArduinoCommsSerial.print("Attempting WiFi connection to '"); ArduinoCommsSerial.print(WIFI_SSID); ArduinoCommsSerial.print("'");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); 
  int connect_timeout = 30; 
  while (WiFi.status() != WL_CONNECTED && connect_timeout > 0) { delay(500); ArduinoCommsSerial.print("."); connect_timeout--;}
  ArduinoCommsSerial.println();
  if (WiFi.status() != WL_CONNECTED) {
    ArduinoCommsSerial.println("Initial WiFi connection FAILED. Entering persistent retry mode...");
    while(WiFi.status() != WL_CONNECTED) { 
        ArduinoCommsSerial.print("Retrying WiFi..."); delay(10000); 
        WiFi.disconnect(); delay(100); WiFi.mode(WIFI_STA); delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        int r_wait = 10; while(WiFi.status() != WL_CONNECTED && r_wait-- > 0) { delay(500); ArduinoCommsSerial.print(",");} ArduinoCommsSerial.println();
        if(WiFi.status() != WL_CONNECTED) ArduinoCommsSerial.println("Retry failed."); else break;
    }
  }
  ArduinoCommsSerial.println(">>> WIFI SUCCESSFULLY CONNECTED! <<<");
  ArduinoCommsSerial.print("IP Address: http://"); ArduinoCommsSerial.println(WiFi.localIP());
  ArduinoCommsSerial.println("Waiting for Python client to register its IP via /register_python_client for UDP target.");


  //mDNS Setup
  ArduinoCommsSerial.println("Setting up mDNS responder...");
  if (!MDNS.begin("esp32-cam-tracker")) { ArduinoCommsSerial.println("Error setting up MDNS responder!"); } 
  else {
    ArduinoCommsSerial.println("mDNS responder started: esp32-cam-tracker.local");
    MDNS.addService("http", "tcp", 8081);
    ArduinoCommsSerial.println("mDNS service added: _http._tcp.8081 for commands");
  }

  //Camera Initialization
  ArduinoCommsSerial.println("\n--- Camera Initialization ---");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM; config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM; config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA; 
  config.jpeg_quality = 12;          //Good quality
  config.fb_count = 2; 
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { ArduinoCommsSerial.printf("Camera init failed with error 0x%x. Rebooting...\n", err); delay(10000); ESP.restart(); return; }
  ArduinoCommsSerial.println("Camera initialized successfully.");
  sensor_t * s = esp_camera_sensor_get();
  if (s == NULL) { ArduinoCommsSerial.println("!!! ERROR: Camera sensor not detected. Halting."); while(true) delay(10000); } 
  else { ArduinoCommsSerial.println("Camera sensor detected successfully.");}
  delay(500); 
  
  //Setup TCP/HTTP Command Server
  ArduinoCommsSerial.println("\n--- TCP/HTTP Command Server Setup (Port 8081) ---");
  commandServer.on("/", HTTP_GET, handleRoot); 
  commandServer.on("/command", HTTP_POST, handleCommand); 
  commandServer.on("/ping", HTTP_GET, handlePing); 
  commandServer.on("/signal_video_ready", HTTP_GET, handleSignalVideoReady); 
  commandServer.on("/register_python_client", HTTP_GET, handleRegisterPythonClient); 
  commandServer.begin();
  ArduinoCommsSerial.println("TCP/HTTP Command Server started on port 8081.");
  ArduinoCommsSerial.println("Waiting for Python client to signal ready for UDP video via /signal_video_ready endpoint...");
  
  ArduinoCommsSerial.println("\nSetup complete. Entering main loop...");
}

unsigned long lastFrameTime = 0;
const unsigned long frameInterval = 100; //Target ~10 FPS for VGA
unsigned int udp_send_failures = 0;
unsigned long lastStatsPrintTime = 0; 
const unsigned long statsPrintInterval = 5000; //Print frame stats every 5 seconds

void loop() {
  // MDNS.update(); 

  if (WiFi.status() == WL_CONNECTED) {
    commandServer.handleClient(); 

    if (pythonClientIPLearned && pythonClientReadyForVideo && (millis() - lastFrameTime > frameInterval)) {
      unsigned long t_start_capture = millis();
      camera_fb_t * fb = esp_camera_fb_get();
      unsigned long t_end_capture = millis();

      if (!fb) { /* ... */ } 
      else {
        unsigned long t_start_send = millis();
        videoUdp.beginPacket(pythonClientIP, videoUdpPort); 
        videoUdp.write((const uint8_t*)START_OF_FRAME_MARKER, strlen(START_OF_FRAME_MARKER));
        videoUdp.endPacket();
        size_t bytesSent = 0; int chunks = 0; bool frame_send_success = true;
        while(bytesSent < fb->len) {
            size_t chunkSize = fb->len - bytesSent;
            if (chunkSize > UDP_PACKET_SIZE) chunkSize = UDP_PACKET_SIZE;
            videoUdp.beginPacket(pythonClientIP, videoUdpPort); 
            videoUdp.write(fb->buf + bytesSent, chunkSize);
            if (videoUdp.endPacket()) { bytesSent += chunkSize; chunks++;} 
            else { 
                //This message goes to USB and Arduino
                ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): UDP video chunk send FAILED!"); 
                udp_send_failures++; 
                frame_send_success = false; 
                delay(5); 
                break; 
            }
        }
        if (frame_send_success && bytesSent == fb->len) { 
            videoUdp.beginPacket(pythonClientIP, videoUdpPort); 
            videoUdp.write((const uint8_t*)END_OF_FRAME_MARKER, strlen(END_OF_FRAME_MARKER));
            videoUdp.endPacket();
        } else if (!frame_send_success) { 
            //This message goes to USB and Arduino
            ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): Frame sending incomplete, not sending EOF.");
        }
        unsigned long t_end_send = millis();
        
        //Print frame stats less frequently TO THE SHARED SERIAL PORT
        if (millis() - lastStatsPrintTime > statsPrintInterval) {
            ArduinoCommsSerial.printf("Frame: %uB, %uchunks. Cap: %lums, Send: %lums. Total: %lums. UDP Fails: %u (Target: %s)\n", 
                          fb->len, chunks, (t_end_capture - t_start_capture), (t_end_send - t_start_send), 
                          (t_end_send - t_start_capture), udp_send_failures, pythonClientIP.toString().c_str());
            lastStatsPrintTime = millis();
        }
        esp_camera_fb_return(fb); 
        lastFrameTime = millis();
      }
    }

    //Check for and forward log messages from Arduino (received on PRIMARY Serial)
    #ifndef DISABLE_UDP_LOG_FORWARDING
    if (pythonClientIPLearned) { 
        while (ArduinoCommsSerial.available() > 0) { 
          char receivedChar = ArduinoCommsSerial.read();
          arduinoLogBuffer += receivedChar;
          if (receivedChar == '\n') {
            arduinoLogBuffer.trim(); 
            if (arduinoLogBuffer.startsWith("LOG_START:") || arduinoLogBuffer.startsWith("LOG_STOP:")) {
              
             

              logUdp.beginPacket(pythonClientIP, logUdpPort); 
              logUdp.print(arduinoLogBuffer); 
              if(!logUdp.endPacket()){ ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): UDP log send failed!");}
            } 
            arduinoLogBuffer = ""; 
          }
          if (arduinoLogBuffer.length() > 60) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): Arduino log buffer overflow."); arduinoLogBuffer = "";}
        }
    }
    #else 
    if (ArduinoCommsSerial.available() > 0) { while(ArduinoCommsSerial.available()) ArduinoCommsSerial.read(); }
    #endif

  } else { 
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 30000) { 
        ArduinoCommsSerial.println("WiFi lost. Reconnecting..."); 
        WiFi.disconnect(); delay(100); WiFi.mode(WIFI_STA); delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastReconnectAttempt = millis();
    }
  }
  delay(1); 
}

//TCP/HTTP Command Server Handlers
void handleRoot() { 
  String html = "<html><body><h1>ESP32-CAM (Primary Serial, Quieter Debug)</h1>";
  html += "<p>Pan/Tilt commands on port 8081 (POST to /command). GET /ping, GET /signal_video_ready, GET /register_python_client.</p>";
  html += "<p>Video is sent via UDP to "; 
  if(pythonClientIPLearned) { html += pythonClientIP.toString(); } else { html += "[Python IP Not Yet Registered]"; }
  html += ":"; html += String(videoUdpPort); html += ".</p>";
  #ifndef DISABLE_UDP_LOG_FORWARDING
    html += "<p>Arduino logs are forwarded via UDP to "; 
    if(pythonClientIPLearned) { html += pythonClientIP.toString(); } else { html += "[Python IP Not Yet Registered]"; }
    html += ":"; html += String(logUdpPort); html += ".</p>";
  #else
    html += "<p>Arduino log forwarding via UDP is currently DISABLED.</p>";
  #endif
  if (WiFi.status() == WL_CONNECTED) { html += "<p>Current IP: " + WiFi.localIP().toString() + "</p>"; }
  html += "<p>Python Client Ready for Video: "; html += (pythonClientReadyForVideo ? "Yes" : "No - Waiting for /signal_video_ready"); html += "</p>";
  html += "</body></html>";
  commandServer.send(200, "text/html", html);
}

void handleCommand(void) { 
  if (WiFi.status() != WL_CONNECTED) { commandServer.send(503, "text/plain", "WiFi not connected."); return; }
  if (!commandServer.hasArg("plain")) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): HTTP /command - Body not received!"); commandServer.send(400, "text/plain", "Body not received for command"); return; }
  String base64EncryptedCommand = commandServer.arg("plain");
  ArduinoCommsSerial.print("ESP32-CAM (SharedSerial): Received Base64 Encrypted Command (len "); 
  ArduinoCommsSerial.print(base64EncryptedCommand.length()); ArduinoCommsSerial.print(")"); ArduinoCommsSerial.println();
  String decryptedCommand = decryptCommand(base64EncryptedCommand);
  if (decryptedCommand.length() == 0) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): Cmd decryption failed."); commandServer.send(400, "text/plain", "Cmd decryption failed."); return;}
  if (decryptedCommand.startsWith("S") && decryptedCommand.indexOf(':') > 0) {
    ArduinoCommsSerial.println(decryptedCommand); //Send to Arduino via PRIMARY Serial
    
    commandServer.send(200, "text/plain", "Encrypted cmd received, decrypted, forwarded: " + decryptedCommand);
  } else {
    ArduinoCommsSerial.print("ESP32-CAM (SharedSerial): Invalid format after decryption: '"); ArduinoCommsSerial.print(decryptedCommand); ArduinoCommsSerial.println("'");
    commandServer.send(400, "text/plain", "Invalid command format after decryption.");
  }
}

void handlePing() { 
    ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): Received /ping request on command server (port 8081).");
    commandServer.send(200, "text/plain", "pong_8081_tcp");
}

void handleSignalVideoReady() {
  ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): Received /signal_video_ready from Python client.");
  pythonClientReadyForVideo = true;
  commandServer.send(200, "text/plain", "OK, ESP32-CAM will now start sending UDP video if Python IP is registered.");
}

void handleRegisterPythonClient() {
  pythonClientIP = commandServer.client().remoteIP();
  pythonClientIPLearned = true;
  String ipStr = pythonClientIP.toString();
  ArduinoCommsSerial.print("ESP32-CAM (SharedSerial): Python client registered. UDP Target IP set to: ");
  ArduinoCommsSerial.println(ipStr);
  commandServer.send(200, "text/plain", "Python client IP registered for UDP: " + ipStr);
  
  ArduinoCommsSerial.println("--- Updated UDP Target ---");
  ArduinoCommsSerial.print("Will send UDP video to: "); ArduinoCommsSerial.print(pythonClientIP.toString()); 
  ArduinoCommsSerial.print(":"); ArduinoCommsSerial.println(videoUdpPort);
  #ifndef DISABLE_UDP_LOG_FORWARDING
    ArduinoCommsSerial.print("Will send UDP logs to: "); ArduinoCommsSerial.print(pythonClientIP.toString());
    ArduinoCommsSerial.print(":"); ArduinoCommsSerial.println(logUdpPort);
  #endif
}
