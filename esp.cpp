#define CAMERA_MODEL_AI_THINKER 

#include "esp_camera.h" 
#include <WiFi.h>
#include <ESPmDNS.h>     
#include <WebServer.h>   
#include <WiFiUdp.h>     
#include <AESLib.h>      
#include "mbedtls/base64.h" 

//WiFi Configuration
const char* WIFI_SSID = "S1ght";         
const char* WIFI_PASSWORD = "armut1998x"; 

//Encryption Configuration
uint8_t aesKey[] = { 
  0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 
  0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C 
};
const uint8_t original_aesIV[]  = { 
  0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
  0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
};
AESLib aesLib; 
const int AES_EFFECTIVE_BLOCK_SIZE = 16; 

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

//Web Server
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

//Serial Communication with Arduino using PRIMARY Serial (UART0)
// This 'Serial' object (UART0 on GPIO1 TX, GPIO3 RX) will be used for 
// Arduino communication AND for ESP32-CAM's own USB debug output.
HardwareSerial& ArduinoCommsSerial = Serial; 

String arduinoLogBuffer = ""; 

//Forward Declarations for WebServer Handlers
void handleRoot();
void handleCommand();
void handlePing(); 
void handleSignalVideoReady();
void handleRegisterPythonClient();

//Function to decrypt data (AES-128 CBC PKCS7 unpadding)
String decryptCommand(const String& base64Ciphertext) {
  if (base64Ciphertext.length() > 88) { 
    ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): B64 too long"); 
    return "";
  }
  unsigned char ciphertext_bytes[64]; size_t ciphertext_len = 0;
  int ret = mbedtls_base64_decode(ciphertext_bytes, sizeof(ciphertext_bytes), &ciphertext_len, 
                                  (const unsigned char*)base64Ciphertext.c_str(), base64Ciphertext.length());
  if (ret != 0) { ArduinoCommsSerial.printf("ESP32-CAM (SharedSerial): B64 decode fail! Err: %d\n", ret); return "";} 
  if (ciphertext_len == 0 || ciphertext_len % AES_EFFECTIVE_BLOCK_SIZE != 0) { 
    ArduinoCommsSerial.printf("ESP32-CAM (SharedSerial): Cipher len invalid (%u)\n", ciphertext_len); return "";} 
  unsigned char decrypted_padded_bytes[64]; memset(decrypted_padded_bytes, 0, sizeof(decrypted_padded_bytes));
  uint8_t currentIV[16]; memcpy(currentIV, original_aesIV, 16);
  uint16_t decrypted_padded_len = aesLib.decrypt(ciphertext_bytes, (uint16_t)ciphertext_len, 
                                                 decrypted_padded_bytes, aesKey, sizeof(aesKey) * 8, currentIV); 
  if (decrypted_padded_len == 0) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): AES Decrypt fail"); return "";} 
  if (decrypted_padded_len > ciphertext_len) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): AES Decrypt len error"); return "";} 
  int pad_val = (int)decrypted_padded_bytes[decrypted_padded_len - 1];
  if (pad_val <= 0 || pad_val > AES_EFFECTIVE_BLOCK_SIZE) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): Invalid PKCS7 pad val"); return ""; } 
  for (int i = 0; i < pad_val; i++) { if ((int)decrypted_padded_bytes[decrypted_padded_len - 1 - i] != pad_val) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): PKCS7 verify fail"); return ""; }} 
  int actual_data_len = decrypted_padded_len - pad_val;
  if (actual_data_len < 0) { ArduinoCommsSerial.println("ESP32-CAM (SharedSerial): Unpad error"); return "";} 
  String result = ""; for(int i=0; i<actual_data_len; i++){ result += (char)decrypted_padded_bytes[i]; }
  ArduinoCommsSerial.print("ESP32-CAM (SharedSerial): Decrypted: '"); ArduinoCommsSerial.print(result); ArduinoCommsSerial.println("'"); 
  return result;
}

//WiFi Event Handler
void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
        case WIFI_EVENT_STA_START:
            ArduinoCommsSerial.println("WiFi Station Started");
            break;
        case IP_EVENT_STA_GOT_IP:
            ArduinoCommsSerial.print("WiFi GOT IP: ");
            ArduinoCommsSerial.println(WiFi.localIP());
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            ArduinoCommsSerial.println("WiFi Disconnected from AP.");
            break;
        default:
            break;
    }
}


void setup() {
  ArduinoCommsSerial.begin(115200); 
  arduinoLogBuffer.reserve(64); 
  ArduinoCommsSerial.println("\nESP32-CAM Booting (Primary Serial, Quieter Debug, Dynamic Python IP)...");
  delay(1000); 

  //WiFi Connection
  ArduinoCommsSerial.println("\n--- WiFi Connection ---"); 
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_STA); delay(100);
  ArduinoCommsSerial.print("Connecting to: "); ArduinoCommsSerial.println(WIFI_SSID); // Shorter
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); 
  int connect_timeout = 30; 
  while (WiFi.status() != WL_CONNECTED && connect_timeout > 0) { delay(500); ArduinoCommsSerial.print("."); connect_timeout--;}
  ArduinoCommsSerial.println();
  if (WiFi.status() != WL_CONNECTED) {
    ArduinoCommsSerial.println("WiFi FAILED. Retrying...");
    while(WiFi.status() != WL_CONNECTED) { 
        ArduinoCommsSerial.print("R..."); delay(10000); WiFi.disconnect(true); delay(100); WiFi.mode(WIFI_STA); delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        int r_wait = 20; while(WiFi.status() != WL_CONNECTED && r_wait-- > 0) { delay(500); ArduinoCommsSerial.print(",");} ArduinoCommsSerial.println();
        if(WiFi.status() != WL_CONNECTED) ArduinoCommsSerial.println("R Fail."); else break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoCommsSerial.println(">>> WIFI CONNECTED! <<<");
    ArduinoCommsSerial.print("IP: "); ArduinoCommsSerial.println(WiFi.localIP());
    ArduinoCommsSerial.println("Waiting Python IP reg...");
  } else {
     ArduinoCommsSerial.println(">>> WIFI FINAL FAIL. Halting. <<<");
     while(true) delay(10000);
  }

  //mDNS Setup
  ArduinoCommsSerial.println("mDNS init...");
  if (!MDNS.begin("esp32-cam-tracker")) { ArduinoCommsSerial.println("mDNS fail!"); } 
  else { MDNS.addService("http", "tcp", 8081); ArduinoCommsSerial.println("mDNS OK: esp32-cam-tracker.local");}

  //Camera Initialization
  ArduinoCommsSerial.println("--- Cam Init ---");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM; config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM; config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM; config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM; config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM; config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM; config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000; config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA; config.jpeg_quality = 12; config.fb_count = 2; 
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) { ArduinoCommsSerial.printf("CamFail:0x%x\n", err); delay(10000); ESP.restart(); return; }
  ArduinoCommsSerial.println("Cam OK.");
  sensor_t * s = esp_camera_sensor_get();
  if (s == NULL) { ArduinoCommsSerial.println("SensorFail!"); while(true) delay(10000); } 
  else { ArduinoCommsSerial.println("Sensor OK.");}
  delay(500); 
  
  //Setup TCP/HTTP Command Server
  ArduinoCommsSerial.println("--- HTTP Server Init ---");
  commandServer.on("/", HTTP_GET, handleRoot); 
  commandServer.on("/command", HTTP_POST, handleCommand); 
  commandServer.on("/ping", HTTP_GET, handlePing); 
  commandServer.on("/signal_video_ready", HTTP_GET, handleSignalVideoReady); 
  commandServer.on("/register_python_client", HTTP_GET, handleRegisterPythonClient); 
  commandServer.begin();
  ArduinoCommsSerial.println("HTTP Server OK.");
  ArduinoCommsSerial.println("Waiting VideoReady sig...");
  ArduinoCommsSerial.println("Setup Done.");
  ArduinoCommsSerial.flush();
}

unsigned long lastFrameTime = 0;
const unsigned long frameInterval = 100; 
unsigned int udp_send_failures = 0;
unsigned long lastStatsPrintTime = 0; 
const unsigned long statsPrintInterval = 10000; // Print frame stats only every 10 seconds

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    commandServer.handleClient(); 
    if (pythonClientIPLearned && pythonClientReadyForVideo && (millis() - lastFrameTime > frameInterval)) {
      unsigned long t_start_capture = millis();
      camera_fb_t * fb = esp_camera_fb_get();
      unsigned long t_end_capture = millis();
      if (fb) {
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
                if (millis() - lastStatsPrintTime > statsPrintInterval) { 
                    ArduinoCommsSerial.println("UDP VidChunkFail!"); 
                }
                udp_send_failures++; frame_send_success = false; delay(5); break; 
            }
        }
        if (frame_send_success && bytesSent == fb->len) { 
            videoUdp.beginPacket(pythonClientIP, videoUdpPort); 
            videoUdp.write((const uint8_t*)END_OF_FRAME_MARKER, strlen(END_OF_FRAME_MARKER));
            videoUdp.endPacket();
        } else if (!frame_send_success) { 
            if (millis() - lastStatsPrintTime > statsPrintInterval) {
                 ArduinoCommsSerial.println("FrameSendIncomplete-NoEOF");
            }
        }
        unsigned long t_end_send = millis();
        if (millis() - lastStatsPrintTime > statsPrintInterval) {
            ArduinoCommsSerial.printf("F:%uB,%uc C:%lums S:%lums T:%lums NetF:%u PyIP:%s\n", 
                          fb->len, chunks, (t_end_capture - t_start_capture), (t_end_send - t_start_send), 
                          (t_end_send - t_start_capture), udp_send_failures, 
                          (pythonClientIPLearned ? pythonClientIP.toString().c_str() : "N/A") );
            lastStatsPrintTime = millis();
        }
        esp_camera_fb_return(fb); 
        lastFrameTime = millis();
      }
    }
    #ifndef DISABLE_UDP_LOG_FORWARDING
    if (pythonClientIPLearned) { 
        while (ArduinoCommsSerial.available() > 0) { 
          char receivedChar = ArduinoCommsSerial.read();
          arduinoLogBuffer += receivedChar;
          if (receivedChar == '\n') {
            arduinoLogBuffer.trim(); 
            if (arduinoLogBuffer.startsWith("LOG_START:") || arduinoLogBuffer.startsWith("LOG_STOP:")) {
              // ArduinoCommsSerial.print("ESP32 (SharedSerial RX): From Arduino: '"); // Avoid echo loop
              // ArduinoCommsSerial.print(arduinoLogBuffer); ArduinoCommsSerial.println("'");
              logUdp.beginPacket(pythonClientIP, logUdpPort); 
              logUdp.print(arduinoLogBuffer); 
              if(!logUdp.endPacket()){ /* ArduinoCommsSerial.println("UDP LogSendFail"); */ } 
            } 
            arduinoLogBuffer = ""; 
          }
          if (arduinoLogBuffer.length() > 60) { 
              // ArduinoCommsSerial.println("ESP32 LogBufOverflow"); // Keep this quiet
              arduinoLogBuffer = ""; 
          }
        }
    }
    #else 
    if (ArduinoCommsSerial.available() > 0) { while(ArduinoCommsSerial.available()) ArduinoCommsSerial.read(); }
    #endif
  } else { 
    static unsigned long lastReconnectAttempt = 0;
    if (millis() - lastReconnectAttempt > 30000) { 
        ArduinoCommsSerial.println("WiFi lost. Reconnecting..."); 
        WiFi.disconnect(true); delay(100); WiFi.mode(WIFI_STA); delay(100);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        lastReconnectAttempt = millis();
        ArduinoCommsSerial.println("Reconnect attempt initiated.");
    }
  }
  delay(1); 
}

void handleRoot() { 
  String html = "<html><body><h1>ESP32-CAM (Primary Serial, Quieter Debug)</h1>";
  html += "<p>Cmds: port 8081. GET /ping, /signal_video_ready, /register_python_client.</p>";
  html += "<p>UDP Vid to "; 
  if(pythonClientIPLearned) { html += pythonClientIP.toString(); } else { html += "[NoPyIP]"; }
  html += ":"; html += String(videoUdpPort); html += ".</p>";
  if (WiFi.status() == WL_CONNECTED) { html += "<p>IP: " + WiFi.localIP().toString() + "</p>"; }
  html += "<p>VidRdy: "; html += (pythonClientReadyForVideo ? "Y" : "N"); html += "</p>";
  html += "</body></html>";
  commandServer.send(200, "text/html", html);
}

void handleCommand(void) { 
  if (WiFi.status() != WL_CONNECTED) { commandServer.send(503, "text/plain", "NoWiFi"); return; }
  if (!commandServer.hasArg("plain")) { ArduinoCommsSerial.println("CmdNoBody!"); commandServer.send(400, "text/plain", "NoBody"); return; }
  String base64EncryptedCommand = commandServer.arg("plain");
  // ArduinoCommsSerial.print("EncCmd(len"); ArduinoCommsSerial.print(base64EncryptedCommand.length()); ArduinoCommsSerial.println(")"); 
  String decryptedCommand = decryptCommand(base64EncryptedCommand);
  if (decryptedCommand.length() == 0) { ArduinoCommsSerial.println("CmdDecryptFail"); commandServer.send(400, "text/plain", "DecryptFail"); return;}
  if (decryptedCommand.startsWith("S") && decryptedCommand.indexOf(':') > 0) {
    ArduinoCommsSerial.println(decryptedCommand); //Send to Arduino via PRIMARY Serial
    commandServer.send(200, "text/plain", "OK: " + decryptedCommand);
  } else {
    ArduinoCommsSerial.print("InvalidCmdFmt: '"); ArduinoCommsSerial.print(decryptedCommand); ArduinoCommsSerial.println("'");
    commandServer.send(400, "text/plain", "InvalidCmdFmt");
  }
}

void handlePing() { ArduinoCommsSerial.println("Pinged"); commandServer.send(200, "text/plain", "pong"); }
void handleSignalVideoReady() { ArduinoCommsSerial.println("VidRdySig"); pythonClientReadyForVideo = true; commandServer.send(200, "text/plain", "OK,VidStart");}
void handleRegisterPythonClient() {
  pythonClientIP = commandServer.client().remoteIP(); pythonClientIPLearned = true;
  String ipStr = pythonClientIP.toString();
  ArduinoCommsSerial.print("PyIPReg: "); ArduinoCommsSerial.println(ipStr);
  commandServer.send(200, "text/plain", "PyIPReg: " + ipStr);
  ArduinoCommsSerial.println("--- UDP Target Updated ---");
  ArduinoCommsSerial.print("UDP Vid To: "); ArduinoCommsSerial.print(pythonClientIP.toString()); ArduinoCommsSerial.print(":"); ArduinoCommsSerial.println(videoUdpPort);
  #ifndef DISABLE_UDP_LOG_FORWARDING
    ArduinoCommsSerial.print("UDP Log To: "); ArduinoCommsSerial.print(pythonClientIP.toString()); ArduinoCommsSerial.print(":"); ArduinoCommsSerial.println(logUdpPort);
  #endif
}
