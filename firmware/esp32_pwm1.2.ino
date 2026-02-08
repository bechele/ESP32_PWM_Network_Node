#include <WiFi.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Ethernet2.h> 
#include <stddef.h> // for offsetof
#include "utility/w5500.h"

#include <WiFiManager.h>
extern SPIClass SPIe;
extern SPISettings wiznet_SPI_settings;

//--------------- W5500 SPI Pins -----------------------------------
#define W5500_SCK   18
#define W5500_MISO  36                                             // Not default, but necessary to have 16 PWM output pins
#define W5500_MOSI  23
#define W5500_CS    5
#define W5500_RST   16

byte mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};                // Standard MAC

//------------- Network-configuration ------------------------------
#define FIRMWARE "Version 1.2"
#define UDP_PORT 7625
#define BROADCAST_IP "255.255.255.255"

//------------- EEPROM-Configuration -------------------------------
#define EEPROM_SIZE 512
#define NV_CONFIG_START 0

//---------- Configuration register Addresses ----------------------
#define REG_ETH_ADDR_1 1
#define REG_ETH_ADDR_2 2  nvConfig.nodeAddr = 0;

#define REG_ETH_ADDR_3 3
#define REG_ETH_ADDR_4 4
#define REG_ETH_ADDR_5 5
#define REG_ETH_ADDR_6 6
#define REG_NODE_ADDR 7
#define REG_PWM_COUNT 8
#define REG_BIN_OUT_COUNT 9
#define REG_PWM_START_WORD 10
#define REG_BIN_OUT_START_WORD_L 11
#define REG_BIN_OUT_START_WORD_H 12

//------------- W5500 Socket 0 Registers with BSB0=1 1-4=0 ---------
#define S0_MR       0x0000      // Socket 0 Mode Register
#define S0_CR       0x0001      // Socket 0 Command Register
#define S0_IR       0x0002      // Socket 0 Interrupt Register
#define S0_SR       0x0003      // Socket 0 Status Register
#define S0_PORT     0x0004      // Socket 0 Source Port (2 bytes)
#define S0_RX_RSR   0x0026      // Socket 0 Received Size Register (2 bytes)
#define S0_TX_WR    0x0024      // Socket 0 TX Write Pointer (2 Bytes)
#define S0_RX_RD    0x0028      // Socket 0 Read Pointer (2 bytes)

//------------- W5500 Common Registers with BSB0-4 =0 --------------
// #define MR       0x0000      // Mode Register - already defined in library
#define GAR         0x0001      // Gateway Address (4 bytes)
#define SUBR        0x0005      // Subnet Mask (4 bytes)
#define SHAR        0x0009      // Source Hardware Address (6 bytes) - MAC
#define SIPR        0x000F      // Source IP Address (4 bytes)
#define IR          0x0015      // Interrupt Register
#define IMR         0x0016      // Interrupt Mask Register
#define RTR         0x0017      // Retry Time Register (2 bytes)
#define RCR         0x0019      // Retry Count Register

//-------------- W5500 Control bytes -------------------------------
#define CB_WRITE_CR     0x0004  // Control Byte write common register
#define CB_WRITE_SR0    0x000C  // Control Byte write SR0 Register
#define CB_WRITE_TX0    0x0014  // Control Byte write TX buffer
#define CB_READ_CR      0x0000  // Control Byte Read common register
#define CB_READ_SR0     0x0008  // Control byte read SR0 register
#define CB_READ_RX0     0x0018  // Control Byte read RX buffer
#define CB_READ_CRS     0x0001  // Control byte read Common register using fixed data length

//--------------- Buffer -------------------------------------------
#define MAX_UDP_PACKET_SIZE 1472
uint8_t udpBuffer[MAX_UDP_PACKET_SIZE];

//--------------- RAM-Register -------------------------------------
#define REG_BLINK_ENABLE 128
#define REG_BLINK_DURATION 129
#define REG_RESET_NODE 130
#define REG_SEND_STATUS 131
#define REG_UNCONFIG_NODE 132

//--------------- Hardware-Pins ------------------------------------
#define CONFIG_JUMPER_PIN 35  // needs external pullup - complete config (MAC and node #)
#define WIFI_CONFIG_BUTTON 34 // needs external pullup - starts Wifi Manager in AP mode
#define STATUS_LED_PIN 2

//--------------- Global Variables ---------------------------------
WiFiUDP udp;
uint8_t ethBuffer[1500];
bool useEthernet = false;
bool configMode = false;                                                             // Basic-config (MAC/WiFi)
bool nodeAddrConfig = false;                                                         // Node adress config
bool pwm_toggle = false;
static bool spiInitialized = false;
unsigned long lastBlinkTime = 0;
bool blinkingActive = false;
bool nodeReady = false;
bool netOK = false;
static uint16_t statusCounter = 0;                                                   // Send counter - only used to judge on the host side if packets have been lost.
IPAddress lastSenderIP;
bool nv_changed=false;

//---------- forward declarations ----------------------------------
bool initW5500Raw();
bool initRawUDPSocket();
void updateBlinking();
void saveIPConfigAfterDHCP();
int parseRawUDPPacket();
void processConfigCommand(uint16_t* data, int wordCount);
void processDataCommand(uint16_t* data, int wordCount);
void SendStatus();
void sendRawUDP(IPAddress destIP, uint16_t destPort, uint8_t* data, uint16_t len);
void sendWiFiUDP(IPAddress destIP, uint16_t destPort, uint8_t* data, uint16_t len);

struct IPConfig {
  uint8_t mac[6];
  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;
};

IPConfig ethConfig;

  struct NVConfig {
    uint8_t ethAddr[6];
    uint16_t nodeAddr;
    uint16_t pwmCount;          // Number of PWM ports in this node
    uint16_t pwmStartWord;      // Here starts the PWM data in the stream for this node
    uint16_t pwmStartbit;       // Here we bind the first Servo for this node
    uint16_t binOutCount;       // Number of binary ports in this node
    uint16_t binOutStartWord;   // Here starts the bit data for this node
    uint16_t binOutStartbit;    // Tells the start bit where digital ports start Range: 0-15 - means, digital outputs may be mixed with PWM outputs - PWM first 0->X, then Binary X->15
    uint16_t dummy1;            // ... a few further known register place holders to keep CRC intact in case of changes
    uint16_t dummy2;
    uint16_t dummy3;
    uint16_t dummy4;
    uint16_t dummy5;
    uint16_t dummy6;
    uint16_t checksum;
  };
//---------------- PWM Config --------------------------------------
NVConfig nvConfig;
uint16_t ramRegisters[256];                                                          // PWM-Configuration - RAM-Register
#define MAX_PWM_OUTPUTS 16                                                           //16 Pins
#define PWM_FREQUENCY 50
#define PWM_RESOLUTION 12                                                            // 12 bit =4096 steps
uint8_t pwmPins[MAX_PWM_OUTPUTS] = {
  32,33,25,26,27,14,12,13,15,17,0,4,19,21,3,22                                       // Ports in order according to the wiring of the PCB
};
//==================================================================
// Setup after boot
//==================================================================
void setup() {
  Serial.begin(115200);
  pinMode(W5500_RST, OUTPUT);
  pinMode(W5500_SCK, OUTPUT);
  pinMode(W5500_CS, OUTPUT);
  pinMode(W5500_MISO, INPUT);
  pinMode(W5500_MOSI, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  initEEPROM();
    Serial.println(FIRMWARE);
    Serial.print("Ethernet Address in NV: ");
  for (int i=0;i < 6;i++){
    Serial.print(nvConfig.ethAddr[i],HEX);                                           // show ethernet address in NV memory
    Serial.print(" ");
  }
  Serial.println();
  Serial.print("Node Adress: ");
  Serial.println(nvConfig.nodeAddr);
  initNetwork();                                                                     // in case of ethernet only use it to get an address via DHCP    
  if (useEthernet) {                                                                 //set up for the raw W5500 communication
    saveIPConfigAfterDHCP();                                                         // save IP settings for use with W5500 raw communication
    if (initW5500Raw() && initRawUDPSocket()){ 
      useEthernet=true;
      applyIPConfigToW5500();
    } else {
      useEthernet=false;
    }
  }
  if (WiFi.status() == WL_CONNECTED || useEthernet ) {netOK=true;} 
  initPullDowns();                                                                   // for PWM pins
  Serial.println("Start ESP32 PWM-Node");
  initPWM();
  Serial.print("configuration mode: ");
  Serial.println(configMode ? "ACTIVE" : "INACTIVE");
  if (nvConfig.nodeAddr > 0 && netOK==true) {
    Serial.print("Turn on the LED - Node address is set to: ");
    Serial.println(nvConfig.nodeAddr);
    digitalWrite(STATUS_LED_PIN, HIGH);                                              // Show that node has a node address
    nodeReady=true;
  } 
  if (!nodeReady) {
    Serial.println("Node is !!!!! NOT !!!!! ready for operation ");
    Serial.print("Network ready status is: ");
    Serial.println(netOK);
  }
}
//==================================================================
// our main loop
//==================================================================
void loop() {
    if (useEthernet) {
      maintainSocket();  // frequently do mainenance
    } else { 
      maintainWiFiSocket();
    }
    processUDPPackets();
    updateBlinking();
    delay (5);                                                                       // keep the CPU cool
}
//==================================================================
// Mini keepalive to prevent of socket timeouts
//==================================================================
void maintainSocket() {
    static unsigned long lastMaintain = 0;
    if (millis() - lastMaintain < 60000) return;
    
    // Mini-Ping to ourself (127.0.0.1)
    uint8_t ping[] = {0x00};
    sendRawUDP(IPAddress(127,0,0,1), UDP_PORT, ping, 1);
    uint8_t sr = readW5500(S0_SR, 0x08);
    if (sr != 0x22) {  // Nicht im UDP-Modus
        Serial.print("Socket inactive - reinit. Status: 0x");
        Serial.println(sr, HEX);
        // carefully close and re-open
        writeW5500(S0_CR, CB_WRITE_SR0, 0x10);  // CLOSE
        delay(5);
        initRawUDPSocket();
    }
    lastMaintain = millis();
}
//------------------------------------------------------------------
void maintainWiFiSocket() {
    static unsigned long lastWiFiMaintain = 0;
    if (millis() - lastWiFiMaintain < 60000) return;
    
    // Mini-Ping to Broadcast
    uint8_t ping[] = {0x00};
    udp.beginPacket("255.255.255.255", UDP_PORT);
    udp.write(ping, 1);
    udp.endPacket();
    lastWiFiMaintain = millis();
}
//==================================================================
// EEPROM-Functions
//==================================================================
void initEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  loadNVConfig();                                                                    // load or set default values
  if (nvConfig.checksum != calculateChecksum()) {                                    // Check if the config is valid
    // set standard values
    Serial.println("NV checksum is invalid!");
    setDefaultConfig();
  }
}
//------------------------------------------------------------------
// Load the NV memory into struct 
//------------------------------------------------------------------
void loadNVConfig() {
  EEPROM.get(NV_CONFIG_START, nvConfig);
}
//------------------------------------------------------------------
// Save the NV memory to Chip after a change
//------------------------------------------------------------------
void saveNVConfig() {
  nvConfig.checksum = calculateChecksum();
  EEPROM.put(NV_CONFIG_START, nvConfig);
  EEPROM.commit();
}
//------------------------------------------------------------------
// calc the NV memory checksum
//------------------------------------------------------------------
uint16_t calculateChecksum() {
  uint16_t sum = 0;
  uint16_t* words = (uint16_t*)&nvConfig;
  int numWords = (sizeof(nvConfig) / sizeof(uint16_t)) - 1; // Ohne checksum
  
  for (int i = 0; i < numWords; i++) {
    sum += words[i];
  }
  return sum;
}
//------------------------------------------------------------------
// Init the eeprom to default
//------------------------------------------------------------------
void setDefaultConfig() {
  memset(nvConfig.ethAddr, 0, 6);                                                    // leave MAC completely empty -> 0x00,0x00,0x00,0x00,0x00,0x00
  nvConfig.nodeAddr = 0;
  nvConfig.pwmCount = 0;
  nvConfig.pwmStartWord = 0;
  nvConfig.pwmStartbit = 0;
  nvConfig.binOutCount = 0;
  nvConfig.binOutStartWord = 0;
  nvConfig.binOutStartbit = 0;
  saveNVConfig();                                                                    // ... other default values
  Serial.println("NV Configuration - set all values to default");
}
//==================================================================
// Stop execution - Wait for a reset
//==================================================================
void stopExec() {
  while (1) { delay (1000); }                                                        // stay here and wait for a reset
}
//==================================================================
// Network-Init
//==================================================================
void initNetwork() {
  pinMode(CONFIG_JUMPER_PIN, INPUT);
  pinMode(WIFI_CONFIG_BUTTON, INPUT);
  configMode = (digitalRead(CONFIG_JUMPER_PIN) == LOW);
  bool wifiConfigMode = (digitalRead(WIFI_CONFIG_BUTTON) == LOW);
  checkEthernet();
  if (useEthernet) {                                                                 // Ethernet has Priority when available - means does not start wifi
    initEthernet(configMode);
  } else {                                                                           // No Ethernet -> WiFi  
    if (wifiConfigMode) {
      startWiFiConfigMode();                                                         // start the configuration mode and stay there until the wifi config jumper is removed
    } else {                                                                         // initMAC(configMode);
      connectToWiFi();                                                               // Normal operation using saved WiFi credentials
    }
  }
}
//------------------------------------------------------------------
// check, if a W5500 is connected to the ESP
//------------------------------------------------------------------
void checkEthernet() {                                                               //Tells if the unit has a Ethernet W5500 Module
  digitalWrite(W5500_RST, LOW);                                                      // Hardware Reset
  delay(100);
  digitalWrite(W5500_RST, HIGH);
  delay(1000);
  SPIe.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);                           // SPI Initialisieren
  digitalWrite(W5500_CS, HIGH);                                                      // CS inaktiv
  SPIe.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(W5500_CS, LOW);                                                       // read W5500 version register ( Register 039 )
  SPIe.transfer(0x00);                                                               // Read command + Adress-High (nur 0x0F für High-Byte)
  SPIe.transfer(0x39);                                                               // Adress-Low
  SPIe.transfer(0x01);           
  byte version = SPIe.transfer(0x00);
  digitalWrite(W5500_CS, HIGH);
  SPIe.endTransaction();
  SPIe.end();
  if (version == 0x04) {
    Serial.println("W5500 hardware found - Version: 0x04");
    useEthernet = true;
  } else {
    Serial.println("W5500 hardware not found - use WiFi");
    useEthernet = false;
  }
}
//------------------------------------------------------------------
// check for existing MAC and use default MAC in basic config mode
//------------------------------------------------------------------
bool initMAC(bool configMode) {
  if (configMode) {                                                                  // choose MAC
    if (!hasStoredMAC()) {
      byte fixedMac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01};                       // Config-Mode and no saved MAC -> use temporary default MAC
      memcpy(mac, fixedMac, 6);
      Serial.print("Use temporary default MAC during configuration: ");
      for(int i=0; i<6; i++) {
        Serial.print(mac[i], HEX); 
        if(i < 5) Serial.print(":");
      }
      Serial.println();
    }
  } else {
    if (hasStoredMAC()){                                                             // if a stored MAC exists use this in any case
      memcpy(mac, nvConfig.ethAddr, 6);
      Serial.print("use saved MAC: ");
    } else {
      byte noMac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      memcpy(mac, noMac, 6);
      Serial.println("No saved MAC Address - please set jumper and MAC: ");
      return 0;
    }
    Serial.print("MAC: ");                                                           // Print MAC on serial monitor
    for(int i=0; i<6; i++) {
      Serial.print(mac[i], HEX); 
      if(i < 5) Serial.print(":");
    }
    Serial.println();
  }  
  return 1;
}
//------------------------------------------------------------------
// Init the W5500 ethernet Module
//------------------------------------------------------------------
void initEthernet(bool configMode) {
  if (! initMAC(configMode)) {
    useEthernet=false;
    return;                                                                          // MAC setting failed
  }
  digitalWrite(W5500_RST, LOW);
  delay(100);
  digitalWrite(W5500_RST, HIGH);
  delay(1000);
  Serial.println("Start DHCP...");                                                   // Run DHCP 
  if (Ethernet.begin(mac) == 0) {
    Serial.println("init: DHCP failed - got no IP");
    useEthernet = false; 
    return; 
  }
  delay(2000);                                                                       // Wait a moment and check IP
  IPAddress ip = Ethernet.localIP();
  Serial.print("got IP Address: ");
  Serial.println(ip);
  if (ip == INADDR_NONE || ip[0] == 0 || ip[0] == 96) {                              // Check for a valid IP
    Serial.println("initEthernet: got invalid IP");
    useEthernet = false;
    return;
  }
  Serial.println("Ethernet successfully initialized!");
  WiFi.mode(WIFI_OFF);
  useEthernet = true;
  return;
}
//------------------------------------------------------------------
// Ckecks if a stored MAC exists
//------------------------------------------------------------------
bool hasStoredMAC() {                                                                // Checks if a valid IP is saved (not 00:00:00:00:00:00)
  for(int i = 0; i < 6; i++) {
    if(nvConfig.ethAddr[i] != 0x00) return true;
  }
  Serial.println("Oops - seems that we have no stored Ethernet in NV memory");
  return false;
}
//==============================================================================
// Wifi handling
//==============================================================================
void startWiFiConfigMode() {
  Serial.println("Start WiFi Configuration-Mode ...");
  digitalWrite(STATUS_LED_PIN, HIGH);
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);                                                    // 3 Minutes Timeout
  if (!wm.startConfigPortal("ESP32_PWM_Node")) {
    Serial.println("Config Portal failed - restarting");
    ESP.restart();
  }
  Serial.println("WiFi configured and connected!");
  digitalWrite(STATUS_LED_PIN, LOW);
}
//------------------------------------------------------------------
// Connect using stored wifi setup
//------------------------------------------------------------------
void connectToWiFi() {
  WiFi.mode(WIFI_STA);  
  Serial.print("Connect with saved WiFi...");                                        // connect with stored Credentials
  WiFi.begin();
  for (int i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
    startWiFiUDP();
  } else {
    Serial.println("\nWiFi connection failed");                                      // Kein automatischer Fallback!
  }
}
void startWiFiUDP() {
  if (udp.begin(UDP_PORT)) {
    Serial.print("WiFi UDP started on port ");
    Serial.println(UDP_PORT);
    udp.beginPacket("255.255.255.255", UDP_PORT);                                    // allow to receive Broadcasts
    udp.endPacket();
  } else {
    Serial.println("WiFi UDP start failed");
  }
}
//------------------------------------------------------------------
// Send via wifi UDP 
//------------------------------------------------------------------
void sendWiFiUDP(IPAddress destIP, uint16_t destPort, uint8_t* data, uint16_t len) {
  udp.beginPacket(destIP, destPort);
  udp.write(data, len);
  Serial.print("Sent a data packet via Wifi to Port:/IP: ");
  Serial.print(destPort);
  Serial.print(" ");
  Serial.println(destIP);
  udp.endPacket();
}
//==================================================================
// PWM-Initializing and port init
//==================================================================
void initPWM() {
  for (int i = nvConfig.pwmStartbit; i < (nvConfig.pwmCount+nvConfig.pwmStartbit) && i < MAX_PWM_OUTPUTS; i++) {
    analogWriteFrequency(pwmPins[i],PWM_FREQUENCY);                                  // 50Hz for Servos
    analogWriteResolution(pwmPins[i],PWM_RESOLUTION);
    pinMode(pwmPins[i], OUTPUT);
    analogWrite(pwmPins[i], 0);
  }
}
void setPWMOutput(uint8_t channel, uint16_t value) {
  if (channel < (nvConfig.pwmCount+nvConfig.pwmStartbit) && value <= 4095) {
    analogWrite(pwmPins[channel], value);
  }
}
void setBinOutput(uint8_t channel, bool value){
  for (int i = nvConfig.binOutStartbit; i < (nvConfig.binOutCount+nvConfig.binOutStartbit) && i < MAX_PWM_OUTPUTS; i++) {
    pinMode(pwmPins[i], OUTPUT);
  }
  digitalWrite(pwmPins[channel], value);
}  
void initPullDowns() {
  for(int i = 0; i < MAX_PWM_OUTPUTS; i++) {
    pinMode(pwmPins[i], OUTPUT);
    digitalWrite(pwmPins[i], LOW);                                                   // safe state
  }
}
//==================================================================
// UDP-Packet-processing - wifi and Ethernet
//==================================================================
void processUDPPackets() {
  int packetSize;
  if (useEthernet) {
    Ethernet.maintain();
    packetSize = parseRawUDPPacket();
    if(packetSize > 0) {
      processDatagram(udpBuffer, packetSize);                                        // udpBuffer contains only user data (without W5500 Header)
    }
  } else {
    packetSize = udp.parsePacket();
    if (packetSize) {
      lastSenderIP=udp.remoteIP();
      int len = udp.read(ethBuffer, sizeof(ethBuffer)); 
      if (len >= 4) {
        processDatagram(ethBuffer, len);
      }
    }
  }
 
}
//-----------------------------------------------------------------
// calculate the CRC of the received data
//-----------------------------------------------------------------
uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for(size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for(uint8_t bit = 0; bit < 8; bit++) {
            if(crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
//----------------------------------------------------------------
// process the recieved data , check for CRC and change Byte order
//----------------------------------------------------------------
void processDatagram(uint8_t* data, int length) {
  if(length < 2) {                                                                   // 1. check CRC (last 2 Bytes are CRC)
    Serial.println("Packet too short - packet discarded");
    return;                                                                          // Too short for CRC
  }
  uint16_t received_crc = data[length-2] | (data[length-1] << 8);
  uint16_t calculated_crc = crc16(data, length - 2);
  
  if(received_crc != calculated_crc) {
    Serial.println("CRC error - packet discarded");
    return;                                                                          // Paket verwerfen
  }
  for(int i = 0; i < length - 2; i += 2) {                                           // 2. Byte-Swap for 16-bit Words (if W5500 delivers Little-Endian)
    uint8_t temp = data[i];
    data[i] = data[i + 1];
    data[i + 1] = temp;
  }
  uint16_t* wordData = (uint16_t*)data; 
  uint16_t lengthWord = wordData[0];
  uint16_t counter = wordData[1];
  if (lengthWord & 0x8000) {                                                         // configuration command?
    processConfigCommand(wordData, length/2-1);                                      // cut off crc
  } else {
    processDataCommand(wordData, length/2-1);                                        //cut off crc
  }
}
//--------------------------------------------------------------------
// stop the PWM output temporarily - this command will be issued after a session
//--------------------------------------------------------------------
void pausePWM() {
  for (int i = nvConfig.pwmStartbit; i < (nvConfig.pwmCount+nvConfig.pwmStartbit) && i < MAX_PWM_OUTPUTS; i++) {
    setPWMOutput(i, 0);
  }
}
//--------------------------------------------------------------------
// if the data received was a PWM packet, set the PWM pins accordingly
//--------------------------------------------------------------------
void processDataCommand(uint16_t* data, int wordCount) {
  uint16_t lengthWord = data[0];
  if (nvConfig.pwmCount > 0) {                                                       // Process PWM-data
    uint16_t startWord = nvConfig.pwmStartWord+2;                                    // +1 = start at first real data word - means omit lengthword and counter
    for (int i = 0; i < nvConfig.pwmCount; i++) {
      if ((startWord + i) < wordCount) {
        uint16_t pwmValue = data[startWord + i];
        if (pwmValue <= 4095) {
          setPWMOutput(i+nvConfig.pwmStartbit, pwmValue);
        } else {
          setPWMOutput(i+nvConfig.pwmStartbit, 4095);
        }
      }
    }
  }
  if (nvConfig.binOutCount > 0) {
    uint16_t mask;
    for (int i=0;i<nvConfig.binOutCount;i++) {
      mask= 1 << i;                                                                  // set the bitmask to define which pin is to be processed
      setBinOutput(i+nvConfig.binOutStartbit,data[nvConfig.binOutStartWord+2] & mask); //set the bit pattern according to data word
    }
  }
}
//==================================================================
// Configuration commands
//==================================================================
void processConfigCommand(uint16_t* data, int wordCount) {                           // determine the type of configuration and pass the data to the proper function
  uint16_t command = data[0];
  if (command == 32768 && configMode) {                                              // Basic-Configuration (length word = 32768)
    processBaseConfig(data, wordCount);                                              // Program will reboot within processBaseConfig
  }
  if (command == 65535) {                                                            // this is the disabling of the PWM ithat goes to all nodes - usually issued after a session
    pausePWM();
    return;
  }
    if (command == 65534) {                                                          // general reboot that goes to all nodes received, so do that - may be called in fault cases 
      digitalWrite(STATUS_LED_PIN, LOW);
      ESP.restart();
    return;
  }
  if ((command-32768) == nvConfig.nodeAddr) {                                        // node-specific configuration Node address in packet is coded as 32768 + nodeaddress
    Serial.print("Node specific command received: ");                                // therefore we have to subtact 32768 from the first byte in the packet
    for (int i=0; i < wordCount; i++) {                                              // show config data on screen
      Serial.print(data[i]);
      Serial.print(" ");
    }
    Serial.println();
    processNodeConfig(data, wordCount); 
  }                                                                                  // else -> this command is not intended for this node - so silently ignore it
}
//-------------------------------------------------------------------
// Do a basic config - means set the MAC address and the node number
//-------------------------------------------------------------------
void processBaseConfig(uint16_t* data, int wordCount) {
  if (wordCount < 9) return;                                                         // at least 7 data words + checksum
  for (int i = 0; i < 6; i++) {                                                      // Ethernet-Adress out of the first 6 Data words
    nvConfig.ethAddr[i] = (uint8_t)(data[2 + i] & 0xFF);
  }
  nvConfig.nodeAddr = data[8];                                                       // node address
  saveNVConfig();
  Serial.println("Base config ready - wait for jumper removal - reboots automatically");
  while (digitalRead(CONFIG_JUMPER_PIN) == LOW) {delay (500);}                       // do a reboot as soon as the jumper pin is removed otherwise stay in loop
  ESP.restart();                                                                     // restart
}
//-------------------------------------------------------------------
// Set the register defined in the configuration command
//-------------------------------------------------------------------
void processNodeConfig(uint16_t* data, int wordCount) {
  Serial.print("wordcount: ");
  Serial.println(wordCount);
  if (wordCount >= 4) {                                                              // data 0 and 1 are nodenumber and counter
    nv_changed=0;
    for (int i=0;i < (wordCount-2); i=i+2) {                                         // Do multiple register commands - always 2 words are a register command
      uint16_t regAddr = data[i+2];
      uint16_t registerValue = data[i+3];
      if (regAddr == 1) {
        nvConfig.pwmCount=registerValue;
        nv_changed=1;
      } else if ( regAddr==2 ){
        nvConfig.pwmStartWord=registerValue;
        nv_changed=1;
      } else if ( regAddr==3){
        nvConfig.pwmStartbit=registerValue;
        nv_changed=1;
      } else if ( regAddr==4 ){
        nvConfig.binOutCount=registerValue;
        nv_changed=1;
      } else if ( regAddr==5 ){
        nvConfig.binOutStartWord=registerValue;
        nv_changed=1;
      } else if ( regAddr==6 ){
        nvConfig.binOutStartbit=registerValue;
        nv_changed=1;
      } else if ( regAddr == REG_UNCONFIG_NODE && registerValue & 0x0001) {          // Command to unconfigure the node completely
        setDefaultConfig();
        digitalWrite(STATUS_LED_PIN, LOW);
        ESP.restart();
      } else if (regAddr == REG_RESET_NODE && registerValue & 0x0001) {
        digitalWrite(STATUS_LED_PIN, LOW);
        ESP.restart();
      } else if (regAddr == REG_BLINK_ENABLE) {
        ramRegisters[REG_BLINK_ENABLE]=registerValue;
      } else if (regAddr == REG_BLINK_DURATION) {
          ramRegisters[REG_BLINK_DURATION]=registerValue;
      } else if (regAddr == REG_SEND_STATUS) {
        if (nv_changed) {saveNVConfig();delay(800);nv_changed=0;}
        SendStatus();
      }
    } 
  if (nv_changed) {saveNVConfig();delay(800);nv_changed=0;}   
  }
}
//==================================================================
// Send the status of the node back to the calling host
//==================================================================
void SendStatus() {
  uint16_t destPort = UDP_PORT;                                                      // same port like receive
  uint16_t response[32];                                                             // Build response - MAX 32 words (64 bytes)
  uint8_t responseIndex = 0;
  response[responseIndex++] = 0;                                                     // 1. Length word (will be set later)
  response[responseIndex++] = statusCounter++;                                       // 2. Counter
  response[responseIndex++] = nvConfig.nodeAddr;                                     // 3. Node address
  response[responseIndex++] = nvConfig.pwmCount;                                     // 4. PWM Count
  response[responseIndex++] = nvConfig.pwmStartWord;                                 // 5. PWM Start Word
  response[responseIndex++] = nvConfig.pwmStartbit;                                 // 5. PWM Start bit
  response[responseIndex++] = nvConfig.binOutCount;                                  // 6. BinOut Count
  response[responseIndex++] = nvConfig.binOutStartWord;                              // 7. BinOut Start Word
  response[responseIndex++] = nvConfig.binOutStartbit;                               // 8. Binout Start bit
  response[0] = responseIndex;                                                       // Now set the length word (number of words including length word itself)
  uint8_t sendBuffer[64];                                                            // Convert to byte array with proper byte order Max 32 words * 2 bytes
  int byteIndex = 0;
  for (int i = 0; i < responseIndex; i++) {                                          // 1. First, copy all words to byte array with byte-swap (ESP32 is little-endian)
    sendBuffer[byteIndex++] = (response[i] >> 8) & 0xFF;  // High byte first         // Byte-swap: little-endian to network (big-endian) order
    sendBuffer[byteIndex++] = response[i] & 0xFF;         // Low byte second
  }
  uint16_t crc = crc16(sendBuffer, byteIndex);  // 2. Calculate CRC on the byte-swapped data
  sendBuffer[byteIndex++] = (crc >> 8) & 0xFF;  // 3. Append CRC (also byte-swapped for network order) - CRC high byte
  sendBuffer[byteIndex++] = crc & 0xFF;         // CRC low byte
  Serial.print("Sending status response: ");                                         // Debug output
  Serial.print(byteIndex);
  Serial.println(" bytes");
  Serial.print("CRC: 0x");
  Serial.println(crc, HEX);
  // Send via appropriate interface
  if (useEthernet) {                                    
    sendRawUDP(lastSenderIP, destPort, sendBuffer, byteIndex);
  } else {
    sendWiFiUDP(lastSenderIP, destPort, sendBuffer, byteIndex);
  }
}
//==================================================================
// LED-blink-functionality
//==================================================================
void updateBlinking() {
  bool blinkEnabled = (ramRegisters[REG_BLINK_ENABLE] & 0x0001);
  if (!blinkEnabled) {
    if (blinkingActive) {
      digitalWrite(STATUS_LED_PIN, (nvConfig.nodeAddr > 0) ? HIGH : LOW);            // Blinking was just disabled -> LED to Node-Status
      blinkingActive = false;
    }
    return;
  }
  blinkingActive = true;                                                             // Blinking active
  unsigned long blinkInterval = ramRegisters[REG_BLINK_DURATION];
  if (blinkInterval == 0) blinkInterval = 250;                                       // Default 0,5 Hz
  if (millis() - lastBlinkTime >= blinkInterval) {
    bool currentState = digitalRead(STATUS_LED_PIN);
    digitalWrite(STATUS_LED_PIN, !currentState);
    lastBlinkTime = millis();
  }
}
//==================================================================
// SPI Raw access to get broadcast and UDP in ethernet mode - 
// necessary since Ethernet2 does not support UDP :-(
//==================================================================
void writeW5500(uint16_t addr, uint8_t cb, uint8_t data) {
    SPIe.beginTransaction(wiznet_SPI_settings);
    digitalWrite(W5500_CS, LOW);
    SPIe.transfer(addr >> 8);
    SPIe.transfer(addr & 0xFF);
    SPIe.transfer(cb);                                                               // vorgegebenes Control Byte
    SPIe.transfer(data);
    digitalWrite(SPI_CS, HIGH);
    SPIe.endTransaction();
}
//----------------------------------------------------------------
uint8_t readW5500(uint16_t addr, uint8_t cb) {
    SPIe.beginTransaction(wiznet_SPI_settings);
    digitalWrite(W5500_CS, LOW);
    SPIe.transfer(addr >> 8);
    SPIe.transfer(addr & 0xFF);
    SPIe.transfer(cb);                                                               // Original aus Datei: cb unverändert
    uint8_t data = SPIe.transfer(0);
    digitalWrite(SPI_CS, HIGH);
    SPIe.endTransaction();
    return data;
}
void writeW5500Bytes(uint16_t addr, uint8_t cb, const uint8_t* data, uint16_t len) {
    Serial.print("writeW5500Bytes - addr: 0x");
    Serial.print(addr, HEX);
    Serial.print(", cb: 0x");
    Serial.print(cb, HEX);
    Serial.print(", len: ");
    Serial.println(len);
    SPIe.beginTransaction(wiznet_SPI_settings);
    digitalWrite(W5500_CS, LOW);
    SPIe.transfer(addr >> 8);
    SPIe.transfer(addr & 0xFF);
    SPIe.transfer(cb);
    for(uint16_t i = 0; i < len; i++) {
        SPIe.transfer(data[i]);
    }
    digitalWrite(W5500_CS, HIGH);
    SPIe.endTransaction();
}
//----------------------------------------------------------------
void readW5500Bytes(uint16_t addr, uint8_t cb, uint8_t* data, uint16_t len) {
    SPIe.beginTransaction(wiznet_SPI_settings);
    digitalWrite(SPI_CS, LOW);
    SPIe.transfer(addr >> 8);
    SPIe.transfer(addr & 0xFF);
    SPIe.transfer(cb);
    for(uint16_t i = 0; i < len; i++) {
        data[i] = SPIe.transfer(0);
    }
    digitalWrite(SPI_CS, HIGH);
    SPIe.endTransaction();
}
//----------------------------------------------------------------
bool initW5500Raw() {                                                                // 1. Hardware Reset
  pinMode(W5500_RST, OUTPUT);
  digitalWrite(W5500_RST, LOW);
  delay(100);
  digitalWrite(W5500_RST, HIGH);
  delay(1000);
  SPIe.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);                           // 2. re- initialize W5500
  digitalWrite(W5500_CS, HIGH);
  writeW5500(0x0000, CB_WRITE_CR, 0x80);                                             // 3. MR: Soft Reset
  delay(10);                                                                         // Wait until Reset ready
  uint8_t cntl_byte = 0x0C;                                                          // Set buffer-sizes for Socket 0 (2KB RX, 2KB TX) - Socket 0 Register Block, Write
  writeW5500(0x1E, cntl_byte, 0x02);                                                 // Sn_RXBUF_SIZE = 2KB
  writeW5500(0x1F, cntl_byte, 0x02);                                                 // Sn_TXBUF_SIZE = 2KB
  delay(1);
  uint8_t rx_size = readW5500(0x001E, 0x08);                                         // Optional: read back and output
  uint8_t tx_size = readW5500(0x001F, 0x08);
  Serial.print("RX Buffer Size: ");
  Serial.print(rx_size); Serial.println(" (0=1KB, 1=2KB, 2=4KB, 3=8KB, 4=16KB)");
  Serial.print("TX Buffer Size: ");
  Serial.print(tx_size); Serial.println(" (0=1KB, 1=2KB, 2=4KB, 3=8KB, 4=16KB)");
  uint8_t version = readW5500(0x0039, 0x01);                                         // 4. Test- read Version (should be 0x04) read Version Register
  Serial.print("W5500 Version: 0x");
  Serial.println(version, HEX);
  return (version == 0x04);
}
//----------------------------------------------------------------
bool initRawUDPSocket() {
  writeW5500(S0_CR, CB_WRITE_SR0, 0x10);                                             // 1. close Socket (if open) - CLOSE command (0x10)
  delay(10);                                                                         // 2. wait for SOCK_CLOSED
  uint8_t status;
  do {
    status = readW5500(S0_SR, CB_READ_SR0);                                          // read Status of socket 0
    delay(1);
  } while(status != 0x00);                                                           // 0x00 = SOCK_CLOSED
  writeW5500(S0_MR, CB_WRITE_SR0, 0x02);                                             // 3. Set Socket to UDP mode - UDP mode (0x02)
  writeW5500(S0_PORT, CB_WRITE_SR0, (UDP_PORT >> 8) & 0xFF);                         // 4. Set Port (7625) - High byte
  writeW5500(S0_PORT + 1, CB_WRITE_SR0, UDP_PORT & 0xFF);                            // Low byte
  writeW5500(S0_CR, CB_WRITE_SR0, 0x01);                                             // 5. Open Socket-  OPEN command (0x01)
  delay(10);
  writeW5500(S0_TX_WR, 0x0C, 0x00);                                                  //6. Setze TX Write Pointer auf 0 -  High byte 
  writeW5500(S0_TX_WR + 1, 0x0C, 0x00);                                              // Low byte
  status = readW5500(S0_SR, CB_READ_SR0);                                            // 7. check Status
  Serial.print("S0_SR Status: 0x");
  Serial.println(status,HEX);
  if(status == 0x22) {                                                               // 0x22 = SOCK_UDP
    Serial.print("Raw UDP socket opened on port:");
    Serial.println(UDP_PORT);
    return true;
  } else {
    Serial.print("Failed to open UDP socket. Status: 0x");
    Serial.println(status, HEX);
    return false;
  }
}
//----------------------------------------------------------------
// Check for incoming UDP packets
int parseRawUDPPacket() {
  uint8_t status = readW5500(S0_SR, CB_READ_SR0);                                    // 1. check Status - Socket 0 Status (Block1, Read)
  if(status != 0x22) return 0;
  uint8_t ir = readW5500(S0_IR, CB_READ_SR0);                                        // 2. check Interrupt - Socket 0 IR (Block1, Read)
  if(!(ir & 0x04)) return 0;
  uint16_t size = readW5500(S0_RX_RSR, CB_READ_SR0) << 8;                            // 3. Read size (2 Byte)
  size |= readW5500(S0_RX_RSR + 1, CB_READ_SR0);
  if(size == 0 || size > MAX_UDP_PACKET_SIZE + 8) {
    writeW5500(S0_CR, CB_WRITE_SR0, 0x40);                                           // 4. Send RECV-command to delete Buffer
    writeW5500(S0_IR, CB_WRITE_SR0, 0x04);                                           // 5. delete interrupt - delete RECV Interrupt (Write)
    return 0;
  }
  uint16_t rx_rd = readW5500(S0_RX_RD, CB_READ_SR0) << 8;                            // 6. get read pointer (2 Byte)
  rx_rd |= readW5500(S0_RX_RD + 1, CB_READ_SR0); 
  readW5500Bytes(rx_rd, CB_READ_RX0, udpBuffer, size);                               // 7. read data from this offset into rx buffer - rx_rd is Offset in Buffer!
  lastSenderIP = IPAddress(udpBuffer[0], udpBuffer[1], udpBuffer[2], udpBuffer[3]);
  memmove(udpBuffer, udpBuffer + 8, size - 8);                                       // 8. Remove header, move data to front
  uint16_t new_rx_rd = rx_rd + size;                                                 // 9. update Read Pointer (rx_rd + size)
  writeW5500(S0_RX_RD, 0x0C, (new_rx_rd >> 8) & 0xFF);                               // High byte
  writeW5500(S0_RX_RD + 1, 0x0C, new_rx_rd & 0xFF);                                  // Low byte
  writeW5500(S0_CR, CB_WRITE_SR0, 0x40);                                             // 10. send RECV command
  writeW5500(S0_IR, CB_WRITE_SR0, 0x04);                                             // 11. delete Interrupt
  return size - 8;
}
//----------------------------------------------------------------
void saveIPConfigAfterDHCP() {
  ethConfig.ip = Ethernet.localIP();                                                 // read mach from Ethernet2 (if possible)
  ethConfig.gateway = Ethernet.gatewayIP();
  ethConfig.subnet = Ethernet.subnetMask();
  ethConfig.dns = Ethernet.dnsServerIP();
  Serial.println("DHCP Config gespeichert:");
  Serial.print("IP: "); Serial.println(ethConfig.ip);
  Serial.print("Gateway: "); Serial.println(ethConfig.gateway);
  Serial.print("Subnet: "); Serial.println(ethConfig.subnet);
}
//----------------------------------------------------------------
void applyIPConfigToW5500() {
  writeW5500(0x0001, CB_WRITE_CR, ethConfig.gateway[0]);                             // 1. Gateway (GAR) - Common Block Write (cb=0x04)
  writeW5500(0x0002, CB_WRITE_CR, ethConfig.gateway[1]);
  writeW5500(0x0003, CB_WRITE_CR, ethConfig.gateway[2]);
  writeW5500(0x0004, CB_WRITE_CR, ethConfig.gateway[3]);
  writeW5500(0x0005, CB_WRITE_CR, ethConfig.subnet[0]);                              // 2. Subnet Mask (SUBR) - Common Block Write (cb=0x04)
  writeW5500(0x0006, CB_WRITE_CR, ethConfig.subnet[1]);
  writeW5500(0x0007, CB_WRITE_CR, ethConfig.subnet[2]);
  writeW5500(0x0008, CB_WRITE_CR, ethConfig.subnet[3]);
  writeW5500(0x000F, CB_WRITE_CR, ethConfig.ip[0]);                                  // 3. Source IP (SIPR) - Common Block Write (cb=0x04)
  writeW5500(0x0010, CB_WRITE_CR, ethConfig.ip[1]);
  writeW5500(0x0011, CB_WRITE_CR, ethConfig.ip[2]);
  writeW5500(0x0012, CB_WRITE_CR, ethConfig.ip[3]);
  for(int i = 0; i < 6; i++) {                                                       // 4. MAC (SHAR) - write 6 Bytes
    writeW5500(0x0009 + i, CB_WRITE_CR, ethConfig.mac[i]);
  }
  Serial.print("IP in W5500 Registers: ");                                           // Optional: read back for a check
  Serial.print(readW5500(0x000F, CB_READ_CRS)); Serial.print(".");
  Serial.print(readW5500(0x0010, CB_READ_CRS)); Serial.print(".");
  Serial.print(readW5500(0x0011, CB_READ_CRS)); Serial.print(".");
  Serial.println(readW5500(0x0012, CB_READ_CRS));
}
//----------------------------------------------------------------
void sendRawUDP(IPAddress destIP, uint16_t destPort, uint8_t* data, uint16_t len) {
    Serial.println("=== sendRawUDP ===");
    Serial.print("Dest IP: "); Serial.println(destIP);
    Serial.print("Dest Port: "); Serial.println(destPort);
    Serial.print("Data length: "); Serial.println(len);
    uint16_t tx_free = getTXFreeSize();                                              // Check TX Free Size
    Serial.print("TX Free Size before send: "); Serial.println(tx_free);
    if (tx_free < len) {
        Serial.println("ERROR: TX Buffer full!");
        return;
    }
    writeW5500(0x000C, 0x0C, destIP[0]);                                             // 1. Set destination IP and port
    writeW5500(0x000D, 0x0C, destIP[1]);
    writeW5500(0x000E, 0x0C, destIP[2]);
    writeW5500(0x000F, 0x0C, destIP[3]);
    writeW5500(0x0010, 0x0C, (destPort >> 8) & 0xFF);
    writeW5500(0x0011, 0x0C, destPort & 0xFF);
    uint16_t tx_wr = readW5500(S0_TX_WR, 0x08) << 8;                                 // 2. Get current TX Write Pointer
    tx_wr |= readW5500(S0_TX_WR + 1, 0x08);
    Serial.println(tx_wr, HEX);
    writeW5500Bytes(tx_wr, 0x14, data, len);                                         // 3. Write data to TX Buffer
    uint16_t new_tx_wr = tx_wr + len;                                                // 4. Update TX Write Pointer
    writeW5500(S0_TX_WR, 0x0C, (new_tx_wr >> 8) & 0xFF);
    writeW5500(S0_TX_WR + 1, 0x0C, new_tx_wr & 0xFF);
    uint16_t tx_wr_after = readW5500(S0_TX_WR, 0x08) << 8;                           // 5. Verify TX Write Pointer update
    tx_wr_after |= readW5500(S0_TX_WR + 1, 0x08);
    Serial.println(tx_wr_after, HEX);        
    writeW5500(S0_CR, 0x0C, 0x20);                                                   // 6. Send command
    Serial.println("SEND command issued");
    unsigned long start = millis();                                                  // 7. Wait for SEND_OK
    while(!(readW5500(S0_IR, 0x08) & 0x10)) {
        if(millis() - start > 1000) {
            Serial.println("Timeout waiting for SEND_OK");
            break;
        }
        delay(1);
    }
    writeW5500(S0_IR, 0x0C, 0x10);                                                   // 8. Clear SEND_OK interrupt
    tx_free = getTXFreeSize();
    Serial.print("TX Free Size after send: "); Serial.println(tx_free);              // 9. Check TX Free Size after sending
    uint8_t sr = readW5500(S0_SR, 0x08);
    Serial.print("Socket status after send: 0x"); Serial.println(sr, HEX);           // 10. Log socket status
    uint8_t ir = readW5500(S0_IR, 0x08);
    Serial.print("Interrupt register: 0x"); Serial.println(ir, HEX);
    Serial.println("=== end sendRawUDP ===");
}
//----------------------------------------------------------------
uint16_t getTXFreeSize() {
  uint16_t val = readW5500(0x0020, 0x08) << 8;
  val |= readW5500(0x0021, 0x08);
  return val;
}