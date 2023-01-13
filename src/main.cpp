#include <Arduino.h>
#include "BLEDevice.h"
#include <WiFi.h>
#include <PubSubClient.h>

typedef struct smartbmsutilRunInfo {
   int8_t header1;
   int8_t header2;
   int8_t contentLength;
   int16_t batteryVoltages[32];
   int16_t batteryTemp[8];
   int16_t currentV;
   int16_t currentA;
   int16_t chargePercent;
   int16_t maxCellVoltage;
   int16_t minCellVoltage;
   int16_t dummy1;
   int16_t dummy2;
   int16_t dummy3;
   int16_t dummy4;
   int16_t countBatteryVoltages;
   int16_t countBatteryTemp;
   int16_t cycle;
   int16_t jh;
   int16_t cdmos;
   int16_t fdmos;
   int16_t avgVoltage;
   int16_t diffVoltage;
   int16_t currentKw;
   int16_t alarm1;
   int16_t alarm2;
   int16_t alarm3;
   int16_t alarm4;
   int8_t crcHigh;
   int8_t crcLow;
} __attribute__ ((packed)) SmartbmsutilRunInfo;

#define RECEIVE_BUFFER_SIZE 500
#define READ_PACKET_HEADER_LENGTH 3 // the number of bytes for header of read packets (header, content length)
#define READ_PACKET_CRC_LENGTH 2 // the number of bytes for CRC of read packets
#define READ_PACKET_OVERHEAD_LENGTH  READ_PACKET_HEADER_LENGTH + READ_PACKET_CRC_LENGTH 

// TODO: WiFiManager
const char* ssid = "your-ssid-here";
const char* password = "pReShArEdKeY";
const char* mqtt_server = "10.10.10.10";
String bleServerName = "DL-02112334A039"; // The name of your daly bms (probably mac address)

unsigned long lastMsg = 0;


static BLEUUID rxCharacteristicUUID("0000fff1");
static BLEUUID txCharacteristicUUID("0000fff2");

static BLEUUID bmeServiceUUID("fff0");

static BLERemoteCharacteristic* rxCharacteristic;
static BLERemoteCharacteristic* txCharacteristic;

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};


byte req[8] = { 210, 3, 0, 0, 0, 62, 215, 185};

byte smartBmsReceiveBuffer[RECEIVE_BUFFER_SIZE];
uint16_t indexSmartBmsReceiveBuffer = 0;

WiFiClient espClient;
PubSubClient client(espClient);

void smartbmsutilSwapBmsBytesEndian(byte *buffer, int size) {
  byte tmpValue;
  for (int i = 0; i < ((size - READ_PACKET_HEADER_LENGTH) / 2); i++) {
    tmpValue = buffer[READ_PACKET_HEADER_LENGTH + 2*i];
    buffer[READ_PACKET_HEADER_LENGTH + 2*i] = buffer[READ_PACKET_HEADER_LENGTH + 2*i + 1];
    buffer[READ_PACKET_HEADER_LENGTH + 2*i + 1] = tmpValue;
  }
}

SmartbmsutilRunInfo smartbmsutilGetRunInfo(byte *buffer, int size) {
  // use copy of buffer to not change original buffer for endian conversion
  byte tmpBuffer[size];
  memcpy(tmpBuffer, buffer, size);

  // swap bytes to little endian (as structs are organized in little endian in ESP32)
  smartbmsutilSwapBmsBytesEndian(tmpBuffer, size - 2);

  // copy tmpBuffer to struct
  SmartbmsutilRunInfo result;
  memcpy(&result, tmpBuffer, sizeof(result));  
  return result;
}

void smartbmsutilPrintRunInfo(SmartbmsutilRunInfo runInfo) {
  char msg[50];

  Serial.print("Battery voltages: ");
  for (int i = 0; i < runInfo.countBatteryVoltages; i++) {
    Serial.print(runInfo.batteryVoltages[i] / 1000.0, 3);
    Serial.print("V ");

    String topic = String("daly_bms/cell_voltages/" + String(i + 1));
    char char_array[topic.length() + 1];
    topic.toCharArray(char_array, topic.length() +1);

    snprintf (msg, 50, "%.3f", runInfo.batteryVoltages[i] / 1000.0);

    client.publish(char_array, msg);
    memset(msg, 0, sizeof(msg));
    
  }
  Serial.println();

  Serial.print("Battery temps: ");
  for (int i = 0; i < runInfo.countBatteryTemp; i++) {
    Serial.print(runInfo.batteryTemp[i] - 40);
    Serial.print("°C ");

    String topic = String("daly_bms/temperatures/external" + String(i + 1));
    char char_array[topic.length() + 1];
    topic.toCharArray(char_array, topic.length() +1);
    
    snprintf (msg, 50, "%.1f", runInfo.batteryTemp[i] - 40.0);
    client.publish(char_array, msg);
    memset(msg, 0, sizeof(msg));
  }
  Serial.println();

  Serial.print("Max Cell voltage: ");
  Serial.println(runInfo.maxCellVoltage / 1000.0, 3);

  Serial.print("Min Cell voltage: ");
  Serial.println(runInfo.minCellVoltage / 1000.0, 3);

  Serial.print("Current voltage: ");
  Serial.println(runInfo.currentV / 10.0);
  

  snprintf (msg, 50, "%.3f", (runInfo.currentV / 10.0));
  client.publish("daly_bms/soc/total_voltage", msg);
  memset(msg, 0, sizeof(msg));

  Serial.print("Current A: ");
  Serial.println((runInfo.currentA - 30000) / 10.0);


  snprintf (msg, 50, "%.3f", (runInfo.currentA - 30000) / 10.0);
  client.publish("daly_bms/soc/current", msg);
  memset(msg, 0, sizeof(msg));


  Serial.print("Current KW: ");
  Serial.println(runInfo.currentKw / 1000.0, 3);

  snprintf (msg, 50, "%.1f", (runInfo.chargePercent / 10.0));
  client.publish("daly_bms/soc/soc_percent", msg);
  memset(msg, 0, sizeof(msg));

  snprintf (msg, 50, "%i", runInfo.countBatteryVoltages);
  client.publish("daly_bms/cell_count", msg);
  memset(msg, 0, sizeof(msg));

  snprintf (msg, 50, "%i", runInfo.cycle);
  client.publish("daly_bms/cycle_count", msg);
  memset(msg, 0, sizeof(msg));

  snprintf (msg, 50, "%i", runInfo.jh);
  client.publish("daly_bms/jh", msg);
  memset(msg, 0, sizeof(msg));

  snprintf (msg, 50, "%i", runInfo.cdmos);
  client.publish("daly_bms/cdmos", msg);
  memset(msg, 0, sizeof(msg));

  snprintf (msg, 50, "%i", runInfo.fdmos);
  client.publish("daly_bms/fdmos", msg);
  memset(msg, 0, sizeof(msg));

  snprintf (msg, 50, "%i", runInfo.alarm1);
  client.publish("daly_bms/alarm/1", msg);
  memset(msg, 0, sizeof(msg));

  snprintf (msg, 50, "%i", runInfo.alarm2);
  client.publish("daly_bms/alarm/2", msg);
  memset(msg, 0, sizeof(msg));

  snprintf (msg, 50, "%i", runInfo.alarm3);
  client.publish("daly_bms/alarm/3", msg);
  memset(msg, 0, sizeof(msg));

  snprintf (msg, 50, "%i", runInfo.alarm4);
  client.publish("daly_bms/alarm/4", msg);
  memset(msg, 0, sizeof(msg));
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String deviceName = advertisedDevice.getName().c_str();
    String deviceAddr = advertisedDevice.getAddress().toString().c_str();
    Serial.println(deviceName + " - " + deviceAddr);
    if (deviceName == bleServerName.c_str()) { //Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
      doConnect = true; //Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};

static void rxNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    byte *pData, size_t length, bool isNotify) {

  Serial.println("Got new rx data!");
  for (int i = 0; i < length; i++) {
    smartBmsReceiveBuffer[indexSmartBmsReceiveBuffer] = pData[i];
    indexSmartBmsReceiveBuffer++;
  }
  SmartbmsutilRunInfo runInfo = smartbmsutilGetRunInfo(smartBmsReceiveBuffer, indexSmartBmsReceiveBuffer);
  smartbmsutilPrintRunInfo(runInfo);

  memset(smartBmsReceiveBuffer, 0, sizeof(smartBmsReceiveBuffer));
  indexSmartBmsReceiveBuffer = 0;
  Serial.println("Buffer reset");

}

//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
  BLEClient* pClient = BLEDevice::createClient();
 
  // Connect to the remote BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");
  pClient->setMTU(517);

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    pClient->disconnect();
    return (false);
  }

  rxCharacteristic = pRemoteService->getCharacteristic(rxCharacteristicUUID);
  if (rxCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(rxCharacteristicUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our rx characteristic");

  if(rxCharacteristic->canNotify())
    rxCharacteristic->registerForNotify(rxNotifyCallback);

  txCharacteristic = pRemoteService->getCharacteristic(txCharacteristicUUID);
  if (txCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(txCharacteristicUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our tx characteristic");

  connected = true;
  return true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  client.setServer(mqtt_server, 1883);


  //Init BLE device
  BLEDevice::init("");
 
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(60);
}

void loop() {
  if (!client.connected()) {
    client.connect("daly-bms");
  }
  client.loop();
  
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
    }
    doConnect = false;
  }

  unsigned long now = millis();
  if (now - lastMsg > 15000) {
    lastMsg = now;
    if (connected) {
      Serial.println("requesting new data");
      txCharacteristic->writeValue(req, sizeof(req), false);
    } else {
      doConnect = true;
    }
  }
}