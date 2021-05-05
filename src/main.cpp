#include <Arduino.h>
/**
   A BLE client for the Xiaomi Mi Plant Sensor, pushing measurements to an MQTT server.

   See https://github.com/nkolban/esp32-snippets/blob/master/Documentation/BLE%20C%2B%2B%20Guide.pdf
   on how bluetooth low energy and the library used are working.

   See https://github.com/ChrisScheffler/miflora/wiki/The-Basics for details on how the 
   protocol is working.
   
   MIT License

   Copyright (c) 2017 Sven Henkel
   Multiple units reading by Grega Lebar 2018

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#include "BLEDevice.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

// boot count used to check if battery status should be read
RTC_DATA_ATTR int bootCount = 0;

// device count
static int deviceCount = sizeof FLORA_DEVICES / sizeof FLORA_DEVICES[0];

// the remote service we wish to connect to
static BLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");

// the characteristic of the remote service we are interested in
static BLEUUID uuid_version_battery("00001a02-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_sensor_data("00001a01-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_write_mode("00001a00-0000-1000-8000-00805f9b34fb");

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void connectWifi()
{
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("");
}

void disconnectWifi()
{
  WiFi.disconnect(true);
  Serial.println("WiFi disonnected");
}

void connectMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  int count = 0;
  while (!mqttClient.connected())
  {
    if (!mqttClient.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD))
    {
      if (count < 5)
      {
        Serial.print("MQTT connection failed:");
        Serial.print(mqttClient.state());
        Serial.println("Retrying...");
        delay(MQTT_RETRY_WAIT);
        ++count;
      }
      else
      {
        Serial.print("MQTT connection failed:");
        Serial.println("Trying...on next cicle");
        return;
      }
    }
  }

  Serial.println("MQTT connected");
  Serial.println("");
}

void disconnectMqtt()
{
  mqttClient.disconnect();
  Serial.println("MQTT disconnected");
}

BLEClient *getFloraClient(floraDevice *device)
{
  char *deviceMacAddress = device->address;
  BLEAddress floraAddress(deviceMacAddress);
  BLEClient *floraClient = BLEDevice::createClient();

  if (!floraClient->connect(floraAddress))
  {
    Serial.println("- Connection failed, skipping");
    return nullptr;
  }

  Serial.println("- Connection successful");
  return floraClient;
}

BLERemoteService *getFloraService(BLEClient *floraClient)
{
  BLERemoteService *floraService = nullptr;

  try
  {
    floraService = floraClient->getService(serviceUUID);
  }
  catch (...)
  {
     Serial.println("Something wrong1");
  }
  if (floraService == nullptr)
  {
    Serial.println("- Failed to find data service");
  }
  else
  {
    Serial.println("- Found data service");
  }

  return floraService;
}

bool forceFloraServiceDataMode(BLERemoteService *floraService)
{
  BLERemoteCharacteristic *floraCharacteristic;

  // get device mode characteristic, needs to be changed to read data
  Serial.println("- Force device in data mode");
  floraCharacteristic = nullptr;
  try
  {
    floraCharacteristic = floraService->getCharacteristic(uuid_write_mode);
  }
  catch (...)
  {
    Serial.println("Something wrong1");
  }
  if (floraCharacteristic == nullptr)
  {
    Serial.println("-- Failed, skipping device");
    return false;
  }

  // write the magic data
  uint8_t buf[2] = {0xA0, 0x1F};
  floraCharacteristic->writeValue(buf, 2, true);

  delay(500);
  return true;
}

bool readFloraDataCharacteristic(BLERemoteService *floraService, floraDevice *device)
{
  BLERemoteCharacteristic *floraCharacteristic = nullptr;

  // get the main device data characteristic
  Serial.println("- Access characteristic from device");
  try
  {
    floraCharacteristic = floraService->getCharacteristic(uuid_sensor_data);
  }
  catch (...)
  {
    Serial.println("Something wrong2");
  }
  if (floraCharacteristic == nullptr)
  {
    Serial.println("-- Failed, skipping device");
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from characteristic");
  std::string value;
  try
  {
    value = floraCharacteristic->readValue();
  }
  catch (...)
  {
    // something went wrong
    Serial.println("-- Failed, skipping device");
    return false;
  }
  const char *val = value.c_str();

  Serial.print("Hex: ");
  for (int i = 0; i < 16; i++)
  {
    Serial.print((int)val[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

  int16_t *temp_raw = (int16_t *)val;
  device->temperature = (*temp_raw) / ((float)10.0);
  Serial.print("-- Temperature: ");
  Serial.println(device->temperature);

  device->moisture = val[7];
  Serial.print("-- Moisture: ");
  Serial.println(device->moisture);

  device->light = val[3] + val[4] * 256;
  Serial.print("-- Light: ");
  Serial.println(device->light);

  device->conductivity = val[8] + val[9] * 256;
  Serial.print("-- Conductivity: ");
  Serial.println(device->conductivity);

  if (device->temperature > 200)
  {
    Serial.println("-- Unreasonable values received, skip publish");
    return false;
  }

  return true;
}

bool readFloraBatteryCharacteristic(BLERemoteService *floraService, floraDevice *device)
{
  BLERemoteCharacteristic *floraCharacteristic = nullptr;

  // get the device battery characteristic
  Serial.println("- Access battery characteristic from device");
  try
  {
    floraCharacteristic = floraService->getCharacteristic(uuid_version_battery);
  }
  catch (...)
  {
    // something went wrong
  }
  if (floraCharacteristic == nullptr)
  {
    Serial.println("-- Failed, skipping battery level");
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from characteristic");
  std::string value;
  try
  {
    value = floraCharacteristic->readValue();
  }
  catch (...)
  {
    // something went wrong
    Serial.println("-- Failed, skipping battery level");
    return false;
  }
  const char *val2 = value.c_str();
  device->battery = val2[0];

  Serial.print("-- Battery: ");
  Serial.println(device->battery);

  return true;
}

bool processFloraService(BLERemoteService *floraService, floraDevice *device, bool readBattery)
{
  // set device in data mode
  if (!forceFloraServiceDataMode(floraService))
  {
    return false;
  }
  bool dataSuccess = readFloraDataCharacteristic(floraService, device);

  bool batterySuccess = true;
  if (readBattery)
  {
    batterySuccess = readFloraBatteryCharacteristic(floraService, device);
  }

  return dataSuccess && batterySuccess;
}

bool processFloraDevice(floraDevice *device, bool getBattery, int tryCount)
{
  
  Serial.print("Processing Flora device at ");
  Serial.print(device->address);
  Serial.print(" (try ");
  Serial.print(tryCount);
  Serial.println(")");

  // connect to flora ble server
  BLEClient *floraClient = getFloraClient(device);
  if (floraClient == nullptr)
  {
    return false;
  }

  // connect data service
  BLERemoteService *floraService = getFloraService(floraClient);
  if (floraService == nullptr)
  {
    floraClient->disconnect();
    return false;
  }

  // process devices data
  bool success = processFloraService(floraService, device, getBattery);

  // disconnect from device
  floraClient->disconnect();

  return success;
}

void publishJsonData(floraDevice *device)
{

  if (mqttClient.connected())
  {
    String baseTopic = MQTT_BASE_TOPIC;
    String output;
    StaticJsonDocument<200> data;
    data["measurement"] = "miData";
    data["tag"] = device->address;
    data["temperature"] = device->temperature;
    data["moisture"] = device->moisture;
    data["light"] = device->light;
    data["conductivity"] = device->conductivity;
    serializeJsonPretty(data, Serial);
    serializeJson(data, output);
    mqttClient.publish(baseTopic.c_str(), output.c_str());
  }
  delay(100);
}

void setup()
{
  // all action is done when device is woken up
  Serial.begin(115200);
  delay(1000);

  // increase boot count
  bootCount++;
}

void loop()
{
  bool readBattery = ((bootCount % BATTERY_INTERVAL) == 0);
  Serial.println("Initialize BLE client...");
  BLEDevice::init("");
  BLEDevice::setPower(ESP_PWR_LVL_P7);
  delay(1000);
  // process devices
  for (int i = 0; i < deviceCount; i++)
  {
    int tryCount = 0;
    char *deviceMacAddress = FLORA_DEVICES[i].address;
    BLEAddress floraAddress(deviceMacAddress);

    while (tryCount < RETRY)
    {
      tryCount++;
      if (processFloraDevice(&FLORA_DEVICES[i], readBattery, tryCount))
      {

        break;
      }
      delay(1000);
    }
  }
  Serial.println("Bluetooth Deinitialisation");
  BLEDevice::deinit(false);
  connectWifi();
  connectMqtt();
  for (int i = 0; i < deviceCount; i++)
  {
    publishJsonData(&FLORA_DEVICES[i]);
  }
  disconnectMqtt();
  disconnectWifi();
  delay(60000);
}