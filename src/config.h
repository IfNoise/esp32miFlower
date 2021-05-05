struct floraDevice
{
   char* address;
   double temperature;
   int moisture;
   int light;
   int conductivity;
   int battery;
};

// array of different xiaomi flora MAC addresses
floraDevice FLORA_DEVICES[] = {
   { "C4:7C:8D:6C:86:98",0,0,0,0,0},
   { "C4:7C:8D:6C:80:FC",0,0,0,0,0},
   { "C4:7C:8D:6C:91:A4",0,0,0,0,0},
   { "C4:7C:8D:6C:81:96",0,0,0,0,0}
};

// sleep between to runs in seconds
#define SLEEP_DURATION 30 * 60
// emergency hibernate countdown in seconds
#define EMERGENCY_HIBERNATE 3 * 60
// how often should the battery be read - in run count
#define BATTERY_INTERVAL 6
// how often should a device be retried in a run when something fails
#define RETRY 3

const char*   WIFI_SSID       = "Garage";
const char*   WIFI_PASSWORD   = "00000006";

// MQTT topic gets defined by "<MQTT_BASE_TOPIC>/<MAC_ADDRESS>/<property>"
// where MAC_ADDRESS is one of the values from FLORA_DEVICES array
// property is either temperature, moisture, conductivity, light or battery

const char*   MQTT_HOST       = "192.168.1.209";
const int     MQTT_PORT       = 1883;
const char*   MQTT_CLIENTID   = "miflora-client";
const char*   MQTT_USERNAME   = "miflora";
const char*   MQTT_PASSWORD   = "neuro8301";
const String  MQTT_BASE_TOPIC = "flora"; 
const int     MQTT_RETRY_WAIT = 10000;
