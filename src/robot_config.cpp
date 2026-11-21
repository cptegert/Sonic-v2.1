#include "robot_config.h"
#include <Preferences.h>
#include <esp_mac.h>

struct RobotProfile {
  const char *key;        // stored on the board, used by tools/robot.py --robot
  const char *name;       // robot name: logs, params, Wi-Fi "<name>-log"
  const char *bleName;    // Bluetooth name in the RemoteXY app
  RobotGui gui;           // RemoteXY GUI array
  float motorScale;       // ESC scale around neutral, 1.0 = Sonic's ESC
};

// One line per robot. Keep in sync with tools/robots.json (key, name, ssid).
static const RobotProfile PROFILES[] = {
  {"sonic",    "Sonic-X",  "Sonic-X",                GUI_SONIC,  1.0f},
  {"shadow",   "Shadow",   "Shadow_v2_Bluetooth_v4", GUI_SHADOW, 1.0f},
  {"knuckles", "Knuckles", "Knuckles",               GUI_SONIC,  1.0f},
};
static const int PROFILE_COUNT = sizeof(PROFILES) / sizeof(PROFILES[0]);

static char keyBuf[16] = "";
static char nameBuf[24] = "Robot";
static char bleBuf[40] = "Robot";
static char ssidBuf[40] = "Robot-log";
static RobotGui gui = GUI_SONIC;
static float motorScale = 1.0f;
static bool stored = false;

static const RobotProfile *findProfile(const char *key) {
  for (int i = 0; i < PROFILE_COUNT; i++) {
    if (strcasecmp(PROFILES[i].key, key) == 0) return &PROFILES[i];
  }
  return nullptr;
}

void robot_identity_begin() {
  Preferences p;
  p.begin("robot", true);
  String key = p.getString("id", "");
  p.end();

  const RobotProfile *prof = findProfile(key.c_str());
  if (prof) {
    stored = true;
    strlcpy(keyBuf, prof->key, sizeof(keyBuf));
    strlcpy(nameBuf, prof->name, sizeof(nameBuf));
    strlcpy(bleBuf, prof->bleName, sizeof(bleBuf));
    gui = prof->gui;
    motorScale = prof->motorScale;
  } else {
    // No (known) identity on this board: "Robot-xxxx" from the last MAC bytes
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(nameBuf, sizeof(nameBuf), "Robot-%02x%02x", mac[4], mac[5]);
    strlcpy(bleBuf, nameBuf, sizeof(bleBuf));
    keyBuf[0] = 0;
  }
  snprintf(ssidBuf, sizeof(ssidBuf), "%s-log", nameBuf);
}

const char *robotKey() { return keyBuf; }
const char *robotName() { return nameBuf; }
const char *robotBleName() { return bleBuf; }
const char *robotWifiSsid() { return ssidBuf; }
RobotGui robotGui() { return gui; }
float robotMotorScale() { return motorScale; }
bool robotIdentityStored() { return stored; }

bool robotSetIdentity(const char *key) {
  const RobotProfile *prof = findProfile(key);
  if (!prof) return false;
  Preferences p;
  p.begin("robot", false);
  p.putString("id", prof->key);
  p.end();
  return true;
}

void robotListProfiles(Print &out) {
  out.print("known robots:");
  for (int i = 0; i < PROFILE_COUNT; i++) out.printf(" %s(%s)", PROFILES[i].key, PROFILES[i].name);
  out.println();
}
