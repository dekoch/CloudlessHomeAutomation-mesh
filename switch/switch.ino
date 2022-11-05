//************************************************************
// CloudlessHomeAutomation
// https://github.com/dekoch/CloudlessHomeAutomation-mesh
//
// Switch Device
//
// connected with painlessMesh
//************************************************************

#include <painlessMesh.h>
#include "HLW8012.h"
#include "core/Edge.h"
#include "core/Fs.h"
#include "core/Config.h"
#include "core/ConfigWifi.h"
#include "core/ConfigMesh.h"
#include "core/ConfigSwitch.h"

#define SHELLY_1PM
//#define WEMOS_D1_MINI
//#define WEMOS_MINI32

#define VERSION       "1.001"

// https://www.shelly.cloud/products/shelly-1pm-smart-home-automation-relay/
// https://www.shelly.cloud/knowledge-base/devices/shelly-1pm/
// https://templates.blakadder.com/shelly_1PM.html
// https://www.esphome-devices.com/devices/Shelly-1PM
// https://github.com/arendst/Tasmota/issues/5716
// https://github.com/Mollayo/Shelly-1PM/blob/master/config.h
//
// Generic ESP8266 Module Flash 2MB (FS:1MB OTA:~512KB)
// Pinout: TX | RX | 3.3V | GPIO0 | GND
#ifdef SHELLY_1PM
#define HARDWARE          "SHELLY_1PM"
#define FIRMWARE          "fw_ESP8266_shelly1pm.bin" // OTA-Updates
#define LED               0
#define LED_INV           false
#define DI_0              4
#define DI_0_PULLUP       false
#define DI_1              2       // Button
#define DI_1_PULLUP       true
#define DI_CF             5       // HLW8012 Power Sensor https://esphome.io/components/sensor/hlw8012.html
#define DI_CF1            13
#define DI_SEL            14
#define POWER_MULTIPLIER  2214.010646
#define DO_0              15
#define TEMP_0            A0      // 32kOhm Resistor (10kOhm @ 25C)?!
#endif

// Test Device
#ifdef WEMOS_D1_MINI
#define HARDWARE          "WEMOS_D1_MINI"
#define FIRMWARE          "fw_ESP8266_wemosD1Mini.bin"  // OTA-Updates
#define LED               2
#define LED_INV           false
#define DI_0              D1      //GPIO5
#define DI_0_PULLUP       false
#define DI_1              -1
#define DI_1_PULLUP       false
#define DI_CF             -1
#define DI_CF1            -1
#define DI_SEL            -1
#define POWER_MULTIPLIER  0
#define DO_0              D2      //GPIO4
#endif

// Test Device
#ifdef WEMOS_MINI32
#define HARDWARE          "WEMOS_MINI32"
#define FIRMWARE          "fw_ESP32_wemosMini32.bin"      // OTA-Updates
#define LED               2
#define LED_INV           true
#define DI_0              5       //GPIO5
#define DI_0_PULLUP       false
#define DI_1              -1
#define DI_1_PULLUP       false
#define DI_CF             -1
#define DI_CF1            -1
#define DI_SEL            -1
#define POWER_MULTIPLIER  0
#define DO_0              4       //GPIO4
#endif

#define BLINK_PERIOD        3000  // milliseconds until cycle repeat
#define BLINK_DURATION      100   // milliseconds LED is on for
#define CHECK_DI_PERIOD     100   // milliseconds until cycle repeat
#define CHECK_POWER_PERIOD  5000  // milliseconds until cycle repeat
#define CHECK_TEMP_PERIOD   5000  // milliseconds until cycle repeat

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 5 * 470000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

#define arr_len(x) (sizeof(x) / sizeof(*x))

// Prototypes
void sendMessage();
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void delayReceivedCallback(uint32_t from, int32_t delay);

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;

bool calc_delay = false;
SimpleList<uint32_t> nodes;

void sendMessage() ; // Prototype
Task taskSendMessage(TASK_SECOND * 10, TASK_FOREVER, &sendMessage);

// Task to blink the number of nodes
Task blinkNoNodes;
Task checkDi;
Task checkPower;
bool onFlag = false;
String myName = "";

HLW8012 hlw8012;

ConfigMesh configMesh = ConfigMesh();
ConfigSwitch configSwitch = ConfigSwitch();

Edge edgeDi0 = Edge();
Edge edgeDi1 = Edge();

class DiState {
  public:
    int Num;
    bool State;

    DiState() {}

    DiState(JsonObject jsonObj) {
      Num = jsonObj["num"].as<int>();
      State = jsonObj["state"].as<bool>();
    }

    String GetJson() {
      DynamicJsonDocument doc(1024);
      doc["t"] = "di";
      doc["num"] = Num;
      doc["state"] = State;

      String json;
      serializeJson(doc, json);
      return json;
    }
};


class DoState {
  public:
    int Num;
    bool State;

    DoState() {}

    DoState(JsonObject jsonObj) {
      Num = jsonObj["num"].as<int>();
      State = jsonObj["state"].as<bool>();
    }

    String GetJson() {
      DynamicJsonDocument doc(1024);
      doc["t"] = "do";
      doc["num"] = Num;
      doc["state"] = State;

      String json;
      serializeJson(doc, json);
      return json;
    }
};

DoState do0 = DoState();


class PowerState {
  public:
    int Num;
    unsigned int Value;
    double PowerMultiplier;

    PowerState() {}

    PowerState(JsonObject jsonObj) {
      Num = jsonObj["num"].as<int>();
      Value = jsonObj["value"].as<bool>();
      PowerMultiplier = jsonObj["powerMultiplier"].as<bool>();
    }

    String GetJson() {
      unsigned int tempValue = Value;

      if (tempValue < 2) {
        // Make everything below 2W appear as just 0W
        tempValue = 0;
      }

      DynamicJsonDocument doc(1024);
      doc["t"] = "power";
      doc["num"] = Num;
      doc["value"] = tempValue;
      doc["powerMultiplier"] = PowerMultiplier;

      String json;
      serializeJson(doc, json);
      return json;
    }
};


void GroupCheckTrigger(uint32_t from, DiState di, uint32_t myNodeId, String json) {

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json);
  JsonObject jsonObj = doc.as<JsonObject>();

  //Serial.println("GroupCheckTrigger " + json);

  String node = configMesh.GetNodeName(from);

  for (int i = 0; i < jsonObj["gCnt"].as<int>(); i++) {

    bool validTrigger = false;

    for (int ii = 0; ii < jsonObj["g"][i]["tCnt"].as<int>(); ii++) {

      if (jsonObj["g"][i]["t"][ii]["node"].as<String>() == node &&
          jsonObj["g"][i]["t"][ii]["di"].as<int>() == di.Num &&
          jsonObj["g"][i]["t"][ii]["state"].as<bool>() == di.State) {

        validTrigger = true;
      }
    }

    if (validTrigger == false) {
      continue;
    }

    for (int ii = 0; ii < jsonObj["g"][i]["nCnt"].as<int>(); ii++) {

      // its Me?
      if (jsonObj["g"][i]["n"][ii]["node"].as<String>() != myName) {
        continue;
      }

      Serial.println("group " + jsonObj["g"][i]["name"].as<String>());

      DoState out = DoState();
      out.Num = jsonObj["g"][i]["n"][ii]["do"].as<int>();

      String setState = jsonObj["g"][i]["n"][ii]["setState"].as<String>();

      Serial.println("out " + String(out.Num));
      Serial.println("setState " + setState);

      int gpio = -1;

      switch (out.Num) {
        case 0:
          gpio = DO_0;
          break;
      }

      if (gpio == -1) {
        mesh.sendSingle(from, "invalid out " + String(out.Num));
        return;
      }

      bool oldState = digitalRead(gpio);
      out.State = false;

      if (setState == "S") {

        out.State = true;

      } else if (setState == "R") {

        out.State = false;

      } else if (setState == "Tog") {

        out.State = !oldState;

      } else {
        mesh.sendSingle(from, "setState " + setState + " not supported");
        return;
      }

      digitalWrite(gpio, out.State);

      String msg = out.GetJson();
      mesh.sendBroadcast(msg);
      Serial.println("Send msg=" + msg);

      switch (out.Num) {
        case 0:
          do0.State = out.State;
          break;
      }
    }
  }
}


void setup() {
  delay(500);
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("starting...");
  Serial.println(" ");

  pinMode(LED, OUTPUT);

  if (DI_0 >= 0) {

    if (DI_0_PULLUP) {
      pinMode(DI_0, INPUT_PULLUP);
    }
    else {
      pinMode(DI_0, INPUT);
    }
  }

  if (DI_1 >= 0) {

    if (DI_1_PULLUP) {
      pinMode(DI_1, INPUT_PULLUP);
    }
    else {
      pinMode(DI_1, INPUT);
    }
  }

  pinMode(DO_0, OUTPUT);
  digitalWrite(DO_0, false);
  do0.Num = 0;
  do0.State = false;

  for (int i = 1; i <= 3; i++) {
    // LED test
    digitalWrite(LED, true);
    delay(300);
    digitalWrite(LED, false);
    delay(500);
  }

  Serial.print(HARDWARE);
  Serial.print(" ");
  Serial.print(FIRMWARE);
  Serial.print(" v");
  Serial.println(VERSION);
  Serial.println("LED " + String(LED));
  Serial.println("DI_0 " + String(DI_0));
  Serial.println("DI_1 " + String(DI_1));
  Serial.println("DO_0 " + String(DO_0));

  LittleFS.begin();
  configMesh.Read();
  configSwitch.Read();

  if (DI_CF >= 0) {

    // Initialize HLW8012
    // void begin(unsigned char cf_pin, unsigned char cf1_pin, unsigned char sel_pin, unsigned char currentWhen = HIGH, bool use_interrupts = false, unsigned long pulse_timeout = PULSE_TIMEOUT);
    // * cf_pin, cf1_pin and sel_pin are GPIOs to the HLW8012 IC
    // * currentWhen is the value in sel_pin to select current sampling
    // * set use_interrupts to false, we will have to call handle() in the main loop to do the sampling
    // * set pulse_timeout to 500ms for a fast response but losing precision (that's ~24W precision :( )
    hlw8012.begin(DI_CF, DI_CF1, DI_SEL, LOW, false, 500000);

    // These values are used to calculate current, voltage and power factors as per datasheet formula
    // These are the nominal values for the Sonoff POW resistors:
    // * The CURRENT_RESISTOR is the 1milliOhm copper-manganese resistor in series with the main line
    // * The VOLTAGE_RESISTOR_UPSTREAM are the 5 470kOhm resistors in the voltage divider that feeds the V2P pin in the HLW8012
    // * The VOLTAGE_RESISTOR_DOWNSTREAM is the 1kOhm resistor in the voltage divider that feeds the V2P pin in the HLW8012
    hlw8012.setResistors(CURRENT_RESISTOR, VOLTAGE_RESISTOR_UPSTREAM, VOLTAGE_RESISTOR_DOWNSTREAM);

    if (POWER_MULTIPLIER > 0.0) {
      hlw8012.setPowerMultiplier(POWER_MULTIPLIER);
    }
  }


  mesh.setDebugMsgTypes(ERROR | STARTUP | DEBUG);  // set before init() so that you can see error messages

  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);
  mesh.initOTAReceive(FIRMWARE);

  String nodeId = String(mesh.getNodeId());
  myName = configMesh.GetNodeName(mesh.getNodeId());

  if (myName == "") {
    myName = nodeId;
  }

  Serial.println("NodeID " + nodeId);
  Serial.println("NodeName " + myName);

  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();

  blinkNoNodes.set(BLINK_PERIOD, (mesh.getNodeList().size() + 1) * 2, []() {
    // If on, switch off, else switch on
    if (onFlag)
      onFlag = false;
    else
      onFlag = true;
    blinkNoNodes.delay(BLINK_DURATION);

    if (blinkNoNodes.isLastIteration()) {
      // Finished blinking. Reset task for next run
      // blink number of nodes (including this node) times
      blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
      // Calculate delay based on current mesh time and BLINK_PERIOD
      // This results in blinks between nodes being synced
      blinkNoNodes.enableDelayed(BLINK_PERIOD -
                                 (mesh.getNodeTime() % (BLINK_PERIOD * 1000)) / 1000);
    }
  });
  userScheduler.addTask(blinkNoNodes);
  blinkNoNodes.enable();


  checkDi.set(CHECK_DI_PERIOD, TASK_FOREVER, []() {

    bool state;

    if (DI_0 >= 0) {

      if (DI_0_PULLUP) {
        state = !digitalRead(DI_0);
      }
      else {
        state = digitalRead(DI_0);
      }

      //onFlag = state;

      if (edgeDi0.edge(state))
      {
        DiState di = DiState();
        di.Num = 0;
        di.State = state;

        String msg = di.GetJson();
        mesh.sendBroadcast(msg);
        Serial.println("Send msg=" + msg);

        GroupCheckTrigger(mesh.getNodeId(), di, mesh.getNodeId(), configSwitch.Json);
      }
    }

    if (DI_1 >= 0) {

      if (DI_1_PULLUP) {
        state = !digitalRead(DI_1);
      }
      else {
        state = digitalRead(DI_1);
      }

      if (edgeDi1.edge(state))
      {
        DiState di = DiState();
        di.Num = 1;
        di.State = state;

        String msg = di.GetJson();
        mesh.sendBroadcast(msg);
        Serial.println("Send msg=" + msg);

        GroupCheckTrigger(mesh.getNodeId(), di, mesh.getNodeId(), configSwitch.Json);
      }
    }

  });
  userScheduler.addTask(checkDi);
  checkDi.enable();


  if (DI_CF >= 0) {

    checkPower.set(CHECK_POWER_PERIOD, TASK_FOREVER, []() {

      if (do0.State == false) {
        // output is off, dont send broadcast
        return;
      }

      PowerState power = PowerState();
      power.Num = 0;
      power.Value = hlw8012.getActivePower();

      if (POWER_MULTIPLIER == 0.0) {

        hlw8012.expectedActivePower(27.0);
      }

      power.PowerMultiplier = hlw8012.getPowerMultiplier();

      String msg = power.GetJson();
      mesh.sendBroadcast(msg);
      Serial.println("Send msg=" + msg);

    });
    userScheduler.addTask(checkPower);
    checkPower.enable();
  }

  randomSeed(analogRead(A0));
}

void loop() {
  mesh.update();
  digitalWrite(LED, !onFlag == !LED_INV);
}

void sendMessage() {
  /*String msg = "Hello from node ";
    msg += mesh.getNodeId();
    //msg += " myFreeMemory: " + String(ESP.getFreeHeap());
    msg += " time: " + String(mesh.getNodeTime());
    mesh.sendBroadcast(msg);

    if (calc_delay) {
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      mesh.startDelayMeas(*node);
      node++;
    }
    calc_delay = false;
    }

    Serial.printf("Sending message: %s\n", msg.c_str());*/

  Serial.printf("Connections, %s\n", mesh.subConnectionJson(true).c_str());
}

void receivedCallback(uint32_t from, String & msg) {
  Serial.printf("Received from %u msg=%s\n", from, msg.c_str());

  if (msg.startsWith("{") == false ||
      msg.endsWith("}") == false) {
    return;
  }

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, msg);
  JsonObject jsonObj = doc.as<JsonObject>();

  String type = jsonObj["t"].as<String>();

  if (type == "di") {

    DiState di = DiState(jsonObj);
    GroupCheckTrigger(from, di, mesh.getNodeId(), configSwitch.Json);

  } else if (type == CONFIG_MESH_FILE) {

    configMesh.UpdateConfigMesh(msg);

  } else if (type == CONFIG_SWITCH_FILE) {

    configSwitch.UpdateConfigSwitch(msg);
  }
}

void newConnectionCallback(uint32_t nodeId) {
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD * 1000)) / 1000);

  Serial.printf("New Connection, nodeId = %u\n", nodeId);
  Serial.printf("New Connection, %s\n", mesh.subConnectionJson(true).c_str());

  mesh.sendSingle(nodeId, configMesh.Json);
  Serial.println("Send to " + String(nodeId) + " msg=" + configMesh.Json);
  mesh.sendSingle(nodeId, configSwitch.Json);
  Serial.println("Send to " + String(nodeId) + " msg=" + configSwitch.Json);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD * 1000)) / 1000);

  nodes = mesh.getNodeList();

  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");

  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    Serial.printf(" %u", *node);
    node++;
  }
  Serial.println();
  calc_delay = true;
}

void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}
