//************************************************************
// CloudlessHomeAutomation
// https://github.com/dekoch/CloudlessHomeAutomation-mesh
//
// Switch Device
//
// connected with painlessMesh
//************************************************************

#define SHELLY_1PM
//#define SHELLY_2_5
//#define SONOFF_S20
//#define WEMOS_D1_MINI
//#define WEMOS_MINI32

#define VERSION       "1.003"

// SINGLE-CHANNEL RELAY WITH POWER METERING
// Generic ESP8266 Module Flash 2MB (FS:1MB OTA:~512KB)
// Pinout: TX | RX | 3.3V | GPIO0 | GND
//
// https://www.shelly.cloud/products/shelly-1pm-smart-home-automation-relay/
// https://www.shelly.cloud/knowledge-base/devices/shelly-1pm/
// https://templates.blakadder.com/shelly_1PM.html
// https://www.esphome-devices.com/devices/Shelly-1PM
// https://github.com/arendst/Tasmota/issues/5716
// https://github.com/Mollayo/Shelly-1PM/blob/master/config.h
#ifdef SHELLY_1PM
#define HARDWARE          "SHELLY_1PM"
#define FIRMWARE          "fw_ESP8266_shelly1pm.bin" // OTA-Updates
#define LED               0
#define LED_INV           false
#define DI_0              4
#define DI_0_PULLUP       false
#define DI_1              2       // Button
#define DI_1_PULLUP       true
#define DI_2              -1
#define DI_2_PULLUP       false
#define DI_CF             5       // HLW8012 Power Sensor https://esphome.io/components/sensor/hlw8012.html
#define DI_CF1            13
#define DI_SEL            14
#define POWER_MULTIPLIER  2214.010646
#define DO_0              15
#define DO_1              -1
#define TEMP_0            A0      // 32kOhm Resistor (10kOhm @ 25C)?!
#endif

// DOUBLE RELAY SWITCH & ROLLER SHUTTER
// Generic ESP8266 Module Flash 2MB (FS:1MB OTA:~512KB)
// Pinout: TX | RX | 3.3V | RST | GPIO0 | GND
//
// https://www.shelly.cloud/knowledge-base/devices/shelly-25/
#ifdef SHELLY_2_5
#define HARDWARE          "SHELLY_2_5"
#define FIRMWARE          "fw_ESP8266_shelly2_5.bin" // OTA-Updates
#define LED               0
#define LED_INV           false
#define DI_0              13
#define DI_0_PULLUP       false
#define DI_1              5
#define DI_1_PULLUP       false
#define DI_2              2       // Button
#define DI_2_PULLUP       true
#define DI_CF             -1
#define DI_CF1            -1
#define DI_SEL            -1
#define POWER_MULTIPLIER  0
#define DO_0              4
#define DO_1              15
#define TEMP_0            A0
#endif

#ifdef SONOFF_S20
#define HARDWARE          "SONOFF_S20"
#define FIRMWARE          "fw_ESP8266_sonoff_s20.bin" // OTA-Updates
#define LED               13
#define LED_INV           false
#define DI_0              0       // Button
#define DI_0_PULLUP       true
#define DI_1              -1
#define DI_1_PULLUP       false
#define DI_CF             -1
#define DI_CF1            -1
#define DI_SEL            -1
#define POWER_MULTIPLIER  0
#define DO_0              12
#define TEMP_0            A0
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
#define DI_2              -1
#define DI_2_PULLUP       false
#define DI_CF             -1
#define DI_CF1            -1
#define DI_SEL            -1
#define POWER_MULTIPLIER  0
#define DO_0              D2      //GPIO4
#define DO_1              -1
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
#define CHECK_TIMER_PERIOD  500   // milliseconds until cycle repeat
#define CHECK_TEMP_PERIOD   5000  // milliseconds until cycle repeat

// These are the nominal values for the resistors in the circuit
#define CURRENT_RESISTOR                0.001
#define VOLTAGE_RESISTOR_UPSTREAM       ( 5 * 470000 ) // Real: 2280k
#define VOLTAGE_RESISTOR_DOWNSTREAM     ( 1000 ) // Real 1.009k

#define arr_len(x) (sizeof(x) / sizeof(*x))

#include "core/CHAMesh.h"
#include "HLW8012.h"
#include "DeviceState.h"

// Prototypes
void sendMessage();
void receivedCallback(uint32_t from, String & msg);

void sendMessage() ; // Prototype
Task taskSendMessage(TASK_SECOND * 10, TASK_FOREVER, &sendMessage);
Task checkDi;
Task checkPower;
Task checkTimer;

HLW8012 hlw8012;
Edge edgeDi0 = Edge();
Edge edgeDi1 = Edge();
Edge edgeDi2 = Edge();
TimerItem NodeTimer[5];

class DiState {
  public:
    int Num;
    bool State;

    DiState() {}

    DiState(DynamicJsonDocument doc) {
      Num = doc["num"].as<int>();
      State = doc["state"].as<bool>();
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

    DoState(DynamicJsonDocument doc) {
      Num = doc["num"].as<int>();
      State = doc["state"].as<bool>();
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
DoState do1 = DoState();


class PowerState {
  public:
    int Num;
    unsigned int Value;
    double PowerMultiplier;

    PowerState() {}

    PowerState(DynamicJsonDocument doc) {
      Num = doc["num"].as<int>();
      Value = doc["value"].as<bool>();
      PowerMultiplier = doc["powerMultiplier"].as<bool>();
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


void SetOutputState(uint32_t from, DoState out, String setState) {

  /*Serial.println("out " + String(out.Num));
  Serial.println("setState " + setState);*/

  int gpio = -1;

  switch (out.Num) {
    case 0:
      gpio = DO_0;
      break;

    case 1:
      gpio = DO_1;
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

    String msg = "setState " + setState + " not supported";

    if (from > 0) {
      mesh.sendSingle(from, msg);
    } else {
      Serial.println(msg);
    }
    return;
  }

  digitalWrite(gpio, out.State);

  String msg = out.GetJson();
  mesh.sendBroadcast(msg);
  Serial.println("Send msg=" + msg);

  switch (out.Num) {
    case 0:
      do0.State = out.State;
      SaveDoState(0, out.State);
      break;

    case 1:
      do1.State = out.State;
      SaveDoState(1, out.State);
      break;
  }
}


void GroupCheckTrigger(uint32_t from, DiState di, uint32_t myNodeId, String json) {

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json);

  //Serial.println("GroupCheckTrigger " + json);

  String node = configMesh.GetNodeName(from);

  for (int i = 0; i < doc["gCnt"].as<int>(); i++) {

    bool validTrigger = false;

    for (int ii = 0; ii < doc["g"][i]["tCnt"].as<int>(); ii++) {

      if (doc["g"][i]["t"][ii]["node"].as<String>() == node &&
          doc["g"][i]["t"][ii]["di"].as<int>() == di.Num &&
          doc["g"][i]["t"][ii]["state"].as<bool>() == di.State) {

        validTrigger = true;
      }
    }

    if (validTrigger == false) {
      continue;
    }

    for (int ii = 0; ii < doc["g"][i]["nCnt"].as<int>(); ii++) {

      // its Me?
      if (doc["g"][i]["n"][ii]["node"].as<String>() != myName) {
        continue;
      }

      Serial.println("group " + doc["g"][i]["name"].as<String>());

      DoState out = DoState();
      out.Num = doc["g"][i]["n"][ii]["do"].as<int>();

      String setState = doc["g"][i]["n"][ii]["setState"].as<String>();

      SetOutputState(from, out, setState);
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

  if (DI_2 >= 0) {

    if (DI_2_PULLUP) {
      pinMode(DI_2, INPUT_PULLUP);
    }
    else {
      pinMode(DI_2, INPUT);
    }
  }


  for (int i = 1; i <= 3; i++) {
    // LED test
    digitalWrite(LED, true);
    delay(300);
    digitalWrite(LED, false);
    delay(500);
  }


  LittleFS.begin();

  if (DO_0 >= 0) {

    do0.Num = 0;
    // restore last state
    do0.State = ReadDoState(0);
    pinMode(DO_0, OUTPUT);
    digitalWrite(DO_0, do0.State);
  }

  if (DO_1 >= 0) {

    do1.Num = 0;
    // restore last state
    do1.State = ReadDoState(1);
    pinMode(DO_1, OUTPUT);
    digitalWrite(DO_1, do1.State);
  }

  Serial.print(HARDWARE);
  Serial.print(" ");
  Serial.print(FIRMWARE);
  Serial.print(" v");
  Serial.println(VERSION);
  Serial.println("LED " + String(LED));
  Serial.println("DI_0 " + String(DI_0));
  Serial.println("DI_1 " + String(DI_1));
  Serial.println("DO_0 " + String(DO_0) + "=" + String(do0.State));

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

  MeshSetup();
  mesh.onReceive(&receivedCallback);

  //userScheduler.addTask(taskSendMessage);
  //taskSendMessage.enable();

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

    if (DI_2 >= 0) {

      if (DI_2_PULLUP) {
        state = !digitalRead(DI_2);
      }
      else {
        state = digitalRead(DI_2);
      }

      if (edgeDi2.edge(state))
      {
        DiState di = DiState();
        di.Num = 2;
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


  for (int i = 0; i < arr_len(NodeTimer); i++) {
    NodeTimer[i] = configTimer.GetItems(myName, i);
  }

  checkTimer.set(CHECK_TIMER_PERIOD, TASK_FOREVER, []() {

    for (int i = 0; i < arr_len(NodeTimer); i++) {

      if (NodeTimer[i].Trigger == "DO0") {

        NodeTimer[i].CheckTrigger(do0.State);
        
      } else if (NodeTimer[i].Trigger == "DO1") {

        NodeTimer[i].CheckTrigger(do1.State);
        
      } else {
        continue;
      }
      
      if (NodeTimer[i].Elapsed()) {

        Serial.println("Timer " + NodeTimer[i].Name);

        DoState out = DoState();
        out.Num = NodeTimer[i].Output;
        SetOutputState(0, out, NodeTimer[i].SetState);
      }
    }

  });
  userScheduler.addTask(checkTimer);
  checkTimer.enable();

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

  String type = doc["t"].as<String>();

  if (type == "di") {

    DiState di = DiState(doc);
    GroupCheckTrigger(from, di, mesh.getNodeId(), configSwitch.Json);

  } else {
    ConfigReceivedCallback(type, msg);
  }
}
