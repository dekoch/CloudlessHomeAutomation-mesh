//************************************************************
// CloudlessHomeAutomation
// https://github.com/dekoch/CloudlessHomeAutomation-mesh
//
// EFOY fuel cell
//
// connected with painlessMesh
//************************************************************

#include <painlessMesh.h>
#include "Fs.h"
#include "Config.h"
#include "ConfigWifi.h"
#include "ConfigMesh.h"

#define VERSION   "1.001"

#define HARDWARE  "WEMOS_D1_MINI"
#define FIRMWARE  "fw_efoy.bin"       // OTA-Updates
#define LED       2
#define LED_INV   false

#define BLINK_PERIOD                  3000  // milliseconds until cycle repeat
#define BLINK_DURATION                100   // milliseconds LED is on for
#define CHECK_OPERATING_STATE_PERIOD  5000  // milliseconds until cycle repeat

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
Task checkOperatingState;
bool onFlag = false;
String myName = "";

ConfigMesh configMesh = ConfigMesh();

class OperatingState {
  public:
    String State;

    OperatingState() {}

    OperatingState(JsonObject jsonObj) {
      State = jsonObj["state"].as<String>();
    }

    String GetJson() {
      DynamicJsonDocument doc(1024);
      doc["t"] = "efoy_state";
      doc["state"] = State;

      String json;
      serializeJson(doc, json);
      return json;
    }
};


void setup() {
  delay(500);
  Serial.begin(115200);
  Serial.println(" ");
  Serial.println(" ");
  Serial.println("starting...");
  Serial.println(" ");

  pinMode(LED, OUTPUT);

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

  LittleFS.begin();
  configMesh.Read();

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


  checkOperatingState.set(CHECK_OPERATING_STATE_PERIOD, TASK_FOREVER, []() {

    OperatingState op = OperatingState();
    op.State = "off";

    String msg = op.GetJson();
    mesh.sendBroadcast(msg);
    Serial.println("Send msg=" + msg);

  });
  userScheduler.addTask(checkOperatingState);
  checkOperatingState.enable();

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

  if (type == CONFIG_MESH_FILE) {

    configMesh.UpdateConfigMesh(msg);

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
