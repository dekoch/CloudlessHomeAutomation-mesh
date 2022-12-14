//************************************************************
// CloudlessHomeAutomation
// https://github.com/dekoch/CloudlessHomeAutomation-mesh
//
// RootNode
//
// connected with painlessMesh
//************************************************************

#include "painlessMesh.h"
#include <FS.h>
#include "SD.h"
#include "SPI.h"
#include "core/Fs.h"
#include "core/Edge.h"
#include "core/Timer.h"
#include "core/Config.h"
#include "core/ConfigWifi.h"
#include "core/ConfigMesh.h"
#include "core/ConfigSwitch.h"
#include "core/ConfigTimer.h"

#define VERSION "1.002"

#define HARDWARE  "WEMOS_D1_MINI"
#define LED       2

#define BLINK_PERIOD            3000  // milliseconds until cycle repeat
#define BLINK_DURATION          100   // milliseconds LED is on for
#define UPDATE_FIRMWARE_PERIOD  60000
#define OTA_PART_SIZE           512   //How many bytes to send per OTA data packet

// change this to match your SD shield or module;
// WeMos Micro SD Shield V1.0.0: D8
// LOLIN Micro SD Shield V1.2.0: D4 (Default)
const int chipSelect = D8;

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;

bool calc_delay = false;
SimpleList<uint32_t> nodes;

// Task to blink the number of nodes
Task blinkNoNodes;
bool onFlag = false;

ConfigMesh configMesh = ConfigMesh();
ConfigSwitch configSwitch = ConfigSwitch();
ConfigTimer configTimer = ConfigTimer();

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
  Serial.print(" v");
  Serial.println(VERSION);
  Serial.println("LED " + String(LED));

  if (!SD.begin(SS)) {
    Serial.println("initialization failed!");
    return;
  }

  createMissingFiles();
  restoreFromSd();

  Serial.println(String(CONFIG_MESH_FILE) + ": " + configMesh.Json);
  Serial.println(String(CONFIG_SWITCH_FILE) + ": " + configSwitch.Json);
  Serial.println(String(CONFIG_TIMER_FILE) + ": " + configTimer.Json);


  mesh.setDebugMsgTypes(ERROR | STARTUP | DEBUG);  // set before init() so that you can see startup messages

  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);
  // Bridge node, should (in most cases) be a root node. See [the
  // wiki](https://gitlab.com/painlessMesh/painlessMesh/wikis/Possible-challenges-in-mesh-formation)
  // for some background
  mesh.setRoot(true);
  // This node and all other nodes should ideally know the mesh contains a root,
  // so call this on all nodes
  mesh.setContainsRoot(true);

  Serial.println("NodeID " + String(mesh.getNodeId()));

  sendUpdates();

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
}

void loop() {
  mesh.update();
  digitalWrite(LED, onFlag);
}

void sendUpdates() {

  File root = SD.open("/");

  while (true) {

    File entry =  root.openNextFile();
    if (!entry) {
      // no more files
      break;
    }

    if (entry.isDirectory() == false) {

      TSTRING name = entry.name();

      if (name.endsWith(".bin") == false) {
        entry.close();
        continue;
      }

      TSTRING hardware = name.substring(name.indexOf('_') + 1, name.lastIndexOf('_'));

      Serial.println("BROADCASTING " + name + " FOR " + hardware);

      //This is the important bit for OTA, up to now was just getting the file.
      //If you are using some other way to upload firmware, possibly from
      //mqtt or something, this is what needs to be changed.
      //This function could also be changed to support OTA of multiple files
      //at the same time, potentially through using the pkg.md5 as a key in
      //a map to determine which to send
      mesh.initOTASend([&entry](painlessmesh::plugin::ota::DataRequest pkg, char* buffer) {

        //fill the buffer with the requested data packet from the node.
        entry.seek(OTA_PART_SIZE * pkg.partNo);
        entry.readBytes(buffer, OTA_PART_SIZE);

        //The buffer can hold OTA_PART_SIZE bytes, but the last packet may
        //not be that long. Return the actual size of the packet.
        return min((unsigned)OTA_PART_SIZE, entry.size() - (OTA_PART_SIZE * pkg.partNo));
      },
      OTA_PART_SIZE);

      //Calculate the MD5 hash of the firmware we are trying to send. This will be used
      //to validate the firmware as well as tell if a node needs this firmware.
      MD5Builder md5;
      md5.begin();
      md5.addStream(entry, entry.size());
      md5.calculate();

      //Make it known to the network that there is OTA firmware available.
      //This will send a message every minute for an hour letting nodes know
      //that firmware is available.
      //This returns a task that allows you to do things on disable or more,
      //like closing your files or whatever.
      mesh.offerOTA(name, hardware, md5.toString(), ceil(((float)entry.size()) / OTA_PART_SIZE), false);

      while (true) {
        //This program will not reach loop() so we dont have to worry about file scope.
        mesh.update();
      }
    }
  }
}

void createMissingFiles() {

  if (SD.exists(CONFIG_MESH_FILE) == false) {

    writeSd(CONFIG_MESH_FILE, configMesh.Json);
  }

  if (SD.exists(CONFIG_SWITCH_FILE) == false) {

    writeSd(CONFIG_SWITCH_FILE, configSwitch.Json);
  }

  if (SD.exists(CONFIG_TIMER_FILE) == false) {

    writeSd(CONFIG_TIMER_FILE, configTimer.Json);
  }
}

void restoreFromSd() {

  if (SD.exists(CONFIG_MESH_FILE)) {

    configMesh.Json = readSd(CONFIG_MESH_FILE);
  }

  if (SD.exists(CONFIG_SWITCH_FILE)) {

    configSwitch.Json = readSd(CONFIG_SWITCH_FILE);
  }

  if (SD.exists(CONFIG_TIMER_FILE)) {

    configTimer.Json = readSd(CONFIG_TIMER_FILE);
  }
}

void writeSd(String fileName, String contents) {

  SD.remove(fileName);

  if (SD.exists(fileName)) {
    Serial.println("error deleteing file " + fileName);
    return;
  }

  File myFile = SD.open(fileName, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to " + String(myFile.name()));
    myFile.print(contents);
    // close the file:
    myFile.close();
    Serial.println(" done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening " + String(myFile.name()));
  }
}

String readSd(String fileName) {

  File myFile = SD.open(fileName);

  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Reading from " + String(myFile.name()));

    String ret = "";

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      ret += myFile.readString();
    }

    // close the file:
    myFile.close();
    Serial.println(" done.");

    ret.trim();
    return ret;
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening " + String(myFile.name()));
    return "";
  }
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

    if (configMesh.Update(msg)) {
      // backup to SD
      writeSd(CONFIG_MESH_FILE, configMesh.Json);
    }

  } else if (type == CONFIG_SWITCH_FILE) {

    if (configSwitch.Update(msg)) {
      // backup to SD
      writeSd(CONFIG_SWITCH_FILE, configSwitch.Json);
    }
  } else if (type == CONFIG_TIMER_FILE) {

    if (configTimer.Update(msg)) {
      // backup to SD
      writeSd(CONFIG_TIMER_FILE, configTimer.Json);
    }
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
  mesh.sendSingle(nodeId, configTimer.Json);
  Serial.println("Send to " + String(nodeId) + " msg=" + configTimer.Json);
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
