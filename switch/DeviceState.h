//************************************************************
// restore devices last states after reset
//************************************************************

#define DEVICE_STATE_FILE "device_state.json"
#define DEVICE_STATE_JSON "{\"doCnt\":1,\"do\":[{\"num\":0,\"state\":false}]}"

/*
  device_state.json
  {
    "doCnt":1,
    "do":[
      {
        "num":0,
        "state":false
      }
    ]
  }

*/

bool ReadDoState(int num) {

  String json = ReadFile(DEVICE_STATE_FILE);

  //Serial.println("read " + String(DEVICE_STATE_FILE) + " " + json);

  if (json == "") {
    // initial value
    json = DEVICE_STATE_JSON;
  }

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json);

  for (int i = 0; i < doc["doCnt"].as<int>(); i++) {

    if (doc["do"][i]["num"].as<int>() == num) {

      return doc["do"][i]["state"].as<bool>();
    }
  }

  return false;
}


bool SaveDoState(int num, bool state) {

  String json = ReadFile(DEVICE_STATE_FILE);

  //Serial.println("read " + String(DEVICE_STATE_FILE) + " " + json);

  if (json == "") {
    // initial value
    json = DEVICE_STATE_JSON;
  }

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json);

  for (int i = 0; i < doc["doCnt"].as<int>(); i++) {

    if (doc["do"][i]["num"].as<int>() == num) {

      if (doc["do"][i]["state"].as<bool>() == state) {
        // nothing to save
        return false;
      }

      doc["do"][i]["state"] = state;
    }
  }

  json = "";
  serializeJson(doc, json);

  WriteFile(DEVICE_STATE_FILE, json);
  //Serial.println("updated " + String(DEVICE_STATE_FILE) + " " + json);
  return true;
}
