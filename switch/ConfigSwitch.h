
#define CONFIG_SWITCH_FILE  "config_switch.json"

/*
config_switch.json
{
  "t":"config_switch.json",
  "rev":1,
  "gCnt":1,
  "g":[
    {
      "name":"Toggle",
      "tCnt":2,
      "t":[
        {
          "node":"KeWkSw",
          "di":0,
          "state":false
        },
        {
          "node":"KeWkSw",
          "di":0,
          "state":true
        }
      ],
      "nCnt":1,
      "n":[
        {
          "node":"KeGaLi",
          "do":0,
          "setState":"Tog"
        }
      ]
    }
  ]
}

*/

class ConfigSwitch {
  public:
    // initial value
    String Json = "{\"t\":\"" + String(CONFIG_SWITCH_FILE) + "\",\"rev\":10,\"gCnt\":1,\"g\":[{\"name\":\"Toggle\",\"tCnt\":2,\"t\":[{\"node\":\"KeWkSw\",\"di\":0,\"state\":false},{\"node\":\"KeWkSw\",\"di\":0,\"state\":true}],\"nCnt\":1,\"n\":[{\"node\":\"KeGaLi\",\"do\":0,\"setState\":\"Tog\"}]}]}";

    ConfigSwitch() {}

    void Read() {
      String temp = ReadFile(CONFIG_SWITCH_FILE);

      if (temp == "") {
        WriteFile(CONFIG_SWITCH_FILE, Json);
      } else {
        Serial.println(String(CONFIG_SWITCH_FILE) + " " + temp);
        UpdateConfigSwitch(temp);
      }
    }

    void UpdateConfigSwitch(String newConfig) {
      uint32_t myRev = GetRevision(Json);
      uint32_t newRev = GetRevision(newConfig);

      Serial.println("myRev=" + String(myRev) + " newRev=" + String(newRev));

      if (myRev >= newRev) {
        Serial.println("keep configSwitch " + Json);
        return;
      }

      Json = newConfig;
      Serial.println("updated configSwitch " + Json);

      String temp = ReadFile(CONFIG_SWITCH_FILE);
      uint32_t fileRev;

      if (temp != "") {
        fileRev = GetRevision(temp);
      }

      if (fileRev >= newRev) {
        return;
      }

      WriteFile(CONFIG_SWITCH_FILE, Json);
      Serial.println("updated " + String(CONFIG_SWITCH_FILE) + " " + Json);
    }
};
