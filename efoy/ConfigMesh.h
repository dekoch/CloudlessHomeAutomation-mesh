
#define CONFIG_MESH_FILE  "config_mesh.json"

/*
config_mesh.json
{
  "t":"config_mesh.json",
  "rev":1,
  "nCnt":2,
  "n":[
    {
      "nodeId":2503482707,
      "name":"KeGaLi"
    },
    {
      "nodeId":2786085503,
      "name":"KeWkSw"
    }
  ]
}

*/

class ConfigMesh {
  public:
    // initial value
    String Json = "{\"t\":\"" + String(CONFIG_MESH_FILE) + "\",\"rev\":2,\"nCnt\":2,\"n\":[{\"nodeId\":2503482707,\"name\":\"KeGaLi\"},{\"nodeId\":2786085503,\"name\":\"KeWkSw\"}]}";

    ConfigMesh() {}

    void Read() {
      String temp = ReadFile(CONFIG_MESH_FILE);

      if (temp == "") {
        WriteFile(CONFIG_MESH_FILE, Json);
      } else {
        Serial.println(String(CONFIG_MESH_FILE) + " " + temp);
        UpdateConfigMesh(temp);
      }
    }

    void UpdateConfigMesh(String newConfig) {
      uint32_t myRev = GetRevision(Json);
      uint32_t newRev = GetRevision(newConfig);

      Serial.println("myRev=" + String(myRev) + " newRev=" + String(newRev));

      if (myRev >= newRev) {
        Serial.println("keep configMesh " + Json);
        return;
      }

      Json = newConfig;
      Serial.println("updated configMesh " + Json);

      String temp = ReadFile(CONFIG_MESH_FILE);
      uint32_t fileRev;

      if (temp != "") {
        fileRev = GetRevision(temp);
      }

      if (fileRev >= newRev) {
        return;
      }

      WriteFile(CONFIG_MESH_FILE, Json);
      Serial.println("updated " + String(CONFIG_MESH_FILE) + " " + Json);
    }

    uint32_t GetNodeId(String node) {

      DynamicJsonDocument doc(1024);
      deserializeJson(doc, Json);
      JsonObject jsonObj = doc.as<JsonObject>();

      for (int i = 0; i < jsonObj["nCnt"].as<int>(); i++) {

        if (jsonObj["n"][i]["name"].as<String>() == node) {
          return jsonObj["n"][i]["nodeId"].as<uint32_t>();
        }
      }

      return 0;
    }

    String GetNodeName(uint32_t nodeId) {

      DynamicJsonDocument doc(1024);
      deserializeJson(doc, Json);
      JsonObject jsonObj = doc.as<JsonObject>();

      for (int i = 0; i < jsonObj["nCnt"].as<int>(); i++) {

        if (jsonObj["n"][i]["nodeId"].as<uint32_t>() == nodeId) {
          return jsonObj["n"][i]["name"].as<String>();
        }
      }

      return String(nodeId);
    }
};
