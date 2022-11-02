
uint32_t GetRevision(String json) {

  DynamicJsonDocument doc(1024);
  deserializeJson(doc, json);
  JsonObject jsonObj = doc.as<JsonObject>();

  return jsonObj["rev"].as<uint32_t>();
}
