#include "LittleFS.h"

String ReadFile(String fileName) {
  String result = "";

  File file = LittleFS.open(fileName, "r");
  if (!file) { // failed to open the file, retrn empty result
    return result;
  }
  while (file.available()) {
    result += (char)file.read();
  }

  file.close();
  return result;
}

bool WriteFile(String fileName, String contents) {
  //Serial.println("WriteFile " + fileName + " " + contents);

  File file = LittleFS.open(fileName, "w");
  if (!file) { // failed to open the file, return false
    return false;
  }
  int bytesWritten = file.print(contents);

  if (bytesWritten == 0) { // write failed
    return false;
  }

  file.close();
  return true;
}

bool DeleteFile(String fileName) {
  Serial.println("DeleteFile " + fileName);

  if (!LittleFS.remove(fileName)) { // failed to open the file, return false
    return false;
  }

  return true;
}
