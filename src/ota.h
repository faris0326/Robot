#pragma once
// Over-the-air firmware uploads (ArduinoOTA) over the robot's own Wi-Fi.
// Join the robot's access point, then: pio run -e robot_ota -t upload
//
// Password-protected (OTA_PASSWORD, from secrets.ini via platformio.ini). An upload
// puts the robot in Stop before anything is written, and it reboots into
// the new firmware (which starts in Stop) when done. Runs on the network
// task (core 0); the transfer blocks that task, so telemetry pauses.

void otaBegin();   // after the Wi-Fi access point is up
void otaHandle();  // call often from the network task
