/*
 * IRCamSimplePacketComsServer.cpp
 *
 *  Created on: Oct 19, 2018
 *      Author: hephaestus
 */

#include "IRCamSimplePacketComsServer.h"

IRCamSimplePacketComsServer::IRCamSimplePacketComsServer(
		DFRobotIRPosition * cam) :
		PacketEventAbstract(1590) {
	camera = cam;
}

IRCamSimplePacketComsServer::~IRCamSimplePacketComsServer() {
	// TODO Auto-generated destructor stub
}

void IRCamSimplePacketComsServer::event(float * buffer) {
	for (int i = 0; i < 8; i++) {
		buffer[i] = bufferCache[i];
	}
}

bool IRCamSimplePacketComsServer::loop() {
	//int64_t start = esp_timer_get_time();
	Wire.flush();
	camera->requestPosition();
	if (camera->available()) {
		for (int i = 0; i < 4; i++) {
			bufferCache[(i * 2)] = ((float) camera->readX(i)); ///1024.0;
			//bufferCache[(i * 2)] = bufferCache[(i * 2)]>0?33.0/bufferCache[(i * 2)]:0;
			bufferCache[(i * 2) + 1] = ((float) camera->readY(i));	///1024.0;
			//bufferCache[(i * 2) + 1] = bufferCache[(i * 2) + 1]>0?23.0/bufferCache[(i * 2) + 1]:0;
		}
		return true;
	}else{
		Serial.println("Error reading IR cam");
		return false;
	}

}

float IRCamSimplePacketComsServer::readX(int i){
	return bufferCache[(i * 2)];
}
float IRCamSimplePacketComsServer::readY(int i){
	return bufferCache[(i * 2) + 1];
}

void IRCamSimplePacketComsServer::print() {
	Serial.print("\r\nValues\r\n");
	for (int i = 0; i < 4; i++) {
		Serial.print("\t");
		Serial.print(bufferCache[(i * 2)]);
		Serial.print(",");

		Serial.print(bufferCache[(i * 2)+1]);
		Serial.print(";\r\n");
	}
}
