/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net> and Leopold Sayous <leosayous@gmail.com>
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DW1000Ranging.h
 * Arduino global library (source file) working with the DW1000 library 
 * for the Decawave DW1000 UWB transceiver IC.
 *
 * @TODO
 * - remove or debugmode for Serial.print
 * - move strings to flash to reduce ram usage
 * - do not safe duplicate of pin settings
 * - maybe other object structure
 * - use enums instead of preprocessor constants
 */


#include "DW1000Ranging.h"
#include "DW1000Device.h"

DW1000RangingClass DW1000Ranging;


//other devices we are going to communicate with which are on our network:
DW1000Device DW1000RangingClass::_networkDevices[MAX_DEVICES];
byte         DW1000RangingClass::_currentAddress[8];
byte         DW1000RangingClass::_currentShortAddress[2];
byte         DW1000RangingClass::_lastSentToShortAddress[2];
volatile uint8_t DW1000RangingClass::_networkDevicesNumber = 0; // TODO short, 8bit?
int16_t      DW1000RangingClass::_lastDistantDevice    = 0; // TODO short, 8bit?
DW1000Mac    DW1000RangingClass::_globalMac;

//module type (anchor or tag)
int16_t      DW1000RangingClass::_type; // TODO enum??

// message flow state
volatile byte    DW1000RangingClass::_expectedMsgId;

// range filter
volatile boolean DW1000RangingClass::_useRangeFilter = false;
uint16_t DW1000RangingClass::_rangeFilterValue = 15;

// message sent/received state
volatile boolean DW1000RangingClass::_sentAck     = false;
volatile boolean DW1000RangingClass::_receivedAck = false;

// protocol error state
boolean          DW1000RangingClass::_protocolFailed = false;

// timestamps to remember
int32_t            DW1000RangingClass::timer           = 0;
int16_t            DW1000RangingClass::counterForBlink = 0; // TODO 8 bit?
long long int	   DW1000RangingClass::cycleCounter = 0;
uint32_t           DW1000RangingClass::currentTimeStamp = 0;
bool               DW1000RangingClass::protocolEnd = false;
int 			   DW1000RangingClass::timeOutCounter = 0;
int 			   DW1000RangingClass::timeOutResetCount = 10;
byte               DW1000RangingClass::destinationAddress[2];
bool			   DW1000RangingClass::minimalSerialPrint = false;


// data buffer
byte          DW1000RangingClass::data[LEN_DATA];
// reset line to the chip
uint8_t   DW1000RangingClass::_RST;
uint8_t   DW1000RangingClass::_SS;
// watchdog and reset period
uint32_t  DW1000RangingClass::_lastActivity;
uint32_t  DW1000RangingClass::_resetPeriod;
uint32_t  DW1000RangingClass::timeoutPeriod = 400;

// reply times (same on both sides for symm. ranging)
uint16_t  DW1000RangingClass::_replyDelayTimeUS;
//timer delay
uint16_t  DW1000RangingClass::_timerDelay;
// ranging counter (per second)
uint16_t  DW1000RangingClass::_successRangingCount = 0;
uint32_t  DW1000RangingClass::_rangingCountPeriod  = 0;
//Here our handlers
void (* DW1000RangingClass::_handleNewRange)(void) = 0;
void (* DW1000RangingClass::_handleBlinkDevice)(DW1000Device*) = 0;
void (* DW1000RangingClass::_handleNewDevice)(DW1000Device*) = 0;
void (* DW1000RangingClass::_handleInactiveDevice)(DW1000Device*) = 0;

/* ###########################################################################
 * #### Init and end #######################################################
 * ######################################################################### */

void DW1000RangingClass::initCommunication(uint8_t myRST, uint8_t mySS, uint8_t myIRQ) {
	// reset line to the chip
	_RST              = myRST;
	_SS               = mySS;
	_resetPeriod      = DEFAULT_RESET_PERIOD;
	// reply times (same on both sides for symm. ranging)
	_replyDelayTimeUS = DEFAULT_REPLY_DELAY_TIME;
	//we set our timer delay
	_timerDelay       = DEFAULT_TIMER_DELAY;
	
	
	DW1000.begin(myIRQ, myRST);
	DW1000.select(mySS);
}


void DW1000RangingClass::configureNetwork(uint16_t deviceAddress, uint16_t networkId, const byte mode[]) {
	// general configuration
	DW1000.newConfiguration();
	DW1000.setDefaults();
	DW1000.setDeviceAddress(deviceAddress);
	DW1000.setNetworkId(networkId);
	DW1000.enableMode(mode);
	DW1000.commitConfiguration();
	
}

void DW1000RangingClass::generalStart() {
	// attach callback for (successfully) sent and received messages
	DW1000.attachSentHandler(handleSent);
	DW1000.attachReceivedHandler(handleReceived);
	// anchor starts in receiving mode, awaiting a ranging poll message
	
	
	if(DEBUG) {
		// DEBUG monitoring
		Serial.println("DW1000-arduino");
		// initialize the driver
		
		
		Serial.println("configuration..");
		// DEBUG chip info and registers pretty printed
		char msg[90];
		DW1000.getPrintableDeviceIdentifier(msg);
		Serial.print("Device ID: ");
		Serial.println(msg);
		DW1000.getPrintableExtendedUniqueIdentifier(msg);
		Serial.print("Unique ID: ");
		Serial.print(msg);
		char string[6];
		sprintf(string, "%02X:%02X", _currentShortAddress[0], _currentShortAddress[1]);
		Serial.print(" short: ");
		Serial.println(string);
		
		DW1000.getPrintableNetworkIdAndShortAddress(msg);
		Serial.print("Network ID & Device Address: ");
		Serial.println(msg);
		DW1000.getPrintableDeviceMode(msg);
		Serial.print("Device mode: ");
		Serial.println(msg);
	}
	
	// Vincent changes
	DW1000.large_power_init();
	
	// anchor starts in receiving mode, awaiting a ranging poll message
	receiver();
	// for first time ranging frequency computation
	_rangingCountPeriod = millis();
}


void DW1000RangingClass::startAsAnchor(char address[], const byte mode[], const bool randomShortAddress) {
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	Serial.print("device address: ");
	Serial.println(address);
	if (randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else {
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}
	
	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000Ranging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode);
	
	//general start:
	generalStart();
	
	//defined type as anchor
	_type = ANCHOR;

	//added by debils
	 _expectedMsgId == POLL;
	
	Serial.println("### ANCHOR ###");
	
}

void DW1000RangingClass::startAsTag(char address[], const byte mode[], const bool randomShortAddress) {
	//save the address
	DW1000.convertToByte(address, _currentAddress);
	//write the address on the DW1000 chip
	DW1000.setEUI(address);
	if(DEBUG) {
		Serial.print("device address: ");
		Serial.println(address);
	}
	
	if (randomShortAddress) {
		//we need to define a random short address:
		randomSeed(analogRead(0));
		_currentShortAddress[0] = random(0, 256);
		_currentShortAddress[1] = random(0, 256);
	}
	else {
		// we use first two bytes in addess for short address
		_currentShortAddress[0] = _currentAddress[0];
		_currentShortAddress[1] = _currentAddress[1];
	}
	
	//we configur the network for mac filtering
	//(device Address, network ID, frequency)
	DW1000Ranging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode);
	
	generalStart();
	//defined type as tag
	_type = TAG;
	if(DEBUG) {
		Serial.println("### TAG ###");
	}
	
}

boolean DW1000RangingClass::addNetworkDevices(DW1000Device* device, boolean shortAddress) {
	boolean   addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
		if(_networkDevices[i].isAddressEqual(device) && !shortAddress) {
			//the device already exists
			addDevice = false;
			return false;
		}
		else if(_networkDevices[i].isShortAddressEqual(device) && shortAddress) {
			//the device already exists
			addDevice = false;
			return false;
		}
		
	}
	
	if(addDevice) {
		device->setRange(0);
		memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
		_networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
		_networkDevicesNumber++;
		return true;
	}
	
	return false;
}

boolean DW1000RangingClass::addNetworkDevices(DW1000Device* device) {
	boolean addDevice = true;
	//we test our network devices array to check
	//we don't already have it
	for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
		if(_networkDevices[i].isAddressEqual(device) && _networkDevices[i].isShortAddressEqual(device)) {
			//the device already exists
			addDevice = false;
			return false;
		}
		
	}
	
	if(addDevice) {
		if(_type == ANCHOR) //for now let's start with 1 TAG
		{
			_networkDevicesNumber = 0;
		}
		memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
		_networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
		_networkDevicesNumber++;
		return true;
	}
	
	return false;
}

void DW1000RangingClass::removeNetworkDevices(int16_t index) {
	//if we have just 1 element
	if(_networkDevicesNumber == 1) {
		_networkDevicesNumber = 0;
	}
	else if(index == _networkDevicesNumber-1) //if we delete the last element
	{
		_networkDevicesNumber--;
	}
	else {
		//we translate all the element wich are after the one we want to delete.
		for(int16_t i = index; i < _networkDevicesNumber-1; i++) { // TODO 8bit?
			memcpy(&_networkDevices[i], &_networkDevices[i+1], sizeof(DW1000Device));
			_networkDevices[i].setIndex(i);
		}
		_networkDevicesNumber--;
	}
}

/* ###########################################################################
 * #### Setters and Getters ##################################################
 * ######################################################################### */

//setters
void DW1000RangingClass::setReplyTime(uint16_t replyDelayTimeUs) { _replyDelayTimeUS = replyDelayTimeUs; }

void DW1000RangingClass::setResetPeriod(uint32_t resetPeriod) { _resetPeriod = resetPeriod; }

void DW1000RangingClass::setSentAck(bool sent) { _sentAck = sent; }


DW1000Device* DW1000RangingClass::searchDistantDevice(byte shortAddress[]) {
	//we compare the 2 bytes address with the others
	for(uint16_t i = 0; i < _networkDevicesNumber; i++) { // TODO 8bit?
		if(memcmp(shortAddress, _networkDevices[i].getByteShortAddress(), 2) == 0) {
			//we have found our device !
			return &_networkDevices[i];
		}
	}
	
	return nullptr;
}

DW1000Device* DW1000RangingClass::getDistantDevice() {
	//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
	
	return &_networkDevices[_lastDistantDevice];
	
}


/* ###########################################################################
 * #### Public methods #######################################################
 * ######################################################################### */

void DW1000RangingClass::checkForReset() {
	uint32_t curMillis = millis();
	if(!_sentAck && !_receivedAck) {
		// check if inactive
		if(curMillis-_lastActivity > _resetPeriod) {
			resetInactive();
		}
		return; // TODO cc
	}
}

void DW1000RangingClass::timeoutTAG() {
	uint32_t curMillis = millis();
	if(!_sentAck && !_receivedAck) {
		// check if inactive
		if(curMillis-_lastActivity > timeoutPeriod) {
			if(timeOutCounter++ >= timeOutResetCount)
			{
				ESP.restart();
			}
			Serial.println("Timed out!");
			protocolEnd = true;
			startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
			beginProtocol();
		}
		return; // TODO cc
	}
}

void DW1000RangingClass::prepareForAnotherRound() {				
	if(currentTimeStamp + (150) < millis()) {
	DW1000RangingClass::protocolEnd = false;
	DW1000RangingClass::initProtocol = true;
	}
}

void DW1000RangingClass::timeoutANCHOR() {
	uint32_t curMillis = millis();
	//int diff = curMillis - _lastActivity;
			//Serial.println(diff);
	
	if(!_sentAck && !_receivedAck) {
		// check if inactive
		if(curMillis-_lastActivity > _resetPeriod) {
			if(DEBUG) {
			Serial.println("timeout reset");
			}
			resetAnchor();
			// receiver();
		}
		//return; // TODO cc
	}
}

void DW1000RangingClass::resetAnchor() 
{
	if(DEBUG) {
		Serial.println("resetAnchor initiated.");
	}
	_expectedMsgId = POLL;
	noteActivity();
}

void DW1000RangingClass::checkForInactiveDevices() {
	for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
		if(_networkDevices[i].isInactive()) {
			if(_handleInactiveDevice != 0) {
				(*_handleInactiveDevice)(&_networkDevices[i]);
			}
			//we need to delete the device from the array:
			removeNetworkDevices(i);
			
		}
	}
}

// TODO check return type
int16_t DW1000RangingClass::detectMessageType(byte datas[]) {
	if(datas[0] == FC_1_BLINK) {
		return BLINK;
	}
	else if(datas[0] == FC_1 && datas[1] == FC_2) {
		//we have a long MAC frame message (ranging init)
		return datas[LONG_MAC_LEN];
	}
	else if(datas[0] == FC_1 && datas[1] == FC_2_SHORT) {
		//we have a short mac frame message (poll, range, range report, etc..)
		return datas[SHORT_MAC_LEN];
	}
}

void DW1000RangingClass::loop_tag(char anchor_address[], int cyclesLimit) {
	// checkForReset();
	if(cycleCounter <= cyclesLimit)
	{
	timeoutTAG();
	if(protocolEnd) {
		prepareForAnotherRound();
	}
	}
	if(DW1000RangingClass::initProtocol) {
		DW1000RangingClass::initProtocol = false;
		byte anchor_address_byte[8];
		
		DW1000.convertToByte(anchor_address, anchor_address_byte);


		byte anchor_address_short_byte[2];
		anchor_address_short_byte[0] = anchor_address_byte[0];
		anchor_address_short_byte[1] = anchor_address_byte[1];
		destinationAddress[0] = anchor_address_short_byte[0];
		destinationAddress[1] = anchor_address_short_byte[1];


		myStaticAnchor = new DW1000Device(anchor_address_byte, anchor_address_short_byte);
		myStaticAnchor->setReplyTime(DEFAULT_REPLY_DELAY_TIME);
		transmitPoll(myStaticAnchor);
		noteActivity();
		_expectedMsgId = POLL_ACK;
	}
	
	
	if(_sentAck) {
		if(DEBUG) {
			Serial.print("Reply time:");
			Serial.println(_replyDelayTimeUS);
		}		
		_sentAck = false;
		int messageType = detectMessageType(data);
		if(DEBUG) {
			Serial.println("Data sent:");
			visualizeDatas(data);
			Serial.print("sent messageType: ");
			Serial.println(messageType);
		}
		

		if(messageType == POLL) {
			DW1000Time timePollSent;
			DW1000.getTransmitTimestamp(timePollSent);
			myStaticAnchor->timePollSent = timePollSent;

			if(DEBUG) {
				Serial.print("POLL sent at timestamp: ");
				myStaticAnchor->timePollSent.print();
			}
		} else if(messageType == RANGE) {
			DW1000Time timeRangeSent;
			DW1000.getTransmitTimestamp(timeRangeSent);
			myStaticAnchor->timeRangeSent = timeRangeSent;

			if(DEBUG) {
				Serial.print("RANGE sent at timestamp: ");
				myStaticAnchor->timeRangeSent.print();
			}
		}		
	}
		

	if(_receivedAck) {
		//TODO check recepient
		_receivedAck = false;
		DW1000.getData(data, LEN_DATA);
		byte destination_address_short_byte[2];
		_globalMac.decodeDestenationMACFrame(data, destination_address_short_byte);
		//TODO as part of ^ read data before ifs
		if(destination_address_short_byte[0] == _currentShortAddress[0] && destination_address_short_byte[1] == _currentShortAddress[1]) {
			//TODO handle unexpected message type
			int messageType = detectMessageType(data);

				if(messageType != _expectedMsgId) {
					if(DEBUG) {
						Serial.println("Received unexpected message type. Communication terminated.");
						Serial.print("Expected: ");
						Serial.println(_expectedMsgId);
						Serial.print("Received: ");
						Serial.println(messageType);
					}	
					beginProtocol();
					startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY, false);
					return;
				}

			noteActivity();
			timeOutCounter = 0;

			if(_expectedMsgId == POLL_ACK) {
				//_receivedAck = false;
				//DW1000.getData(data, LEN_DATA);
				if(DEBUG) {
					Serial.println("POLL_ACK expected, data received: ");
					visualizeDatas(data);
				}
				

				//TODO handle unexpected message type
				if(messageType == POLL_ACK) {
					if(DEBUG) {
						Serial.println("POLL_ACK RECIEVED");
					}
					DW1000.getReceiveTimestamp(myStaticAnchor->timePollAckReceived);
					//send FINAL to POLL recepient
					transmitRange(myStaticAnchor);
					noteActivity();
					
					_expectedMsgId = RANGE_REPORT;
					if(DEBUG) {
						Serial.print(" at timestamp: ");
						myStaticAnchor->timePollAckReceived.print();
						Serial.println("RANGE SENT");
					}
				}
			}
			// expect REPORT
			else if(_expectedMsgId == RANGE_REPORT) {
				//_receivedAck = false;
				//detect data type 
				//DW1000.getData(data, LEN_DATA);
				
				if(DEBUG) {
					Serial.println("POLL_ACK expected, data received: ");
					visualizeDatas(data);
					Serial.println(messageType);
				}
				
				//TODO handle unexpected message type
				if(messageType == RANGE_REPORT) {
					if(DEBUG) {
						Serial.println("RANGE_REPORT RECIEVED");
					}
					//read and decode REPORT
					float curRange;
					memcpy(&curRange, data+1+SHORT_MAC_LEN, 4);
					float curRXPower;
					memcpy(&curRXPower, data+5+SHORT_MAC_LEN, 4);
					++cycleCounter;
					printShortAddresses();
					if(!minimalSerialPrint) {
					Serial.print("Range: ");
					}
					Serial.println(curRange);
					// curRXPower/=100.0f;


					//spit out data to ROS
					/*if(_handleNewRange != 0) {
						(*_handleNewRange)();
					}*/
					//prepare for another round
					
					//DW1000RangingClass::initProtocol=true;
					//resets protocol to deafult settings
					//beginProtocol();
				    
					//wait for 250ms	
					currentTimeStamp = millis();
					protocolEnd = true;
				}
			}
		}
		else {
			if(DEBUG) {
				Serial.println("Incorrect message recipient. Ignoring request.");
			}
			//_receivedAck = false;
		}
	}
}

void DW1000RangingClass::loop_anchor() {
	// if(_expectedMsgId != POLL) {
	// 	timeoutANCHOR();
	// }

	checkForReset();
	
	if(_sentAck) {
		_sentAck = false;
		int messageType = detectMessageType(data);
		if(messageType == POLL_ACK) {
			if (myStaticTag) {
				DW1000.getTransmitTimestamp(myStaticTag->timePollAckSent);
			}
		}

		if(DEBUG) {
			if(messageType == RANGE_REPORT) {
				Serial.println("Range report sent^");
			}
			Serial.println("Data sent:");
			visualizeDatas(data);
		}
	} 
	
	if(_receivedAck) {
		_receivedAck = false;
		DW1000.getData(data, LEN_DATA);
		if(DEBUG) {
			visualizeDatas(data);
		}

		byte destenation_address_short_byte[2];
		_globalMac.decodeDestenationMACFrame(data, destenation_address_short_byte);

		if(DEBUG) {
			Serial.print("Own address: ");
			displayShortAddress(_currentShortAddress);
			Serial.print("Decoded address: ");
			displayShortAddress(destenation_address_short_byte);
		}
		
		if(destenation_address_short_byte[0] == _currentShortAddress[0] && destenation_address_short_byte[1] == _currentShortAddress[1]){
			noteActivity();
			if(DEBUG) {
				Serial.println("self address matched!");
			}

			int messageType = detectMessageType(data);
			//TODO - reset protocole
			if(messageType != _expectedMsgId) {
				if(DEBUG) {
					Serial.println("Unexpected msg type!");
					Serial.print("Expected message: ");
					Serial.println(_expectedMsgId);
					Serial.print("Received message: ");
					Serial.println(messageType);
				}
				resetAnchor();
				return;
			}

			
			//exepect POLL
			if(messageType == POLL) {
				if(DEBUG) {
					Serial.print("POLL received");
				}
				byte tag_address_short_byte[2];
				_globalMac.decodeShortMACFrame(data, tag_address_short_byte);
				//TODO - save address as next FINAL recepient
				myStaticTag = new DW1000Device(tag_address_short_byte, true);
				// myStaticTag->setReplyTime(DEFAULT_REPLY_DELAY_TIME);

				_expectedMsgId = RANGE;

				uint16_t replyTime;

				//TMP
				// memcpy(&replyTime, data+SHORT_MAC_LEN+2, 2);
				replyTime = DEFAULT_REPLY_DELAY_TIME;

				//we configure our replyTime;
				if(DEBUG) {
					Serial.print("Reply time: ");
					Serial.println(replyTime);
				}
				
				_replyDelayTimeUS = replyTime;

				DW1000.getReceiveTimestamp(myStaticTag->timePollReceived);
				transmitPollAck(myStaticTag);
				noteActivity();

				if(DEBUG) {
					Serial.print(" at timestamp: ");
					myStaticTag->timePollReceived.print();
					Serial.println("Sending POLL_ACK");
				}

			} else if(messageType == RANGE) {
				if(DEBUG) {
					Serial.println("RANGE RECIEVED");
				}
				DW1000.getReceiveTimestamp(myStaticTag->timeRangeReceived);
				
				myStaticTag->timePollSent.setTimestamp(data+10);
				myStaticTag->timePollAckReceived.setTimestamp(data+15);
				myStaticTag->timeRangeSent.setTimestamp(data+20);

				DW1000Time myTOF;
				computeRangeAsymmetric(myStaticTag, &myTOF); 
				float distance = myTOF.getAsMeters();
				Serial.print("distance: ");
				Serial.println(distance);

				myStaticTag->setRXPower(DW1000.getReceivePower());
				myStaticTag->setRange(distance);
				
				myStaticTag->setFPPower(DW1000.getFirstPathPower());
				myStaticTag->setQuality(DW1000.getReceiveQuality());
				if(DEBUG)
				{
				cycleCounter++;
				Serial.print("Cycle number: ");
				Serial.println(cycleCounter);
				}
				//we send the range to TAG
				transmitRangeReport(myStaticTag);
				noteActivity();
				if(DEBUG) {
					Serial.println("RANGE_REPORT SENDING");
				}

				_expectedMsgId = POLL; //??
			}	
		}
	}

	
}

void DW1000RangingClass::loop() {
	//we check if needed to reset !
	checkForReset();
	uint32_t time = millis(); // TODO other name - too close to "timer"
	if(time-timer > _timerDelay) {
		timer = time;
		timerTick();
	}
	
	if(_sentAck) {
		_sentAck = false;
		
		// TODO cc
		int messageType = detectMessageType(data);

		if(DEBUG) {
			visualizeDatas(data);
			Serial.print("Sent reply time: ");
			Serial.println(_replyDelayTimeUS);
		}
		
		if(messageType != POLL_ACK && messageType != POLL && messageType != RANGE)
			return;
		
		//A msg was sent. We launch the ranging protocole when a message was sent
		if(_type == ANCHOR) {
			if(messageType == POLL_ACK) {
				DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
				
				if (myDistantDevice) {
					DW1000.getTransmitTimestamp(myDistantDevice->timePollAckSent);
				}
			}
		}
		else if(_type == TAG) {
			if(messageType == POLL) {
				DW1000Time timePollSent;
				DW1000.getTransmitTimestamp(timePollSent);
				//if the last device we send the POLL is broadcast:
				if(_lastSentToShortAddress[0] == 0xFF && _lastSentToShortAddress[1] == 0xFF) {
					//we save the value for all the devices !
					for(uint16_t i = 0; i < _networkDevicesNumber; i++) {
						_networkDevices[i].timePollSent = timePollSent;
					}
				}
				else {
					//we search the device associated with the last send address
					DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
					//we save the value just for one device
					if (myDistantDevice) {
						myDistantDevice->timePollSent = timePollSent;
					}
				}
			}
			else if(messageType == RANGE) {
				DW1000Time timeRangeSent;
				DW1000.getTransmitTimestamp(timeRangeSent);
				//if the last device we send the POLL is broadcast:
				if(_lastSentToShortAddress[0] == 0xFF && _lastSentToShortAddress[1] == 0xFF) {
					//we save the value for all the devices !
					for(uint16_t i = 0; i < _networkDevicesNumber; i++) {
						_networkDevices[i].timeRangeSent = timeRangeSent;
					}
				}
				else {
					//we search the device associated with the last send address
					DW1000Device* myDistantDevice = searchDistantDevice(_lastSentToShortAddress);
					//we save the value just for one device
					if (myDistantDevice) {
						myDistantDevice->timeRangeSent = timeRangeSent;
					}
				}
				
			}
		}
		
	}
	
	//check for new received message
	if(_receivedAck) {
		_receivedAck = false;
		
		//we read the datas from the modules:
		// get message and parse
		DW1000.getData(data, LEN_DATA);
		if(DEBUG) {
			visualizeDatas(data);
			Serial.print("Recieved reply time: ");
			Serial.println(_replyDelayTimeUS);
		}
		
		int messageType = detectMessageType(data);
		
		//we have just received a BLINK message from tag
		if(messageType == BLINK && _type == ANCHOR) {
			byte address[8];
			byte shortAddress[2];
			_globalMac.decodeBlinkFrame(data, address, shortAddress);
			displayFullAddress(address);
			//we crate a new device with th tag
			DW1000Device myTag(address, shortAddress);
			
			if(addNetworkDevices(&myTag)) {
				if(_handleBlinkDevice != 0) {
					(*_handleBlinkDevice)(&myTag);
				}
				//we reply by the transmit ranging init message
				transmitRangingInit(&myTag);
				noteActivity();
			}
			_expectedMsgId = POLL;
		}
		else if(messageType == RANGING_INIT && _type == TAG) {
			
			byte address[2];
			_globalMac.decodeLongMACFrame(data, address);
			//we crate a new device with the anchor
			DW1000Device myAnchor(address, true);
			
			if(addNetworkDevices(&myAnchor, true)) {
				if(_handleNewDevice != 0) {
					(*_handleNewDevice)(&myAnchor);
				}
			}
			
			noteActivity();
		}
		else {
			//we have a short mac layer frame !
			byte address[2];
			_globalMac.decodeShortMACFrame(data, address);
			
			
			
			//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			DW1000Device* myDistantDevice = searchDistantDevice(address);
			
			
			if((_networkDevicesNumber == 0) || (myDistantDevice == nullptr)) {
				//we don't have the short address of the device in memory
				if (DEBUG) {
					Serial.println("Not found");
					/*
					Serial.print("unknown: ");
					Serial.print(address[0], HEX);
					Serial.print(":");
					Serial.println(address[1], HEX);
					*/
				}
				return;
			}
			
			
			//then we proceed to range protocole
			if(_type == ANCHOR) {
				if(messageType != _expectedMsgId) {
					// unexpected message, start over again (except if already POLL)
					_protocolFailed = true;
				}
				if(messageType == POLL) {
					//we receive a POLL which is a broacast message
					//we need to grab info about it
					int16_t numberDevices = 0;
					memcpy(&numberDevices, data+SHORT_MAC_LEN+1, 1);
					
					for(uint16_t i = 0; i < numberDevices; i++) {
						//we need to test if this value is for us:
						//we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, data+SHORT_MAC_LEN+2+i*4, 2);
						
						//we test if the short address is our address
						if(shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
							//we grab the replytime wich is for us
							uint16_t replyTime;
							memcpy(&replyTime, data+SHORT_MAC_LEN+2+i*4+2, 2);
							//we configure our replyTime;
							_replyDelayTimeUS = replyTime;
							
							// on POLL we (re-)start, so no protocol failure
							_protocolFailed = false;
							
							DW1000.getReceiveTimestamp(myDistantDevice->timePollReceived);
							//we note activity for our device:
							myDistantDevice->noteActivity();
							//we indicate our next receive message for our ranging protocole
							_expectedMsgId = RANGE;
							transmitPollAck(myDistantDevice);
							noteActivity();
							
							return;
						}
						
					}
					
					
				}
				else if(messageType == RANGE) {
					//we receive a RANGE which is a broacast message
					//we need to grab info about it
					uint8_t numberDevices = 0;
					memcpy(&numberDevices, data+SHORT_MAC_LEN+1, 1);
					
					
					for(uint8_t i = 0; i < numberDevices; i++) {
						//we need to test if this value is for us:
						//we grab the mac address of each devices:
						byte shortAddress[2];
						memcpy(shortAddress, data+SHORT_MAC_LEN+2+i*17, 2);
						
						//we test if the short address is our address
						if(shortAddress[0] == _currentShortAddress[0] && shortAddress[1] == _currentShortAddress[1]) {
							//we grab the replytime wich is for us
							DW1000.getReceiveTimestamp(myDistantDevice->timeRangeReceived);
							noteActivity();
							_expectedMsgId = POLL;
							
							if(!_protocolFailed) {
								
								myDistantDevice->timePollSent.setTimestamp(data+SHORT_MAC_LEN+4+17*i);
								myDistantDevice->timePollAckReceived.setTimestamp(data+SHORT_MAC_LEN+9+17*i);
								myDistantDevice->timeRangeSent.setTimestamp(data+SHORT_MAC_LEN+14+17*i);
								
								// (re-)compute range as two-way ranging is done
								DW1000Time myTOF;
								computeRangeAsymmetric(myDistantDevice, &myTOF); // CHOSEN RANGING ALGORITHM
								
								float distance = myTOF.getAsMeters();
								
								if (_useRangeFilter) {
									//Skip first range
									if (myDistantDevice->getRange() != 0.0f) {
										distance = filterValue(distance, myDistantDevice->getRange(), _rangeFilterValue);
									}
								}
								
								myDistantDevice->setRXPower(DW1000.getReceivePower());
								myDistantDevice->setRange(distance);
								
								myDistantDevice->setFPPower(DW1000.getFirstPathPower());
								myDistantDevice->setQuality(DW1000.getReceiveQuality());
								
								//we send the range to TAG
								transmitRangeReport(myDistantDevice);
								
								//we have finished our range computation. We send the corresponding handler
								_lastDistantDevice = myDistantDevice->getIndex();
								if(_handleNewRange != 0) {
									(*_handleNewRange)();
								}
								
							}
							else {
								transmitRangeFailed(myDistantDevice);
							}
							
							
							return;
						}
						
					}
					
					
				}
			}
			else if(_type == TAG) {
				// get message and parse
				if(messageType != _expectedMsgId) {
					// unexpected message, start over again
					//not needed ?
					return;
					_expectedMsgId = POLL_ACK;
					return;
				}
				if(messageType == POLL_ACK) {
					DW1000.getReceiveTimestamp(myDistantDevice->timePollAckReceived);
					//we note activity for our device:
					myDistantDevice->noteActivity();
					
					//in the case the message come from our last device:
					if(myDistantDevice->getIndex() == _networkDevicesNumber-1) {
						_expectedMsgId = RANGE_REPORT;
						//and transmit the next message (range) of the ranging protocole (in broadcast)
						transmitRange(nullptr);
					}
				}
				else if(messageType == RANGE_REPORT) {
					
					float curRange;
					memcpy(&curRange, data+1+SHORT_MAC_LEN, 4);
					float curRXPower;
					memcpy(&curRXPower, data+5+SHORT_MAC_LEN, 4);
					
					if (_useRangeFilter) {
						//Skip first range
						if (myDistantDevice->getRange() != 0.0f) {
							curRange = filterValue(curRange, myDistantDevice->getRange(), _rangeFilterValue);
						}
					}

					//we have a new range to save !
					myDistantDevice->setRange(curRange);
					myDistantDevice->setRXPower(curRXPower);
					
					
					//We can call our handler !
					//we have finished our range computation. We send the corresponding handler
					_lastDistantDevice = myDistantDevice->getIndex();
					if(_handleNewRange != 0) {
						(*_handleNewRange)();
					}
				}
				else if(messageType == RANGE_FAILED) {
					//not needed as we have a timer;
					return;
					_expectedMsgId = POLL_ACK;
				}
			}
		}
		
	}
}

void DW1000RangingClass::useRangeFilter(boolean enabled) {
	_useRangeFilter = enabled;
}

void DW1000RangingClass::setRangeFilterValue(uint16_t newValue) {
	if (newValue < 2) {
		_rangeFilterValue = 2;
	}else{
		_rangeFilterValue = newValue;
	}
}


/* ###########################################################################
 * #### Private methods and Handlers for transmit & Receive reply ############
 * ######################################################################### */


void DW1000RangingClass::handleSent() {
	// status change on sent success
	_sentAck = true;
}

void DW1000RangingClass::handleReceived() {
	// status change on received success
	_receivedAck = true;
}


void DW1000RangingClass::noteActivity() {
	// update activity timestamp, so that we do not reach "resetPeriod"
	if(DEBUG) {
		Serial.println("Activity noted");
	}
	_lastActivity = millis();
}

void DW1000RangingClass::resetInactive() {
	//if inactive
	if(_type == ANCHOR) {
		_expectedMsgId = POLL;
		receiver();
	}
	noteActivity();
}

void DW1000RangingClass::timerTick() {
	if(_networkDevicesNumber > 0 && counterForBlink != 0) {
		if(_type == TAG) {
			_expectedMsgId = POLL_ACK;
			//send a prodcast poll
			transmitPoll(nullptr);
		}
	}
	else if(counterForBlink == 0) {
		if(_type == TAG) {
			transmitBlink();
		}
		//check for inactive devices if we are a TAG or ANCHOR
		checkForInactiveDevices();
	}
	counterForBlink++;
	if(counterForBlink > 20) {
		counterForBlink = 0;
	}
}


void DW1000RangingClass::copyShortAddress(byte address1[], byte address2[]) {
	*address1     = *address2;
	*(address1+1) = *(address2+1);
}

/* ###########################################################################
 * #### Methods for ranging protocole   ######################################
 * ######################################################################### */

void DW1000RangingClass::transmitInit() {
	DW1000.newTransmit();
	DW1000.setDefaults();
}

void DW1000RangingClass::printShortAddresses() {
	if(!minimalSerialPrint) {
		Serial.print("From: ");
		displayShortAddress(destinationAddress);
		Serial.print(" To: ");
		displayShortAddress(_currentShortAddress);
		Serial.print(" ");
	}
	else {
		displayShortAddress(destinationAddress);
		displayShortAddress(_currentShortAddress);
	}
}


void DW1000RangingClass::transmit(byte datas[]) {
	DW1000.setData(datas, LEN_DATA);
	DW1000.startTransmit();
}


void DW1000RangingClass::transmit(byte datas[], DW1000Time time) {
	DW1000.setDelay(time);
	DW1000.setData(data, LEN_DATA);
	DW1000.startTransmit();
}

void DW1000RangingClass::transmitBlink() {
	transmitInit();
	_globalMac.generateBlinkFrame(data, _currentAddress, _currentShortAddress);
	transmit(data);
}

void DW1000RangingClass::transmitRangingInit(DW1000Device* myDistantDevice) {
	transmitInit();
	//we generate the mac frame for a ranging init message
	_globalMac.generateLongMACFrame(data, _currentShortAddress, myDistantDevice->getByteAddress());
	//we define the function code
	data[LONG_MAC_LEN] = RANGING_INIT;
	
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	
	transmit(data);
}

void DW1000RangingClass::transmitPoll(DW1000Device* myDistantDevice) {
	
	transmitInit();
	
	if(myDistantDevice == nullptr) {
		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/1000);
		
		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN]   = POLL;
		//we enter the number of devices
		data[SHORT_MAC_LEN+1] = _networkDevicesNumber;
		
		for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
			//each devices have a different reply delay time.
			_networkDevices[i].setReplyTime((2*i+1)*DEFAULT_REPLY_DELAY_TIME);
			//we write the short address of our device:
			memcpy(data+SHORT_MAC_LEN+2+4*i, _networkDevices[i].getByteShortAddress(), 2);
			
			//we add the replyTime
			uint16_t replyTime = _networkDevices[i].getReplyTime();
			memcpy(data+SHORT_MAC_LEN+2+2+4*i, &replyTime, 2);
			
		}
		
		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
		
	}
	else {
		//we redefine our default_timer_delay for just 1 device;
		_timerDelay = DEFAULT_TIMER_DELAY;
		
		_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
		
		data[SHORT_MAC_LEN]   = POLL;
		data[SHORT_MAC_LEN+1] = 1;
		uint16_t replyTime = myDistantDevice->getReplyTime();
		memcpy(data+SHORT_MAC_LEN+2, &replyTime, sizeof(uint16_t)); // todo is code correct?
		
		copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	}
	
	transmit(data);
}


void DW1000RangingClass::transmitPollAck(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = POLL_ACK;
	// delay the same amount as ranging tag
	DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data, deltaTime);
}

void DW1000RangingClass::transmitRange(DW1000Device* myDistantDevice) {
	//transmit range need to accept broadcast for multiple anchor
	transmitInit();
	
	
	if(myDistantDevice == nullptr) {
		//we need to set our timerDelay:
		_timerDelay = DEFAULT_TIMER_DELAY+(uint16_t)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/1000);
		
		byte shortBroadcast[2] = {0xFF, 0xFF};
		_globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
		data[SHORT_MAC_LEN]   = RANGE;
		//we enter the number of devices
		data[SHORT_MAC_LEN+1] = _networkDevicesNumber;
		
		// delay sending the message and remember expected future sent timestamp
		DW1000Time deltaTime     = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW1000Time::MICROSECONDS);
		DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);
		
		for(uint8_t i = 0; i < _networkDevicesNumber; i++) {
			//we write the short address of our device:
			memcpy(data+SHORT_MAC_LEN+2+17*i, _networkDevices[i].getByteShortAddress(), 2);
			
			
			//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
			_networkDevices[i].timeRangeSent = timeRangeSent;
			_networkDevices[i].timePollSent.getTimestamp(data+SHORT_MAC_LEN+4+17*i);
			_networkDevices[i].timePollAckReceived.getTimestamp(data+SHORT_MAC_LEN+9+17*i);
			_networkDevices[i].timeRangeSent.getTimestamp(data+SHORT_MAC_LEN+14+17*i);
			
		}
		
		copyShortAddress(_lastSentToShortAddress, shortBroadcast);
		
	}
	else {
		_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
		data[SHORT_MAC_LEN] = RANGE;
		// delay sending the message and remember expected future sent timestamp
		DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS);
		//we get the device which correspond to the message which was sent (need to be filtered by MAC address)
		myDistantDevice->timeRangeSent = DW1000.setDelay(deltaTime);
		myDistantDevice->timePollSent.getTimestamp(data+1+SHORT_MAC_LEN);
		myDistantDevice->timePollAckReceived.getTimestamp(data+6+SHORT_MAC_LEN);
		myDistantDevice->timeRangeSent.getTimestamp(data+11+SHORT_MAC_LEN);
		copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	}
	
	
	transmit(data);
}


void DW1000RangingClass::transmitRangeReport(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = RANGE_REPORT;
	// write final ranging result
	float curRange   = myDistantDevice->getRange();
	float curRXPower = myDistantDevice->getRXPower();
	//We add the Range and then the RXPower
	memcpy(data+1+SHORT_MAC_LEN, &curRange, 4);
	memcpy(data+5+SHORT_MAC_LEN, &curRXPower, 4);
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data, DW1000Time(_replyDelayTimeUS, DW1000Time::MICROSECONDS));
}

void DW1000RangingClass::transmitRangeFailed(DW1000Device* myDistantDevice) {
	transmitInit();
	_globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
	data[SHORT_MAC_LEN] = RANGE_FAILED;
	
	copyShortAddress(_lastSentToShortAddress, myDistantDevice->getByteShortAddress());
	transmit(data);
}

void DW1000RangingClass::receiver() {
	DW1000.newReceive();
	DW1000.setDefaults();
	// so we don't need to restart the receiver manually
	DW1000.receivePermanently(true);
	DW1000.startReceive();
}

void DW1000RangingClass::initializeVariables(uint32_t timeoutTime, int resetCount, bool minimalPrint) {
	timeoutPeriod = timeoutTime;
	timeOutResetCount = resetCount;
	minimalSerialPrint = minimalPrint;

}

void DW1000RangingClass::decodeSerial(char serialString[]) {
	//Serial.println(serialString);
	//Serial.println(serialString);
	char anchorAddressChar[6] = {serialString[0], serialString[1], serialString[2], serialString[3], serialString[4]}; 
	Serial.println(anchorAddressChar);
	std::string numberOfProtocolsString;
	numberOfProtocolsString.push_back(serialString[5]);
	numberOfProtocolsString.push_back(serialString[6]);
	try {
	int numberOfProtocols = std::stoi(numberOfProtocolsString);
	Serial.println(numberOfProtocols);
	}
	catch(std::invalid_argument& e)
	{
		if(DEBUG)
		{
		Serial.println("Parsing failure.");
		}
	}
	
	
}

int DW1000RangingClass::getCycleCounter() {
	return cycleCounter;
}


/* ###########################################################################
 * #### Methods for range computation and corrections  #######################
 * ######################################################################### */


void DW1000RangingClass::computeRangeAsymmetric(DW1000Device* myDistantDevice, DW1000Time* myTOF) {
	// asymmetric two-way ranging (more computation intense, less error prone)
	DW1000Time round1 = (myDistantDevice->timePollAckReceived-myDistantDevice->timePollSent).wrap();
	DW1000Time reply1 = (myDistantDevice->timePollAckSent-myDistantDevice->timePollReceived).wrap();
	DW1000Time round2 = (myDistantDevice->timeRangeReceived-myDistantDevice->timePollAckSent).wrap();
	DW1000Time reply2 = (myDistantDevice->timeRangeSent-myDistantDevice->timePollAckReceived).wrap();
	
	myTOF->setTimestamp((round1*round2-reply1*reply2)/(round1+round2+reply1+reply2));
	if(DEBUG) {
		Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
		Serial.print("timePollSent ");myDistantDevice->timePollSent.print();
		Serial.print("round1 "); Serial.println((long)round1.getTimestamp());
		
		Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
		Serial.print("timePollReceived ");myDistantDevice->timePollReceived.print();
		Serial.print("reply1 "); Serial.println((long)reply1.getTimestamp());
		
		Serial.print("timeRangeReceived ");myDistantDevice->timeRangeReceived.print();
		Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
		Serial.print("round2 "); Serial.println((long)round2.getTimestamp());
		
		Serial.print("timeRangeSent ");myDistantDevice->timeRangeSent.print();
		Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
		Serial.print("reply2 "); Serial.println((long)reply2.getTimestamp());
	}
}


/* FOR DEBUGGING*/
void DW1000RangingClass::visualizeDatas(byte datas[]) {
	char string[60];
	sprintf(string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
					datas[0], datas[1], datas[2], datas[3], datas[4], datas[5], datas[6], datas[7], datas[8],
					datas[9], datas[10], datas[11], datas[12], datas[13], datas[14], datas[15],
					datas[16],datas[17],datas[18],datas[19]
					);
	Serial.println(string);
}

void DW1000RangingClass::displayFullAddress(byte datas[]) {	
	char string[60];
	sprintf(string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
					datas[0], datas[1], datas[2], datas[3], datas[4], datas[5], datas[6], datas[7]);
	Serial.println(string);
}

void DW1000RangingClass::displayShortAddress(byte datas[]) {	
	char string[60];
	sprintf(string, "%02X:%02X",
					datas[0], datas[1]);
	Serial.print(string);
}

void DW1000RangingClass::beginProtocol() {
	initProtocol = true;
	myStaticAnchor = nullptr;
	myStaticTag = nullptr;
	noteActivity();
}



/* ###########################################################################
 * #### Utils  ###############################################################
 * ######################################################################### */

float DW1000RangingClass::filterValue(float value, float previousValue, uint16_t numberOfElements) {
	
	float k = 2.0f / ((float)numberOfElements + 1.0f);
	return (value * k) + previousValue * (1.0f - k);
}



