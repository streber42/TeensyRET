/*
 GEV-RET.ino

 Created: 7/2/2014 10:10:14 PM
 Author: Collin Kidder

Copyright (c) 2014-2015 Collin Kidder, Michael Neuweiler, Charles Galpin

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */

#include <Arduino.h>
#include "TeensyRET.h"
#include "config.h"
#include <EEPROM.h>
#include <FlexCAN_T4.h>
#include <Wire.h>
#include <SdFat.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;

byte i = 0;

byte serialBuffer[SER_BUFF_SIZE];
int serialBufferLength = 0; //not creating a ring buffer. The buffer should be large enough to never overflow
uint32_t lastFlushMicros = 0;

EEPROMSettings settings;
SystemSettings SysSettings;
DigitalCANToggleSettings digToggleSettings;

// file system on sdcard
SdFile sd;

SerialConsole console;

bool digTogglePinState;
uint8_t digTogglePinCounter;

//initializes all the system EEPROM values. Chances are this should be broken out a bit but
//there is only one checksum check for all of them so it's simple to do it all here.
void loadSettings()
{
	EEPROM.get(EEPROM_ADDRESS, settings);

	if (settings.version != EEPROM_VER) //if settings are not the current version then erase them and set defaults
	{
		Logger::console("Resetting to factory defaults");
		settings.version = EEPROM_VER;
		settings.appendFile = false;
		settings.CAN0Speed = 500000;
		settings.CAN0_Enabled = false;
		settings.CAN1Speed = 500000;
		settings.CAN1_Enabled = false;
		sprintf((char *)settings.fileNameBase, "CANBUS");
		sprintf((char *)settings.fileNameExt, "TXT");
		settings.fileNum = 1;
		for (int i = 0; i < 6; i++) 
		{
			settings.CAN0Filters[i].enabled = true;
			settings.CAN0Filters[i].extended = true;
			settings.CAN0Filters[i].id = 0;
			settings.CAN0Filters[i].mask = 0;
			settings.CAN1Filters[i].enabled = true;
			settings.CAN1Filters[i].extended = true;
			settings.CAN1Filters[i].id = 0;
			settings.CAN1Filters[i].mask = 0;
		}
		for (int j = 6; j < 14; j++)
		{
			settings.CAN0Filters[j].enabled = true;
			settings.CAN0Filters[j].extended = false;
			settings.CAN0Filters[j].id = 0;
			settings.CAN0Filters[j].mask = 0;
			settings.CAN1Filters[j].enabled = true;
			settings.CAN1Filters[j].extended = false;
			settings.CAN1Filters[j].id = 0;
			settings.CAN1Filters[j].mask = 0;
		}
		settings.fileOutputType = CRTD;
		settings.useBinarySerialComm = false;
		settings.autoStartLogging = false;
		settings.logLevel = 1; //info
		settings.sysType = 0; //CANDUE as default
		settings.valid = 0; //not used right now
		settings.CAN0ListenOnly = false;
		settings.CAN1ListenOnly = false;
		EEPROM.put(EEPROM_ADDRESS, settings);
	}
	else {
		Logger::console("Using stored values from EEPROM");
        if (settings.CAN0ListenOnly > 1) settings.CAN0ListenOnly = 0;
        if (settings.CAN1ListenOnly > 1) settings.CAN1ListenOnly = 0;
	}
	
	EEPROM.get(EEPROM_ADDRESS + 500, digToggleSettings);
	if (digToggleSettings.mode == 255)
    {
        Logger::console("Resetting digital toggling system to defaults");
        digToggleSettings.enabled = false;
        digToggleSettings.length = 0;
        digToggleSettings.mode = 0;
        digToggleSettings.pin = 1;
        digToggleSettings.rxTxID = 0x700;
        for (int c=0 ; c<8 ; c++) digToggleSettings.payload[c] = 0;
        EEPROM.put(EEPROM_ADDRESS + 500, digToggleSettings);        
    }
    else
    {
        Logger::console("Using stored values for digital toggling system");
    }
    
	Logger::setLoglevel((Logger::LogLevel)settings.logLevel);

	SysSettings.SDCardInserted = false;

	//switch (settings.sysType) {
        
		//case 1:  
			Logger::console("Running on Teensy Hardware");
			SysSettings.CAN0EnablePin = 2;
			SysSettings.CAN1EnablePin = 35;
			SysSettings.LED_CANTX = 13; //We do have an LED at pin 13. Use it for both
			SysSettings.LED_CANRX = 13; //RX and TX.
			SysSettings.LED_LOGGING = 255; //we just don't have an LED to use for this.
			pinMode(13, OUTPUT);
			digitalWrite(13, LOW);
			//break;
    //}
	if (SysSettings.CAN0EnablePin != 255) pinMode(SysSettings.CAN0EnablePin, OUTPUT);
	if (SysSettings.CAN1EnablePin != 255) pinMode(SysSettings.CAN1EnablePin, OUTPUT);
}

void canSniff20(const CAN_message_t &msg) { // global callback
  Serial.print("T4: ");
  Serial.print("MB "); Serial.print(msg.mb);
  Serial.print(" OVERRUN: "); Serial.print(msg.flags.overrun);
  Serial.print(" BUS "); Serial.print(msg.bus);
  Serial.print(" LEN: "); Serial.print(msg.len);
  Serial.print(" EXT: "); Serial.print(msg.flags.extended);
  Serial.print(" REMOTE: "); Serial.print(msg.flags.remote);
  Serial.print(" TS: "); Serial.print(msg.timestamp);
  Serial.print(" ID: "); Serial.print(msg.id, HEX);
  Serial.print(" IDHIT: "); Serial.print(msg.idhit);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  } Serial.println();
  /*if (SerialUSB)*/ bool isConnected = true;
  toggleRXLED();
  if (isConnected) sendFrameToUSB(msg, 0);
  if (SysSettings.logToFile) sendFrameToFile(msg, 0);
  if (digToggleSettings.enabled && (digToggleSettings.mode & 1) && (digToggleSettings.mode & 2)) processDigToggleFrame(msg);
  //fwGotFrame(&incoming);
}

void setup()
{
	delay(5000); //just for testing. Don't use in production
    pinMode(BLINK_LED, OUTPUT);
    digitalWrite(BLINK_LED, LOW);

    Serial.begin(115200);
	Wire.begin();

	loadSettings();

    if (SysSettings.useSD) {	
		if (!sd.availableForWrite())
		{
			Logger::error("Could not initialize SDCard! No file logging will be possible!");
		}
		else SysSettings.SDCardInserted = true;
		if (settings.autoStartLogging) {
			SysSettings.logToFile = true;
			Logger::info("Automatically logging to file.");
		}
	}

    Serial.print("Build number: ");
    Serial.println(CFG_BUILD_NUM);
    
    if (digToggleSettings.enabled)
    {
        Serial.println("Digital Toggle System Enabled");
        if (digToggleSettings.mode & 1) { //input CAN and output pin state mode
            Serial.println("In Output Mode");
            pinMode(digToggleSettings.pin, OUTPUT);
            if (digToggleSettings.mode & 0x80) {
                digitalWrite(digToggleSettings.pin, LOW);
                digTogglePinState = false;
            }
            else {
                digitalWrite(digToggleSettings.pin, HIGH);
                digTogglePinState = true;
            }
        }
        else { //read pin and output CAN mode
            Serial.println("In Input Mode");
            pinMode(digToggleSettings.pin, INPUT);
            digTogglePinCounter = 0;
            if (digToggleSettings.mode & 0x80) digTogglePinState = false;
            else digTogglePinState = true;          
        }
    }

	if (settings.CAN0_Enabled)
	{
        Serial.println("Init CAN0");
		Can0.begin();
    	Can0.setBaudRate(500000);
    	Can0.enableFIFO();
    	Can0.enableFIFOInterrupt();
    	Can0.onReceive(FIFO, canSniff20);
    	Can0.mailboxStatus();

        if (SysSettings.CAN0EnablePin < 255) 
        {
            pinMode(SysSettings.CAN0EnablePin, OUTPUT);
            digitalWrite(SysSettings.CAN0EnablePin, HIGH);
        }
        // if (settings.CAN0ListenOnly)
        // {
        //     Can0.setListenOnly(true);
        // }
        // else
        // {
        //     Can0.setListenOnly(false);
        // }
	}
	else {
        Serial.println("CAN0 disabled.");
        //TODO: apparently calling end while it isn't inialized actually locks it up
        //Can0.end();
    }
	if (settings.CAN1_Enabled)
	{
        Serial.println("Init CAN0");
		Can1.begin();
		Can1.setBaudRate(500000);
    	Can1.enableFIFO();
    	Can1.enableFIFOInterrupt();
    	Can1.onReceive(FIFO, canSniff20);
    	Can1.mailboxStatus();

        if (SysSettings.CAN1EnablePin < 255)
        {
            pinMode(SysSettings.CAN1EnablePin, OUTPUT);
            digitalWrite(SysSettings.CAN1EnablePin, HIGH);
        }
        // if (settings.CAN1ListenOnly)
        // {
        //     Can1.setListenOnly(true);
        // }
        // else
        // {
        //     Can1.setListenOnly(false);
        // }        
	}
	else {
        Serial.println("CAN1 disabled.");
        //Can1.end();
    }
    /*
	for (int i = 0; i < 7; i++) 
	{
		if (settings.CAN0Filters[i].enabled) 
		{
			Can0.setRXFilter(i, settings.CAN0Filters[i].id,
				settings.CAN0Filters[i].mask, settings.CAN0Filters[i].extended);
		}
		if (settings.CAN1Filters[i].enabled)
		{
			Can1.setRXFilter(i, settings.CAN1Filters[i].id,
				settings.CAN1Filters[i].mask, settings.CAN1Filters[i].extended);
		}
	}
	*/
	
	SysSettings.lawicelMode = false;
	SysSettings.lawicelAutoPoll = false;
	SysSettings.lawicelTimestamping = false;
	SysSettings.lawicelPollCounter = 0;

	Serial.print("Done with init\n");
	digitalWrite(BLINK_LED, HIGH);
}

void setPromiscuousMode() {
   //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept extended
  //or standard frames
  int filter;
  //extended
  /*
  for (filter = 0; filter < 3; filter++) {
	Can0.setRXFilter(filter, 0, 0, true);
	Can1.setRXFilter(filter, 0, 0, true);
  }  
  //standard
  for (filter = 3; filter < 7; filter++) {
	Can0.setRXFilter(filter, 0, 0, false);
	Can1.setRXFilter(filter, 0, 0, false);
  } 
  */
}

//Get the value of XOR'ing all the bytes together. This creates a reasonable checksum that can be used
//to make sure nothing too stupid has happened on the comm.
uint8_t checksumCalc(uint8_t *buffer, int length) 
{
	uint8_t valu = 0;
	for (int c = 0; c < length; c++) {
		valu ^= buffer[c];
	}
	return valu;
}

void toggleRXLED()
{
	SysSettings.rxToggle = !SysSettings.rxToggle;
	//setLED(SysSettings.LED_CANRX, SysSettings.rxToggle);
}

void sendFrameToUSB(const CAN_message_t &in_frame, int whichBus) 
{
	uint8_t buff[22];
	uint8_t temp;
	uint32_t now = micros();
	CAN_message_t frame = in_frame;

	if (SysSettings.lawicelMode)
	{
		if (frame.flags.extended)
		{
			Serial.print("T");
			sprintf((char *)buff, "%08x", frame.id);
			Serial.print((char *)buff);
		}
		else
		{
			Serial.print("t");
			sprintf((char *)buff, "%03x", frame.id);
			Serial.print((char *)buff);
		}
		Serial.print(frame.len);
		for (int i = 0; i < frame.len; i++)
		{
			sprintf((char *)buff, "%02x", frame.buf[i]);
			Serial.print((char *)buff);
		}
		if (SysSettings.lawicelTimestamping)
		{
			uint16_t timestamp = (uint16_t)millis();
			sprintf((char *)buff, "%04x", timestamp);
			Serial.print((char *)buff);
		}
		Serial.write(13);
	}
	else
	{
		if (settings.useBinarySerialComm) {
			if (frame.flags.extended) frame.id |= 1 << 31;
			serialBuffer[serialBufferLength++] = 0xF1;
			serialBuffer[serialBufferLength++] = 0; //0 = canbus frame sending
			serialBuffer[serialBufferLength++] = (uint8_t)(now & 0xFF);
			serialBuffer[serialBufferLength++] = (uint8_t)(now >> 8);
			serialBuffer[serialBufferLength++] = (uint8_t)(now >> 16);
			serialBuffer[serialBufferLength++] = (uint8_t)(now >> 24);
			serialBuffer[serialBufferLength++] = (uint8_t)(frame.id & 0xFF);
			serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 8);
			serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 16);
			serialBuffer[serialBufferLength++] = (uint8_t)(frame.id >> 24);
			serialBuffer[serialBufferLength++] = frame.len + (uint8_t)(whichBus << 4);
			for (int c = 0; c < frame.len; c++)
			{
				serialBuffer[serialBufferLength++] = frame.buf[c];
			}
			//temp = checksumCalc(buff, 11 + frame.length);
			temp = 0;
			serialBuffer[serialBufferLength++] = temp;
			//SerialUSB.write(buff, 12 + frame.length);
		}
		else 
		{
			Serial.print(micros());
			Serial.print(" - ");
			Serial.print(frame.id, HEX);
			if (frame.flags.extended) Serial.print(" X ");
			else Serial.print(" S ");
			Serial.print(whichBus);
			Serial.print(" ");
			Serial.print(frame.len);
			for (int c = 0; c < frame.len; c++)
			{
				Serial.print(" ");
				Serial.print(frame.buf[c], HEX);
			}
			Serial.println();
		}
	}
}

void sendFrameToFile(const CAN_message_t &in_frame, int whichBus)
{
	uint8_t buff[40];
	uint8_t temp;
	uint32_t timestamp;
	CAN_message_t frame = in_frame;

	if (settings.fileOutputType == BINARYFILE) {
		if (frame.flags.extended) frame.id |= 1 << 31;
		timestamp = micros();
		buff[0] = (uint8_t)(timestamp & 0xFF);
		buff[1] = (uint8_t)(timestamp >> 8);
		buff[2] = (uint8_t)(timestamp >> 16);
		buff[3] = (uint8_t)(timestamp >> 24);
		buff[4] = (uint8_t)(frame.id & 0xFF);
		buff[5] = (uint8_t)(frame.id >> 8);
		buff[6] = (uint8_t)(frame.id >> 16);
		buff[7] = (uint8_t)(frame.id >> 24);
		buff[8] = frame.len + (uint8_t)(whichBus << 4);
		for (int c = 0; c < frame.len; c++)
		{
			buff[9 + c] = frame.buf[c];
		}
		Logger::fileRaw(buff, 9 + frame.len);
	}
	else if (settings.fileOutputType == GVRET)
	{
		sprintf((char *)buff, "%i,%x,%i,%i,%i", millis(), frame.id, frame.flags.extended, whichBus, frame.len);
		Logger::fileRaw(buff, strlen((char *)buff));

		for (int c = 0; c < frame.len; c++)
		{
			sprintf((char *) buff, ",%x", frame.buf[c]);
			Logger::fileRaw(buff, strlen((char *)buff));
		}
		buff[0] = '\r';
		buff[1] = '\n';
		Logger::fileRaw(buff, 2);
	}
	else if (settings.fileOutputType == CRTD)
	{
		int idBits = 11;
		if (frame.flags.extended) idBits = 29;
		sprintf((char *)buff, "%f R%i %x", millis() / 1000.0f, idBits, frame.id);
		Logger::fileRaw(buff, strlen((char *)buff));

		for (int c = 0; c < frame.len; c++)
		{
			sprintf((char *) buff, " %x", frame.buf[c]);
			Logger::fileRaw(buff, strlen((char *)buff));
		}
		buff[0] = '\r';
		buff[1] = '\n';
		Logger::fileRaw(buff, 2);
	}
}

void processDigToggleFrame(const CAN_message_t &in_frame) {
	CAN_message_t frame = in_frame;

    bool gotFrame = false;
    if (digToggleSettings.rxTxID == frame.id)
    {
        if (digToggleSettings.length == 0) gotFrame = true;
        else {
            gotFrame = true;
            for (int c = 0; c < digToggleSettings.length; c++) {
                if (digToggleSettings.payload[c] != frame.buf[c]) {
                    gotFrame = false;
                    break;
                }
            }
        }
    }
    
    if (gotFrame) { //then toggle output pin
        Logger::console("Got special digital toggle frame. Toggling the output!");
        digitalWrite(digToggleSettings.pin, digTogglePinState?LOW:HIGH);
        digTogglePinState = !digTogglePinState;
    }
}

void sendDigToggleMsg() {
    CAN_message_t frame;
    Serial.println("Got digital input trigger.");
    frame.id = digToggleSettings.rxTxID;
    if (frame.id > 0x7FF) frame.flags.extended = 1;
    else frame.flags.extended = 0;
    frame.len = digToggleSettings.length;
    for (int c = 0; c < frame.len; c++) frame.buf[c] = digToggleSettings.payload[c];
    if (digToggleSettings.mode & 2) {
        Serial.println("Sending digital toggle message on CAN0");
        Can0.write(frame);
    }
    if (digToggleSettings.mode & 4) {
        Serial.println("Sending digital toggle message on CAN1");
        Can1.write(frame);
    }
}

/*
Loop executes as often as possible all the while interrupts fire in the background.
The serial comm protocol is as follows:
All commands start with 0xF1 this helps to synchronize if there were comm issues
Then the next byte specifies which command this is. 
Then the command data bytes which are specific to the command
Lastly, there is a checksum byte just to be sure there are no missed or duped bytes
Any bytes between checksum and 0xF1 are thrown away

Yes, this should probably have been done more neatly but this way is likely to be the
fastest and safest with limited function calls
*/
void loop()
{
	static int loops = 0;
	CAN_message_t incoming;
	static CAN_message_t build_out_frame;
	static int out_bus;
	int in_byte;
	static byte buff[20];
	static int step = 0;
	static STATE state = IDLE;
	static uint32_t build_int;
	uint8_t temp8;
	uint16_t temp16;
	static bool markToggle = false;
	bool isConnected = false;
	int serialCnt;
	uint32_t now = micros();

	/*if (SerialUSB)*/ isConnected = true;

	//if (!SysSettings.lawicelMode || SysSettings.lawicelAutoPoll || SysSettings.lawicelPollCounter > 0)
	//{
		Can0.events();
		Can1.events();
		if (SysSettings.lawicelPollCounter > 0) SysSettings.lawicelPollCounter--;
	//}
        
  if (digToggleSettings.enabled && !(digToggleSettings.mode & 1)) {
      if (digTogglePinState) { //pin currently high. Look for it going low
          if (!digitalRead(digToggleSettings.pin)) digTogglePinCounter++; //went low, increment debouncing counter 
          else digTogglePinCounter = 0; //whoops, it bounced or never transitioned, reset counter to 0
            
          if (digTogglePinCounter > 3) { //transitioned to LOW for 4 checks in a row. We'll believe it then.
              digTogglePinState = false;
              sendDigToggleMsg();
          }                
      }
      else { //pin currently low. Look for it going high
          if (digitalRead(digToggleSettings.pin)) digTogglePinCounter++; //went high, increment debouncing counter 
          else digTogglePinCounter = 0; //whoops, it bounced or never transitioned, reset counter to 0
            
          if (digTogglePinCounter > 3) { //transitioned to HIGH for 4 checks in a row. We'll believe it then.
              digTogglePinState = true;
              sendDigToggleMsg();
          }                          
      }      
  }

  if (micros() - lastFlushMicros > SER_BUFF_FLUSH_INTERVAL)
  {
	if (serialBufferLength > 0)
	{
		Serial.write(serialBuffer, serialBufferLength);
        	serialBufferLength = 0;
		lastFlushMicros = micros();
	}
  }

  serialCnt = 0;
  while (isConnected && (Serial.available() > 0) && serialCnt < 128) {
	serialCnt++;
	in_byte = Serial.read();
	   switch (state) {
	   case IDLE:
		   if (in_byte == 0xF1) state = GET_COMMAND;
		   else if (in_byte == 0xE7) 
		  {
			settings.useBinarySerialComm = true;
			SysSettings.lawicelMode = false;
		  }
		   else 
		   {
			   console.rcvCharacter((uint8_t)in_byte);
		   }
		   break;
	   case GET_COMMAND:
		   switch (in_byte) {
		   case 0:
			   state = BUILD_CAN_FRAME;
			   buff[0] = 0xF1;
			   step = 0;
			   break;
		   case 1:
			   state = TIME_SYNC;
			   step = 0;
			   buff[0] = 0xF1;
			   buff[1] = 1; //time sync
			   buff[2] = (uint8_t)(now & 0xFF);
			   buff[3] = (uint8_t)(now >> 8);
			   buff[4] = (uint8_t)(now >> 16);
			   buff[5] = (uint8_t)(now >> 24);
			   Serial.write(buff, 6);
			   break;
		   case 2:
			   //immediately return the data for digital inputs
               /*
			   temp8 = getDigital(0) + (getDigital(1) << 1) + (getDigital(2) << 2) + (getDigital(3) << 3);
			   buff[0] = 0xF1;
			   buff[1] = 2; //digital inputs
			   buff[2] = temp8;
			   temp8 = checksumCalc(buff, 2);
			   buff[3] = temp8;
			   Serial.write(buff, 4);
			   */
			   state = IDLE;
			   break;
		   case 3:
			   //immediately return data on analog inputs
               /*
			   temp16 = getAnalog(0);
			   buff[0] = 0xF1;
			   buff[1] = 3;
			   buff[2] = temp16 & 0xFF;
			   buff[3] = uint8_t(temp16 >> 8);
			   temp16 = getAnalog(1);
			   buff[4] = temp16 & 0xFF;
			   buff[5] = uint8_t(temp16 >> 8);
			   temp16 = getAnalog(2);
			   buff[6] = temp16 & 0xFF;
			   buff[7] = uint8_t(temp16 >> 8);
			   temp16 = getAnalog(3);
			   buff[8] = temp16 & 0xFF;
			   buff[9] = uint8_t(temp16 >> 8);
			   temp8 = checksumCalc(buff, 9);
			   buff[10] = temp8;
			   Serial.write(buff, 11);
			   */
			   state = IDLE;
			   break;
		   case 4:
			   state = SET_DIG_OUTPUTS;
			   buff[0] = 0xF1;
			   break;
		   case 5:
			   state = SETUP_CANBUS;
			   step = 0;
			   buff[0] = 0xF1;
			   break;
		   case 6:
			   //immediately return data on canbus params
			   buff[0] = 0xF1;
			   buff[1] = 6;
			   buff[2] = settings.CAN0_Enabled + ((unsigned char)settings.CAN0ListenOnly << 4);
			   buff[3] = settings.CAN0Speed;
			   buff[4] = settings.CAN0Speed >> 8;
			   buff[5] = settings.CAN0Speed >> 16;
			   buff[6] = settings.CAN0Speed >> 24;
			   buff[7] = settings.CAN1_Enabled + ((unsigned char)settings.CAN1ListenOnly << 4);
			   buff[8] = settings.CAN1Speed;
			   buff[9] = settings.CAN1Speed >> 8;
			   buff[10] = settings.CAN1Speed >> 16;
			   buff[11] = settings.CAN1Speed >> 24;
			   Serial.write(buff, 12);
			   state = IDLE;
			   break;
		   case 7:
			   //immediately return device information
			   buff[0] = 0xF1;
			   buff[1] = 7;
			   buff[2] = CFG_BUILD_NUM & 0xFF;
			   buff[3] = (CFG_BUILD_NUM >> 8);
			   buff[4] = EEPROM_VER;
			   buff[5] = (unsigned char)settings.fileOutputType;
			   buff[6] = (unsigned char)settings.autoStartLogging;
			   buff[7] = 0;
			   Serial.write(buff, 8);
			   state = IDLE;
			   break; 
		   case 8:
			   buff[0] = 0xF1;
			   state = IDLE;//SET_SINGLEWIRE_MODE;
			   step = 0;
			   break;
		   case 9:
			   buff[0] = 0xF1;
			   buff[1] = 0x09;
			   buff[2] = 0xDE;
			   buff[3] = 0xAD;
			   Serial.write(buff, 4);
			   state = IDLE;
			   break;
		   case 10:
			   buff[0] = 0xF1;
			   state = SET_SYSTYPE;
			   step = 0;
			   break;
		   case 11:
			   state = ECHO_CAN_FRAME;
			   buff[0] = 0xF1;
			   step = 0;
			   break;
		   }
		   break;
	   case BUILD_CAN_FRAME:
		   buff[1 + step] = in_byte;
		   switch (step) {
		   case 0:
			   build_out_frame.id = in_byte;
			   break;
		   case 1:
			   build_out_frame.id |= in_byte << 8;
			   break;
		   case 2:
			   build_out_frame.id |= in_byte << 16;
			   break;
		   case 3:
			   build_out_frame.id |= in_byte << 24;
			   if (build_out_frame.id & 1 << 31) 
			   {
				   build_out_frame.id &= 0x7FFFFFFF;
				   build_out_frame.flags.extended = 1;
			   }
			   else build_out_frame.flags.extended = 0;
			   break;
		   case 4:
		       out_bus = in_byte & 1;
		       break;
		   case 5:
			   build_out_frame.len = in_byte & 0xF;
			   if (build_out_frame.len > 8) build_out_frame.len = 8;
			   break;
		   default:
			   if (step < build_out_frame.len + 6)
			   {
			      build_out_frame.buf[step - 6] = in_byte;
			   }
			   else 
			   {
				   state = IDLE;
				   //this would be the checksum byte. Compute and compare.
				   temp8 = checksumCalc(buff, step);
				   //if (temp8 == in_byte) 
				   //{
				   if (out_bus == 0) Can0.write(build_out_frame);
				   if (out_bus == 1) Can1.write(build_out_frame);
				   //}
			   }
			   break;
		   }
		   step++;
		   break;
	   case TIME_SYNC:
		   state = IDLE;
		   break;
	   case SET_DIG_OUTPUTS: //todo: validate the XOR byte
		   buff[1] = in_byte;
		   //temp8 = checksumCalc(buff, 2);
           /*
		   for (int c = 0; c < 8; c++) 
		   {
			   if (in_byte & (1 << c)) setOutput(c, true);
			   else setOutput(c, false);
		   }
		   */
		   state = IDLE;
		   break;
	   case SETUP_CANBUS: //todo: validate checksum
		   switch (step)
		   {
		   case 0:
			   build_int = in_byte;
			   break;
		   case 1:
			   build_int |= in_byte << 8;
			   break;
		   case 2:
			   build_int |= in_byte << 16;
			   break;
		   case 3:
			   build_int |= in_byte << 24;
			   if (build_int > 0) 
			   {
				   if (build_int & 0x80000000) //signals that enabled and listen only status are also being passed
				   {
					   if (build_int & 0x40000000)
					   {
						   settings.CAN0_Enabled = true;
					   }
					   else
					   {
						   settings.CAN0_Enabled = false;
					   }
					   if (build_int & 0x20000000)
					   {
						   settings.CAN0ListenOnly = true;
						//    Can0.setListenOnly(true);
					   }
					   else
					   {
						   settings.CAN0ListenOnly = false;
						//    Can0.setListenOnly(false);
					   }
				   }
				   else
				   {
					   settings.CAN0_Enabled = true;
				   }
				   build_int = build_int & 0xFFFFF;
				   if (build_int > 1000000) build_int = 1000000;				   
				   Can0.begin();
                   if (SysSettings.CAN0EnablePin < 255 && settings.CAN0_Enabled)
                   {
                       pinMode(SysSettings.CAN0EnablePin, OUTPUT);
                       digitalWrite(SysSettings.CAN0EnablePin, HIGH);
                   }
                   else digitalWrite(SysSettings.CAN0EnablePin, LOW);
				   //Can0.set_baudrate(build_int);
				   settings.CAN0Speed = build_int;				   
			   }
			   else //disable first canbus
			   {
				//    Can0.end();
                   digitalWrite(SysSettings.CAN0EnablePin, LOW);
				   settings.CAN0_Enabled = false;
			   }
			   break;
		   case 4:
			   build_int = in_byte;
			   break;
		   case 5:
			   build_int |= in_byte << 8;
			   break;
		   case 6:
			   build_int |= in_byte << 16;
			   break;
		   case 7:
			   build_int |= in_byte << 24;
			   if (build_int > 0) 
			   {
				   if (build_int & 0x80000000) //signals that enabled and listen only status are also being passed
				   {
					   if (build_int & 0x40000000)
					   {
						   settings.CAN1_Enabled = true;
					   }
					   else
					   {
						   settings.CAN1_Enabled = false;
					   }
					   if (build_int & 0x20000000)
					   {
						   settings.CAN1ListenOnly = true;
						//    Can1.setListenOnly(true);
					   }
					   else
					   {
						   settings.CAN1ListenOnly = false;
						//    Can1.setListenOnly(false);
					   }
				   }
				   else
				   {
					   settings.CAN1_Enabled = true;
				   }
				   build_int = build_int & 0xFFFFF;
				   if (build_int > 1000000) build_int = 1000000;
				   Can1.begin();
                   if (SysSettings.CAN1EnablePin < 255 && settings.CAN1_Enabled)
                   {
                       pinMode(SysSettings.CAN1EnablePin, OUTPUT);
                       digitalWrite(SysSettings.CAN1EnablePin, HIGH);
                   }
                   else digitalWrite(SysSettings.CAN1EnablePin, LOW);
				   //Can1.set_baudrate(build_int);

				   settings.CAN1Speed = build_int;				   
			   }
			   else //disable second canbus
			   {
				//    Can1.end();
                   digitalWrite(SysSettings.CAN1EnablePin, LOW);
				   settings.CAN1_Enabled = false;
			   }
			   state = IDLE;
			    //now, write out the new canbus settings to EEPROM
				EEPROM.put(EEPROM_ADDRESS, settings);
				setPromiscuousMode();
			   break;
		   }
		   step++;
		   break;
	   case SET_SINGLEWIRE_MODE:
		   state = IDLE;
		   break;
	   case SET_SYSTYPE:
		   settings.sysType = in_byte;		   
		   EEPROM.put(EEPROM_ADDRESS, settings);
		   loadSettings();
		   state = IDLE;
		   break;
	   case ECHO_CAN_FRAME:
		   buff[1 + step] = in_byte;
		   switch (step) {
		   case 0:
			   build_out_frame.id = in_byte;
			   break;
		   case 1:
			   build_out_frame.id |= in_byte << 8;
			   break;
		   case 2:
			   build_out_frame.id |= in_byte << 16;
			   break;
		   case 3:
			   build_out_frame.id |= in_byte << 24;
			   if (build_out_frame.id & 1 << 31) 
			   {
				   build_out_frame.id &= 0x7FFFFFFF;
				   build_out_frame.flags.extended = 1;
			   }
			   else build_out_frame.flags.extended = 0;
			   break;
		   case 4:
		       out_bus = in_byte & 1;
		       break;
		   case 5:
			   build_out_frame.len = in_byte & 0xF;
			   if (build_out_frame.len > 8) build_out_frame.len = 8;
			   break;
		   default:
			   if (step < build_out_frame.len + 6)
			   {
			      build_out_frame.buf[step - 6] = in_byte;
			   }
			   else 
			   {
				   state = IDLE;
				   //this would be the checksum byte. Compute and compare.
				   temp8 = checksumCalc(buff, step);
				   //if (temp8 == in_byte) 
				   //{
				   toggleRXLED();
				   if (isConnected) sendFrameToUSB(build_out_frame, 0);
				   //}
			   }
			   break;
		   }
		   step++;
		   break;
	   }
  }
	Logger::loop();
}

SerialConsole::SerialConsole() {
	init();
}

void SerialConsole::init() {
	//State variables for serial console
	ptrBuffer = 0;
	state = STATE_ROOT_MENU;      
}

void SerialConsole::printMenu() {
	char buff[80];
	//Show build # here as well in case people are using the native port and don't get to see the start up messages
	Serial.print("Build number: ");
	Serial.println(CFG_BUILD_NUM);
	Serial.println("System Menu:");
	Serial.println();
	Serial.println("Enable line endings of some sort (LF, CR, CRLF)");
	Serial.println();
	Serial.println("Short Commands:");
	Serial.println("h = help (displays this message)");
	Serial.println("K = set all outputs high");
	Serial.println("J = set all outputs low");
	Serial.println("R = reset to factory defaults");
	Serial.println("s = Start logging to file");
	Serial.println("S = Stop logging to file");
	Serial.println();
	Serial.println("Config Commands (enter command=newvalue). Current values shown in parenthesis:");
    Serial.println();

    Logger::console("LOGLEVEL=%i - set log level (0=debug, 1=info, 2=warn, 3=error, 4=off)", settings.logLevel);
	Logger::console("SYSTYPE=%i - set board type (0 = Teensy 3.1/3.2/3.5, 1 = Teensy 3.6)", settings.sysType);
	Serial.println();

	Logger::console("CAN0EN=%i - Enable/Disable CAN0 (0 = Disable, 1 = Enable)", settings.CAN0_Enabled);
	Logger::console("CAN0SPEED=%i - Set speed of CAN0 in baud (125000, 250000, etc)", settings.CAN0Speed);
	Logger::console("CAN0LISTENONLY=%i - Enable/Disable Listen Only Mode (0 = Dis, 1 = En)", settings.CAN0ListenOnly);
	for (int i = 0; i < 8; i++) {
		sprintf(buff, "CAN0FILTER%i=0x%%x,0x%%x,%%i,%%i (ID, Mask, Extended, Enabled)", i);
		Logger::console(buff, settings.CAN0Filters[i].id, settings.CAN0Filters[i].mask,
			settings.CAN0Filters[i].extended, settings.CAN0Filters[i].enabled);
	}
	Serial.println();

	Logger::console("CAN1EN=%i - Enable/Disable CAN1 (0 = Disable, 1 = Enable)", settings.CAN1_Enabled);
	Logger::console("CAN1SPEED=%i - Set speed of CAN1 in baud (125000, 250000, etc)", settings.CAN1Speed);
	Logger::console("CAN1LISTENONLY=%i - Enable/Disable Listen Only Mode (0 = Dis, 1 = En)", settings.CAN1ListenOnly);
	for (int i = 0; i < 8; i++) {
		sprintf(buff, "CAN1FILTER%i=0x%%x,0x%%x,%%i,%%i (ID, Mask, Extended, Enabled)", i);
		Logger::console(buff, settings.CAN1Filters[i].id, settings.CAN1Filters[i].mask,
			settings.CAN1Filters[i].extended, settings.CAN1Filters[i].enabled);
	}
	Logger::console("CAN0SEND=ID,LEN,<BYTES SEPARATED BY COMMAS> - Ex: C0SEND=0x200,4,1,2,3,4");
	Logger::console("CAN1SEND=ID,LEN,<BYTES SEPARATED BY COMMAS> - Ex: C1SEND=0x200,8,00,00,00,10,0xAA,0xBB,0xA0,00");
	Logger::console("MARK=<Description of what you are doing> - Set a mark in the log file about what you are about to do.");
	Serial.println();

	Logger::console("BINSERIAL=%i - Enable/Disable Binary Sending of CANBus Frames to Serial (0=Dis, 1=En)", settings.useBinarySerialComm);
	Logger::console("FILETYPE=%i - Set type of file output (0=None, 1 = Binary, 2 = GVRET, 3 = CRTD)", settings.fileOutputType);
	Serial.println();

	Logger::console("FILEBASE=%s - Set filename base for saving", (char *)settings.fileNameBase);
	Logger::console("FILEEXT=%s - Set filename ext for saving", (char *)settings.fileNameExt);
	Logger::console("FILENUM=%i - Set incrementing number for filename", settings.fileNum);
	Logger::console("FILEAPPEND=%i - Append to file (no numbers) or use incrementing numbers after basename (0=Incrementing Numbers, 1=Append)", settings.appendFile);
	Logger::console("FILEAUTO=%i - Automatically start logging at startup (0=No, 1 = Yes)", settings.autoStartLogging);
    Serial.println();
    
    Logger::console("DIGTOGEN=%i - Enable digital toggling system (0 = Dis, 1 = En)", digToggleSettings.enabled);
    Logger::console("DIGTOGMODE=%i - Set digital toggle mode (0 = Read pin, send CAN, 1 = Receive CAN, set pin)", digToggleSettings.mode & 1);
    Logger::console("DIGTOGLEVEL=%i - Set default level of digital pin (0 = LOW, 1 = HIGH)", digToggleSettings.mode >> 7);
    Logger::console("DIGTOGPIN=%i - Pin to use for digital toggling system (Use Arduino Digital Pin Number)", digToggleSettings.pin);
    Logger::console("DIGTOGID=%X - CAN ID to use for Rx or Tx", digToggleSettings.rxTxID);
    Logger::console("DIGTOGCAN0=%i - Use CAN0 with Digital Toggling System? (0 = No, 1 = Yes)", (digToggleSettings.mode >> 1) & 1);
    Logger::console("DIGTOGCAN1=%i - Use CAN1 with Digital Toggling System? (0 = No, 1 = Yes)", (digToggleSettings.mode >> 2) & 1);
    Logger::console("DIGTOGLEN=%i - Length of frame to send (Tx) or validate (Rx)", digToggleSettings.length);
    Logger::console("DIGTOGPAYLOAD=%X,%X,%X,%X,%X,%X,%X,%X - Payload to send or validate against (comma separated list)", digToggleSettings.payload[0],
                    digToggleSettings.payload[1], digToggleSettings.payload[2], digToggleSettings.payload[3], digToggleSettings.payload[4],
                    digToggleSettings.payload[5], digToggleSettings.payload[6], digToggleSettings.payload[7]);
}

/*	There is a help menu (press H or h or ?)
 This is no longer going to be a simple single character console.
 Now the system can handle up to 80 input characters. Commands are submitted
 by sending line ending (LF, CR, or both)
 */
void SerialConsole::rcvCharacter(uint8_t chr) {
	if (chr == 10 || chr == 13) { //command done. Parse it.
		handleConsoleCmd();
		ptrBuffer = 0; //reset line counter once the line has been processed
	} else {
		cmdBuffer[ptrBuffer++] = (unsigned char) chr;
		if (ptrBuffer > 79)
			ptrBuffer = 79;
	}
}

void SerialConsole::handleConsoleCmd() {
	if (state == STATE_ROOT_MENU) {
		if (ptrBuffer == 1) 
		{ //command is a single ascii character
			handleShortCmd();
		} 
		else //at least two bytes 
		{
		        boolean equalSign = false;
			for (int i = 0; i < ptrBuffer; i++) if (cmdBuffer[i] == '=') equalSign = true;
			if (equalSign) handleConfigCmd();
			else handleLawicelCmd();
		}
		ptrBuffer = 0; //reset line counter once the line has been processed
	}
}

void SerialConsole::handleLawicelCmd()
{
    cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
    CAN_message_t outFrame;
    char buff[80];
    int val;
    
    switch (cmdBuffer[0])
    {
      case 't': //transmit standard frame	
	outFrame.id = parseHexString(cmdBuffer + 1, 3);
	outFrame.len = cmdBuffer[4] - '0';
	outFrame.flags.extended = 0;
	if (outFrame.len < 0) outFrame.len = 0;
	if (outFrame.len > 8) outFrame.len = 8;
	for (int data = 0; data < outFrame.len; data++)
	{
		 outFrame.buf[data] = parseHexString(cmdBuffer + 5 + (2 * data), 2);
	}
	Can0.write(outFrame);
	if (SysSettings.lawicelAutoPoll) Serial.print("z");
	break;
      case 'T': //transmit extended frame
	outFrame.id = parseHexString(cmdBuffer + 1, 8);
	outFrame.len = cmdBuffer[9] - '0';
	outFrame.flags.extended = 1;
	if (outFrame.len < 0) outFrame.len = 0;
	if (outFrame.len > 8) outFrame.len = 8;
	for (int data = 0; data < outFrame.len; data++)
	{
		 outFrame.buf[data] = parseHexString(cmdBuffer + 10 + (2 * data), 2);
	}
	Can0.write(outFrame);
	if (SysSettings.lawicelAutoPoll) Serial.print("Z");
	break;	
      case 'S': //setup canbus baud via predefined speeds
	val = parseHexCharacter(cmdBuffer[1]);
	switch (val)
	{
	  case 0:
	    settings.CAN0Speed = 10000;
	    break;
	  case 1:
	    settings.CAN0Speed = 20000;
	    break;
	  case 2:
	    settings.CAN0Speed = 50000;
	    break;	    
	  case 3:
	    settings.CAN0Speed = 100000;
	    break;	    
	  case 4:
	    settings.CAN0Speed = 125000;
	    break;	    
	  case 5:
	    settings.CAN0Speed = 250000;
	    break;	    
	  case 6:
	    settings.CAN0Speed = 500000;
	    break;	    
	  case 7:
	    settings.CAN0Speed = 800000;
	    break;	    
	  case 8:
	    settings.CAN0Speed = 1000000;
	    break;	    
	}
      case 's': //setup canbus baud via register writes (we can't really do that...)
	//settings.CAN0Speed = 250000;
	break;
      case 'r': //send a standard RTR frame (don't really... that's so deprecated its not even funny)
	break;
      case 'R': //send extended RTR frame (NO! DON'T DO IT!)
	break;
      case 'X': //Set autopoll off/on
	if (cmdBuffer[1] == '1') SysSettings.lawicelAutoPoll = true;
	else SysSettings.lawicelAutoPoll = false;	
	break;
      case 'W': //Dual or single filter mode
	break; //don't actually support this mode
      case 'm': //set acceptance mask - these things seem to be odd and aren't actually implemented yet
      case 'M': //set acceptance code 
	break; //don't do anything here yet either
      case 'U': //set uart speed. We just ignore this. You can't set a baud rate on a USB CDC port
	break; //also no action here
      case 'Z': //Turn timestamp off/on
	if (cmdBuffer[1] == '1') SysSettings.lawicelTimestamping = true;
	else SysSettings.lawicelTimestamping =  false;
	break;
      case 'Q': //turn auto start up on/off - probably don't need to actually implement this at the moment.	
	break; //no action yet or maybe ever
    }
    Serial.write(13);
}

/*For simplicity the configuration setting code uses four characters for each configuration choice. This makes things easier for
 comparison purposes.
 */
void SerialConsole::handleConfigCmd() {
	int i;
	int newValue;
	char *newString;
	bool writeEEPROM = false;
    bool writeDigEE = false;
    char *dataTok;

	//Logger::debug("Cmd size: %i", ptrBuffer);
	if (ptrBuffer < 6)
		return; //4 digit command, =, value is at least 6 characters
	cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
	String cmdString = String();
	unsigned char whichEntry = '0';
	i = 0;

	while (cmdBuffer[i] != '=' && i < ptrBuffer) {
	 cmdString.concat(String(cmdBuffer[i++]));
	}
	i++; //skip the =
	if (i >= ptrBuffer)
	{
		Logger::console("Command needs a value..ie TORQ=3000");
		Logger::console("");
		return; //or, we could use this to display the parameter instead of setting
	}

	// strtol() is able to parse also hex values (e.g. a string "0xCAFE"), useful for enable/disable by device id
	newValue = strtol((char *) (cmdBuffer + i), NULL, 0); //try to turn the string into a number
	newString = (char *)(cmdBuffer + i); //leave it as a string

	cmdString.toUpperCase();

	if (cmdString == String("CAN0EN")) {
		if (newValue < 0) newValue = 0;
		if (newValue > 1) newValue = 1;
		Logger::console("Setting CAN0 Enabled to %i", newValue);
		settings.CAN0_Enabled = newValue;
		if (newValue == 1) {
            Can0.begin();
			// settings.CAN0Speed);
            if (SysSettings.CAN0EnablePin < 255) {
                pinMode(SysSettings.CAN0EnablePin, OUTPUT);
                digitalWrite(SysSettings.CAN0EnablePin, HIGH);
            }
        }
		else {
            // Can0.end();
            digitalWrite(SysSettings.CAN0EnablePin, LOW);
        }
		writeEEPROM = true;
	} else if (cmdString == String("CAN1EN")) {
		if (newValue < 0) newValue = 0;
		if (newValue > 1) newValue = 1;
		Logger::console("Setting CAN1 Enabled to %i", newValue);
		if (newValue == 1) {
            Can1.begin();
			// settings.CAN1Speed);
            if (SysSettings.CAN1EnablePin < 255)
            {
                pinMode(SysSettings.CAN1EnablePin, OUTPUT);
                digitalWrite(SysSettings.CAN1EnablePin, HIGH);
            }
        }
		else {
            // Can1.end();
            digitalWrite(SysSettings.CAN1EnablePin, LOW);
        }
		settings.CAN1_Enabled = newValue;
		writeEEPROM = true;
	} else if (cmdString == String("CAN0SPEED")) {
		if (newValue > 0 && newValue <= 1000000) 
		{
			Logger::console("Setting CAN0 Baud Rate to %i", newValue);
			settings.CAN0Speed = newValue;
			Can0.begin();
			// settings.CAN0Speed);
			writeEEPROM = true;
		}
		else Logger::console("Invalid baud rate! Enter a value 1 - 1000000");
	} else if (cmdString == String("CAN1SPEED")) {
		if (newValue > 0 && newValue <= 1000000)
		{
			Logger::console("Setting CAN1 Baud Rate to %i", newValue);
			settings.CAN1Speed = newValue;
			Can1.begin();
			// settings.CAN1Speed);
			writeEEPROM = true;
		}
		else Logger::console("Invalid baud rate! Enter a value 1 - 1000000");

	} else if (cmdString == String("CAN0LISTENONLY")) {
		if (newValue >= 0 && newValue <= 1)
		{
			Logger::console("Setting CAN0 Listen Only to %i", newValue);
			settings.CAN0ListenOnly = newValue;
			if (settings.CAN0ListenOnly)
			{
				// Can0.setListenOnly(true);
			}
			else
			{
				// Can0.setListenOnly(false);
			}
			writeEEPROM = true;
		}
		else Logger::console("Invalid setting! Enter a value 0 - 1");
	} else if (cmdString == String("CAN1LISTENONLY")) {
		if (newValue >= 0 && newValue <= 1)
		{
			Logger::console("Setting CAN1 Listen Only to %i", newValue);
			settings.CAN1ListenOnly = newValue;
			if (settings.CAN1ListenOnly)
			{
				// Can1.setListenOnly(true);
			}
			else
			{
				// Can1.setListenOnly(false);
			}
			writeEEPROM = true;
		}
		else Logger::console("Invalid setting! Enter a value 0 - 1");
	} else if (cmdString == String("CAN0FILTER0")) { //someone should kick me in the face for this laziness... FIX THIS!
		handleFilterSet(0, 0, newString);
	}
	else if (cmdString == String("CAN0FILTER1")) {
		if (handleFilterSet(0, 1, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN0FILTER2")) {
		if (handleFilterSet(0, 2, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN0FILTER3")) {
		if (handleFilterSet(0, 3, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN0FILTER4")) {
		if (handleFilterSet(0, 4, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN0FILTER5")) {
		if (handleFilterSet(0, 5, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN0FILTER6")) {
		if (handleFilterSet(0, 6, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN0FILTER7")) {
		if (handleFilterSet(0, 7, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN1FILTER0")) {
		if (handleFilterSet(1, 0, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN1FILTER1")) {
		if (handleFilterSet(1, 1, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN1FILTER2")) {
		if (handleFilterSet(1, 2, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN1FILTER3")) {
		if (handleFilterSet(1, 3, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN1FILTER4")) {
		if (handleFilterSet(1, 4, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN1FILTER5")) {
		if (handleFilterSet(1, 5, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN1FILTER6")) {
		if (handleFilterSet(1, 6, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN1FILTER7")) {
		if (handleFilterSet(1, 7, newString)) writeEEPROM = true;
	}
	else if (cmdString == String("CAN0SEND")) {
		handleCAN0Send(Can0, newString);
	}
	else if (cmdString == String("CAN1SEND")) {
		handleCAN1Send(Can1, newString);
	}
	else if (cmdString == String("MARK")) { //just ascii based for now
		if (settings.fileOutputType == GVRET) Logger::file("Mark: %s", newString);
		if (settings.fileOutputType == CRTD) 
		{
			uint8_t buff[40];
			sprintf((char *)buff, "%f CEV ", millis() / 1000.0f);
			Logger::fileRaw(buff, strlen((char *)buff));
			Logger::fileRaw((uint8_t *)newString, strlen(newString));
			buff[0] = '\r';
			buff[1] = '\n';
			Logger::fileRaw(buff, 2);			
		}
		if (!settings.useBinarySerialComm) Logger::console("Mark: %s", newString);
	} else if (cmdString == String("BINSERIAL")) {
		if (newValue < 0) newValue = 0;
		if (newValue > 1) newValue = 1;
		Logger::console("Setting Serial Binary Comm to %i", newValue);
		settings.useBinarySerialComm = newValue;
		writeEEPROM = true;
	} else if (cmdString == String("FILETYPE")) {
		if (newValue < 0) newValue = 0;
		if (newValue > 3) newValue = 3;
		Logger::console("Setting File Output Type to %i", newValue);
		settings.fileOutputType = (FILEOUTPUTTYPE)newValue; //the numbers all intentionally match up so this works
		writeEEPROM = true;
	} else if (cmdString == String("FILEBASE")) {
		Logger::console("Setting File Base Name to %s", newString);
		strcpy((char *)settings.fileNameBase, newString);
		writeEEPROM = true;
	} else if (cmdString == String("FILEEXT")) {
		Logger::console("Setting File Extension to %s", newString);
		strcpy((char *)settings.fileNameExt, newString);
		writeEEPROM = true;
	} else if (cmdString == String("FILENUM")) {
		Logger::console("Setting File Incrementing Number Base to %i", newValue);
		settings.fileNum = newValue;
		writeEEPROM = true;
	} else if (cmdString == String("FILEAPPEND")) {
		if (newValue < 0) newValue = 0;
		if (newValue > 1) newValue = 1;
		Logger::console("Setting File Append Mode to %i", newValue);
		settings.appendFile = newValue;
		writeEEPROM = true;
	}
	else if (cmdString == String("FILEAUTO")) {
		if (newValue < 0) newValue = 0;
		if (newValue > 1) newValue = 1;
		Logger::console("Setting Auto File Logging Mode to %i", newValue);
		settings.autoStartLogging = newValue;
		writeEEPROM = true;
	}
	else if (cmdString == String("SYSTYPE")) {
		if (newValue < 2 && newValue >= 0) {
			settings.sysType = newValue;			
			writeEEPROM = true;
			Logger::console("System type updated. Power cycle to apply.");
		}
		else Logger::console("Invalid system type. Please enter a value of 0 for CanDue or 1 for GEVCU");
    } else if (cmdString == String("DIGTOGEN")) {
        if (newValue >= 0 && newValue <= 1)
        {
            Logger::console("Setting Digital Toggle System Enable to %i", newValue);
            digToggleSettings.enabled = newValue;
            writeDigEE = true;
        }
        else Logger::console("Invalid enable value. Must be either 0 or 1");
    } else if (cmdString == String("DIGTOGMODE")) {
        if (newValue >= 0 && newValue <= 1)
        {
            Logger::console("Setting Digital Toggle Mode to %i", newValue);
            if (newValue == 0) digToggleSettings.mode &= ~1;
            if (newValue == 1) digToggleSettings.mode |= 1;
            writeDigEE = true;
        }
        else Logger::console("Invalid mode. Must be either 0 or 1");
    } else if (cmdString == String("DIGTOGLEVEL")) {
        if (newValue >= 0 && newValue <= 1)
        {
            Logger::console("Setting Digital Toggle Starting Level to %i", newValue);
            if (newValue == 0) digToggleSettings.mode &= ~0x80;
            if (newValue == 1) digToggleSettings.mode |= 0x80;
            writeDigEE = true;
        }
        else Logger::console("Invalid default level. Must be either 0 or 1");
    } else if (cmdString == String("DIGTOGPIN")) {
        if (newValue >= 0 && newValue <= 77)
        {
            Logger::console("Setting Digital Toggle Pin to %i", newValue);
            digToggleSettings.pin = newValue;
            writeDigEE = true;
        }
        else Logger::console("Invalid pin. Must be between 0 and 77");
    } else if (cmdString == String("DIGTOGID")) {
        if (newValue >= 0 && newValue < (1 << 30))
        {
            Logger::console("Setting Digital Toggle CAN ID to %X", newValue);
            digToggleSettings.rxTxID = newValue;
            writeDigEE = true;
        }
        else Logger::console("Invalid CAN ID. Must be either an 11 or 29 bit ID");
     } else if (cmdString == String("DIGTOGCAN0")) {
        if (newValue >= 0 && newValue <= 1)
        {
            Logger::console("Setting Digital Toggle CAN0 Usage to %i", newValue);
            if (newValue == 0) digToggleSettings.mode &= ~2;
            if (newValue == 1) digToggleSettings.mode |= 2;
            writeDigEE = true;
        }
        else Logger::console("Invalid value. Must be either 0 or 1");       
     } else if (cmdString == String("DIGTOGCAN1")) {
        if (newValue >= 0 && newValue <= 1)
        {
            Logger::console("Setting Digital Toggle CAN1 Usage to %i", newValue);
            if (newValue == 0) digToggleSettings.mode &= ~4;
            if (newValue == 1) digToggleSettings.mode |= 4;
            writeDigEE = true;
        }
        else Logger::console("Invalid value. Must be either 0 or 1");                       
    } else if (cmdString == String("DIGTOGLEN")) {
        if (newValue >= 0 && newValue <= 8)
        {
            Logger::console("Setting Digital Toggle Frame Length to %i", newValue);
            digToggleSettings.length = newValue;
            writeDigEE = true;
        }
        else Logger::console("Invalid length. Must be between 0 and 8");
    } else if (cmdString == String("DIGTOGPAYLOAD")) {
        dataTok = strtok(newString, ",");
        if (dataTok) 
        {
            digToggleSettings.payload[0] = strtol(dataTok, NULL, 0);
            i = 1;
            while (i < 8 && dataTok)
            {
                dataTok = strtok(NULL, ",");
                if (dataTok) {
                    digToggleSettings.payload[i] = strtol(dataTok, NULL, 0);
                    i += 1;
                }                    
            }            
            writeDigEE = true;
            Logger::console("Set new payload bytes");
        }
        else Logger::console("Error processing payload");
	} else if (cmdString == String("LOGLEVEL")) {
		switch (newValue) {
		case 0:
			Logger::setLoglevel(Logger::Debug);
			Logger::console("setting loglevel to 'debug'");
			writeEEPROM = true;
			break;
		case 1:
			Logger::setLoglevel(Logger::Info);
			Logger::console("setting loglevel to 'info'");
			writeEEPROM = true;
			break;
		case 2:
			Logger::console("setting loglevel to 'warning'");
			Logger::setLoglevel(Logger::Warn);
			writeEEPROM = true;
			break;
		case 3:
			Logger::console("setting loglevel to 'error'");
			Logger::setLoglevel(Logger::Error);
			writeEEPROM = true;
			break;
		case 4:
			Logger::console("setting loglevel to 'off'");
			Logger::setLoglevel(Logger::Off);
			writeEEPROM = true;
			break;
		}

	} 
	else {
		Logger::console("Unknown command");
	}
	if (writeEEPROM) 
	{
		EEPROM.put(EEPROM_ADDRESS, settings);
	}
	if (writeDigEE)
    {
        EEPROM.put(EEPROM_ADDRESS + 500, digToggleSettings);
    }
}

/*
LAWICEL single letter commands are now mixed in with the other commands here.
*/
void SerialConsole::handleShortCmd() {
	uint8_t val;

	switch (cmdBuffer[0]) {
	case 'h':
	case '?':
	case 'H':
		printMenu();
		break;
	case 'R': //reset to factory defaults.
		settings.version = 0xFF;
		EEPROM.put(EEPROM_ADDRESS, settings);
		Logger::console("Power cycle to reset to factory defaults");
		break;
	case 's': //start logging canbus to file
		SysSettings.logToFile = true;
		break;
	case 'S': //stop logging canbus to file
		SysSettings.logToFile = false;
		break;
	case 'O': //LAWICEL open canbus port (first one only because LAWICEL has no concept of dual canbus
		//Can0.begin(settings.CAN0Speed, SysSettings.CAN1EnablePin);
		//Can0.enable();
		Serial.write(13); //send CR to mean "ok"
		SysSettings.lawicelMode = true;
		break;
	case 'C': //LAWICEL close canbus port (First one)
		// Can0.end();
        digitalWrite(SysSettings.CAN0EnablePin, LOW);
		Serial.write(13); //send CR to mean "ok"
		break;
	case 'L': //LAWICEL open canbus port in listen only mode
		Can0.begin();
		// settings.CAN0Speed); //this is NOT really listen only mode but it isn't supported yet so for now...
        if (SysSettings.CAN0EnablePin < 255)
        {
            pinMode(SysSettings.CAN0EnablePin, OUTPUT);
            digitalWrite(SysSettings.CAN0EnablePin, HIGH);
        }
		// Can0.setListenOnly(true);
		Serial.write(13); //send CR to mean "ok"
		SysSettings.lawicelMode = true;
		break;
	case 'P': //LAWICEL - poll for one waiting frame. Or, just CR if no frames
		// if (Can0.available()) SysSettings.lawicelPollCounter = 1;
		// else Serial.write(13); //no waiting frames
		break;
	case 'A': //LAWICEL - poll for all waiting frames - CR if no frames
		// SysSettings.lawicelPollCounter = Can0.available();
		if (SysSettings.lawicelPollCounter == 0) Serial.write(13);
		break;
	case 'F': //LAWICEL - read status bits 
		Serial.print("F00"); //bit 0 = RX Fifo Full, 1 = TX Fifo Full, 2 = Error warning, 3 = Data overrun, 5= Error passive, 6 = Arb. Lost, 7 = Bus Error
		Serial.write(13);
		break;
	case 'V': //LAWICEL - get version number
		Serial.print("V1013\n");
		SysSettings.lawicelMode = true;
		break;
	case 'N': //LAWICEL - get serial number
		Serial.print("ND00D\n");
		SysSettings.lawicelMode = true;
		break;
	}
}

//CAN0FILTER%i=%%i,%%i,%%i,%%i (ID, Mask, Extended, Enabled)", i);
bool SerialConsole::handleFilterSet(uint8_t bus, uint8_t filter, char *values) 
{
	if (filter < 0 || filter > 7) return false;
	if (bus < 0 || bus > 1) return false;

	//there should be four tokens
	char *idTok = strtok(values, ",");
	char *maskTok = strtok(NULL, ",");
	char *extTok = strtok(NULL, ",");
	char *enTok = strtok(NULL, ",");

	if (!idTok) return false; //if any of them were null then something was wrong. Abort.
	if (!maskTok) return false;
	if (!extTok) return false;
	if (!enTok) return false;

	int idVal = strtol(idTok, NULL, 0);
	int maskVal = strtol(maskTok, NULL, 0);
	int extVal = strtol(extTok, NULL, 0);
	int enVal = strtol(enTok, NULL, 0);

	Logger::console("Setting CAN%iFILTER%i to ID 0x%x Mask 0x%x Extended %i Enabled %i", bus, filter, idVal, maskVal, extVal, enVal);

	if (bus == 0)
	{
		settings.CAN0Filters[filter].id = idVal;
		settings.CAN0Filters[filter].mask = maskVal;
		settings.CAN0Filters[filter].extended = extVal;
		settings.CAN0Filters[filter].enabled = enVal;
		//Can0.setRXFilter(filter, idVal, maskVal, extVal);
	}
	else if (bus == 1) 
	{
		settings.CAN1Filters[filter].id = idVal;
		settings.CAN1Filters[filter].mask = maskVal;
		settings.CAN1Filters[filter].extended = extVal;
		settings.CAN1Filters[filter].enabled = enVal;
		//Can1.setRXFilter(filter, idVal, maskVal, extVal);
	}

	return true;
}

bool SerialConsole::handleCAN0Send(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &port, char *inputString)
{
	char *idTok = strtok(inputString, ",");
	char *lenTok = strtok(NULL, ",");
	char *dataTok;
	CAN_message_t frame;

	if (!idTok) return false;
	if (!lenTok) return false;

	int idVal = strtol(idTok, NULL, 0);
	int lenVal = strtol(lenTok, NULL, 0);

	for (int i = 0; i < lenVal; i++)
	{
		dataTok = strtok(NULL, ",");
		if (!dataTok) return false;
		frame.buf[i] = strtol(dataTok, NULL, 0);
	}

	//things seem good so try to send the frame.
	frame.id = idVal;
	if (idVal >= 0x7FF) frame.flags.extended = 1;
	else frame.flags.extended = 0;
	frame.len = lenVal;
	port.write(frame);
	Logger::console("Sending frame with id: 0x%x len: %i", frame.id, frame.len);
	SysSettings.txToggle = !SysSettings.txToggle;
	//setLED(SysSettings.LED_CANTX, SysSettings.txToggle);
	return true;
}
bool SerialConsole::handleCAN1Send(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &port, char *inputString)
{
	char *idTok = strtok(inputString, ",");
	char *lenTok = strtok(NULL, ",");
	char *dataTok;
	CAN_message_t frame;

	if (!idTok) return false;
	if (!lenTok) return false;

	int idVal = strtol(idTok, NULL, 0);
	int lenVal = strtol(lenTok, NULL, 0);

	for (int i = 0; i < lenVal; i++)
	{
		dataTok = strtok(NULL, ",");
		if (!dataTok) return false;
		frame.buf[i] = strtol(dataTok, NULL, 0);
	}

	//things seem good so try to send the frame.
	frame.id = idVal;
	if (idVal >= 0x7FF) frame.flags.extended = 1;
	else frame.flags.extended = 0;
	frame.len = lenVal;
	port.write(frame);
	Logger::console("Sending frame with id: 0x%x len: %i", frame.id, frame.len);
	SysSettings.txToggle = !SysSettings.txToggle;
	//setLED(SysSettings.LED_CANTX, SysSettings.txToggle);
	return true;
}

unsigned int SerialConsole::parseHexCharacter(char chr)
{
	unsigned int result = 0;
	if (chr >= '0' && chr <= '9') result = chr - '0';
	else if (chr >= 'A' && chr <= 'F') result = 10 + chr - 'A';
	else if (chr >= 'a' && chr <= 'f') result = 10 + chr - 'a';
	
	return result;
}

unsigned int SerialConsole::parseHexString(char *str, int length)
{
    unsigned int result = 0;
    for (int i = 0; i < length; i++) result += parseHexCharacter(str[i]) << (4 * (length - i - 1));
    return result;
}

Logger::LogLevel Logger::logLevel = Logger::Info;
uint32_t Logger::lastLogTime = 0;
uint16_t Logger::fileBuffWritePtr = 0;
SdFile Logger::fileRef; //file we're logging to
uint8_t Logger::filebuffer[SD_BUFF_SIZE]; //size of buffer for file output
uint32_t Logger::lastWriteTime = 0;

/*
 * Output a debug message with a variable amount of parameters.
 * printf() style, see Logger::log()
 *
 */
void Logger::debug(const char *message, ...)
{
    if (logLevel > Debug) {
        return;
    }

    va_list args;
    va_start(args, message);
    Logger::log(Debug, message, args);
    va_end(args);
}

/*
 * Output a info message with a variable amount of parameters
 * printf() style, see Logger::log()
 */
void Logger::info(const char *message, ...)
{
    if (logLevel > Info) {
        return;
    }

    va_list args;
    va_start(args, message);
    Logger::log(Info, message, args);
    va_end(args);
}

/*
 * Output a warning message with a variable amount of parameters
 * printf() style, see Logger::log()
 */
void Logger::warn(const char *message, ...)
{
    if (logLevel > Warn) {
        return;
    }

    va_list args;
    va_start(args, message);
    Logger::log(Warn, message, args);
    va_end(args);
}

/*
 * Output a error message with a variable amount of parameters
 * printf() style, see Logger::log()
 */
void Logger::error(const char *message, ...)
{
    if (logLevel > Error) {
        return;
    }

    va_list args;
    va_start(args, message);
    Logger::log(Error, message, args);
    va_end(args);
}

/*
 * Output a comnsole message with a variable amount of parameters
 * printf() style, see Logger::logMessage()
 */
void Logger::console(const char *message, ...)
{
    va_list args;
    va_start(args, message);
    Logger::logMessage(message, args);
    va_end(args);
}

void Logger::buffPutChar(char c)
{
	*(filebuffer + fileBuffWritePtr++) = c;
}

void Logger::buffPutString(const char *c)
{
	while (*c) *(filebuffer + fileBuffWritePtr++) = *c++;
}

void Logger::flushFileBuff()
{
	Logger::debug("Write to SD Card %i bytes", fileBuffWritePtr);
	lastWriteTime = millis();
	if (fileRef.write(filebuffer, fileBuffWritePtr) != fileBuffWritePtr) {
		Logger::error("Write to SDCard failed!");
		SysSettings.useSD = false; //borked so stop trying.
		fileBuffWritePtr = 0;
		return;
	}
	fileRef.sync(); //needed in order to update the file if you aren't closing it ever
	SysSettings.logToggle = !SysSettings.logToggle;
	//setLED(SysSettings.LED_LOGGING, SysSettings.logToggle);
	fileBuffWritePtr = 0;

}

boolean Logger::setupFile()
{
	if (!fileRef.isOpen())  //file not open. Try to open it.
	{
		String filename;
		if (settings.appendFile == 1)
		{
			filename = String(settings.fileNameBase);
			filename.concat(".");
			filename.concat(settings.fileNameExt);
			// fileRef.open(filename.c_str(), uint8_t(O_APPEND | O_WRITE));
		}
		else {
			filename = String(settings.fileNameBase);
			filename.concat(settings.fileNum++);
			filename.concat(".");
			filename.concat(settings.fileNameExt);
			EEPROM.put(EEPROM_ADDRESS, settings); //save settings to save updated filenum
			// fileRef.open(filename.c_str(), O_CREAT | O_TRUNC | O_WRITE);
		}
		if (!fileRef.isOpen())
		{
			Logger::error("open failed");
			return false;
		}
	}

	//Before we add the next frame see if the buffer is nearly full. if so flush it first.
	if (fileBuffWritePtr > SD_BUFF_SIZE - 40)
	{
		flushFileBuff();
	}
	return true;
}

void Logger::loop()
{
	if (fileBuffWritePtr > 0) {
		if (millis() > (lastWriteTime + 1000)) //if it's been at least 1 second since the last write and we have data to write
		{
			flushFileBuff();
		}
	}
}

void Logger::file(const char *message, ...)
{
	if (!SysSettings.SDCardInserted) return; // not possible to log without card

	char buff[20];

	va_list args;
	va_start(args, message);	

	if (!setupFile()) return;
	
	for (; *message != 0; ++message) {
		if (*message == '%') {
			++message;

			if (*message == '\0') {
				break;
			}

			if (*message == '%') {
				buffPutChar(*message);
				continue;
			}

			if (*message == 's') {
				register char *s = (char *)va_arg(args, int);
				buffPutString(s);
				continue;
			}

			if (*message == 'd' || *message == 'i') {
				sprintf(buff, "%i", va_arg(args, int));
				buffPutString(buff);
				continue;
			}

			if (*message == 'f') {
				sprintf(buff, "%f0.2", va_arg(args, double));
				buffPutString(buff);
				continue;
			}

			if (*message == 'x') {
				sprintf(buff, "%x", va_arg(args, int));
				buffPutString(buff);				
				continue;
			}

			if (*message == 'X') {
				buffPutString("0x");
				sprintf(buff, "%x", va_arg(args, int));
				buffPutString(buff);
				continue;
			}

			if (*message == 'l') {
				// sprintf(buff, "%l", va_arg(args, long));
				buffPutString(buff);
				continue;
			}

			if (*message == 'c') {
				buffPutChar(va_arg(args, int));
				continue;
			}

			if (*message == 't') {
				if (va_arg(args, int) == 1) {
					buffPutString("T");
				}
				else {
					buffPutString("F");
				}

				continue;
			}

			if (*message == 'T') {
				if (va_arg(args, int) == 1) {
					buffPutString("TRUE");
				}
				else {
					buffPutString("FALSE");
				}
				continue;
			}

		}

		buffPutChar(*message);
	}

	buffPutString("\r\n");

	va_end(args);

}

void Logger::fileRaw(uint8_t* buff, int sz) 
{
	if (!SysSettings.SDCardInserted) return; // not possible to log without card

	if (!setupFile()) return;

	for (int i=0; i < sz; i++) {
		buffPutChar(*buff++);
	}
}

/*
 * Set the log level. Any output below the specified log level will be omitted.
 */
void Logger::setLoglevel(LogLevel level)
{
    logLevel = level;
}

/*
 * Retrieve the current log level.
 */
Logger::LogLevel Logger::getLogLevel()
{
    return logLevel;
}

/*
 * Return a timestamp when the last log entry was made.
 */
uint32_t Logger::getLastLogTime()
{
    return lastLogTime;
}

/*
 * Returns if debug log level is enabled. This can be used in time critical
 * situations to prevent unnecessary string concatenation (if the message won't
 * be logged in the end).
 *
 * Example:
 * if (Logger::isDebug()) {
 *    Logger::debug("current time: %d", millis());
 * }
 */
boolean Logger::isDebug()
{
    return logLevel == Debug;
}

/*
 * Output a log message (called by debug(), info(), warn(), error(), console())
 *
 * Supports printf() like syntax:
 *
 * %% - outputs a '%' character
 * %s - prints the next parameter as string
 * %d - prints the next parameter as decimal
 * %f - prints the next parameter as double float
 * %x - prints the next parameter as hex value
 * %X - prints the next parameter as hex value with '0x' added before
 * %b - prints the next parameter as binary value
 * %B - prints the next parameter as binary value with '0b' added before
 * %l - prints the next parameter as long
 * %c - prints the next parameter as a character
 * %t - prints the next parameter as boolean ('T' or 'F')
 * %T - prints the next parameter as boolean ('true' or 'false')
 */
void Logger::log(LogLevel level, const char *format, va_list args)
{
    lastLogTime = millis();
    Serial.print(lastLogTime);
    Serial.print(" - ");

    switch (level) {
        case Debug:
            Serial.print("DEBUG");
            break;

        case Info:
            Serial.print("INFO");
            break;

        case Warn:
            Serial.print("WARNING");
            break;

        case Error:
            Serial.print("ERROR");
            break;
		case Off:
		    break;
    }

    Serial.print(": ");

    logMessage(format, args);
}

/*
 * Output a log message (called by log(), console())
 *
 * Supports printf() like syntax:
 *
 * %% - outputs a '%' character
 * %s - prints the next parameter as string
 * %d - prints the next parameter as decimal
 * %f - prints the next parameter as double float
 * %x - prints the next parameter as hex value
 * %X - prints the next parameter as hex value with '0x' added before
 * %b - prints the next parameter as binary value
 * %B - prints the next parameter as binary value with '0b' added before
 * %l - prints the next parameter as long
 * %c - prints the next parameter as a character
 * %t - prints the next parameter as boolean ('T' or 'F')
 * %T - prints the next parameter as boolean ('true' or 'false')
 */
void Logger::logMessage(const char *format, va_list args)
{
    for (; *format != 0; ++format) {
        if (*format == '%') {
            ++format;

            if (*format == '\0') {
                break;
            }

            if (*format == '%') {
                Serial.print(*format);
                continue;
            }

            if (*format == 's') {
                register char *s = (char *) va_arg(args, int);
                Serial.print(s);
                continue;
            }

            if (*format == 'd' || *format == 'i') {
                Serial.print(va_arg(args, int), DEC);
                continue;
            }

            if (*format == 'f') {
                Serial.print(va_arg(args, double), 2);
                continue;
            }

            if (*format == 'x') {
                Serial.print(va_arg(args, int), HEX);
                continue;
            }

            if (*format == 'X') {
                Serial.print("0x");
                Serial.print(va_arg(args, int), HEX);
                continue;
            }

            if (*format == 'b') {
                Serial.print(va_arg(args, int), BIN);
                continue;
            }

            if (*format == 'B') {
                Serial.print("0b");
                Serial.print(va_arg(args, int), BIN);
                continue;
            }

            if (*format == 'l') {
                Serial.print(va_arg(args, long), DEC);
                continue;
            }

            if (*format == 'c') {
                Serial.print(va_arg(args, int));
                continue;
            }

            if (*format == 't') {
                if (va_arg(args, int) == 1) {
                    Serial.print("T");
                } else {
                    Serial.print("F");
                }

                continue;
            }

            if (*format == 'T') {
                if (va_arg(args, int) == 1) {
                    Serial.print("TRUE");
                } else {
                    Serial.print("FALSE");
                }

                continue;
            }

        }

        Serial.print(*format);
    }

    Serial.println();
}
