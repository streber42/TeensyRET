/*
 * GEVCU.h
 *
Copyright (c) 2013 Collin Kidder, Michael Neuweiler, Charles Galpin

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

#ifndef GVRET_H_
#define GVRET_H_

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SdFat.h>
#include <EEPROM.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

enum STATE {
	IDLE,
	GET_COMMAND,
	BUILD_CAN_FRAME,
	TIME_SYNC,
	GET_DIG_INPUTS,
	GET_ANALOG_INPUTS,
	SET_DIG_OUTPUTS,
	SETUP_CANBUS,
	GET_CANBUS_PARAMS,
	GET_DEVICE_INFO,
	SET_SINGLEWIRE_MODE,
	SET_SYSTYPE,
	ECHO_CAN_FRAME
};

void loadSettings();
void processDigToggleFrame(CAN_message_t &frame);
void sendDigToggleMsg();

class SerialConsole {
public:
    SerialConsole();
	void printMenu();
	void rcvCharacter(uint8_t chr);

protected:
	enum CONSOLE_STATE
	{
		STATE_ROOT_MENU
	};

private:
	char cmdBuffer[80];
	int ptrBuffer;
	int state;
    
	void init();
	void handleConsoleCmd();
	void handleShortCmd();
	void handleConfigCmd();
	void handleLawicelCmd();
	bool handleFilterSet(uint8_t bus, uint8_t filter, char *values);
	bool handleCAN0Send(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> &port, char *inputString);
	bool handleCAN1Send(FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> &port, char *inputString);
	unsigned int parseHexCharacter(char chr);
	unsigned int parseHexString(char *str, int length);
};

class Logger
{
public:
    enum LogLevel {
        Debug = 0, Info = 1, Warn = 2, Error = 3, Off = 4
    };
    static void debug(const char *, ...);
	static void info(const char *, ...);
	static void warn(const char *, ...);
	static void error(const char *, ...);
	static void console(const char *, ...);
	static void file(const char *, ...);
	static void fileRaw(uint8_t*, int);
    static void setLoglevel(LogLevel);
    static LogLevel getLogLevel();
    static uint32_t getLastLogTime();
    static boolean isDebug();
	static void loop();
private:
    static LogLevel logLevel;
    static uint32_t lastLogTime;

	static SdFile fileRef; //file we're logging to
	static uint8_t filebuffer[SD_BUFF_SIZE]; //size of buffer for file output
	static uint16_t fileBuffWritePtr;
	static uint32_t lastWriteTime;

	static void log(LogLevel, const char *format, va_list);
	static void logMessage(const char *format, va_list args);
	static void buffPutChar(char c);
	static void buffPutString(const char *c);
	static boolean setupFile();
	static void flushFileBuff();
};
#endif /* GVRET_H_ */




