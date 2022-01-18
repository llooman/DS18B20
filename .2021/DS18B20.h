/*
 2020-05-23 refactor add staticLockId
 
 2020-05-18 refactor to myTimers
            refactor read when ! timerOff
 2019-03-25 Update desc
            Add overflow to timers
 2017-01-10 DS18B20::sendCurrentTemp sendError
			DS18B20::sendCurrentTemp retry netw.sendData

 Copyright (c) 2013-2013 J&L Computing.  All right reserved.
 Rev 1.0 - May 15th, 2013

 on the pin a 4.7K resistor is necessary for combined power and data

 #define DS18B20_PIN 6

 DS18B20 xxx( int dsPin );
 DS18B20 xxx( int dsPin, byte[] dsAddress );
 DS18B20 xxx( int dsPin, int uploadId );
 DS18B20 xxx( int dsPin, byte[] dsAddress, int uploadId );

 setup:
 xxx.setup();								use DS18B20_PIN and defaults
 xxx.setTimerPeriod ( xx );  				sense every xx seconds default = 7
 xxx.setResolution( [9, 10, 11, 12] ); 	default = 9
 xxx.listAdres();							find adress
 xxx.listAdres( true );					print adresses found
 xxx.deltaTempMax;      					error when change > delatTempMax, default 0
 xxx.tempMax;								error when temp > tempMax, default 100
 xxx.tempMin;								error when temp < tempMin, default -10
 xxx.async;
 loop:
 int retCode = xxx.loop();					check timer to read
 //  int retCode = xxx.read();
 //  int retCode = xxx.read( float *var );
 //  int retCode = xxx.read( addr, float *var);

 data:
 xxx.trace();
 int  temp = xxx.temp;  			in 1/100 graden
 float temp = xxx.getTemp()		in 1/100 graden
 float temp = xxx.getCelcius()		in graden
 time_t lastRead = xxx.timeStamp;
 xxx.errorCnt;
 xxx.lastError

 */

#ifndef DS18B20_h
#define DS18B20_h

// #define DEBUG

#define TEMP_NOT_SET                -27500
#define TEMP_INVALID                -27400

#define ERR_DS18B20_NOT_FOUND		-30
#define ERR_DS18B20_CONVERSION		-31
#define ERR_DS18B20_READ_CRC		-32
#define ERR_DS18B20_ADDRESS_CRC		-33
#define ERR_DS18B20_ADDRESS_0X28	-34  // Address does not start with 0x28
#define ERR_DS18B20_INVALID_TEMP	-35
#define ERR_DS18B20_STATIC_TIMER_RELEASED	-36
#define ERR_DS18B20_MAX_TEMP		-37
#define ERR_DS18B20_MIN_TEMP		-38
#define ERR_DS18B20_DELTA_TEMP		-39

 
#include <Arduino.h> 
#include <inttypes.h>
#include <OneWire.h>
#include <avr/pgmspace.h>


static unsigned long staticLockTimer = 0; // prevent conflicts with other DS18B20 on the same pin
static int           staticLockId = 0;
//const byte ds00[] PROGMEM = {0x00};

//const byte ds01[] PROGMEM = {0x28, 0x80, 0x7e, 0x00, 0x05, 0x00, 0x00, 0x0e};
//const byte ds03[] PROGMEM = {0x28, 0x11, 0x41, 0x00, 0x05, 0x00, 0x00, 0x63};
//const byte ds04[] PROGMEM = {0x28, 0x2b, 0x57, 0x2b, 0x05, 0x00, 0x00, 0x45};
//const byte ds05[] PROGMEM = {0x28, 0x53, 0x06, 0x01, 0x05, 0x00, 0x00, 0x4f};
//const byte ds06[] PROGMEM = {0x28, 0x00, 0x57, 0x2b, 0x05, 0x00, 0x00, 0x0b};
//const byte ds07[] PROGMEM = {0x28, 0xc4, 0xb0, 0x2b, 0x05, 0x00, 0x00, 0xc3};
//const byte ds08[] PROGMEM = {0x28, 0x92, 0x5f, 0x2b, 0x05, 0x00, 0x00, 0xea};
//const byte ds09[] PROGMEM = {0x28, 0xad, 0xb4, 0x2b, 0x05, 0x00, 0x00, 0x89};
//const byte ds10[] PROGMEM = {0x28, 0x60, 0x9E, 0x8C, 0x06, 0x00, 0x00, 0xD2};
//const byte ds11[] PROGMEM = {0x28, 0x24, 0xDF, 0x8C, 0x06, 0x00, 0x00, 0x5F};
//
//const byte ds12[] PROGMEM = {0x28, 0xff, 0xc8, 0x75, 0x82, 0x15, 0x01, 0x1d};  //28 FF C8 75 82 15 1 1D
//const byte ds13[] PROGMEM = {0x28, 0xff, 0x0e, 0x77, 0x82, 0x15, 0x01, 0xa4};  //28 FF  E 77 82 15 1 A4
//
////28 FF 1C 5E 0 16 1  28 4F 90 54 5 0 0 1B
//
//const byte ds101[] PROGMEM = {0x28, 0x66, 0x7f, 0x58, 0x05, 0x00, 0x00, 0x9c};
//const byte ds102[] PROGMEM = {0x28, 0x4f, 0x90, 0x54, 0x05, 0x00, 0x00, 0x1b};
//const byte ds103[] PROGMEM = {0x28, 0x2f, 0xec, 0x54, 0x05, 0x00, 0x00, 0x94};
//const byte ds104[] PROGMEM = {0x28, 0xde, 0xc3, 0x57, 0x05, 0x00, 0x00, 0xce};
//const byte ds105[] PROGMEM = {0x28, 0xFF, 0x37, 0x36, 0x66, 0x14, 0x03, 0x9D};  //28 FF 37 36 66 14 3 9D
//const byte ds106[] PROGMEM = {0x28, 0xff, 0x1c, 0x5e, 0x00, 0x16, 0x01, 0xc0};  //28 FF 1C 5E 0 16 1 C0

// ds107 28 37 DB 8C 6 0 0 42

//const byte ds108[] PROGMEM = {0x28, 0xff, 0x5e, 0x97, 0x66, 0x14, 0x02, 0x07};  //28 FF 5E 97 66 14 2 7

//const byte t30[] PROGMEM = {0x28, 0xff, 0x4d, 0x5f, 0xa6, 0x16, 0x03, 0x8c}; //28 FF 4D 5F A6 16 3 8C
//const byte t31[] PROGMEM = {0x28, 0xa7, 0x70, 0x8f, 0x33, 0x14, 0x01, 0x5a}; //28 A7 70 8F 33 14 1 5A
//const byte t32[] PROGMEM = {0x28, 0xff, 0xfd, 0x9b, 0xa6, 0x16, 0x03, 0xf4}; //28 FF FD 9B A6 16 3 F4
//const byte t33[] PROGMEM = {0x28, 0x92, 0xda, 0xac, 0x33, 0x14, 0x01, 0x2a}; //28 92 DA AC 33 14 1 2A
//const byte t34[] PROGMEM = {0x28, 0x68, 0xf5, 0x8f, 0x33, 0x14, 0x01, 0xf8}; //28 68 F5 8F 33 14 1 F8

//const byte los01[] PROGMEM = {0x28, 0xff, 0x15, 0x56, 0x80, 0x16, 0x04, 0x72}; //28 FF 15 56 80 16 4 72
//const byte los02[] PROGMEM = {0x28, 0xff, 0x9d, 0x5a, 0x80, 0x16, 0x04, 0x95}; // 28 FF 9D 5A 80 16 4 95


//const byte ds110[] PROGMEM = {0x28, 0xff, 0x68, 0x18, 0xa0, 0x16, 0x05, 0xcc}; //28 FF 68 18 A0 16 5 CC
//const byte ds111[] PROGMEM = {0x28, 0xff, 0x7f, 0xd9, 0x83, 0x16, 0x03, 0xfa}; //28 FF 7F D9 83 16 3 FA
//const byte ds112[] PROGMEM = {0x28, 0xff, 0xec, 0xf5, 0x83, 0x16, 0x03, 0x61}; //28 FF EC F5 83 16 3 61

//const byte ds113[] PROGMEM = {0x28, 0xff, 0x94, 0x31, 0x85, 0x16, 0x05, 0x09}; //addr: 28 FF 94 31 85 16 5 9
//const byte ds114[] PROGMEM = {0x28, 0xff, 0xff, 0x36, 0xa0, 0x16, 0x05, 0x4f}; //addr: 28 FF FF 36 A0 16 5 4F

#define DS18B20_TIMER_COUNT 3
#define DS18B20_TIMER_REQUEST 0
#define DS18B20_TIMER_READ 1
#define DS18B20_TIMER_UPLOAD 2

class DS18B20: public OneWire
{
public:
 	unsigned long 	timers[DS18B20_TIMER_COUNT];

	int 			id=0;

	int 			temp = TEMP_NOT_SET;    // in 1/100 graden
	int 			prevTemp = TEMP_NOT_SET;
	int 			tempUploaded = 0;
	unsigned long 	timeStamp = 0;

	// int 			retCode = -1;
	int 			errorCnt = 3;		

    volatile int  	lastError=0;
    int 			lastErrorUploaded = lastError;

	int 			deltaTempMax = 1000; 		// 0 = inactive, when the temp change is > delatTempMax we assume its an error
	int 			tempMax	= TEMP_NOT_SET;  	// + 100 C
	int 			tempMin = TEMP_NOT_SET;  	// -10 C

	int				sensorFrequency_s = 21;
	int				uploadFrequency_s = 60;

	// int 			readInterval_s = 5;   // default sence every 2 * 7 sec

	// unsigned long 	readStart;

	// used by wpomp
	// bool 		valid = false;
	// bool		updated = false;

	// uint8_t 		uploadInterval_s = 58;   // default sence every 60 sec


	byte 			addr[8];
	bool			fixedAddr = false;
	bool 			initAdress = false;

	bool 			async = true;



	DS18B20(int pin) : OneWire(pin)
	{
		_pin = pin;
		addr[0] = 0x00;
		setup( );
	}
	DS18B20(int pin, int ID) : OneWire(pin)
	{
		_pin = pin;
		id = ID;
		addr[0] = 0x00;
		setup( );
	}

	DS18B20(int pin, const byte addres[8]) : OneWire(pin)  // a 4.7K resistor is necessary for 2 wire power steal
	{
		_pin = pin;
		setup( );
		for(int i = 0; i < 8; i++) this->addr[i] = pgm_read_word_near(addres +i);
		initAdress = true;
		fixedAddr = true;
	}

	DS18B20(int pin, const byte addres[8], int ID) : OneWire(pin)  // a 4.7K resistor is necessary for 2 wire power steal
	{
		_pin = pin;
		id = ID;
		setup( );
		for(int i = 0; i < 8; i++) this->addr[i] = pgm_read_word_near(addres +i);
		initAdress = true;
		fixedAddr = true;
	}

	~DS18B20()
	{
		//delete _OneWire;
	}			// release memory

	void setup( )
	{
		temp 		 = TEMP_NOT_SET;
		prevTemp 	 = TEMP_NOT_SET;
		tempUploaded = TEMP_NOT_SET;

		staticLockTimer = 0;
		staticLockId = 0;

		initTimers(DS18B20_TIMER_COUNT);
	}


	int (*uploadFunc) (int id, long val, unsigned long timeStamp) = 0;
    void onUpload( int (*function)(int id, long val, unsigned long timeStamp) )
    {
    	uploadFunc = function;
    }
    void onUpload( int (*function)(int id, long val, unsigned long timeStamp), int id )
    {
    	this->id = id;
    	uploadFunc = function;
    }
    void upload(void);

	int (*errorFunc) (int id, long val ) = 0;
    void onError( int (*function)(int id, long val ) )
    {
    	errorFunc = function;
    }

	void initTimers(int count);
	bool isTime( int id);
	bool isTimerActive( int id );
	bool isTimerInactive( int id );
	void nextTimer( int id){ nextTimerMillis(id, sensorFrequency_s * 1000L );}	
	void nextTimer( int id, int periode){ nextTimerMillis(id, periode * 1000L );}	
	void nextTimerMillis( int id, unsigned long periode);
	void timerOff( int id );

    bool loop(void);
	// void loop(int retries);
	int requestTemp();
	int readTemp();

	void setAddr( const byte addres[8]);
	void setSenseInterval(int);
	void setSyncInterval(int);

	void setResolution(int resolution);

	void searchAdres();
	void printAdres();
	void searchAdres(boolean);
	int checkTemp(long refTemp);
	int getTemp();
	int getGraden();
	float getCelcius();

	void trace(void);
	void trace(const char sensorId[]);

private:
	void setErrorCnt(bool);

	uint8_t resolution = 9;   // default precision
	int readDelay_ms = 300;

	uint8_t _pin;

};


#endif
