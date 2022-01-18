#include <DS18B20.h>

bool DS18B20::loop(void)  //  0 when nothing changed,
/*
 * - periodically upload
 * - upload new error
 * - release staticTimer lock after 3 seconds
 * - when nothing pending
 *   - request temp from ds18b20
 *   - if ! async then wait
 * - when pending & timer expired
 *   - get response from ds18b20
 *   - additional error check: min, max & delta previous temp
 *
 *
 * - staticReadTimer > 0  will lock whole library
 * */

//  1 when a new valid value was found.
//   < 0 error -1 when an error occurred
{

	if( isTime(DS18B20_TIMER_UPLOAD)){
		upload() ;
	} else if( errorFunc!=0
	 && lastError!=lastErrorUploaded
	){
		errorFunc(id, lastError);
		lastErrorUploaded = lastError;
	}


	// unsigned long staticLockTimeout =  staticLockTimer + 1200;
	// unsigned long staticLockDuration = staticLockTimeout - millis() 

	// skip when locked by other   !! double check dead lock possible
	if( staticLockId != 0
	 && id != 0 				// when my id = 0 then no others use the same pin ???
	 && staticLockId != id

	){
		return false;
	}


	// release lock after 3 seconds. Skip to next
	if( id != 0
	 && staticLockId == id 
	 && millis() > staticLockTimer + 1200
	 && ( millis() > staticLockTimer + 1200 ? millis()-staticLockTimer + 1200 : staticLockTimer + 1200-millis()) < 0x0fffffff 
	){
		staticLockTimer = 0;
		staticLockId = 0;
		// readStart = 0;

		timerOff(DS18B20_TIMER_READ);
		nextTimer(DS18B20_TIMER_REQUEST , sensorFrequency_s);

		lastError = ERR_DS18B20_STATIC_TIMER_RELEASED;

		#ifdef DEBUG
				Serial.println(F("reset staticReadTimer"));
		#endif
	}


	if( isTime(DS18B20_TIMER_REQUEST))
	{
		nextTimer(DS18B20_TIMER_REQUEST , sensorFrequency_s);

		int retCode = requestTemp();

		#ifdef DEBUG
			Serial.print(F("reqTmp")); Serial.print(id);Serial.print(F(" ret=")); Serial.println(retCode);
		#endif

		if(retCode < 0) {

			lastError = retCode;
			if(errorCnt<58) errorCnt++;

			nextTimer(DS18B20_TIMER_REQUEST, 2+errorCnt);

			#ifdef DEBUG
				Serial.print(F("req err=")); Serial.println(retCode);
			#endif

		} else {

			// readStart = millis();

			if(async)
			{
				nextTimerMillis(DS18B20_TIMER_READ, readDelay_ms);
			}
			else
			{
				delay(readDelay_ms);
				nextTimerMillis(DS18B20_TIMER_READ, 0);
			}

			staticLockId = id;
			staticLockTimer = timers[DS18B20_TIMER_READ]; //readTimer;	// lock the library by this instance
		}
	}


	if( isTime(DS18B20_TIMER_READ)
	){

		timerOff(DS18B20_TIMER_READ);
		int retCode = readTemp();

		#ifdef DEBUG
			Serial.print(F("readTmp")); Serial.print(id);Serial.print(F(" ret=")); Serial.println(retCode);
			Serial.print(F("temp="));   Serial.println(temp);
			Serial.print(F("prevTemp="));   Serial.println(prevTemp);
		#endif


		staticLockTimer = 0;	// release library lock
		staticLockId = 0;

		if(retCode<0){

			lastError = retCode;
			if(errorCnt<58) errorCnt++;

			nextTimer(DS18B20_TIMER_REQUEST, 2+errorCnt);

			if(!fixedAddr ) initAdress = false; //searchAdres(false);	// here we re-try to get an address

		} else {

			timeStamp = millis();
			errorCnt = 0;

			nextTimer(DS18B20_TIMER_REQUEST );

			return true;
		}
	}

	return false;
} // end temp.loop()


int DS18B20::requestTemp()
/*
 * - init address if needed.
 * */
{
	if( !initAdress
     && !fixedAddr
	){
		searchAdres(false); // just pick one of the adresses found. If only one connected this is it!
		if( addr[0] == 0x00)
		{
			return ERR_DS18B20_NOT_FOUND;
		}
		else
		{
			if (OneWire::crc8(addr, 7) != addr[7])
			{
			    return ERR_DS18B20_ADDRESS_CRC;
			}
			if (addr[0] != 0x28)
			{
			    return ERR_DS18B20_ADDRESS_0X28;
			}
			initAdress = true;
		}
	}

	if(OneWire::reset() != 1) return ERR_DS18B20_NOT_FOUND;

	OneWire::select(addr);

	if(resolution != 12)
	{
		OneWire::write(0x4e, 1);
		OneWire::write(0x00, 1);
		OneWire::write(0x00, 1);
		if(resolution == 9)
		OneWire::write(0x00, 1);
		if(resolution == 10)
		OneWire::write(0x20, 1);
		if(resolution == 11)
		OneWire::write(0x40, 1);
		if(resolution == 12)
		OneWire::write(0x70, 1);   // default setting
	}

	if(OneWire::reset() != 1) return ERR_DS18B20_NOT_FOUND;

	OneWire::select(addr);

	OneWire::write(0x44, 1); // start conversion, with parasite power on at the end

	return 0;
}


int DS18B20::readTemp()   // set temp (C *100), errorCnt ret 1=ok <0= error
{
	byte dataBuf[12];  //9 zou genoeg moeten zijn ???

	// we might do a ds.depower() here, but the reset will take care of it.

	OneWire::reset();
	OneWire::select(addr);
	OneWire::write(0xBE);         // request to read 8 bytes (Scratchpad)

	for(int i = 0; i < 9; i++)
	{           // we need 9 bytes
		dataBuf[i] = OneWire::read();
	}

	#ifdef compileWithDebug
		Serial.print(F("data="));
		for( int i = 0; i < 9; i++)
		{
			Serial.print(dataBuf[i], HEX);Serial.write(' ');
		}Serial.println();
	#endif

	if(dataBuf[2] != 0x00 || dataBuf[3] != 0x00){ return ERR_DS18B20_CONVERSION;}  // check conversion ready by comparing high low temp 
	if(OneWire::crc8(dataBuf, 8) != dataBuf[8]) { return ERR_DS18B20_READ_CRC; }


	int16_t rawTemp = (dataBuf[1] << 8) | dataBuf[0];   //data=80 1 0  0  1F FF 10 10 AE  = 24.00 Celsius,
	                                          //data=50 5 4B 46 7F FF C  10 1C = 85.00 Celsius,

	byte cfg = (dataBuf[4] & 0x60);
	// at lower res, the low bits are undefined, so let's zero them

	// default is 12 bit resolution, 750 ms conversion time

	if(cfg == 0x00)								// 9 bit resolution, 93.75 ms
		rawTemp = rawTemp & ~7;
	else if(cfg == 0x20)						// 10 bit res, 187.5 ms
		rawTemp = rawTemp & ~3;
	else if(cfg == 0x40)						// 11 bit res, 375 ms
		rawTemp = rawTemp & ~1;


	prevTemp = temp;
	temp = (rawTemp / 16.0) * 100;  			// C *100

	if( tempMax != TEMP_NOT_SET 
	 && temp > tempMax
	){
		 return ERR_DS18B20_MAX_TEMP; 
	}

	if( tempMin != TEMP_NOT_SET 
	 && temp < tempMin
	){ 
		return ERR_DS18B20_MIN_TEMP; 
	} 

	if( deltaTempMax > 0
	 && prevTemp != TEMP_NOT_SET
	 && ((temp>prevTemp?temp-prevTemp:prevTemp-temp)) > deltaTempMax
	){
		return ERR_DS18B20_DELTA_TEMP;   
	}

	if(temp != tempUploaded){

		nextTimer( DS18B20_TIMER_REQUEST, 7);

		// prevent klepperen tussen 1 graad verschil
		int delta =  temp>tempUploaded ? temp-tempUploaded : tempUploaded-temp;
		if( delta > 100 ){
			nextTimer( DS18B20_TIMER_UPLOAD, 0);		
		}
	} 

	return 1;
}


void DS18B20::upload()
{
	nextTimer(DS18B20_TIMER_UPLOAD, uploadFrequency_s);

	if( uploadFunc==0
	 || errorCnt >= 3 ) return;

	uploadFunc(id, temp, timeStamp );
	tempUploaded = temp;	
}

void DS18B20::setAddr( const byte addres[8])
{
	//memcpy(this->addr, addres, sizeof(addr));
	for(int i = 0; i < 8; i++) this->addr[i] = addres[i];
}

void DS18B20::setSenseInterval(int periode_s)
{
	sensorFrequency_s = periode_s ;
}
void DS18B20::setSyncInterval(int periode_s)
{
	uploadFrequency_s = periode_s;
}


void DS18B20::setResolution(int resolution)
{
	this->resolution = resolution;
	if(resolution == 9) readDelay_ms = 300;
	if(resolution == 10) readDelay_ms = 450;
	if(resolution == 11) readDelay_ms = 750;
	if(resolution == 12) readDelay_ms = 900; // maybe 750ms is enough, maybe not
}

int   DS18B20::getTemp()   { return temp;}
int   DS18B20::getGraden() { return temp/100;}
float DS18B20::getCelcius(){ return (float) temp/100;}

void DS18B20::trace(){ trace(""); }
void DS18B20::trace(const char*  id  )
{
	Serial.print(F("@"));
	Serial.print(millis()/1000);
	Serial.print(F(" "));
	Serial.print(id);
	Serial.print(F(": "));
	Serial.print(getCelcius());
	Serial.print(F("C"));
	// if(valid)Serial.print(F("&"));else Serial.print(F("X"));
	Serial.print(F(", temp="));	Serial.print( temp);
	// Serial.print(F(", rqTmr="));	Serial.print(requestTimer/1000L);
	Serial.print(F(", errorCnt="));	Serial.print(errorCnt);
	Serial.print(F(", lastError="));	Serial.print(lastError);
//	Serial.print(F(", errUploaded="));	Serial.print(lastErrorUploaded);
//	Serial.print(F(", errSend="));	Serial.print(lastErrorSend);



//	Serial.print(F(", nextUpl="));	Serial.print( uploadTimer/1000L);

	Serial.print(F(", "));

	printAdres( );
	//Serial.println();
}

void DS18B20::searchAdres()
{
	searchAdres(false);
}

void DS18B20::printAdres( )
{
	Serial.print(F(" addr:"));	// this will be used when a read without addr is fired !!
	for(int i = 0; i < 8; i++)
	{
		Serial.write(' ');
	    Serial.print(addr[i], HEX);
	}
	Serial.println();
}

void DS18B20::searchAdres(boolean print)
{
	OneWire::reset_search();
	// Serial.println(F("listAdres"));
	while(OneWire::search(addr))
	{ 	// the last one is remembered in addr.
		if(print) printAdres();
	}

	return;
}

void DS18B20::initTimers(int count)
{
	for(int i=0; i<count; i++){
		timers[i]=0L;
	}

	nextTimerMillis(DS18B20_TIMER_REQUEST, 500);
}

// bool DS18B20::isTime( int id){
// 	if(timers[id] == 0L) return false;

// 	if( millis() > timers[id]) return true;
// 	return (timers[id] -  millis() ) >  0x0fffffff;  
// }
bool DS18B20::isTime( int id){
	if(timers[id] == 0L) return false;

	unsigned long delta = millis() > timers[id] ? millis() - timers[id] : timers[id] - millis() ;
	return delta > 0x0fffffff ? false : millis() >= timers[id];
	// if( millis() > timers[id]) return true;
	// return (timers[id] -  millis() ) >  0x0fffffff;  
}


bool DS18B20::isTimerActive( int id ){
	return timers[id] > 0;
}
bool DS18B20::isTimerInactive( int id ){
	return timers[id] == 0;
}
void DS18B20::timerOff( int id ){
	timers[id]=0;
}

void DS18B20::nextTimerMillis( int id, unsigned long periode){

	if(periode<0) periode=0;
	timers[id] = millis() + periode;
	if(timers[id]==0) timers[id]=1;
}


//class QueueItem
//{
//public:
//	int delta;
//	int temp;
//	int opt;
//	int delta2;
//	//   QueueItem( int delta, int temp, int opt )
////   {
////     this->delta 	= delta;
////     this->temp 	= temp;
////     this->opt 		= opt;
////     this->delta2 	= 0;
////   }
//	QueueItem(int delta, int temp, int opt, int delta2)
//	{
//		this->delta = delta;
//		this->temp = temp;
//		this->opt = opt;
//		this->delta2 = delta2;
//	}
//};

//
//#ifndef dsxx
//#define dsxx() byte dsxx[] {0x28, 0x66, 0x7f, 0x58, 0x05, 0x00, 0x00, 0x9c};
//#endif
/*
 switch (addr[0]) {
 case 0x10:
 Serial.println("  Chip = DS18S20");  // or old DS1820
 break;
 case 0x28:
 Serial.println("  Chip = DS18B20");
 break;
 case 0x22:
 Serial.println("  Chip = DS1822");
 break;
 default:
 Serial.println("Device is not a DS18x20 family device.");
 return;
 }
 */


