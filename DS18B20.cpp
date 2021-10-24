#include <DS18B20.h>

#ifdef ARDUINO_ARCH_RP2040
#include "hardware/gpio.h"
#include <vector>

std::vector<rom_address_t2> found_addresses2;

#endif

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
	if( isTime2(DS18B20_TIMER_UPLOAD)){
		// Serial.print(F("upload")); Serial.println(id);
		// nextTimer2(DS18B20_TIMER_UPLOAD, uploadFrequency_s);
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
		Serial.print(F("reset staticReadTimer")); Serial.println(id);
		staticLockTimer = 0;
		staticLockId = 0;
		// readStart = 0;

		timerOff(DS18B20_TIMER_READ);
		// nextTimer2(DS18B20_TIMER_TST , sensorFrequency_s);
		nextTimer2(DS18B20_TIMER_REQUEST, sensorFrequency_s);

		lastError = ERR_DS18B20_STATIC_TIMER_RELEASED;

		// #ifdef DEBUG
				Serial.println(F("reset staticReadTimer"));
		// #endif
	}

 	// return false;
 

	if( isTime2( DS18B20_TIMER_REQUEST))
	{ 
		// Serial.print(F("timerId:"));     Serial.println(DS18B20_TIMER_REQUEST);
		
	// } 
	 	
 
	// if( (errorCnt < 1 && (millis() % 7000) == 500 )
	//  || (errorCnt > 0 && (millis() % 3000) == 400 && errorCnt < 3 )
	//  || (errorCnt > 2 && (millis() % 60000) == 300  )
	// ){
		int retCode =   requestTemp();

		#ifdef DEBUG
			Serial.print(F("reqTmp: @")); Serial.print( millis());   Serial.print(F(" ")); Serial.println(id);
			Serial.print(F("reqTmp")); Serial.print(id);Serial.print(F(" ret=")); Serial.println(retCode);
		#endif

		if(retCode < 0) {

			lastError = retCode;
			if(errorCnt<58) errorCnt++;

			// nextTimer2(DS18B20_TIMER_TST, 2+errorCnt);
			nextTimer2(DS18B20_TIMER_REQUEST, 2 + errorCnt );

			#ifdef DEBUG
				Serial.print(F("req err=")); Serial.println(retCode);
			#endif

		} else {

			// readStart = millis();
			nextTimer2(DS18B20_TIMER_REQUEST,  sensorFrequency_s);//,  sensorFrequency_s  );

			if(async)
			{
				nextTimerMillis2(DS18B20_TIMER_READ, readDelay_ms);
			}
			else
			{
				delay(readDelay_ms);
				nextTimerMillis2(DS18B20_TIMER_READ, 0);
			}
			
			// staticLockId = id;
			// staticLockTimer = timers2[DS18B20_TIMER_READ]; //readTimer;	// lock the library by this instance
		}
		return false;
	}

 

	if( isTime2(DS18B20_TIMER_READ)
	){
		timerOff(DS18B20_TIMER_READ);
		int retCode = readTemp();

		#ifdef DEBUG
			Serial.print(F("@")); Serial.print( millis());   
			Serial.print(F("readTmp")); Serial.print(id);Serial.print(F(" ret=")); Serial.println(retCode);
			Serial.print(F("temp="));   Serial.println(temp);
			Serial.print(F("prevTemp="));   Serial.println(prevTemp);
		#endif


		// staticLockTimer = 0;	// release library lock
		// staticLockId = 0;

		if(retCode<0){

			lastError = retCode;
			if(errorCnt<58) errorCnt++;

			// nextTimer2(DS18B20_TIMER_TST, 2+errorCnt);
			nextTimer2(DS18B20_TIMER_REQUEST, 2+errorCnt);

			if(!fixedAddr ) initAdress = false; //searchAdres(false);	// here we re-try to get an address

		} else {

			timeStamp = millis();
			errorCnt = 0;

			// nextTimer2(DS18B20_TIMER_TST );
			// nextTimer2(DS18B20_TIMER_REQUEST,5 );
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

		#ifdef DEBUG
			Serial.print(F("requestTemp")); Serial.println("initAdress"); 
		#endif

		searchAdres(false); // just pick one of the adresses found. If only one connected this is it!
		if( addr[0] == 0x00)
		{
			return ERR_DS18B20_NOT_FOUND;
		}
		else
		{
			// if (OneWire::crc8(addr, 7) != addr[7])
			if (crc8(addr, 7) != addr[7])
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

	#ifdef DEBUG
		Serial.println("requestTemp.reset"); 
	#endif

	// if(OneWire::reset() != 1) return ERR_DS18B20_NOT_FOUND;
	if(!resetDevice()) return ERR_DEVICE_NOT_FOUND;

	// OneWire::select(addr);
	selectAddress(addr);

	// if(resolution != 12)
	// {
	// 	OneWire::write(0x4e, 1);
	// 	OneWire::write(0x00, 1);
	// 	OneWire::write(0x00, 1);
	// 	if(resolution == 9)
	// 	OneWire::write(0x00, 1);
	// 	if(resolution == 10)
	// 	OneWire::write(0x20, 1);
	// 	if(resolution == 11)
	// 	OneWire::write(0x40, 1);
	// 	if(resolution == 12)
	// 	OneWire::write(0x70, 1);   // default setting
	// }
	{
		write(0x4e);
		write(0x00);
		write(0x00);
		if(resolution == 9)		write(0x00);
		if(resolution == 10)	write(0x20);
		if(resolution == 11)	write(0x40);
		if(resolution == 12)	write(0x70);   // default setting
	}
	 
	// if(OneWire::reset() != 1) return ERR_DS18B20_NOT_FOUND;
	if(!resetDevice()) return ERR_DEVICE_NOT_FOUND;

	// OneWire::select(addr);
	selectAddress(addr);

	#ifdef DEBUG
		Serial.println("start conversion"); 
	#endif

	// OneWire::write(0x44, 1); // start conversion, with parasite power on at the end
	write(0x44);

	return 0;
}


int DS18B20::readTemp()   // set temp (C *100), errorCnt ret 1=ok <0= error
{
	byte dataBuf[12];  //9 zou genoeg moeten zijn ???

	// we might do a ds.depower() here, but the reset will take care of it.

	// OneWire::reset();
	resetDevice();
	// OneWire::select(addr);
	selectAddress(addr);

	// OneWire::write(0xBE);         // request to read 8 bytes (Scratchpad)
	write(0xBE);	

	for(int i = 0; i < 9; i++)
	{           // we need 9 bytes
		// dataBuf[i] = OneWire::read();
		dataBuf[i] = read();
	}

	#ifdef compileWithDebug
		Serial.print(F("data="));
		for( int i = 0; i < 9; i++)
		{
			Serial.print(dataBuf[i], HEX);Serial.write(' ');
		}Serial.println();
	#endif

	if(dataBuf[2] != 0x00 || dataBuf[3] != 0x00){ return ERR_DS18B20_CONVERSION;}  // check conversion ready by comparing high low temp 
	// if(OneWire::crc8(dataBuf, 8) != dataBuf[8]) { return ERR_DS18B20_READ_CRC; }
	if(crc8(dataBuf, 8) != dataBuf[8]) { return ERR_DS18B20_READ_CRC; }


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

		nextTimer2( DS18B20_TIMER_TST, 7);
		nextTimer2(DS18B20_TIMER_REQUEST, 7);

		// prevent klepperen tussen 1 graad verschil
		int delta =  temp>tempUploaded ? temp-tempUploaded : tempUploaded-temp;
		if( delta >= 100 ){
			nextTimer2( DS18B20_TIMER_UPLOAD, 0);		
		}
	} 

	return 1;
}


void DS18B20::upload()
{
	nextTimer2(DS18B20_TIMER_UPLOAD, uploadFrequency_s);

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
	Serial.print(F(", async="));	Serial.print(async);

	
	Serial.print(F(", req@="));	Serial.print( timers2[DS18B20_TIMER_REQUEST]/1000   );



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

void DS18B20::initTimers(int count)
{
	for(int i=0; i<count; i++){
		timers2[i]=0L;
	}
	nextTimerMillis2(DS18B20_TIMER_REQUEST, 500);
}

// bool DS18B20::isTime( int id){
// 	if(timers2[id] == 0L) return false;

// 	if( millis() > timers2[id]) return true;
// 	return (timers2[id] -  millis() ) >  0x0fffffff;  
// }
bool DS18B20::isTime2( int id){
	if(timers2[id] == 0L) return false;

	unsigned long delta = millis() > timers2[id] ? millis() - timers2[id] : timers2[id] - millis() ;
	return delta > 0x0fffffff ? false : millis() >= timers2[id];
}

bool DS18B20::isTimerActive( int id ){
	return timers2[id] > 0;
}
bool DS18B20::isTimerInactive( int id ){
	return timers2[id] == 0;
}
void DS18B20::timerOff( int id ){
	timers2[id]=0;
}

void DS18B20::nextTimerMillis2( int id, unsigned long periode){

	if(periode<0) periode=0;
	timers2[id] = millis() + periode;
	if(timers2[id]==0) timers2[id]=1;
}

void DS18B20::searchAdres(boolean print)
{
	// Serial.println(F("reset_search"));

#ifndef ARDUINO_ARCH_RP2040
	OneWire::reset_search();
 
	while(OneWire::search(addr))
	{ 	// the last one is remembered in addr.
		if(print) printAdres();
	}

	// Serial.println(F("searchAdres done"));
	return;

#else

	found_addresses2.clear();

	if(search_rom_find_next()){	
		 
		// just take the first	
		for (uint8_t i = 0; i < ROMSize2; i++) {
			addr[i] = get_address(0).rom[i] ;
		}
		if(print) printAdres();

	} else {
		Serial.println(F("searchAdres NOT FOUND!"));			
	}
#endif
}


void DS18B20::write(uint8_t data){
#ifndef ARDUINO_ARCH_RP2040

	OneWire::write(data, 1);
#else

	onewire_byte_out(data);
#endif
}

uint8_t DS18B20::read(){
#ifndef ARDUINO_ARCH_RP2040

	return OneWire::read();
#else

	return onewire_byte_in();
#endif
}


void DS18B20::selectAddress(const uint8_t addr[8]){

#ifndef ARDUINO_ARCH_RP2040

	OneWire::select(addr);
#else
	int i;
 
	onewire_byte_out(0x55 );  //MatchROMCommand

	for (i = 0; i < 8; i++) {
		onewire_byte_out(addr[i]);
	}
#endif
}



bool DS18B20::resetDevice(){

#ifndef ARDUINO_ARCH_RP2040

	return OneWire::reset();
#else
 
	// This will return false if no devices are present on the data bus
	bool presence = false;
	gpio_set_dir(_pin, GPIO_OUT);
	gpio_put(_pin, false); // bring low for 480us
	sleep_us(480);
	gpio_set_dir(_pin, GPIO_IN); // let the data line float high
	sleep_us(70); // wait 70us
	if (!gpio_get(_pin)) {
		// see if any devices are pulling the data line low
		presence = true;
	}
	sleep_us(410);
	return presence;

#endif
}




uint8_t DS18B20::crc8(const uint8_t *addr, uint8_t len){

#ifndef ARDUINO_ARCH_RP2040

	return OneWire::crc8(addr, len  );
#else

	static const uint8_t PROGMEM dscrc2x16_table[] = {
		0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
		0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
		0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
		0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
	};


	uint8_t crc = 0;

	while (len--) {
		crc = *addr++ ^ crc;  // just re-using crc as intermediate
		crc = pgm_read_byte(dscrc2x16_table + (crc & 0x0f)) ^
		pgm_read_byte(dscrc2x16_table + 16 + ((crc >> 4) & 0x0f));
	}

	return crc;
	// return 0x00;

#endif
}



#ifdef ARDUINO_ARCH_RP2040

// for now we use the fast PROGMEM table !!!!
// uint8_t DS18B20::crc_byte(uint8_t crc, uint8_t byte) {
// 	int j;
// 	for (j = 0; j < 8; j++) {
// 		if ((byte & 0x01) ^ (crc & 0x01)) {
// 			// DATA ^ LSB CRC = 1
// 			crc = crc >> 1;
// 			// Set the MSB to 1
// 			crc = (uint8_t) (crc | 0x80);
// 			// Check bit 3
// 			if (crc & 0x04) {
// 				crc = (uint8_t) (crc & 0xFB);// Bit 3 is set, so clear it
// 			} else {
// 				crc = (uint8_t) (crc | 0x04);// Bit 3 is clear, so set it
// 			}
// 			// Check bit 4
// 			if (crc & 0x08) {
// 				crc = (uint8_t) (crc & 0xF7);// Bit 4 is set, so clear it
// 			} else {
// 				crc = (uint8_t) (crc | 0x08);// Bit 4 is clear, so set it
// 			}
// 		} else {
// 			// DATA ^ LSB CRC = 0
// 			crc = crc >> 1;
// 			// clear MSB
// 			crc = (uint8_t) (crc & 0x7F);
// 			// No need to check bits, with DATA ^ LSB CRC = 0, they will remain unchanged
// 		}
// 		byte = byte >> 1;
// 	}
// 	return crc;
// }

rom_address_t2 &DS18B20::get_address(int index) {
	return found_addresses2[index];
}

void DS18B20::onewire_bit_out(bool bit_data) const {
	gpio_set_dir(_pin, GPIO_OUT);
	gpio_put(_pin, false);
	sleep_us(3);// (spec 1-15us)
	if (bit_data) {
		gpio_put(_pin, true);
		sleep_us(55);
	} else {
		sleep_us(60);// (spec 60-120us)
		gpio_put(_pin, true);
		sleep_us(5);// allow bus to float high before next bit_out
	}
}

void DS18B20::onewire_byte_out(uint8_t data) {
	int n;
	for (n = 0; n < 8; n++) {
		onewire_bit_out((bool) (data & 0x01));
		data = data >> 1;// now the next bit is in the least sig bit position.
	}
}

bool DS18B20::onewire_bit_in() const {
	bool answer;
	gpio_set_dir(_pin, GPIO_OUT);
	gpio_put(_pin, false);
	sleep_us(3);// (spec 1-15us)
	gpio_set_dir(_pin, GPIO_IN);
	sleep_us(3);// (spec read within 15us)
	answer = gpio_get(_pin);
	sleep_us(45);
	return answer;
}

uint8_t DS18B20::onewire_byte_in() {
	uint8_t answer = 0x00;
	int i;
	for (i = 0; i < 8; i++) {
		answer = answer >> 1;// shift over to make room for the next bit
		if (onewire_bit_in())
			answer = (uint8_t) (answer | 0x80);// if the data port is high, make this bit a 1
	}
	return answer;
}

bool DS18B20::search_rom_find_next() {
	bool done_flag = false;
	int last_discrepancy = 0;
	uint8_t search_ROM[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	int discrepancy_marker, rom_bit_index;
	bool return_value, bitA, bitB;
	uint8_t byte_counter, bit_mask;

	return_value = false;
	while (!done_flag) {
		if (!resetDevice()) {
			Serial.printf("Failed to reset one wire bus\n");
			return false;
		} else {
			rom_bit_index = 1;
			discrepancy_marker = 0;
			onewire_byte_out( 0xF0 ); //SearchROMCommand
			byte_counter = 0;
			bit_mask = 0x01;
			while (rom_bit_index <= 64) {
				bitA = onewire_bit_in();
				bitB = onewire_bit_in();
				if (bitA & bitB) {
					discrepancy_marker = 0;// data read error, this should never happen
					rom_bit_index = 0xFF;
					Serial.printf("Data read error - no devices on bus?\r\n");
				} else {
					if (bitA | bitB) {
						// Set ROM bit to Bit_A
						if (bitA) {
							search_ROM[byte_counter] =
									search_ROM[byte_counter] | bit_mask;// Set ROM bit to one
						} else {
							search_ROM[byte_counter] =
									search_ROM[byte_counter] & ~bit_mask;// Set ROM bit to zero
						}
					} else {
						// both bits A and B are low, so there are two or more devices present
						if (rom_bit_index == last_discrepancy) {
							search_ROM[byte_counter] =
									search_ROM[byte_counter] | bit_mask;// Set ROM bit to one
						} else {
							if (rom_bit_index > last_discrepancy) {
								search_ROM[byte_counter] =
										search_ROM[byte_counter] & ~bit_mask;// Set ROM bit to zero
								discrepancy_marker = rom_bit_index;
							} else {
								if ((search_ROM[byte_counter] & bit_mask) == 0x00)
									discrepancy_marker = rom_bit_index;
							}
						}
					}
					onewire_bit_out(search_ROM[byte_counter] & bit_mask);
					rom_bit_index++;
					if (bit_mask & 0x80) {
						byte_counter++;
						bit_mask = 0x01;
					} else {
						bit_mask = bit_mask << 1;
					}
				}
			}
			last_discrepancy = discrepancy_marker;
			if (rom_bit_index != 0xFF) {
				uint8_t i = 0;
				while (true) {
					if (i >= found_addresses2.size()) {       //End of list, or empty list
						// if (rom_checksum_error(search_ROM)) {// Check the CRC
						// 	printf("failed crc\r\n");
						// 	return false;
						// }
						rom_address_t2 address{};
						for (byte_counter = 0; byte_counter < 8; byte_counter++) {
							address.rom[byte_counter] = search_ROM[byte_counter];
						}
						found_addresses2.push_back(address);

						return true;
					} else {//Otherwise, check if ROM is already known
						bool equal = true;
						uint8_t *ROM_compare = found_addresses2[i].rom;

						for (byte_counter = 0; (byte_counter < 8) && equal; byte_counter++) {
							if (ROM_compare[byte_counter] != search_ROM[byte_counter]) {
								equal = false;
							}
						}
						if (equal)
							break;
						else
							i++;
					}
				}
			}
		}
		if (last_discrepancy == 0)
			done_flag = true;
	}
	return return_value;
}
#endif


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


