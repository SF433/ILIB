#ifndef ILIB_H
#define ILIB_H

#include "Arduino.h"
#include <SPI.h>
#include <stdint.h>
#include <stdio.h> // for size_t
#include "WString.h"
#include "util/PRINTABLE.h"

//ISEG
#include <wiring_private.h>
#include <pins_arduino.h>
//PWM, LCD
#include <inttypes.h>
//DS3231, LCD
#include <Wire.h>
//LCD
//MODBUS_SLAVE
#include "util/LinkedList.h"

#define SEG_A   0b00000001
#define SEG_B   0b00000010
#define SEG_C   0b00000100
#define SEG_D   0b00001000
#define SEG_E   0b00010000
#define SEG_F   0b00100000
#define SEG_G   0b01000000

#define DEFAULT_BIT_DELAY  100


#ifndef F_CPU 
#define F_CPU 16000000L
#endif // !F_CPU

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                             ISEG                    ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


class ISEG {

public:
	ISEG(uint8_t pinClk, uint8_t pinDIO, unsigned int bitDelay = 100);

	void SET_BRIGHT(uint8_t brightness, bool on = true);
	void SET_SEG(const uint8_t segments[], uint8_t length = 4, uint8_t pos = 0);
	void CLEAR();
	void NUM_DEC(int num, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);
	void NUM_DEC_EX(int num, uint8_t dots = 0, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);
	void NUM_HEX_EX(uint16_t num, uint8_t dots = 0, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);

	uint8_t encodeDigit(uint8_t digit);

protected:
	void bitDelay();
	void start();
	void stop();
	bool writeByte(uint8_t b);
	void showDots(uint8_t dots, uint8_t* digits);
	void showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots = 0, bool leading_zero = false, uint8_t length = 4, uint8_t pos = 0);


private:
	uint8_t m_pinClk;
	uint8_t m_pinDIO;
	uint8_t m_brightness;
	unsigned int m_bitDelay;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////						WATCH			                   ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifdef __AVR_ATmega2560__

#define WDTO_16MS   0
#define WDTO_32MS	1
#define WDTO_64MS	2
#define WDTO_125MS	3
#define WDTO_250MS	4
#define WDTO_500MS	5
#define WDTO_1S	6
#define WDTO_2S	7
#define WDTO_4S	8
#define WDTO_8S	9
#endif // __AVR_ATmega2560__

#ifdef __AVR_ATmega128__
#define WDTO_14MS	0
#define WDTO_28MS	1
#define WDTO_56MS	2
#define WDTO_110MS	3
#define WDTO_220MS	4
#define WDTO_450MS	5
#define WDTO_900MS	6
#define WDTO_1D8S	7
#endif // __AVR_ATmega128__


void WDT_ENABLE(uint8_t ms);
void WDT_DISENABLE();
void WDT_RESET();
//class IWATCH {
//
//public:
//	IWATCH();
//	void SET_DATA(double sv, double inMax, double inMin, uint8_t samplingTime = 10);
//protected:
//
//private:
//	uint16_t repeat_k = 0;
//};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////						PID			                   ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class IPID {

public:
	IPID();
	void SET_DATA(double sv, double inMax, double inMin, uint8_t samplingTime = 10);
	void OUTPUT_SCALE(long max, long min);
	void ITERM_LIMIT(int max, int min);

	long PID_OUTPUT(double pv, double kp, double ki, double kd);

	//void findKuAndTu(double pv, double kp);
	void Ziegler_Nichols_Method(double Ku, double Tu, double* P, double* I = NULL, double* D = NULL);
protected:

private:
	uint16_t repeat_k = 0;
	uint8_t _samplingTime;
	long _max = 255, _min = 0;
	double _sv;
	double _inMax;
	double _inMin;
	double _pv;
	double _kp;
	double _ki;
	double _kd;
	double iMax, iMin = 0;
	double errsum, preError;
	unsigned long timeChange;
	unsigned long lastTime = 0;
	unsigned long lastTime1 = 0;
	unsigned int mScan = 0;
};



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                  PWM, NTEMP                   ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PWM(uint8_t pin, uint16_t val, bool onDutybit16 = 0);
void FDPWM(uint8_t pin, int32_t intHz, float Duty);
void PWM_RESET();
void NPWM_BEGIN(uint8_t pin, uint32_t intHz, float Duty, uint32_t N);
void NPWM(uint8_t pin);
void PWMOFF(uint8_t pin, bool POff);
int PSR(uint8_t channel);//PWM State Read
void PSR_FREQ(uint8_t channel);//PSR_FREQ State Read
void PSR_DUTY(uint8_t channel);//PSR_DUTY State Read
void TCNTSETUP(uint8_t timerNumber, bool on32bit = 0);

uint16_t TCNTOUT(uint8_t timernumer);
int INTC(unsigned int RawADC);
float INTC1(int32_t _ch, float _bValue = 3950.0f);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                            DHT                ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* DHT library 

MIT license
written by Adafruit Industries
*/
// how many timing transitions we need to keep track of. 2 * number bits + extra
#define MAXTIMINGS 85

#define DHT11 11
#define DHT22 22
#define DHT21 21
#define AM2301 21

class DHT {
 private:
  uint8_t data[6];
  uint8_t _pin, _type, _count;
  bool read(void);
  unsigned long _lastreadtime;
  bool firstreading;

 public:
  DHT(uint8_t pin, uint8_t type, uint8_t count=6);
  void BEGIN(void);
  float READ_TEMP(bool S=false);
  float convertCtoF(float);
  float READ_HUMI(void);

};
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                             DS3231                ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class DATE_TIME {
public:
	DATE_TIME(uint32_t t = 0);
	DATE_TIME(uint16_t year, uint8_t month, uint8_t day,
		uint8_t hour = 0, uint8_t min = 0, uint8_t sec = 0);
	DATE_TIME(const char* date, const char* time);
	uint16_t YEAR() const { return 2000 + yOff; }
	uint8_t MONTH() const { return m; }
	uint8_t DAY() const { return d; }
	uint8_t HOUR() const { return hh; }
	uint8_t MINUTE() const { return mm; }
	uint8_t SECOND() const { return ss; }
	uint8_t DAY_OF_THE_WEEK() const;

	// 32-bit times as seconds since 1/1/2000
	long secondstime() const;
	// 32-bit times as seconds since 1/1/1970
	// THE ABOVE COMMENT IS CORRECT FOR LOCAL TIME; TO USE THIS COMMAND TO
	// OBTAIN TRUE UNIX TIME SINCE EPOCH, YOU MUST CALL THIS COMMAND AFTER
	// SETTING YOUR CLOCK TO UTC
	uint32_t unixtime(void) const;
protected:
	uint8_t yOff, m, d, hh, mm, ss;
};

//checks if a year is a leap year
bool isleapYear(const uint8_t);

class RTClib {
public:
	// Get date and time snapshot
	static DATE_TIME NOW();
};

// Eric's original code is everything below this line
class DS3231 {
public:

	//Constructor
	DS3231();

	// Time-retrieval functions

	// the get*() functions retrieve current values of the registers.
	byte GET_SECOND();
	byte GET_MINUTE();
	byte GET_HOUR(bool& h12, bool& PM_time);
	// In addition to returning the hour register, this function
	// returns the values of the 12/24-hour flag and the AM/PM flag.
	byte GET_DOW();
	byte GET_DATE();
	byte GET_MONTH(bool& Century);
	// Also sets the flag indicating century roll-over.
	byte GET_YEAR();
	// Last 2 digits only

// Time-setting functions
// Note that none of these check for sensibility: You can set the
// date to July 42nd and strange things will probably result.

	void SET_SECOND(byte Second);
	// In addition to setting the seconds, this clears the 
	// "Oscillator Stop Flag".
	void SET_MINUTE(byte Minute);
	// Sets the minute
	void SET_HOUR(byte Hour);
	// Sets the hour
	void SET_DOW(byte DoW);
	// Sets the Day of the Week (1-7);
	void SET_DATE(byte Date);
	// Sets the Date of the Month
	void SET_MONTH(byte Month);
	// Sets the Month of the year
	void SET_YEAR(byte Year);
	// Last two digits of the year
	void SET_CLOCK_MODE(bool h12);
	// Set 12/24h mode. True is 12-h, false is 24-hour.

	void SET_ADJUST(uint8_t dow1, uint16_t year1, uint8_t month1, uint8_t date1, uint8_t hour1, uint8_t minute1, uint8_t second1);

	// Temperature function

	float GET_TEMPERATURE();

	// Alarm functions

	void getA1Time(byte& A1Day, byte& A1Hour, byte& A1Minute, byte& A1Second, byte& AlarmBits, bool& A1Dy, bool& A1h12, bool& A1PM);
	/* Retrieves everything you could want to know about alarm
	 * one.
	 * A1Dy true makes the alarm go on A1Day = Day of Week,
	 * A1Dy false makes the alarm go on A1Day = Date of month.
	 *
	 * byte AlarmBits sets the behavior of the alarms:
	 *	Dy	A1M4	A1M3	A1M2	A1M1	Rate
	 *	X	1		1		1		1		Once per second
	 *	X	1		1		1		0		Alarm when seconds match
	 *	X	1		1		0		0		Alarm when min, sec match
	 *	X	1		0		0		0		Alarm when hour, min, sec match
	 *	0	0		0		0		0		Alarm when date, h, m, s match
	 *	1	0		0		0		0		Alarm when DoW, h, m, s match
	 *
	 *	Dy	A2M4	A2M3	A2M2	Rate
	 *	X	1		1		1		Once per minute (at seconds = 00)
	 *	X	1		1		0		Alarm when minutes match
	 *	X	1		0		0		Alarm when hours and minutes match
	 *	0	0		0		0		Alarm when date, hour, min match
	 *	1	0		0		0		Alarm when DoW, hour, min match
	 */
	void getA2Time(byte& A2Day, byte& A2Hour, byte& A2Minute, byte& AlarmBits, bool& A2Dy, bool& A2h12, bool& A2PM);
	// Same as getA1Time();, but A2 only goes on seconds == 00.
	void setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM);
	// Set the details for Alarm 1
	void setA2Time(byte A2Day, byte A2Hour, byte A2Minute, byte AlarmBits, bool A2Dy, bool A2h12, bool A2PM);
	// Set the details for Alarm 2
	void turnOnAlarm(byte Alarm);
	// Enables alarm 1 or 2 and the external interrupt pin.
	// If Alarm != 1, it assumes Alarm == 2.
	void turnOffAlarm(byte Alarm);
	// Disables alarm 1 or 2 (default is 2 if Alarm != 1);
	// and leaves the interrupt pin alone.
	bool checkAlarmEnabled(byte Alarm);
	// Returns T/F to indicate whether the requested alarm is
	// enabled. Defaults to 2 if Alarm != 1.
	bool checkIfAlarm(byte Alarm);
	// Checks whether the indicated alarm (1 or 2, 2 default);
	// has been activated.

// Oscillator functions

	void enableOscillator(bool TF, bool battery, byte frequency);
	// turns oscillator on or off. True is on, false is off.
	// if battery is true, turns on even for battery-only operation,
	// otherwise turns off if Vcc is off.
	// frequency must be 0, 1, 2, or 3.
	// 0 = 1 Hz
	// 1 = 1.024 kHz
	// 2 = 4.096 kHz
	// 3 = 8.192 kHz (Default if frequency byte is out of range);
	void enable32kHz(bool TF);
	// Turns the 32kHz output pin on (true); or off (false).
	bool oscillatorCheck();;
	// Checks the status of the OSF (Oscillator Stop Flag);.
	// If this returns false, then the clock is probably not
	// giving you the correct time.
	// The OSF is cleared by function setSecond();.

private:

	byte decToBcd(byte val);
	// Convert normal decimal numbers to binary coded decimal
	byte bcdToDec(byte val);
	// Convert binary coded decimal to normal decimal numbers

protected:

	byte readControlByte(bool which);
	// Read selected control byte: (0); reads 0x0e, (1) reads 0x0f
	void writeControlByte(byte control, bool which);
	// Write the selected control byte. 
	// which == false -> 0x0e, true->0x0f.

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                            LCD                     ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void yield1();
void DELAY(unsigned long ms);
// commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En B00000100  // Enable bit
#define Rw B00000010  // Read/Write bit
#define Rs B00000001  // Register select bit

class CLCD : public Print {
public:
  CLCD(uint8_t lcd_Addr,uint8_t lcd_cols,uint8_t lcd_rows);
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS );
  void clear();
  void home();
  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void printLeft();
  void printRight();
  void leftToRight();
  void rightToLeft();
  void shiftIncrement();
  void shiftDecrement();
  void noBacklight();
  void backlight();
  void autoscroll();
  void noAutoscroll(); 
  void createChar(uint8_t, uint8_t[]);
  void setCursor(uint8_t, uint8_t);
#if defined(ARDUINO) && ARDUINO >= 100
  virtual size_t write(uint8_t);
#else
  virtual void write(uint8_t);
#endif
  void command(uint8_t);
  void init();

////compatibility API function aliases
void blink_on();						// alias for blink()
void blink_off();       					// alias for noBlink()
void cursor_on();      	 					// alias for cursor()
void cursor_off();      					// alias for noCursor()
void setBacklight(uint8_t new_val);				// alias for backlight() and nobacklight()
void load_custom_character(uint8_t char_num, uint8_t *rows);	// alias for createChar()
void printstr(const char[]);

////Unsupported API functions (not implemented in this library)
uint8_t status();
void setContrast(uint8_t new_val);
uint8_t keypad();
void setDelay(int,int);
void on();
void off();
uint8_t init_bargraph(uint8_t graphtype);
void draw_horizontal_graph(uint8_t row, uint8_t column, uint8_t len,  uint8_t pixel_col_end);
void draw_vertical_graph(uint8_t row, uint8_t column, uint8_t len,  uint8_t pixel_col_end);


private:
  void init_priv();
  void send(uint8_t, uint8_t);
  void write4bits(uint8_t);
  void expanderWrite(uint8_t);
  void pulseEnable(uint8_t);
  uint8_t _Addr;
  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;
  uint8_t _numlines;
  uint8_t _cols;
  uint8_t _rows;
  uint8_t _backlightval;
};

#ifndef Print1_h
#define Print1_h

#include <inttypes.h>
#include <stdio.h> // for size_t

#include "WString.h"
#include "util/PRINTABLE.h"

#define DEC 10
#define HEX 16
#define OCT 8
#ifdef BIN // Prevent warnings if BIN is previously defined in "iotnx4.h" or similar
#undef BIN
#endif
#define BIN 2

class Print1
{
private:
	int write_error;
	size_t printNumber(unsigned long, uint8_t);
	size_t printFloat(double, uint8_t);
protected:
	void setWriteError(int err = 1) { write_error = err; }
public:
	Print1() : write_error(0) {}

	int getWriteError() { return write_error; }
	void clearWriteError() { setWriteError(0); }

	virtual size_t write(uint8_t) = 0;
	size_t write(const char* str) {
		if (str == NULL) return 0;
		return write((const uint8_t*)str, strlen(str));
	}
	virtual size_t write(const uint8_t* buffer, size_t size);
	size_t write(const char* buffer, size_t size) {
		return write((const uint8_t*)buffer, size);
	}

	// default to zero, meaning "a single write may block"
	// should be overriden by subclasses with buffering
	virtual int availableForWrite() { return 0; }

	size_t PRINT(const __FlashStringHelper*);
	size_t PRINT(const String&);
	size_t PRINT(const char[]);
	size_t PRINT(char);
	size_t PRINT(unsigned char, int = DEC);
	size_t PRINT(int, int = DEC);
	size_t PRINT(unsigned int, int = DEC);
	size_t PRINT(long, int = DEC);
	size_t PRINT(unsigned long, int = DEC);
	size_t PRINT(double, int = 2);
	size_t PRINT(const Printable1&);

	size_t PRINTLN(const __FlashStringHelper*);
	size_t PRINTLN(const String& s);
	size_t PRINTLN(const char[]);
	size_t PRINTLN(char);
	size_t PRINTLN(unsigned char, int = DEC);
	size_t PRINTLN(int, int = DEC);
	size_t PRINTLN(unsigned int, int = DEC);
	size_t PRINTLN(long, int = DEC);
	size_t PRINTLN(unsigned long, int = DEC);
	size_t PRINTLN(double, int = 2);
	size_t PRINTLN(const Printable1&);
	size_t PRINTLN(void);

	virtual void flush() { /* Empty implementation for backward compatibility */ }
};

#endif

class LD_CLCD : public Print1 {
public:
	LD_CLCD(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows);
	void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCD_5x8DOTS);
	void CLEAR();
	void HOME();
	void NO_DISPLAY();
	void DIS_PLAY();
	void NO_BLINK();
	void BLINK();
	void NO_CURSOR();
	void CURSOR();
	void SCROLL_DISPLAY_LEFT();
	void SCROLL_DISPLAY_RIGHT();
	void PRINT_LEFT();
	void PRINT_RIGHT();
	void LEFT_TO_RIGHT();
	void RIGHT_TO_LEFT();
	void SHIFT_INCREMENT();
	void SHIFT_DECREMENT();
	void NO_BACKLIGHT();
	void BACKLIGHT();
	void AUTO_SCROLL();
	void NO_AUTO_SCROLL();
	void CREATE_CHAR(uint8_t, uint8_t[]);
	void SET_CURSOR(uint8_t, uint8_t);
#if defined(ARDUINO) && ARDUINO >= 100
	virtual size_t write(uint8_t);
#else
	virtual void write(uint8_t);
#endif
	void command(uint8_t);
	void INIT();

	////compatibility API function aliases
	void BLINK_ON();						// alias for blink()
	void BLINK_OFF();       					// alias for noBlink()
	void CUSROR_ON();      	 					// alias for cursor()
	void CURSOR_OFF();      					// alias for noCursor()
	void SET_BACKLIGHT(uint8_t new_val);				// alias for backlight() and nobacklight()
	void load_custom_character(uint8_t char_num, uint8_t* rows);	// alias for createChar()
	void printstr(const char[]);

	////Unsupported API functions (not implemented in this library)
	uint8_t status();
	void setContrast(uint8_t new_val);
	uint8_t keypad();
	void setDelay(int, int);
	void on();
	void off();
	uint8_t init_bargraph(uint8_t graphtype);
	void draw_horizontal_graph(uint8_t row, uint8_t column, uint8_t len, uint8_t pixel_col_end);
	void draw_vertical_graph(uint8_t row, uint8_t column, uint8_t len, uint8_t pixel_col_end);


private:
	void init_priv();
	void send(uint8_t, uint8_t);
	void write4bits(uint8_t);
	void expanderWrite(uint8_t);
	void pulseEnable(uint8_t);
	uint8_t _Addr;
	uint8_t _displayfunction;
	uint8_t _displaycontrol;
	uint8_t _displaymode;
	uint8_t _numlines;
	uint8_t _cols;
	uint8_t _rows;
	uint8_t _backlightval;
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////         IDAC               IADC                     ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class IDAC {
public:
	IDAC(uint8_t dacNum);
	void INIT(uint8_t channel1, uint32_t maxvalue65 , uint32_t minvalue00 , bool Mode);
	void DACOUT(uint8_t channel2, uint32_t inputValue);
	void swap_maxmin(uint32_t *maxvalues, uint32_t *minvalues);
private:

	//uint32_t _maxvalue;
	//uint32_t _minvalue;
	uint32_t _maxvalue11;
	uint32_t _minvalue11;
	uint32_t _maxvalue12;
	uint32_t _minvalue12;
	uint32_t _maxvalue13;
	uint32_t _minvalue13;

	uint32_t _maxvalue44;
	uint32_t _minvalue44;
	uint32_t _maxvalue45;
	uint32_t _minvalue45;
	uint32_t _maxvalue46;
	uint32_t _minvalue46;

	uint32_t _maxvalue2;
	uint32_t _minvalue2;
	uint32_t _maxvalue3;
	uint32_t _minvalue3;
	uint32_t _maxvalue5;
	uint32_t _minvalue5;

	uint32_t _maxvalue6;
	uint32_t _minvalue6;
	uint32_t _maxvalue7;
	uint32_t _minvalue7;
	uint32_t _maxvalue8;
	uint32_t _minvalue8;
	bool _Mode11;
	bool _Mode12;
	bool _Mode13;

	bool _Mode44;
	bool _Mode45;
	bool _Mode46;

	bool _Mode5;
	bool _Mode2;
	bool _Mode3;

	bool _Mode6;
	bool _Mode7;
	bool _Mode8;
	uint8_t _dacNum;
};

///Union configuration register
union Config1 {
	struct {
		uint8_t reserved : 1;    	///< "Reserved" bit
		uint8_t noOperation : 2; 	///< "NOP" bits
		uint8_t pullUp : 1;	   	///< "PULL_UP_EN" bit	
		uint8_t sensorMode : 1;  	///< "TS_MODE" bit	
		uint8_t rate : 3;		   	///< "DR" bits
		uint8_t operatingMode : 1;///< "MODE" bit		
		uint8_t pga : 3;			///< "PGA" bits
		uint8_t mux : 3;			///< "MUX" bits
		uint8_t singleStart : 1;  ///< "SS" bit
	}bits;
	uint16_t word;				///< Representation in word (16-bits) format
	struct {
		uint8_t lsb;			///< Byte LSB
		uint8_t msb;			///< Byte MSB
	} byte;						///< Representation in bytes (8-bits) format
};


class IADC{
public:
	void begin();				///< This method initialize the SPI port and the config register
	void PTbegin();				///< This method initialize the SPI port and the config register
	int IPT100(uint8_t Fchannel);
	void INIT(int32_t maxvalue = 32768, int32_t minvalue = 0, uint8_t rate_value = 4);
	IADC(uint8_t io_pin_cs = 1);         ///< Constructor
	double GET_TEMPERATURE();			///< Getting the temperature in degrees celsius from the internal sensor of the ADS1118
	long GET_ADC(uint8_t inputs, int8_t minus1_4 = 0);					///< Getting a sample from the specified input
	int GET_ADCVALUE(uint8_t inputs, int8_t minus1_4 = 0);
	bool getADCValueNoWait(uint8_t pin_drdy, uint16_t& value);
	bool getMilliVoltsNoWait(uint8_t pin_drdy, double& volts); ///< Getting the millivolts from the settled inputs
	double getMilliVolts(uint8_t inputs);					///< Getting the millivolts from the specified inputs
	double getMilliVolts();				///< Getting the millivolts from the settled inputs
	void decodeConfigRegister(union Config1 configRegister);	///< Decoding a configRegister structure and then print it out to the Serial port
	void setSamplingRate(uint8_t samplingRate);				///< Setting the sampling rate specified in the config register
	void setFullScaleRange(uint8_t fsr);///< Setting the full scale range in the config register
	void setContinuousMode();			///< Setting to continuous adquisition mode
	void setSingleShotMode();			///< Setting to single shot adquisition and power down mode
	void disablePullup();				///< Disabling the internal pull-up resistor of the DOUT pin
	void enablePullup();				///< Enabling the internal pull-up resistor of the DOUT pin
	void setInputSelected(uint8_t input);///< Setting the inputs to be adquired in the config register.


	int PTdata[4];
	//uint16_t recieve_Data(uint8_t num);

	//Input multiplexer configuration selection for bits "MUX"
	//Differential inputs
	const uint8_t DIFF_0_1 = 0b000; 	///< Differential input: Vin=A0-A1
	const uint8_t DIFF_0_3 = 0b001; 	///< Differential input: Vin=A0-A3
	const uint8_t DIFF_1_3 = 0b010; 	///< Differential input: Vin=A1-A3
	const uint8_t DIFF_2_3 = 0b011; 	///< Differential input: Vin=A2-A3   
//Single ended inputs
	const uint8_t AIN_0 = 0b100;  ///< Single ended input: Vin=A0
	const uint8_t AIN_1 = 0b101;	///< Single ended input: Vin=A1
	const uint8_t AIN_2 = 0b110;	///< Single ended input: Vin=A2
	const uint8_t AIN_3 = 0b111;	///< Single ended input: Vin=A3
	union Config1 configRegister;        ///< Config register

	//Bit constants
	const uint32_t SCLK = 2000000;///< ADS1118 SCLK frequency: 4000000 Hz Maximum for ADS1118

	// Used by "SS" bit
	const uint8_t START_NOW = 1;      ///< Start of conversion in single-shot mode

	// Used by "TS_MODE" bit
	const uint8_t ADC_MODE = 0;      ///< External (inputs) voltage reading mode
	const uint8_t TEMP_MODE = 1;      ///< Internal temperature sensor reading mode

	// Used by "MODE" bit
	const uint8_t CONTINUOUS = 0;      ///< Continuous conversion mode
	const uint8_t SINGLE_SHOT = 1;      ///< Single-shot conversion and power down mode

	// Used by "PULL_UP_EN" bit
	const uint8_t DOUT_PULLUP = 1;      ///< Internal pull-up resistor enabled for DOUT ***DEFAULT
	const uint8_t DOUT_NO_PULLUP = 0;      ///< Internal pull-up resistor disabled

	// Used by "NOP" bits
	const uint8_t VALID_CFG = 0b01;   ///< Data will be written to Config register
	const uint8_t NO_VALID_CFG = 0b00;   ///< Data won't be written to Config register

	// Used by "Reserved" bit
	const uint8_t RESERVED = 1;      ///< Its value is always 1, reserved

		/*Full scale range (FSR) selection by "PGA" bits.
		 [Warning: this could increase the noise and the effective number of bits (ENOB). See tables above]*/
	const uint8_t FSR_6144 = 0b000;  ///< Range: ±6.144 v. LSB SIZE = 187.5μV
	const uint8_t FSR_4096 = 0b001;  ///< Range: ±4.096 v. LSB SIZE = 125μV
	const uint8_t FSR_2048 = 0b010;  ///< Range: ±2.048 v. LSB SIZE = 62.5μV ***DEFAULT
	const uint8_t FSR_1024 = 0b011;  ///< Range: ±1.024 v. LSB SIZE = 31.25μV
	const uint8_t FSR_0512 = 0b100;  ///< Range: ±0.512 v. LSB SIZE = 15.625μV
	const uint8_t FSR_0256 = 0b111;  ///< Range: ±0.256 v. LSB SIZE = 7.8125μV

	/*Sampling rate selection by "DR" bits.
	[Warning: this could increase the noise and the effective number of bits (ENOB). See tables above]
	[경고: 이것은 bits의 숫자값에 영향과 노이즈가 증가될 수 있습니다.*/
	const uint8_t RATE_8SPS = 0b000;  ///< 8 samples/s, Tconv=125ms
	const uint8_t RATE_16SPS = 0b001;  ///< 16 samples/s, Tconv=62.5ms
	const uint8_t RATE_32SPS = 0b010;  ///< 32 samples/s, Tconv=31.25ms
	const uint8_t RATE_64SPS = 0b011;  ///< 64 samples/s, Tconv=15.625ms
	const uint8_t RATE_128SPS = 0b100;  ///< 128 samples/s, Tconv=7.8125ms
	const uint8_t RATE_250SPS = 0b101;  ///< 250 samples/s, Tconv=4ms
	const uint8_t RATE_475SPS = 0b110;  ///< 475 samples/s, Tconv=2.105ms
	const uint8_t RATE_860SPS = 0b111;  ///< 860 samples/s, Tconv=1.163ms	

private:
	bool boardModel = 0;// if boardModel is HIGH, used MPINO Series. but boardModel is Low, used MPAINO Series.
	bool tempOn = 0;
	uint32_t _maxvalue_adc;
 	uint32_t _minvalue_adc;
	uint8_t _adcNum;
	uint8_t lastSensorMode = 3;			///< Last sensor mode selected (ADC_MODE or TEMP_MODE or none)
	uint8_t cs;                         ///< Chip select pin (choose one)		
	const float pgaFSR[8] = { 6.144, 4.096, 2.048, 1.024, 0.512, 0.256, 0.256, 0.256 };
	const uint8_t CONV_TIME[8] = { 125, 63, 32, 16, 8, 4, 3, 2 }; 	///< Array containing the conversions time in ms

	uint8_t config_msb[5] = { 0xA1,0xB1,0xC1,0xD1 };
	uint8_t config_lsb[5] = { 0xA2,0xB2,0xC2,0xD2 };
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                         ITIMER                     ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


namespace ITIMER {
	extern unsigned long msecs1;
	//extern unsigned long msecs2;
	extern unsigned long msecs3;
	extern unsigned long msecs4;
	extern unsigned long msecs5;
	extern void (*func1)();
	//extern void (*func2)();
	extern void (*func3)();
	extern void (*func4)();
	extern void (*func5)();
	extern volatile unsigned long count1;
	//extern volatile unsigned long count2;
	extern volatile unsigned long count3;
	extern volatile unsigned long count4;
	extern volatile unsigned long count5;
	extern volatile char overflowing1;
	//extern volatile char overflowing2;
	extern volatile char overflowing3;
	extern volatile char overflowing4;
	extern volatile char overflowing5;
	extern volatile unsigned int tcnt1;
	//extern volatile unsigned int tcnt2;
	extern volatile unsigned int tcnt3;
	extern volatile unsigned int tcnt4;
	extern volatile unsigned int tcnt5;
	
	void SET(uint8_t Ntimer, unsigned long ms, void (*f)());
	void START(uint8_t Numstart = 10);
	void STOP(uint8_t Numstop = 10);
	void _overflow1();
	//void _overflow2();
	void _overflow3();
	void _overflow4();
	void _overflow5();
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                    MODBUS_M                   ///////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define __MODBUSMASTER_DEBUG__ (0)
#define __MODBUSMASTER_DEBUG_PIN_A__ 4
#define __MODBUSMASTER_DEBUG_PIN_B__ 5
// Define config for Serial.begin(baud, config);
#ifndef SERIAL_5N1
#define SERIAL_5N1 0x00
#define SERIAL_6N1 0x02
#define SERIAL_7N1 0x04
#define SERIAL_8N1 0x06
#define SERIAL_5N2 0x08
#define SERIAL_6N2 0x0A
#define SERIAL_7N2 0x0C
#define SERIAL_8N2 0x0E
#define SERIAL_5E1 0x20
#define SERIAL_6E1 0x22
#define SERIAL_7E1 0x24
#define SERIAL_8E1 0x26
#define SERIAL_5E2 0x28
#define SERIAL_6E2 0x2A
#define SERIAL_7E2 0x2C
#define SERIAL_8E2 0x2E
#define SERIAL_5O1 0x30
#define SERIAL_6O1 0x32
#define SERIAL_7O1 0x34
#define SERIAL_8O1 0x36
#define SERIAL_5O2 0x38
#define SERIAL_6O2 0x3A
#define SERIAL_7O2 0x3C
#define SERIAL_8O2 0x3E

#endif // SERIAL_5N1



/* _____PROJECT INCLUDES_____________________________________________________ */
// functions to calculate Modbus Application Data Unit CRC
#include "util/crc16.h"

// functions to manipulate words
#include "util/word.h"


/* _____CLASS DEFINITIONS____________________________________________________ */
/**
Arduino class library for communicating with Modbus slaves over
RS232/485 (via RTU protocol).
*/
class ModbusMaster
{
public:
	ModbusMaster();

	//void begin(unsigned long baud) { begin(baud, SERIAL_8N1); }
	//void begin(unsigned long, uint8_t);
	void begin(uint8_t slave, HardwareSerial *SerialPort, unsigned long baud, byte serialbit = SERIAL_8N1);
	void idle(void (*)());
	void preTransmission(void (*)());
	void postTransmission(void (*)());

	// Modbus exception codes
	/**
	Modbus protocol illegal function exception.

	*/
	static const uint8_t ku8MBIllegalFunction = 0x01;

	/**
	Modbus protocol illegal data address exception.

	The data address received in the query is not an allowable address for
	the server (or slave). More specifically, the combination of reference
	number and transfer length is invalid. For a controller with 100
	registers, the ADU addresses the first register as 0, and the last one
	as 99. If a request is submitted with a starting register address of 96
	and a quantity of registers of 4, then this request will successfully
	operate (address-wise at least) on registers 96, 97, 98, 99. If a
	request is submitted with a starting register address of 96 and a
	quantity of registers of 5, then this request will fail with Exception
	Code 0x02 "Illegal Data Address" since it attempts to operate on
	registers 96, 97, 98, 99 and 100, and there is no register with address
	100.

	@ingroup constant
	*/
	static const uint8_t ku8MBIllegalDataAddress = 0x02;

	/**
	Modbus protocol illegal data value exception.
	@ingroup constant
	*/
	static const uint8_t ku8MBIllegalDataValue = 0x03;

	/**
	Modbus protocol slave device failure exception.

	An unrecoverable error occurred while the server (or slave) was
	attempting to perform the requested action.

	@ingroup constant
	*/
	static const uint8_t ku8MBSlaveDeviceFailure = 0x04;

	// Class-defined success/exception codes
	/**
	ModbusMaster success.

	Modbus transaction was successful; the following checks were valid:
	  - slave ID
	  - function code
	  - response code
	  - data
	  - CRC

	@ingroup constant
	*/
	static const uint8_t ku8MBSuccess = 0x00;

	/**
	ModbusMaster invalid response slave ID exception.

	The slave ID in the response does not match that of the request.

	@ingroup constant
	*/
	static const uint8_t ku8MBInvalidSlaveID = 0xE0;

	/**
	ModbusMaster invalid response function exception.

	The function code in the response does not match that of the request.

	@ingroup constant
	*/
	static const uint8_t ku8MBInvalidFunction = 0xE1;

	/**
	ModbusMaster response timed out exception.

	The entire response was not received within the timeout period,
	ModbusMaster::ku8MBResponseTimeout.

	@ingroup constant
	*/
	static const uint8_t ku8MBResponseTimedOut = 0xE2;

	/**
	ModbusMaster invalid response CRC exception.

	The CRC in the response does not match the one calculated.

	@ingroup constant
	*/
	static const uint8_t ku8MBInvalidCRC = 0xE3;

	uint16_t GET_RESPONSE_BUFFER(uint8_t);
	uint16_t getResponseBuffer(uint8_t);
	void     CLAER_RESPONSE_BUFFER();
	void     clearResponseBuffer();
	uint8_t  SET_TRANSMIT_BUFFER(uint8_t, uint16_t);
	uint8_t  setTransmitBuffer(uint8_t, uint16_t);
	void     CLEAR_TRANSMIT_BUFFER();
	void     clearTransmitBuffer();

	void beginTransmission(uint16_t);
	uint8_t requestFrom(uint16_t, uint16_t);
	void sendBit(bool);
	void send(uint8_t);
	void send(uint16_t);
	void send(uint32_t);
	uint8_t available(void);
	uint16_t RECEIVE(void);
	uint16_t receive(void);


	uint8_t  READ_COILS(uint16_t, uint16_t);
	uint8_t  readCoils(uint16_t, uint16_t);
	uint8_t  READ_DISCRETE_INPUTS(uint16_t, uint16_t);
	uint8_t  readDiscreteInputs(uint16_t, uint16_t);
	uint8_t  READ_HOLDING_REGISTERS(uint16_t, uint16_t);
	uint8_t  readHoldingRegisters(uint16_t, uint16_t);
	uint8_t  READ_INPUT_REGISTERS(uint16_t, uint8_t);
	uint8_t  readInputRegisters(uint16_t, uint8_t);
	uint8_t  WRITE_SINGLE_COIL(uint16_t, uint8_t);
	uint8_t  writeSingleCoil(uint16_t, uint8_t);
	uint8_t  WRITE_SINGLE_REGISTER(uint16_t, uint16_t);
	uint8_t  writeSingleRegister(uint16_t, uint16_t);
	uint8_t  WRITE_MULTIPLE_COILS(uint16_t, uint16_t);
	uint8_t  writeMultipleCoils(uint16_t, uint16_t);
	uint8_t  WRITE_MULTIPLE_COILS();
	uint8_t  writeMultipleCoils();
	uint8_t  WRITE_MULTIPLE_REGISTERS(uint16_t, uint16_t);
	uint8_t  writeMultipleRegisters(uint16_t, uint16_t);
	uint8_t  WRITE_MULTIPLE_REGISTERS();
	uint8_t  writeMultipleRegisters();
	uint8_t  MASK_WRITE_REGISTER(uint16_t, uint16_t, uint16_t);
	uint8_t  maskWriteRegister(uint16_t, uint16_t, uint16_t);
	uint8_t  READ_WRITE_MULTIPLE_REGISTERS(uint16_t, uint16_t, uint16_t, uint16_t);
	uint8_t  readWriteMultipleRegisters(uint16_t, uint16_t, uint16_t, uint16_t);
	uint8_t  READ_WRITE_MULTIPLE_REGISTERS(uint16_t, uint16_t);
	uint8_t  readWriteMultipleRegisters(uint16_t, uint16_t);

	// Modbus timeout [milliseconds]
	uint16_t ku16MBResponseTimeout = 100; ///< Modbus timeout [milliseconds]

private:
	Stream* _serial;                                             ///< reference to serial port object
	uint8_t  _u8MBSlave;                                         ///< Modbus slave (1..255) initialized in begin()
	static const uint8_t ku8MaxBufferSize = 64;   ///< size of response/transmit buffers    
	uint16_t _u16ReadAddress;                                    ///< slave register from which to read
	uint16_t _u16ReadQty;                                        ///< quantity of words to read
	uint16_t _u16ResponseBuffer[ku8MaxBufferSize];               ///< buffer to store Modbus slave response; read via GetResponseBuffer()
	uint16_t _u16WriteAddress;                                   ///< slave register to which to write
	uint16_t _u16WriteQty;                                       ///< quantity of words to write
	uint16_t _u16TransmitBuffer[ku8MaxBufferSize];               ///< buffer containing data to transmit to Modbus slave; set via SetTransmitBuffer()
	uint16_t* txBuffer; // from Wire.h -- need to clean this up Rx
	uint8_t _u8TransmitBufferIndex;
	uint16_t u16TransmitBufferLength;
	uint16_t* rxBuffer; // from Wire.h -- need to clean this up Rx
	uint8_t _u8ResponseBufferIndex;
	uint8_t _u8ResponseBufferLength;

	// Modbus function codes for bit access
	static const uint8_t ku8MBReadCoils = 0x01; ///< Modbus function 0x01 Read Coils
	static const uint8_t ku8MBReadDiscreteInputs = 0x02; ///< Modbus function 0x02 Read Discrete Inputs
	static const uint8_t ku8MBWriteSingleCoil = 0x05; ///< Modbus function 0x05 Write Single Coil
	static const uint8_t ku8MBWriteMultipleCoils = 0x0F; ///< Modbus function 0x0F Write Multiple Coils

	// Modbus function codes for 16 bit access
	static const uint8_t ku8MBReadHoldingRegisters = 0x03; ///< Modbus function 0x03 Read Holding Registers
	static const uint8_t ku8MBReadInputRegisters = 0x04; ///< Modbus function 0x04 Read Input Registers
	static const uint8_t ku8MBWriteSingleRegister = 0x06; ///< Modbus function 0x06 Write Single Register
	static const uint8_t ku8MBWriteMultipleRegisters = 0x10; ///< Modbus function 0x10 Write Multiple Registers
	static const uint8_t ku8MBMaskWriteRegister = 0x16; ///< Modbus function 0x16 Mask Write Register
	static const uint8_t ku8MBReadWriteMultipleRegisters = 0x17; ///< Modbus function 0x17 Read Write Multiple Registers

	// master function that conducts Modbus transactions
	uint8_t ModbusMasterTransaction(uint8_t u8MBFunction);

	// idle callback function; gets called during idle time between TX and RX
	void (*_idle)();
	// preTransmission callback function; gets called before writing a Modbus message
	void (*_preTransmission)();
	// postTransmission callback function; gets called after a Modbus message has been sent
	void (*_postTransmission)();
};

/**
@example examples/Basic/Basic.pde
@example examples/PhoenixContact_nanoLC/PhoenixContact_nanoLC.pde
@example examples/RS485_HalfDuplex/RS485_HalfDuplex.ino
*/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                    MODBUS_S                   ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ModbusRTUSlaveWordAddress
{
public:
	u16 addr;
	byte len;
	u16* values;
	ModbusRTUSlaveWordAddress(u16 Address, u16* value, int cnt);
};

class ModbusRTUSlaveBitAddress
{
public:
	u16 addr;
	byte len;
	u8* values;
	ModbusRTUSlaveBitAddress(u16 Address, u8* value, int cnt);
};

class ModbusRTUSlave
{
public:
	ModbusRTUSlave(byte Slave, HardwareSerial *serialport, unsigned long baud, byte serialbit = SERIAL_8N1);
	boolean addWordArea(u16 Address, u16* values, int cnt);
	boolean addBitArea(u16 Address, u8* values, int cnt);
	void process();

private:
	byte slave;
	HardwareSerial* ser;
	LinkedList<ModbusRTUSlaveWordAddress*>* words;
	LinkedList<ModbusRTUSlaveBitAddress*>* bits;
	ModbusRTUSlaveWordAddress* getWordAddress(u16 Addr);
	ModbusRTUSlaveBitAddress* getBitAddress(u16 Addr);
	ModbusRTUSlaveWordAddress* getWordAddress(u16 Addr, u16 Len);
	ModbusRTUSlaveBitAddress* getBitAddress(u16 Addr, u16 Len);
	byte lstResponse[300];
	int ResCnt = 0;
	unsigned long lastrecv;
	void getCRC(byte* pby, int arsize, int startindex, int nSize, byte* byFirstReturn, byte* bySecondReturn);
};

const byte auchCRCHi1[] = {
						0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
						0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
						0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
						0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
						0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
						0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
						0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
						0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
						0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
						0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
						0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
						0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
						0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
						0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
						0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
						0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
						0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
						0x40 };

const byte auchCRCLo1[] = {
						0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
						0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
						0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
						0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
						0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
						0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
						0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
						0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
						0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
						0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
						0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
						0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
						0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
						0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
						0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
						0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
						0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
						0x40 };

boolean getBit(u8* area, int index);
void setBit(u8* area, int index, bool value);

namespace MODTIMER {
	extern unsigned long msecs1_m;
	extern void (*func1_m)();
	extern volatile unsigned long count1_m;
	extern volatile char overflowing1_m;
	extern volatile unsigned int tcnt1_m;


	void SET(void (*f)());
	void START(uint8_t Numstart = 10);
	void STOP(uint8_t Numstop = 10);
	void _overflow1_m();
};

#endif 