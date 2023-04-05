extern "C" {
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <stdio.h>
#include <math.h>
}

#include <SPI.h>
#include <Arduino.h>
#include "ILIB.h"
#include "util/LinkedList.h"
#include "HardwareSerial.h"


#ifndef F_CPU 
#define F_CPU 16000000L
#endif // !F_CPU



#if defined(ARDUINO) && ARDUINO >= 100

#define printIIC(args)	Wire.write(args)
inline size_t CLCD::write(uint8_t value) {
	send(value, Rs);
	return 1;
}inline size_t LD_CLCD::write(uint8_t value) {
	send(value, Rs);
	return 1;
}

#else
#include "WProgram.h"

#define printIIC(args)	Wire.send(args)
inline void CLCD::write(uint8_t value) {
	send(value, Rs);
}
inline void LD_CLCD::write(uint8_t value) {
	send(value, Rs);
}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////		ISEG		ISEG		ISEG	///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  Author: avishorp@gmail.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#define ISEG11 1
#ifdef ISEG11

#define TM1637_I2C_COMM1    0x40
#define TM1637_I2C_COMM2    0xC0
#define TM1637_I2C_COMM3    0x80

#define DEFAULT_BIT_DELAY  100
#define ISEG_UPPER 0 
//
//      A
//     ---
//  F |   | B
//     -G-
//  E |   | C
//     ---
//      D
const uint8_t digitToSegment[] = {
	// XGFEDCBA
	 0b00111111,    // 0
	 0b00000110,    // 1
	 0b01011011,    // 2
	 0b01001111,    // 3
	 0b01100110,    // 4
	 0b01101101,    // 5
	 0b01111101,    // 6
	 0b00000111,    // 7
	 0b01111111,    // 8
	 0b01101111,    // 9
	 0b01110111,    // A
	 0b01111100,    // b
	 0b00111001,    // C
	 0b01011110,    // d
	 0b01111001,    // E
	 0b01110001     // F
};

static const uint8_t minusSegments = 0b01000000;

ISEG::ISEG(uint8_t pinClk, uint8_t pinDIO, unsigned int bitDelay)
{
	// Copy the pin numbers
	m_pinClk = pinClk;
	m_pinDIO = pinDIO;
	m_bitDelay = bitDelay;

	// Set the pin direction and default value.
	// Both pins are set as inputs, allowing the pull-up resistors to pull them up
	pinMode(m_pinClk, INPUT);
	pinMode(m_pinDIO, INPUT);
	digitalWrite(m_pinClk, LOW);
	digitalWrite(m_pinDIO, LOW);
}
void ISEG::SET_BRIGHT(uint8_t brightness, bool on)
{
	m_brightness = (brightness & 0x7) | (on ? 0x08 : 0x00);
}
void ISEG::SET_SEG(const uint8_t segments[], uint8_t length, uint8_t pos)
{
	// Write COMM1
	start();
	writeByte(TM1637_I2C_COMM1);
	stop();

	// Write COMM2 + first digit address
	start();
	writeByte(TM1637_I2C_COMM2 + (pos & 0x03));

	// Write the data bytes
	for (uint8_t k = 0; k < length; k++)
		writeByte(segments[k]);

	stop();

	// Write COMM3 + brightness
	start();
	writeByte(TM1637_I2C_COMM3 + (m_brightness & 0x0f));
	stop();
}
void ISEG::CLEAR()
{
	uint8_t data[] = { 0, 0, 0, 0 };
	SET_SEG(data);
}
void ISEG::NUM_DEC(int num, bool leading_zero, uint8_t length, uint8_t pos)
{
	NUM_DEC_EX(num, 0, leading_zero, length, pos);
}
void ISEG::NUM_DEC_EX(int num, uint8_t dots, bool leading_zero,
	uint8_t length, uint8_t pos)
{
	showNumberBaseEx(num < 0 ? -10 : 10, num < 0 ? -num : num, dots, leading_zero, length, pos);
}
void ISEG::NUM_HEX_EX(uint16_t num, uint8_t dots, bool leading_zero,
	uint8_t length, uint8_t pos)
{
	showNumberBaseEx(16, num, dots, leading_zero, length, pos);
}

void ISEG::showNumberBaseEx(int8_t base, uint16_t num, uint8_t dots, bool leading_zero,
	uint8_t length, uint8_t pos)
{
	bool negative = false;
	if (base < 0) {
		base = -base;
		negative = true;
	}


	uint8_t digits[4];

	if (num == 0 && !leading_zero) {
		// Singular case - take care separately
		for (uint8_t i = 0; i < (length - 1); i++)
			digits[i] = 0;
		digits[length - 1] = encodeDigit(0);
	}
	else {
		//uint8_t i = length-1;
		//if (negative) {
		//	// Negative number, show the minus sign
		//    digits[i] = minusSegments;
		//	i--;
		//}

		for (int i = length - 1; i >= 0; --i)
		{
			uint8_t digit = num % base;

			if (digit == 0 && num == 0 && leading_zero == false)
				// Leading zero is blank
				digits[i] = 0;
			else
				digits[i] = encodeDigit(digit);

			if (digit == 0 && num == 0 && negative) {
				digits[i] = minusSegments;
				negative = false;
			}

			num /= base;
		}

		if (dots != 0)
		{
			showDots(dots, digits);
		}
	}
	SET_SEG(digits, length, pos);
}

void ISEG::bitDelay()
{
	delayMicroseconds(m_bitDelay);
}

void ISEG::start()
{
	pinMode(m_pinDIO, OUTPUT);
	bitDelay();
}

void ISEG::stop()
{
	pinMode(m_pinDIO, OUTPUT);
	bitDelay();
	pinMode(m_pinClk, INPUT);
	bitDelay();
	pinMode(m_pinDIO, INPUT);
	bitDelay();
}

bool ISEG::writeByte(uint8_t b)
{
	uint8_t data = b;

	// 8 Data Bits
	for (uint8_t i = 0; i < 8; i++) {
		// CLK low
		pinMode(m_pinClk, OUTPUT);
		bitDelay();

		// Set data bit
		if (data & 0x01)
			pinMode(m_pinDIO, INPUT);
		else
			pinMode(m_pinDIO, OUTPUT);

		bitDelay();

		// CLK high
		pinMode(m_pinClk, INPUT);
		bitDelay();
		data = data >> 1;
	}

	// Wait for acknowledge
	// CLK to zero
	pinMode(m_pinClk, OUTPUT);
	pinMode(m_pinDIO, INPUT);
	bitDelay();

	// CLK to high
	pinMode(m_pinClk, INPUT);
	bitDelay();
	uint8_t ack = digitalRead(m_pinDIO);
	if (ack == 0)
		pinMode(m_pinDIO, OUTPUT);


	bitDelay();
	pinMode(m_pinClk, OUTPUT);
	bitDelay();

	return ack;
}

void ISEG::showDots(uint8_t dots, uint8_t* digits)
{
	for (int i = 0; i < 4; ++i)
	{
		digits[i] |= (dots & 0x80);
		dots <<= 1;
	}
}

uint8_t ISEG::encodeDigit(uint8_t digit)
{
	return digitToSegment[digit & 0x0f];
}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////   WATCHDOG		WATCHDOG		WATCHDOG  /////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define WATCHDOG1 1
#ifdef WATCHDOG1
byte WDTCSR_recode1;
byte WDTCR_recode1;
	//mega2560 WDTCSR RESISTOR
//	WDTCSR |WDIF|WDIE|WDP3|WDCE|WDE|WDP2|WDP1|WDP0|
	//Watchdog timer prescale
	//|WDP3|WDP2|WDP1|WDP0|division ratio|Timeout Time when Vcc = 5v|
	//|	 0 |  0 |  0 |  0 |		2,048	 |	16ms	|
	//|	 0 |  0 |  0 |  1 |		4,096	 |	32ms	|
	//|	 0 |  0 |  1 |  0 |		8,192	 |	64ms	|
	//|	 0 |  0 |  1 |  1 |	   16,384	 |  0.125s	|
	//|	 0 |  1 |  0 |  0 |	   32,768	 |	0.25s	|
	//|	 0 |  1 |  0 |  1 |	   65,536	 |	0.5s	|
	//|	 0 |  1 |  1 |  0 |	  131,072	 |	1s		|
	//|	 0 |  1 |  1 |  1 |	  262,144	 |	2s		|
	//|	 1 |  0 |  0 |  0 |	  524,288	 |	4s		|
	//|	 1 |  0 |  0 |  1 |	1,048,576	 |	8s		|

//mega128 WDTCR RESISTOR
//	|  -  |  -  |  -  |WDCE|WDE|WDP2|WDP1|WDP0|
	//Watchdog timer prescale
	//|WDP2|WDP1|WDP0|division ratio|Timeout Time when Vcc = 3v|Timeout Time when Vcc = 5v|
	//|  0 |  0 |  0 |	   16,384	 |	14.8ms	|	14.0ms	|
	//|  0 |  0 |  1 |	   32,768	 |	29.6ms	|	28.1ms	|
	//|  0 |  1 |  0 |	   65,536	 |	59.1ms	|	56.2ms	|
	//|  0 |  1 |  1 |	  131,072	 |  0.12s	|	0.11s	|
	//|  1 |  0 |  0 |	  262,144	 |	0.24s	|	0.22s	|
	//|  1 |  0 |  1 |	  524,288	 |	0.47s	|	0.45s	|
	//|  1 |  1 |  0 |	1,048,576	 |	0.95s	|	0.9s	|
	//|  1 |  1 |  1 |	2,097,152	 |	1.9s	|	1.8s	|

void WDT_ENABLE(uint8_t ms) {
	uint8_t MS = ms;
	//	WDTCR |WDIF|WDIE|WDP3|WDCE|WDE|WDP2|WDP1|WDP0|
#ifdef __AVR_ATmega2560__   
	if (MS <= 7) {
		WDTCSR_recode1 = 8 + MS;
	}
	else if (MS <= 9 && MS > 7) {
		WDTCSR_recode1 = MS;
		WDTCSR_recode1 |= (1 << 5) | (1 << 3);
	}
	WDTCSR |= (1 << WDCE) | (1 << WDE); // enable the WDT reset
	WDTCSR = WDTCSR_recode1;

#endif // __AVR_ATmega2560__
//mega128 WDTCR RESISTOR
//	|  -  |  -  |  -  |WDCE|WDE|WDP2|WDP1|WDP0|
#ifdef __AVR_ATmega128__
	WDTCR |= 0x18;
	WDTCR |= 0x10 + ms;
#endif // __AVR_ATmega128__

}
void WDT_DISENABLE() {

#ifdef __AVR_ATmega2560__
	WDTCSR |= (1 << WDCE) | (1 << WDE); // enable the WDT reset
	WDTCSR = 0;

#endif // __AVR_ATmega2560__
#ifdef __AVR_ATmega128__

#endif // __AVR_ATmega128__

}
void WDT_RESET() {

#ifdef __AVR_ATmega2560__
	WDTCSR |= (1 << WDCE) | (1 << WDE); // enable the WDT reset
	WDTCSR = 0;
	WDTCSR |= (1 << WDCE) | (1 << WDE); // enable the WDT reset
	WDTCSR = WDTCSR_recode1;

#endif // __AVR_ATmega2560__
#ifdef __AVR_ATmega128__

#endif // __AVR_ATmega128__

}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////   PID			PID			PID		  /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PID1 1
#ifdef PID1
//#define DEBUG_SERIAL_OUT
IPID::IPID() {
	
}
void IPID::OUTPUT_SCALE(long max, long min) {
	if (max > min) {
		_max = max;
		_min = min;
	}
	else if(max < min) {
		_max = min;
		_min = max;
	}
}
void IPID::ITERM_LIMIT(int max, int min) {
	iMax = max;
	iMin = min;
}
void IPID::SET_DATA(double sv, double inMax,double inMin,uint8_t samplingTime) {
	_sv = sv;
	_inMax = inMax;
	_inMin = inMin;
	_samplingTime = samplingTime;

}
/*void IPID::findKuAndTu(double pv, double *outputMemory) {

	_pv = pv;
	_kp = repeat_k;
	_ki = 0;
	_kd = 0;
	unsigned int timeChange1 = millis() - lastTime1;
	//setValue(_sv, _inMax, _inMin, 25);
	outputPID(_pv, _kp, _ki, _kd);
	if (timeChange1 >= 3000)
	{
		repeat_k++;
		lastTime1 = millis();
	}
}*/
void IPID::Ziegler_Nichols_Method(double Ku,double Tu, double *P,double *I,double *D) {
	if ((I == NULL) && (D == NULL))//P
	{
		*P = 0.5 * Ku;
	}
	else if ((I != NULL) && (D == NULL))	//PI
	{
		*P = 0.45 * Ku;
		*I = 0.54 * Ku / Tu;
	}
	else if ((I == NULL) && (D != NULL))  //PD
	{

	}
	else {
		*P = 0.6 * Ku;
		*I = 2.0 * *P / Tu;
		*D = *P * Tu / 8.0;
	}

}
long IPID::PID_OUTPUT(double pv, double kp, double ki, double kd) {
	_pv = pv;
	_kp = kp;
	_ki = ki;
	_kd = kd;
	timeChange = millis() - lastTime;

	if (timeChange >= _samplingTime)
	{
		if (_sv > _inMax) _sv = _inMax; // Setting Value가 설정 최대값을 초과하면 Setting Value를 설정 최대값으로 조정
		if (_sv < _inMin) _sv = _inMin; // Setting Value가 설정 최소값을 초과하면 Setting Value를 설정 최소값으로 조정
		double error = _sv - _pv; // setting Value 와 Present Value의 오차값을 계산
		double Pterm = _kp * error; // 오차값에 kp 비례매개변수를 곱하여 비례항 P값을 계산
		errsum += error * timeChange * 0.001; // 오차값에 samplingTime(ms)을 곱하여 오차값을 합하여 errsum에 저장
		double Iterm = _ki * errsum; // 적분매개변수 ki와 오차값 합을 곱하여 적분항 I값을 계산
		if (iMin == 0) {
			iMax = _max - _min ; // 적분항 I의 누적 한계값을 출력 최대값 - 출력 최소값으로 계산
			iMin = -1.0 * iMax; // 
		}
		if (Iterm > iMax) Iterm = iMax; // 오차값 합을 입력 최대값의 10%를 초과하지 않도록 제한
		if (Iterm < iMin) Iterm = iMin; // 오차값 합을 입력 최소값의 -10%를 초과하지 않도록 제한
		double Dterm = _kd * ((error - preError) / (timeChange * 0.001)); // 현재 오차값과 이전 오차값을 빼고 samplingTime(ms)를 나누고 미분매개변수 kd로 곱하여 미분항 D값을 계산
		double mv = Pterm + Iterm + Dterm; // 출력값 mv는 비례항 P값과 적분항 I값과 미분항 D값을 합하여 계산


		if (mv > _inMax) mv = _inMax; // 출력값 mv를 입력 최대값을 초과하지 않도록 제한
		if (mv < _inMin) mv = _inMin; // 출력값 mv를 입력 최소값 미만이 되지 않도록 제한
					 // scale =				   ((In - InMin) / (InMax - InMin)) * (OutMax - OutMin) + OutMin
		long outValue = (unsigned int)(((mv - _inMin) / (_inMax - _inMin)) * (_max + _min) - _min);
//#define DEBUG_SERIAL_OUT
#ifdef DEBUG_SERIAL_OUT
		mScan++;
		if (mScan >= 5)
		{
			mScan = 0;
			Serial.print("SV: "); Serial.print(_sv);
			Serial.print(", PV: "); Serial.print(_pv);
			Serial.print(" ,MV: "); Serial.print(mv);
			Serial.print(" ,OUT: "); Serial.println(outValue);

			Serial.print("P:"); Serial.print(Pterm);
			Serial.print(" ,I:"); Serial.print(Iterm);
			Serial.print(" ,D:"); Serial.print(Dterm);
			Serial.print(" ,ERROR:"); Serial.println(error);

			Serial.print("iMax: "); Serial.print(iMax);
			Serial.print(" ,iMin: "); Serial.println(iMin);
		}
#endif
		preError = error; // 현재 스캔의 오차값을 이전 오차값 변수에 저장
		lastTime = millis();
		return outValue;
	}
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////   NTEMP			NTEMP		NTEMP     /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define NTEMP1111 1
#ifdef NTEMP1111

const static int16_t ADC_Table[] PROGMEM = { 15813,  15785,  15754,  15720,  15683,  15644,  15602,  15556,  15509,  15458,  15404,  15347,  15287,  15224,  15158,  15089,  15018,  14943,  14865,  14785,  14702,  14618,  14531,  14441,  14349,  14254,  14156,  14054,  13950,  13843,  13733,  13620,  13504,  13384,  13262,  13136,  13008,  12875,  12740,  12602,  12461,  12313,  12163,  12009,  11853,  11694,  11532,  11369,  11202,  11035,  10863,  10691,  10518,  10342,  10166,  9989, 9812, 9630, 9450, 9273, 9092, 8909, 8731, 8548, 8372, 8192, 8014, 7837, 7661, 7487, 7314, 7143, 6973, 6806, 6640, 6477, 6316, 6157, 6000, 5846, 5694, 5546, 5399, 5256, 5115, 4977, 4842, 4709, 4580, 4453, 4330, 4209, 4091, 3976, 3864, 3754, 3647, 3543, 3441, 3343, 3247, 3154, 3062, 2974, 2889, 2805, 2724, 2646, 2569, 2494, 2422, 2353, 2285, 2219, 2156, 2093, 2033, 1975, 1918, 1864, 1811, 1759, 1710, 1661, 1614, 1569, 1526, 1483, 1442, 1403, 1364, 1327, 1290, 1255, 1221, 1188, 1155, 1124, 1094, 1064, 1035, 1008, 982,  956,  931,  907,  883,  859,  836,  814,  792,  771,  751,  731,  711,  692,  674,  656,  638,  622,  605 };
const static int16_t ADC_Table1023[] PROGMEM = { 987, 986, 984, 982, 979, 977, 974, 971, 968, 965, 962, 958, 955, 951, 946, 942, 938, 933, 928, 923, 918, 913, 907, 902, 896, 890, 884, 878, 871, 864, 858, 850, 843, 836, 828, 820, 812, 804, 796, 787, 778, 769, 759, 750, 740, 730, 720, 710, 699, 689, 678, 668, 657, 646, 635, 624, 613, 601, 590, 579, 568, 556, 545, 534, 523, 512, 500, 489, 478, 468, 457, 446, 435, 425, 415, 404, 394, 384, 375, 365, 356, 346, 337, 328, 319, 311, 302, 294, 286, 278, 270, 263, 255, 248, 241, 234, 228, 221, 215, 209, 203, 197, 191, 186, 180, 175, 170, 165, 160, 156, 151, 147, 143, 139, 135, 131, 127, 123, 120, 116, 113, 110, 107, 104, 101, 98, 95, 93, 90, 88, 85, 83, 81, 78, 76, 74, 72, 70, 68, 66, 65, 63, 61, 60, 58, 57, 55, 54, 52, 51, 49, 48, 47, 46, 44, 43, 42, 41, 40, 39, 38 };
const static int16_t TH_Table[] PROGMEM = { -40,  -39,  -38,  -37,  -36,  -35,  -34,  -33,  -32,  -31,  -30,  -29,  -28,  -27,  -26,  -25,  -24,  -23,  -22,  -21,  -20,  -19,  -18,  -17,  -16,  -15,  -14,  -13,  -12,  -11,  -10,  -9, -8, -7, -6, -5, -4, -3, -2, -1, 0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100,  101,  102,  103,  104,  105,  106,  107,  108,  109,  110,  111,  112,  113,  114,  115,  116,  117,  118,  119,  120 };


//데이터표방식
int INTC(unsigned int RawADC)
{
	int analogInput = 0;
	bool tway = 0;
	if (RawADC < 60) {
		analogInput = analogRead(RawADC);
		tway = 1;

	}
	else { tway = 0; }
	uint8_t Cnt = 0;
	uint8_t Cnt1 = 0;

	uint8_t k = 0;
	uint8_t k1 = 0;

	if (tway == 0) {			//spi입력
		for (k = 0; k < 161; k++) {
			if (RawADC > pgm_read_word_near(&(ADC_Table[k]))) {
				Cnt = k;
				break;
			}
		}
		float rPoint = ((float)(RawADC - pgm_read_word_near(&(ADC_Table[Cnt]))) / (float)(pgm_read_word_near(&(ADC_Table[Cnt - 1])) - pgm_read_word_near(&(ADC_Table[Cnt]))));
		float rTemp = pgm_read_word_near(&(TH_Table[Cnt])) - rPoint;
		return (unsigned int)(rTemp * 10.0);
	}
	else if (tway == 1) {		//analogRead입력
		for (int k1 = 0; k1 < 161; k1++) {
			if (analogInput > pgm_read_word_near(&(ADC_Table1023[k1]))) {
				Cnt1 = k1; break;
			}
		}
		float rPoint1 = ((float)(analogInput - pgm_read_word_near(&(ADC_Table1023[Cnt1]))) / (float)(pgm_read_word_near(&(ADC_Table1023[Cnt1 - 1])) - pgm_read_word_near(&(ADC_Table1023[Cnt1]))));
		float rTemp1 = pgm_read_word_near(&(TH_Table[Cnt1])) - rPoint1;
		return (unsigned int)(rTemp1 * 10.0);
	}
}


//로그방식
float INTC1(int32_t _ch, float _bValue)
{
	uint8_t i;
	float average;
	uint16_t samples[5];
	//float _bValue;
	int rValue;
	uint8_t SpiCheck = 0;

	for (i = 0; i < 5; i++) {
		// take N samples in a row, with a slight delay
		if (_ch > 60) {
			samples[i] = _ch;
			SpiCheck = 1;
		}
		else {
			samples[i] = analogRead(_ch);
			SpiCheck = 0;
		}
	}
	if (SpiCheck == 0) {
		// average all the samples out
		average = 0;
		for (i = 0; i < 5; i++) {
			average += samples[i];
		}
		average /= 5;

		// convert the value to resistance
		average = 1023.0 / average - 1.0;
		average = 10000.0 / average;

		float steinhart;
		steinhart = average / 10000;     // (R/Ro)
		steinhart = log(steinhart);                  // ln(R/Ro)
		steinhart /= _bValue;                   // 1/B * ln(R/Ro)
		steinhart += 1.0 / (25 + 273.15); // + (1/To)
		steinhart = 1.0 / steinhart;                 // Invert
		steinhart -= 273.15;                         // convert to C                                           

		return steinhart;
	}
	else if (SpiCheck == 1) {
		// average all the samples out
		average = 0;
		for (i = 0; i < 5; i++) {
			average += samples[i];
		}
		average /= 5;

		// convert the value to resistance
		average = 15999.5 / average - 1.0;
		average = 10000.0 / average;

		float steinhart;
		steinhart = average / 10000;     // (R/Ro)
		steinhart = log(steinhart);                  // ln(R/Ro)
		steinhart /= _bValue;                   // 1/B * ln(R/Ro)
		steinhart += 1.0 / (25 + 273.15); // + (1/To)
		steinhart = 1.0 / steinhart;                 // Invert
		steinhart -= 273.15;                         // convert to C                                           

		return steinhart;
	}
}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////   DHT			DHT			DHT       /////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* DHT library

MIT license
written by Adafruit Industries
*/

#define DHT1 1
#ifdef DHT1

DHT::DHT(uint8_t pin, uint8_t type, uint8_t count) {
	_pin = pin;
	_type = type;
	_count = count;
	firstreading = true;
}

void DHT::BEGIN(void) {
	// set up the pins!
	pinMode(_pin, INPUT);
	digitalWrite(_pin, HIGH);
	_lastreadtime = 0;
}

//boolean S == Scale.  True == Farenheit; False == Celcius
float DHT::READ_TEMP(bool S) {
	float f;

	if (read()) {
		switch (_type) {
		case DHT11:
			f = data[2];
			if (S)
				f = convertCtoF(f);

			return f;
		case DHT22:
		case DHT21:
			f = data[2] & 0x7F;
			f *= 256;
			f += data[3];
			f /= 10;
			if (data[2] & 0x80)
				f *= -1;
			if (S)
				f = convertCtoF(f);

			return f;
		}
	}
	//Serial.print("Read fail");
	return NAN;
}

float DHT::convertCtoF(float c) {
	return c * 9 / 5 + 32;
}

float DHT::READ_HUMI(void) {
	float f;
	if (read()) {
		switch (_type) {
		case DHT11:
			f = data[0];
			return f;
		case DHT22:
		case DHT21:
			f = data[0];
			f *= 256;
			f += data[1];
			f /= 10;
			return f;
		}
	}
	//Serial.print("Read fail");
	return NAN;
}

bool DHT::read(void) {
	uint8_t laststate = HIGH;
	uint8_t counter = 0;
	uint8_t j = 0, i;
	unsigned long currenttime;

	// pull the pin high and wait 250 milliseconds
	digitalWrite(_pin, HIGH);
	delay(250);

	currenttime = millis();
	if (currenttime < _lastreadtime) {
		// ie there was a rollover
		_lastreadtime = 0;
	}
	if (!firstreading && ((currenttime - _lastreadtime) < 2000)) {
		return true; // return last correct measurement
		//delay(2000 - (currenttime - _lastreadtime));
	}
	firstreading = false;
	/*
	  Serial.print("Currtime: "); Serial.print(currenttime);
	  Serial.print(" Lasttime: "); Serial.print(_lastreadtime);
	*/
	_lastreadtime = millis();

	data[0] = data[1] = data[2] = data[3] = data[4] = 0;

	// now pull it low for ~20 milliseconds
	pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
	delay(20);
	cli();
	digitalWrite(_pin, HIGH);
	delayMicroseconds(40);
	pinMode(_pin, INPUT);

	// read in timings
	for (i = 0; i < MAXTIMINGS; i++) {
		counter = 0;
		while (digitalRead(_pin) == laststate) {
			counter++;
			delayMicroseconds(1);
			if (counter == 255) {
				break;
			}
		}
		laststate = digitalRead(_pin);

		if (counter == 255) break;

		// ignore first 3 transitions
		if ((i >= 4) && (i % 2 == 0)) {
			// shove each bit into the storage bytes
			data[j / 8] <<= 1;
			if (counter > _count)
				data[j / 8] |= 1;
			j++;
		}

	}

	sei();

	/*
	Serial.println(j, DEC);
	Serial.print(data[0], HEX); Serial.print(", ");
	Serial.print(data[1], HEX); Serial.print(", ");
	Serial.print(data[2], HEX); Serial.print(", ");
	Serial.print(data[3], HEX); Serial.print(", ");
	Serial.print(data[4], HEX); Serial.print(" =? ");
	Serial.println(data[0] + data[1] + data[2] + data[3], HEX);
	*/

	// check we read 40 bits and that the checksum matches
	if ((j >= 40) &&
		(data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF))) {
		return true;
	}


	return false;

}

#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                             DS3231                //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define DS1 1
#ifdef DS1

#define CLOCK_ADDRESS 0x68
#define SECONDS_FROM_1970_TO_2000 946684800


// Constructor
DS3231::DS3231() {
	// nothing to do for this constructor.
}

// Utilities from JeeLabs/Ladyada

////////////////////////////////////////////////////////////////////////////////
// utility code, some of this could be exposed in the DATE_TIME API if needed

// DS3231 is smart enough to know this, but keepZ		ing it for now so I don't have
// to rewrite their code. -ADW
static const uint8_t daysInMonth[] PROGMEM = { 31,28,31,30,31,30,31,31,30,31,30,31 };

// number of days since 2000/01/01, valid for 2001..2099
static uint16_t date2days(uint16_t y, uint8_t m, uint8_t d) {
	if (y >= 2000)
		y -= 2000;
	uint16_t days = d;
	for (uint8_t i = 1; i < m; ++i)
		days += pgm_read_byte(daysInMonth + i - 1);
	if (m > 2 && isleapYear(y))
		++days;
	return days + 365 * y + (y + 3) / 4 - 1;
}

static long time2long(uint16_t days, uint8_t h, uint8_t m, uint8_t s) {
	return ((days * 24L + h) * 60 + m) * 60 + s;
}

/*****************************************
	Public Functions
 *****************************************/

 /*******************************************************************************
  * TO GET ALL DATE/TIME INFORMATION AT ONCE AND AVOID THE CHANCE OF ROLLOVER
  * DATE_TIME implementation spliced in here from Jean-Claude Wippler's (JeeLabs)
  * RTClib, as modified by Limor Fried (Ladyada); source code at:
  * https://github.com/adafruit/RTClib
  ******************************************************************************/

  ////////////////////////////////////////////////////////////////////////////////
  // DATE_TIME implementation - ignores time zones and DST changes
  // NOTE: also ignores leap seconds, see http://en.wikipedia.org/wiki/Leap_second

DATE_TIME::DATE_TIME(uint32_t t) {
	t -= SECONDS_FROM_1970_TO_2000;    // bring to 2000 timestamp from 1970

	ss = t % 60;
	t /= 60;
	mm = t % 60;
	t /= 60;
	hh = t % 24;
	uint16_t days = t / 24;
	uint8_t leap;
	for (yOff = 0; ; ++yOff) {
		leap = isleapYear(yOff);
		if (days < 365 + leap)
			break;
		days -= 365 + leap;
	}
	for (m = 1; ; ++m) {
		uint8_t daysPerMonth = pgm_read_byte(daysInMonth + m - 1);
		if (leap && m == 2)
			++daysPerMonth;
		if (days < daysPerMonth)
			break;
		days -= daysPerMonth;
	}
	d = days + 1;
}

DATE_TIME::DATE_TIME(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec) {
	if (year >= 2000)
		year -= 2000;
	yOff = year;
	m = month;
	d = day;
	hh = hour;
	mm = min;
	ss = sec;
}

// supported formats are date "Mmm dd yyyy" and time "hh:mm:ss" (same as __DATE__ and __TIME__)
DATE_TIME::DATE_TIME(const char* date, const char* time) {
	static const char month_names[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
	static const char buff[4] = { '0','0','0','0' };
	int y;
	sscanf(date, "%s %d %d", buff, &d, &y);
	yOff = y >= 2000 ? y - 2000 : y;
	m = (strstr(month_names, buff) - month_names) / 3 + 1;
	sscanf(time, "%d:%d:%d", &hh, &mm, &ss);
}

static uint8_t conv2d(const char* p) {
	uint8_t v = 0;
	if ('0' <= *p && *p <= '9')
		v = *p - '0';
	return 10 * v + *++p - '0';
}

// UNIX time: IS CORRECT ONLY WHEN SET TO UTC!!!
uint32_t DATE_TIME::unixtime(void) const {
	uint32_t t;
	uint16_t days = date2days(yOff, m, d);
	t = time2long(days, hh, mm, ss);
	t += SECONDS_FROM_1970_TO_2000;  // seconds from 1970 to 2000

	return t;
}

// Slightly modified from JeeLabs / Ladyada
// Get all date/time at once to avoid rollover (e.g., minute/second don't match)
static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }

bool isleapYear(const uint8_t y) {
	if (y & 3)//check if divisible by 4
		return false;
	//only check other, when first failed
	return (y % 100 || y % 400 == 0);
}

DATE_TIME RTClib::NOW() {
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0);	// This is the first register address (Seconds)
			  // We'll read from here on for 7 bytes: secs reg, minutes reg, hours, days, months and years.
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 7);
	uint8_t ss = bcd2bin(Wire.read() & 0x7F);
	uint8_t mm = bcd2bin(Wire.read());
	uint8_t hh = bcd2bin(Wire.read());
	Wire.read();
	uint8_t d = bcd2bin(Wire.read());
	uint8_t m = bcd2bin(Wire.read());
	uint16_t y = bcd2bin(Wire.read()) + 2000;

	return DATE_TIME(y, m, d, hh, mm, ss);
}

///// ERIC'S ORIGINAL CODE FOLLOWS /////

byte DS3231::GET_SECOND() {
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x00);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 1);
	return bcdToDec(Wire.read());
}

byte DS3231::GET_MINUTE() {
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x01);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 1);
	return bcdToDec(Wire.read());
}

byte DS3231::GET_HOUR(bool& h12, bool& PM_time) {
	byte temp_buffer;
	byte hour;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x02);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 1);
	temp_buffer = Wire.read();
	h12 = temp_buffer & 0b01000000;
	if (h12) {
		PM_time = temp_buffer & 0b00100000;
		hour = bcdToDec(temp_buffer & 0b00011111);
	}
	else {
		hour = bcdToDec(temp_buffer & 0b00111111);
	}
	return hour;
}

byte DS3231::GET_DOW() {
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x03);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 1);
	return bcdToDec(Wire.read());
}


byte DS3231::GET_DATE() {
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x04);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 1);
	return bcdToDec(Wire.read());
}

byte DS3231::GET_MONTH(bool& Century) {
	byte temp_buffer;
	byte hour;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x05);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 1);
	temp_buffer = Wire.read();
	Century = temp_buffer & 0b10000000;
	return (bcdToDec(temp_buffer & 0b01111111));
}

byte DS3231::GET_YEAR() {
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x06);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 1);
	return bcdToDec(Wire.read());
}

void DS3231::SET_SECOND(byte Second) {
	// Sets the seconds 
	// This function also resets the Oscillator Stop Flag, which is set
	// whenever power is interrupted.
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x00);
	Wire.write(decToBcd(Second));
	Wire.endTransmission();
	// Clear OSF flag
	byte temp_buffer = readControlByte(1);
	writeControlByte((temp_buffer & 0b01111111), 1);
}

void DS3231::SET_MINUTE(byte Minute) {
	// Sets the minutes 
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x01);
	Wire.write(decToBcd(Minute));
	Wire.endTransmission();
}

// Following setHour revision by David Merrifield 4/14/2020 correcting handling of 12-hour clock

void DS3231::SET_HOUR(byte Hour) {
	// Sets the hour, without changing 12/24h mode.
	// The hour must be in 24h format.

	bool h12;
	byte temp_hour;

	// Start by figuring out what the 12/24 mode is
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x02);
	Wire.endTransmission();
	Wire.requestFrom(CLOCK_ADDRESS, 1);
	h12 = (Wire.read() & 0b01000000);
	// if h12 is true, it's 12h mode; false is 24h.

	if (h12) {
		// 12 hour
		bool am_pm = (Hour > 11);
		temp_hour = Hour;
		if (temp_hour > 11) {
			temp_hour = temp_hour - 12;
		}
		if (temp_hour == 0) {
			temp_hour = 12;
		}
		temp_hour = decToBcd(temp_hour) | (am_pm << 5) | 0b01000000;
	}
	else {
		// 24 hour
		temp_hour = decToBcd(Hour) & 0b10111111;
	}

	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x02);
	Wire.write(temp_hour);
	Wire.endTransmission();
}

void DS3231::SET_DOW(byte DoW) {
	// Sets the Day of Week
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x03);
	Wire.write(decToBcd(DoW));
	Wire.endTransmission();
}

void DS3231::SET_DATE(byte Date) {
	// Sets the Date
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x04);
	Wire.write(decToBcd(Date));
	Wire.endTransmission();
}

void DS3231::SET_MONTH(byte Month) {
	// Sets the month
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x05);
	Wire.write(decToBcd(Month));
	Wire.endTransmission();
}

void DS3231::SET_YEAR(byte Year) {
	// Sets the year
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x06);
	Wire.write(decToBcd(Year));
	Wire.endTransmission();
}

void DS3231::SET_CLOCK_MODE(bool h12) {
	// sets the mode to 12-hour (true) or 24-hour (false).
	// One thing that bothers me about how I've written this is that
	// if the read and right happen at the right hourly millisecnd,
	// the clock will be set back an hour. Not sure how to do it better, 
	// though, and as long as one doesn't set the mode frequently it's
	// a very minimal risk. 
	// It's zero risk if you call this BEFORE setting the hour, since
	// the setHour() function doesn't change this mode.

	byte temp_buffer;

	// Start by reading byte 0x02.
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x02);
	Wire.endTransmission();
	Wire.requestFrom(CLOCK_ADDRESS, 1);
	temp_buffer = Wire.read();

	// Set the flag to the requested value:
	if (h12) {
		temp_buffer = temp_buffer | 0b01000000;
	}
	else {
		temp_buffer = temp_buffer & 0b10111111;
	}

	// Write the byte
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x02);
	Wire.write(temp_buffer);
	Wire.endTransmission();
}
void DS3231::SET_ADJUST(uint8_t dow1, uint16_t year1, uint8_t month1, uint8_t date1, uint8_t hour1, uint8_t minute1, uint8_t second1) {
	SET_CLOCK_MODE(0);
	SET_DOW(dow1);
	SET_YEAR(year1);
	SET_MONTH(month1);
	SET_DATE(date1);
	SET_HOUR(hour1);
	SET_MINUTE(minute1);
	SET_SECOND(second1);
}


float DS3231::GET_TEMPERATURE() {
	// Checks the internal thermometer on the DS3231 and returns the 
	// temperature as a floating-point value.

  // Updated / modified a tiny bit from "Coding Badly" and "Tri-Again"
  // http://forum.arduino.cc/index.php/topic,22301.0.html

	byte tMSB, tLSB;
	float temp3231;

	// temp registers (11h-12h) get updated automatically every 64s
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x11);
	Wire.endTransmission();
	Wire.requestFrom(CLOCK_ADDRESS, 2);

	// Should I do more "if available" checks here?
	if (Wire.available()) {
		tMSB = Wire.read(); //2's complement int portion
		tLSB = Wire.read(); //fraction portion

		int16_t  itemp = (tMSB << 8 | (tLSB & 0xC0));  // Shift upper byte, add lower
		temp3231 = ((float)itemp / 256.0);              // Scale and return
	}
	else {
		temp3231 = -9999; // Impossible temperature; error value
	}

	return temp3231;
}

void DS3231::getA1Time(byte& A1Day, byte& A1Hour, byte& A1Minute, byte& A1Second, byte& AlarmBits, bool& A1Dy, bool& A1h12, bool& A1PM) {
	byte temp_buffer;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x07);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 4);

	temp_buffer = Wire.read();	// Get A1M1 and A1 Seconds
	A1Second = bcdToDec(temp_buffer & 0b01111111);
	// put A1M1 bit in position 0 of DS3231_AlarmBits.
	AlarmBits = AlarmBits | (temp_buffer & 0b10000000) >> 7;

	temp_buffer = Wire.read();	// Get A1M2 and A1 minutes
	A1Minute = bcdToDec(temp_buffer & 0b01111111);
	// put A1M2 bit in position 1 of DS3231_AlarmBits.
	AlarmBits = AlarmBits | (temp_buffer & 0b10000000) >> 6;

	temp_buffer = Wire.read();	// Get A1M3 and A1 Hour
	// put A1M3 bit in position 2 of DS3231_AlarmBits.
	AlarmBits = AlarmBits | (temp_buffer & 0b10000000) >> 5;
	// determine A1 12/24 mode
	A1h12 = temp_buffer & 0b01000000;
	if (A1h12) {
		A1PM = temp_buffer & 0b00100000;			// determine am/pm
		A1Hour = bcdToDec(temp_buffer & 0b00011111);	// 12-hour
	}
	else {
		A1Hour = bcdToDec(temp_buffer & 0b00111111);	// 24-hour
	}

	temp_buffer = Wire.read();	// Get A1M4 and A1 Day/Date
	// put A1M3 bit in position 3 of DS3231_AlarmBits.
	AlarmBits = AlarmBits | (temp_buffer & 0b10000000) >> 4;
	// determine A1 day or date flag
	A1Dy = (temp_buffer & 0b01000000) >> 6;
	if (A1Dy) {
		// alarm is by day of week, not date.
		A1Day = bcdToDec(temp_buffer & 0b00001111);
	}
	else {
		// alarm is by date, not day of week.
		A1Day = bcdToDec(temp_buffer & 0b00111111);
	}
}

void DS3231::getA2Time(byte& A2Day, byte& A2Hour, byte& A2Minute, byte& AlarmBits, bool& A2Dy, bool& A2h12, bool& A2PM) {
	byte temp_buffer;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x0b);
	Wire.endTransmission();

	Wire.requestFrom(CLOCK_ADDRESS, 3);
	temp_buffer = Wire.read();	// Get A2M2 and A2 Minutes
	A2Minute = bcdToDec(temp_buffer & 0b01111111);
	// put A2M2 bit in position 4 of DS3231_AlarmBits.
	AlarmBits = AlarmBits | (temp_buffer & 0b10000000) >> 3;

	temp_buffer = Wire.read();	// Get A2M3 and A2 Hour
	// put A2M3 bit in position 5 of DS3231_AlarmBits.
	AlarmBits = AlarmBits | (temp_buffer & 0b10000000) >> 2;
	// determine A2 12/24 mode
	A2h12 = temp_buffer & 0b01000000;
	if (A2h12) {
		A2PM = temp_buffer & 0b00100000;			// determine am/pm
		A2Hour = bcdToDec(temp_buffer & 0b00011111);	// 12-hour
	}
	else {
		A2Hour = bcdToDec(temp_buffer & 0b00111111);	// 24-hour
	}

	temp_buffer = Wire.read();	// Get A2M4 and A1 Day/Date
	// put A2M4 bit in position 6 of DS3231_AlarmBits.
	AlarmBits = AlarmBits | (temp_buffer & 0b10000000) >> 1;
	// determine A2 day or date flag
	A2Dy = (temp_buffer & 0b01000000) >> 6;
	if (A2Dy) {
		// alarm is by day of week, not date.
		A2Day = bcdToDec(temp_buffer & 0b00001111);
	}
	else {
		// alarm is by date, not day of week.
		A2Day = bcdToDec(temp_buffer & 0b00111111);
	}
}

void DS3231::setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM) {
	//	Sets the alarm-1 date and time on the DS3231, using A1* information
	byte temp_buffer;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x07);	// A1 starts at 07h
	// Send A1 second and A1M1
	Wire.write(decToBcd(A1Second) | ((AlarmBits & 0b00000001) << 7));
	// Send A1 Minute and A1M2
	Wire.write(decToBcd(A1Minute) | ((AlarmBits & 0b00000010) << 6));
	// Figure out A1 hour 
	if (A1h12) {
		// Start by converting existing time to h12 if it was given in 24h.
		if (A1Hour > 12) {
			// well, then, this obviously isn't a h12 time, is it?
			A1Hour = A1Hour - 12;
			A1PM = true;
		}
		if (A1PM) {
			// Afternoon
			// Convert the hour to BCD and add appropriate flags.
			temp_buffer = decToBcd(A1Hour) | 0b01100000;
		}
		else {
			// Morning
			// Convert the hour to BCD and add appropriate flags.
			temp_buffer = decToBcd(A1Hour) | 0b01000000;
		}
	}
	else {
		// Now for 24h
		temp_buffer = decToBcd(A1Hour);
	}
	temp_buffer = temp_buffer | ((AlarmBits & 0b00000100) << 5);
	// A1 hour is figured out, send it
	Wire.write(temp_buffer);
	// Figure out A1 day/date and A1M4
	temp_buffer = ((AlarmBits & 0b00001000) << 4) | decToBcd(A1Day);
	if (A1Dy) {
		// Set A1 Day/Date flag (Otherwise it's zero)
		temp_buffer = temp_buffer | 0b01000000;
	}
	Wire.write(temp_buffer);
	// All done!
	Wire.endTransmission();
}

void DS3231::setA2Time(byte A2Day, byte A2Hour, byte A2Minute, byte AlarmBits, bool A2Dy, bool A2h12, bool A2PM) {
	//	Sets the alarm-2 date and time on the DS3231, using A2* information
	byte temp_buffer;
	Wire.beginTransmission(CLOCK_ADDRESS);
	Wire.write(0x0b);	// A1 starts at 0bh
	// Send A2 Minute and A2M2
	Wire.write(decToBcd(A2Minute) | ((AlarmBits & 0b00010000) << 3));
	// Figure out A2 hour 
	if (A2h12) {
		// Start by converting existing time to h12 if it was given in 24h.
		if (A2Hour > 12) {
			// well, then, this obviously isn't a h12 time, is it?
			A2Hour = A2Hour - 12;
			A2PM = true;
		}
		if (A2PM) {
			// Afternoon
			// Convert the hour to BCD and add appropriate flags.
			temp_buffer = decToBcd(A2Hour) | 0b01100000;
		}
		else {
			// Morning
			// Convert the hour to BCD and add appropriate flags.
			temp_buffer = decToBcd(A2Hour) | 0b01000000;
		}
	}
	else {
		// Now for 24h
		temp_buffer = decToBcd(A2Hour);
	}
	// add in A2M3 bit
	temp_buffer = temp_buffer | ((AlarmBits & 0b00100000) << 2);
	// A2 hour is figured out, send it
	Wire.write(temp_buffer);
	// Figure out A2 day/date and A2M4
	temp_buffer = ((AlarmBits & 0b01000000) << 1) | decToBcd(A2Day);
	if (A2Dy) {
		// Set A2 Day/Date flag (Otherwise it's zero)
		temp_buffer = temp_buffer | 0b01000000;
	}
	Wire.write(temp_buffer);
	// All done!
	Wire.endTransmission();
}

void DS3231::turnOnAlarm(byte Alarm) {
	// turns on alarm number "Alarm". Defaults to 2 if Alarm is not 1.
	byte temp_buffer = readControlByte(0);
	// modify control byte
	if (Alarm == 1) {
		temp_buffer = temp_buffer | 0b00000101;
	}
	else {
		temp_buffer = temp_buffer | 0b00000110;
	}
	writeControlByte(temp_buffer, 0);
}

void DS3231::turnOffAlarm(byte Alarm) {
	// turns off alarm number "Alarm". Defaults to 2 if Alarm is not 1.
	// Leaves interrupt pin alone.
	byte temp_buffer = readControlByte(0);
	// modify control byte
	if (Alarm == 1) {
		temp_buffer = temp_buffer & 0b11111110;
	}
	else {
		temp_buffer = temp_buffer & 0b11111101;
	}
	writeControlByte(temp_buffer, 0);
}

bool DS3231::checkAlarmEnabled(byte Alarm) {
	// Checks whether the given alarm is enabled.
	byte result = 0x0;
	byte temp_buffer = readControlByte(0);
	if (Alarm == 1) {
		result = temp_buffer & 0b00000001;
	}
	else {
		result = temp_buffer & 0b00000010;
	}
	return result;
}

bool DS3231::checkIfAlarm(byte Alarm) {
	// Checks whether alarm 1 or alarm 2 flag is on, returns T/F accordingly.
	// Turns flag off, also.
	// defaults to checking alarm 2, unless Alarm == 1.
	byte result;
	byte temp_buffer = readControlByte(1);
	if (Alarm == 1) {
		// Did alarm 1 go off?
		result = temp_buffer & 0b00000001;
		// clear flag
		temp_buffer = temp_buffer & 0b11111110;
	}
	else {
		// Did alarm 2 go off?
		result = temp_buffer & 0b00000010;
		// clear flag
		temp_buffer = temp_buffer & 0b11111101;
	}
	writeControlByte(temp_buffer, 1);
	return result;
}

void DS3231::enableOscillator(bool TF, bool battery, byte frequency) {
	// turns oscillator on or off. True is on, false is off.
	// if battery is true, turns on even for battery-only operation,
	// otherwise turns off if Vcc is off.
	// frequency must be 0, 1, 2, or 3.
	// 0 = 1 Hz
	// 1 = 1.024 kHz
	// 2 = 4.096 kHz
	// 3 = 8.192 kHz (Default if frequency byte is out of range)
	if (frequency > 3) frequency = 3;
	// read control byte in, but zero out current state of RS2 and RS1.
	byte temp_buffer = readControlByte(0) & 0b11100111;
	if (battery) {
		// turn on BBSQW flag
		temp_buffer = temp_buffer | 0b01000000;
	}
	else {
		// turn off BBSQW flag
		temp_buffer = temp_buffer & 0b10111111;
	}
	if (TF) {
		// set ~EOSC to 0 and INTCN to zero.
		temp_buffer = temp_buffer & 0b01111011;
	}
	else {
		// set ~EOSC to 1, leave INTCN as is.
		temp_buffer = temp_buffer | 0b10000000;
	}
	// shift frequency into bits 3 and 4 and set.
	frequency = frequency << 3;
	temp_buffer = temp_buffer | frequency;
	// And write the control bits
	writeControlByte(temp_buffer, 0);
}

void DS3231::enable32kHz(bool TF) {
	// turn 32kHz pin on or off
	byte temp_buffer = readControlByte(1);
	if (TF) {
		// turn on 32kHz pin
		temp_buffer = temp_buffer | 0b00001000;
	}
	else {
		// turn off 32kHz pin
		temp_buffer = temp_buffer & 0b11110111;
	}
	writeControlByte(temp_buffer, 1);
}

bool DS3231::oscillatorCheck() {
	// Returns false if the oscillator has been off for some reason.
	// If this is the case, the time is probably not correct.
	byte temp_buffer = readControlByte(1);
	bool result = true;
	if (temp_buffer & 0b10000000) {
		// Oscillator Stop Flag (OSF) is set, so return false.
		result = false;
	}
	return result;
}

/*****************************************
	Private Functions
 *****************************************/

byte DS3231::decToBcd(byte val) {
	// Convert normal decimal numbers to binary coded decimal
	return ((val / 10 * 16) + (val % 10));
}

byte DS3231::bcdToDec(byte val) {
	// Convert binary coded decimal to normal decimal numbers
	return ((val / 16 * 10) + (val % 16));
}

byte DS3231::readControlByte(bool which) {
	// Read selected control byte
	// first byte (0) is 0x0e, second (1) is 0x0f
	Wire.beginTransmission(CLOCK_ADDRESS);
	if (which) {
		// second control byte
		Wire.write(0x0f);
	}
	else {
		// first control byte
		Wire.write(0x0e);
	}
	Wire.endTransmission();
	Wire.requestFrom(CLOCK_ADDRESS, 1);
	return Wire.read();
}

void DS3231::writeControlByte(byte control, bool which) {
	// Write the selected control byte.
	// which=false -> 0x0e, true->0x0f.
	Wire.beginTransmission(CLOCK_ADDRESS);
	if (which) {
		Wire.write(0x0f);
	}
	else {
		Wire.write(0x0e);
	}
	Wire.write(control);
	Wire.endTransmission();
}

#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////                       LCD                           ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define CLCD1 1
#ifdef CLCD1

CLCD::CLCD(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows)
{
	_Addr = lcd_Addr;
	_cols = lcd_cols;
	_rows = lcd_rows;
	_backlightval = LCD_NOBACKLIGHT;
}

void CLCD::init() {
	init_priv();
}

void CLCD::init_priv()
{
	Wire.begin();
	_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	begin(_cols, _rows);
}

void CLCD::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
	if (lines > 1) {
		_displayfunction |= LCD_2LINE;
	}
	_numlines = lines;

	// for some 1 line displays you can select a 10 pixel high font
	if ((dotsize != 0) && (lines == 1)) {
		_displayfunction |= LCD_5x10DOTS;
	}

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
	delay(50);

	// Now we pull both RS and R/W low to begin commands
	expanderWrite(_backlightval);	// reset expanderand turn backlight off (Bit 8 =1)
	delay(1000);

	//put the LCD into 4 bit mode
	// this is according to the hitachi HD44780 datasheet
	// figure 24, pg 46

	  // we start in 8bit mode, try to set 4 bit mode
	write4bits(0x03 << 4);
	delayMicroseconds(4500); // wait min 4.1ms

	// second try
	write4bits(0x03 << 4);
	delayMicroseconds(4500); // wait min 4.1ms

	// third go!
	write4bits(0x03 << 4);
	delayMicroseconds(150);

	// finally, set to 4-bit interface
	write4bits(0x02 << 4);


	// set # lines, font size, etc.
	command(LCD_FUNCTIONSET | _displayfunction);

	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	display();

	// clear it off
	clear();

	// Initialize to default text direction (for roman languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

	// set the entry mode
	command(LCD_ENTRYMODESET | _displaymode);

	home();

}

/********** high level commands, for the user! */

void CLCD::clear() {
	command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	delayMicroseconds(2000);  // this command takes a long time!
}


void CLCD::home() {
	command(LCD_RETURNHOME);  // set cursor position to zero
	delayMicroseconds(2000);  // this command takes a long time!
}

void CLCD::setCursor(uint8_t col, uint8_t row) {
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (row > _numlines) {
		row = _numlines - 1;    // we count rows starting w/0
	}
	command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void CLCD::noDisplay() {
	_displaycontrol &= ~LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void CLCD::display() {
	_displaycontrol |= LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void CLCD::noCursor() {
	_displaycontrol &= ~LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void CLCD::cursor() {
	_displaycontrol |= LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void CLCD::noBlink() {
	_displaycontrol &= ~LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void CLCD::blink() {
	_displaycontrol |= LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void CLCD::scrollDisplayLeft(void) {
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void CLCD::scrollDisplayRight(void) {
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void CLCD::leftToRight(void) {
	_displaymode |= LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void CLCD::rightToLeft(void) {
	_displaymode &= ~LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void CLCD::autoscroll(void) {
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void CLCD::noAutoscroll(void) {
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void CLCD::createChar(uint8_t location, uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	command(LCD_SETCGRAMADDR | (location << 3));
	for (int i = 0; i < 8; i++) {
		write(charmap[i]);
	}
}

// Turn the (optional) backlight off/on
void CLCD::noBacklight(void) {
	_backlightval = LCD_NOBACKLIGHT;
	expanderWrite(0);
}

void CLCD::backlight(void) {
	_backlightval = LCD_BACKLIGHT;
	expanderWrite(0);
}



/*********** mid level commands, for sending data/cmds */

inline void CLCD::command(uint8_t value) {
	send(value, 0);
}


/************ low level data pushing commands **********/

// write either command or data
void CLCD::send(uint8_t value, uint8_t mode) {
	uint8_t highnib = value & 0xf0;
	uint8_t lownib = (value << 4) & 0xf0;
	write4bits((highnib) | mode);
	write4bits((lownib) | mode);
}

void CLCD::write4bits(uint8_t value) {
	expanderWrite(value);
	pulseEnable(value);
}

void CLCD::expanderWrite(uint8_t _data) {
	Wire.beginTransmission(_Addr);
	printIIC((int)(_data) | _backlightval);
	Wire.endTransmission();
}

void CLCD::pulseEnable(uint8_t _data) {
	expanderWrite(_data | En);	// En high
	delayMicroseconds(1);		// enable pulse must be >450ns

	expanderWrite(_data & ~En);	// En low
	delayMicroseconds(50);		// commands need > 37us to settle
}


// Alias functions

void CLCD::cursor_on() {
	cursor();
}

void CLCD::cursor_off() {
	noCursor();
}

void CLCD::blink_on() {
	blink();
}

void CLCD::blink_off() {
	noBlink();
}

void CLCD::load_custom_character(uint8_t char_num, uint8_t* rows) {
	createChar(char_num, rows);
}

void CLCD::setBacklight(uint8_t new_val) {
	if (new_val) {
		backlight();		// turn backlight on
	}
	else {
		noBacklight();		// turn backlight off
	}
}

void CLCD::printstr(const char c[]) {
	//This function is not identical to the function used for "real" I2C displays
	//it's here so the user sketch doesn't have to be changed 
	print(c);
}


// unsupported API functions
void CLCD::off() {}
void CLCD::on() {}
void CLCD::setDelay(int cmdDelay, int charDelay) {}
uint8_t CLCD::status() { return 0; }
uint8_t CLCD::keypad() { return 0; }
uint8_t CLCD::init_bargraph(uint8_t graphtype) { return 0; }
void CLCD::draw_horizontal_graph(uint8_t row, uint8_t column, uint8_t len, uint8_t pixel_col_end) {}
void CLCD::draw_vertical_graph(uint8_t row, uint8_t column, uint8_t len, uint8_t pixel_row_end) {}
void CLCD::setContrast(uint8_t new_val) {}

#endif


#define PRINT1 1
#ifdef PRINT1

size_t Print1::write(const uint8_t* buffer, size_t size)
{
	size_t n = 0;
	while (size--) {
		if (write(*buffer++)) n++;
		else break;
	}
	return n;
}

size_t Print1::PRINT(const __FlashStringHelper* ifsh)
{
	PGM_P p = reinterpret_cast<PGM_P>(ifsh);
	size_t n = 0;
	while (1) {
		unsigned char c = pgm_read_byte(p++);
		if (c == 0) break;
		if (write(c)) n++;
		else break;
	}
	return n;
}

size_t Print1::PRINT(const String& s)
{
	return write(s.c_str(), s.length());
}

size_t Print1::PRINT(const char str[])
{
	return write(str);
}

size_t Print1::PRINT(char c)
{
	return write(c);
}

size_t Print1::PRINT(unsigned char b, int base)
{
	return PRINT((unsigned long)b, base);
}

size_t Print1::PRINT(int n, int base)
{
	return PRINT((long)n, base);
}

size_t Print1::PRINT(unsigned int n, int base)
{
	return PRINT((unsigned long)n, base);
}

size_t Print1::PRINT(long n, int base)
{
	if (base == 0) {
		return write(n);
	}
	else if (base == 10) {
		if (n < 0) {
			int t = PRINT('-');
			n = -n;
			return printNumber(n, 10) + t;
		}
		return printNumber(n, 10);
	}
	else {
		return printNumber(n, base);
	}
}

size_t Print1::PRINT(unsigned long n, int base)
{
	if (base == 0) return write(n);
	else return printNumber(n, base);
}

size_t Print1::PRINT(double n, int digits)
{
	return printFloat(n, digits);
}

size_t Print1::PRINTLN(const __FlashStringHelper* ifsh)
{
	size_t n = PRINT(ifsh);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINT(const Printable1& x)
{
	return x.printTo(*this);
}

size_t Print1::PRINTLN(void)
{
	return write("\r\n");
}

size_t Print1::PRINTLN(const String& s)
{
	size_t n = PRINT(s);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINTLN(const char c[])
{
	size_t n = PRINT(c);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINTLN(char c)
{
	size_t n = PRINT(c);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINTLN(unsigned char b, int base)
{
	size_t n = PRINT(b, base);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINTLN(int num, int base)
{
	size_t n = PRINT(num, base);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINTLN(unsigned int num, int base)
{
	size_t n = PRINT(num, base);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINTLN(long num, int base)
{
	size_t n = PRINTLN(num, base);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINTLN(unsigned long num, int base)
{
	size_t n = PRINT(num, base);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINTLN(double num, int digits)
{
	size_t n = PRINT(num, digits);
	n += PRINTLN();
	return n;
}

size_t Print1::PRINTLN(const Printable1& x)
{
	size_t n = PRINT(x);
	n += PRINTLN();
	return n;
}

// Private Methods /////////////////////////////////////////////////////////////

size_t Print1::printNumber(unsigned long n, uint8_t base)
{
	char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
	char* str = &buf[sizeof(buf) - 1];

	*str = '\0';

	// prevent crash if called with base == 1
	if (base < 2) base = 10;

	do {
		char c = n % base;
		n /= base;

		*--str = c < 10 ? c + '0' : c + 'A' - 10;
	} while (n);

	return write(str);
}

size_t Print1::printFloat(double number, uint8_t digits)
{
	size_t n = 0;

	if (isnan(number)) return PRINT("nan");
	if (isinf(number)) return PRINT("inf");
	if (number > 4294967040.0) return PRINT("ovf");  // constant determined empirically
	if (number < -4294967040.0) return PRINT("ovf");  // constant determined empirically

	// Handle negative numbers
	if (number < 0.0)
	{
		n += PRINT('-');
		number = -number;
	}

	// Round correctly so that print(1.999, 2) prints as "2.00"
	double rounding = 0.5;
	for (uint8_t i = 0; i < digits; ++i)
		rounding /= 10.0;

	number += rounding;

	// Extract the integer part of the number and print it
	unsigned long int_part = (unsigned long)number;
	double remainder = number - (double)int_part;
	n += PRINT(int_part);

	// Print the decimal point, but only if there are digits beyond
	if (digits > 0) {
		n += PRINT('.');
	}

	// Extract digits from the remainder one at a time
	while (digits-- > 0)
	{
		remainder *= 10.0;
		unsigned int toPrint = (unsigned int)(remainder);
		n += PRINT(toPrint);
		remainder -= toPrint;
	}

	return n;
}




void yield11() {

}
void DELAY(unsigned long ms)
{
	uint32_t start = micros();

	while (ms > 0) {
		yield11();
		while (ms > 0 && (micros() - start) >= 1000) {
			ms--;
			start += 1000;
		}
	}
}
#endif // PRINT1

#define LD_CLCD1 1
#ifdef LD_CLCD1
LD_CLCD::LD_CLCD(uint8_t lcd_Addr, uint8_t lcd_cols, uint8_t lcd_rows)
{
	_Addr = lcd_Addr;
	_cols = lcd_cols;
	_rows = lcd_rows;
	_backlightval = LCD_NOBACKLIGHT;
}

void LD_CLCD::INIT() {
	init_priv();
}

void LD_CLCD::init_priv()
{
	Wire.begin();
	_displayfunction = LCD_4BITMODE | LCD_1LINE | LCD_5x8DOTS;
	begin(_cols, _rows);
}

void LD_CLCD::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
	if (lines > 1) {
		_displayfunction |= LCD_2LINE;
	}
	_numlines = lines;

	// for some 1 line displays you can select a 10 pixel high font
	if ((dotsize != 0) && (lines == 1)) {
		_displayfunction |= LCD_5x10DOTS;
	}

	// SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
	// according to datasheet, we need at least 40ms after power rises above 2.7V
	// before sending commands. Arduino can turn on way befer 4.5V so we'll wait 50
	delay(50);

	// Now we pull both RS and R/W low to begin commands
	expanderWrite(_backlightval);	// reset expanderand turn backlight off (Bit 8 =1)
	delay(1000);

	//put the LCD into 4 bit mode
	// this is according to the hitachi HD44780 datasheet
	// figure 24, pg 46

	  // we start in 8bit mode, try to set 4 bit mode
	write4bits(0x03 << 4);
	delayMicroseconds(4500); // wait min 4.1ms

	// second try
	write4bits(0x03 << 4);
	delayMicroseconds(4500); // wait min 4.1ms

	// third go!
	write4bits(0x03 << 4);
	delayMicroseconds(150);

	// finally, set to 4-bit interface
	write4bits(0x02 << 4);


	// set # lines, font size, etc.
	command(LCD_FUNCTIONSET | _displayfunction);

	// turn the display on with no cursor or blinking default
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	DIS_PLAY();

	// clear it off
	CLEAR();

	// Initialize to default text direction (for roman languages)
	_displaymode = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;

	// set the entry mode
	command(LCD_ENTRYMODESET | _displaymode);

	HOME();

}

/********** high level commands, for the user! */
void LD_CLCD::CLEAR() {
	command(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	delayMicroseconds(2000);  // this command takes a long time!
}

void LD_CLCD::HOME() {
	command(LCD_RETURNHOME);  // set cursor position to zero
	delayMicroseconds(2000);  // this command takes a long time!
}

void LD_CLCD::SET_CURSOR(uint8_t col, uint8_t row) {
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (row > _numlines) {
		row = _numlines - 1;    // we count rows starting w/0
	}
	command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void LD_CLCD::NO_DISPLAY() {
	_displaycontrol &= ~LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LD_CLCD::DIS_PLAY() {
	_displaycontrol |= LCD_DISPLAYON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LD_CLCD::NO_CURSOR() {
	_displaycontrol &= ~LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LD_CLCD::CURSOR() {
	_displaycontrol |= LCD_CURSORON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void LD_CLCD::NO_BLINK() {
	_displaycontrol &= ~LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}
void LD_CLCD::BLINK() {
	_displaycontrol |= LCD_BLINKON;
	command(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void LD_CLCD::SCROLL_DISPLAY_LEFT(void) {
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void LD_CLCD::SCROLL_DISPLAY_RIGHT(void) {
	command(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void LD_CLCD::LEFT_TO_RIGHT(void) {
	_displaymode |= LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void LD_CLCD::RIGHT_TO_LEFT(void) {
	_displaymode &= ~LCD_ENTRYLEFT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void LD_CLCD::AUTO_SCROLL(void) {
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void LD_CLCD::NO_AUTO_SCROLL(void) {
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	command(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LD_CLCD::CREATE_CHAR(uint8_t location, uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	command(LCD_SETCGRAMADDR | (location << 3));
	for (int i = 0; i < 8; i++) {
		write(charmap[i]);
	}
}

// Turn the (optional) backlight off/on
void LD_CLCD::NO_BACKLIGHT(void) {
	_backlightval = LCD_NOBACKLIGHT;
	expanderWrite(0);
}

void LD_CLCD::BACKLIGHT(void) {
	_backlightval = LCD_BACKLIGHT;
	expanderWrite(0);
}



/*********** mid level commands, for sending data/cmds */

inline void LD_CLCD::command(uint8_t value) {
	send(value, 0);
}


/************ low level data pushing commands **********/

// write either command or data
void LD_CLCD::send(uint8_t value, uint8_t mode) {
	uint8_t highnib = value & 0xf0;
	uint8_t lownib = (value << 4) & 0xf0;
	write4bits((highnib) | mode);
	write4bits((lownib) | mode);
}

void LD_CLCD::write4bits(uint8_t value) {
	expanderWrite(value);
	pulseEnable(value);
}

void LD_CLCD::expanderWrite(uint8_t _data) {
	Wire.beginTransmission(_Addr);
	printIIC((int)(_data) | _backlightval);
	Wire.endTransmission();
}

void LD_CLCD::pulseEnable(uint8_t _data) {
	expanderWrite(_data | En);	// En high
	delayMicroseconds(1);		// enable pulse must be >450ns

	expanderWrite(_data & ~En);	// En low
	delayMicroseconds(50);		// commands need > 37us to settle
}


// Alias functions

void LD_CLCD::CUSROR_ON() {
	CURSOR();
}

void LD_CLCD::CURSOR_OFF() {
	NO_CURSOR();
}

void LD_CLCD::BLINK_ON() {
	BLINK();
}

void LD_CLCD::BLINK_OFF() {
	NO_BLINK();
}

void LD_CLCD::load_custom_character(uint8_t char_num, uint8_t* rows) {
	CREATE_CHAR(char_num, rows);
}

void LD_CLCD::SET_BACKLIGHT(uint8_t new_val) {
	if (new_val) {
		BACKLIGHT();		// turn backlight on
	}
	else {
		NO_BACKLIGHT();		// turn backlight off
	}
}

void LD_CLCD::printstr(const char c[]) {
	//This function is not identical to the function used for "real" I2C displays
	//it's here so the user sketch doesn't have to be changed 
	PRINT(c);
}


// unsupported API functions
void LD_CLCD::off() {}
void LD_CLCD::on() {}
void LD_CLCD::setDelay(int cmdDelay, int charDelay) {}
uint8_t LD_CLCD::status() { return 0; }
uint8_t LD_CLCD::keypad() { return 0; }
uint8_t LD_CLCD::init_bargraph(uint8_t graphtype) { return 0; }
void LD_CLCD::draw_horizontal_graph(uint8_t row, uint8_t column, uint8_t len, uint8_t pixel_col_end) {}
void LD_CLCD::draw_vertical_graph(uint8_t row, uint8_t column, uint8_t len, uint8_t pixel_row_end) {}
void LD_CLCD::setContrast(uint8_t new_val) {}

#endif // LD_CLCD1

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////   PWM    PWM    PWM    PWM    /////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef __AVR_ATmega2560__
#define PWM111 1
#define IDAC1 1
#define IADC111 1
#define ITIMER1 1
#define MODTIMER1 1
#define TIMER_OVF 1
#endif
#if defined(MPINO128) || defined(MPAINO128)//mcu128
#define PWM111 2
#define IDAC1 2
#define IADC111 1
#define ITIMER1 2
#define MODTIMER1 2
#define TIMER_OVF 2
#endif


#if PWM111 == 1

uint8_t pin44change46 = 1;
bool Pwm2onoff = 0, Pwm3onoff = 0, Pwm5onoff = 0, Pwm6onoff = 0, Pwm7onoff = 0, Pwm8onoff = 0, Pwm11onoff = 0, Pwm12onoff = 0, Pwm13onoff = 0, Pwm44onoff = 0, Pwm45onoff = 0, Pwm46onoff = 0;
void PWMOFF(uint8_t pin, bool POff) {
	switch (pin)
	{
	case 2:
		if (POff == HIGH) { Pwm2onoff = 1; }
		else if (POff == LOW) { Pwm2onoff = 0; }
		break;
	case 3:
		if (POff == HIGH) { Pwm3onoff = 1; }
		else if (POff == LOW) { Pwm3onoff = 0; }
		break;
	case 5:
		if (POff == HIGH) { Pwm5onoff = 1; }
		else if (POff == LOW) { Pwm5onoff = 0; }
		break;
	case 6:
		if (POff == HIGH) { Pwm6onoff = 1; }
		else if (POff == LOW) { Pwm6onoff = 0; }
		break;
	case 7:
		if (POff == HIGH) { Pwm7onoff = 1; }
		else if (POff == LOW) { Pwm7onoff = 0; }
		break;
	case 8:
		if (POff == HIGH) { Pwm8onoff = 1; }
		else if (POff == LOW) { Pwm8onoff = 0; }
		break;
	case 11:
		if (POff == HIGH) { Pwm11onoff = 1; }
		else if (POff == LOW) { Pwm11onoff = 0; }
		break;
	case 12:
		if (POff == HIGH) { Pwm12onoff = 1; }
		else if (POff == LOW) { Pwm12onoff = 0; }
		break;
	case 13:
		if (POff == HIGH) { Pwm13onoff = 1; }
		else if (POff == LOW) { Pwm13onoff = 0; }
		break;
	case 44:
		if (POff == HIGH) { Pwm44onoff = 1; }
		else if (POff == LOW) { Pwm44onoff = 0; }
		break;
	case 45:
		if (POff == HIGH) { Pwm45onoff = 1; }
		else if (POff == LOW) { Pwm45onoff = 0; }
		break;
	case 46:
		if (POff == HIGH) { Pwm46onoff = 1; }
		else if (POff == LOW) { Pwm46onoff = 0; }
		break;
	}

}
void PWM(uint8_t pin, uint16_t val, bool onDutybit16)
{
	if ((pin == 44) && (pin44change46 == 1)) {
		pin = 46;
		pin44change46 = 0;
	}if ((pin == 46) && (pin44change46 == 1)) {
		pin = 44;
		pin44change46 = 0;
	}
	pin44change46 = 1;

	pinMode(pin, OUTPUT);
	switch (pin)
	{
	case 2:
		if (Pwm2onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR3A |= ((1 << COM3B1) | (1 << WGM31)); TCCR3B |= ((1 << WGM33) | (1 << WGM32) | (1 << CS30));	ICR3 = 65535; }
			else { TCCR3A |= (1 << WGM30); TCCR3B |= ((1 << CS31) | (1 << CS30));}
		}
		else if (Pwm2onoff == HIGH) { TCCR3A &= ~(1 << COM3B1); }
		break;
	case 3:
		if (Pwm3onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR3A |= ((1 << COM3C1) | (1 << WGM31)); TCCR3B |= ((1 << WGM33) | (1 << WGM32) | (1 << CS30)); ICR3 = 65535; }
			else { TCCR3A |= (1 << WGM30); TCCR3B |= ((1 << CS31) | (1 << CS30)); }
		}
		else if (Pwm3onoff == HIGH) { TCCR3A &= ~(1 << COM3C1); }
		break;
	case 5:
		if (Pwm5onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR3A |= ((1 << COM3A1) | (1 << WGM31)); TCCR3B |= ((1 << WGM33) | (1 << WGM32) | (1 << CS30)); ICR3 = 65535; }
			else { TCCR3A |= (1 << WGM30); TCCR3B |= ((1 << CS31) | (1 << CS30)); }
		}
		else if (Pwm5onoff == HIGH) { TCCR3A &= ~(1 << COM3A1); }
		break;
	case 6:
		if (Pwm6onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR4A |= ((1 << COM4A1) | (1 << WGM41)); TCCR4B |= ((1 << WGM43) | (1 << WGM42) | (1 << CS40)); ICR4 = 65535; }
			else { TCCR4A |= (1 << WGM40); TCCR4B |= ((1 << CS41) | (1 << CS40)); }
		}
		else if (Pwm6onoff == HIGH) { TCCR4A &= ~(1 << COM4A1); }
		break;
	case 7:
		if (Pwm7onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR4A |= ((1 << COM4B1) | (1 << WGM41)); TCCR4B |= ((1 << WGM43) | (1 << WGM42) | (1 << CS40)); ICR4 = 65535; }
			else { TCCR4A |= (1 << WGM40); TCCR4B |= ((1 << CS41) | (1 << CS40)); }
		}
		else if (Pwm7onoff == HIGH) { TCCR4A &= ~(1 << COM4B1); }
		break;
	case 8:
		if (Pwm8onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR4A |= ((1 << COM4C1) | (1 << WGM41)); TCCR4B |= ((1 << WGM43) | (1 << WGM42) | (1 << CS40)); ICR4 = 65535; }
			else { TCCR4A |= (1 << WGM40); TCCR4B |= ((1 << CS41) | (1 << CS40)); }
		}
		else if (Pwm8onoff == HIGH) { TCCR4A &= ~(1 << COM4C1); }
		break;
	case 11:
		if (Pwm11onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR1A |= ((1 << COM1A1) | (1 << WGM11)); TCCR1B |= ((1 << WGM13) | (1 << WGM12) | (1 << CS10)); ICR1 = 65535; }
			else { TCCR1A |= (1 << WGM10); TCCR1B |= ((1 << CS11) | (1 << CS10)); }
		}
		else if (Pwm11onoff == HIGH) { TCCR1A &= ~(1 << COM1A1); }
		break;
	case 12:
		if (Pwm12onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR1A |= ((1 << COM1B1) | (1 << WGM11)); TCCR1B |= ((1 << WGM13) | (1 << WGM12) | (1 << CS10)); ICR1 = 65535; }
			else { TCCR1A |= (1 << WGM10); TCCR1B |= ((1 << CS11) | (1 << CS10)); }
		}
		else if (Pwm12onoff == HIGH) { TCCR1A &= ~(1 << COM1B1); }
		break;
	case 13:
		if (Pwm13onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR1A |= ((1 << COM1C1) | (1 << WGM11)); TCCR1B |= ((1 << WGM13) | (1 << WGM12) | (1 << CS10)); ICR1 = 65535; }
			else { TCCR1A |= (1 << WGM10); TCCR1B |= ((1 << CS11) | (1 << CS10)); }
		}
		else if (Pwm13onoff == HIGH) { TCCR1A &= ~(1 << COM1C1); }
		break;
	case 44:
		if (Pwm44onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR5A |= ((1 << COM5C1) | (1 << WGM51)); TCCR5B |= ((1 << WGM53) | (1 << WGM52) | (1 << CS50)); ICR5 = 65535; }
			else { TCCR5A |= (1 << WGM50); TCCR5B |= ((1 << CS51) | (1 << CS50)); }
		}
		else if (Pwm44onoff == HIGH) { TCCR5A &= ~(1 << COM5C1); }
		break;
	case 45:
		if (Pwm45onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR5A |= ((1 << COM5B1) | (1 << WGM51)); TCCR5B |= ((1 << WGM53) | (1 << WGM52) | (1 << CS50)); ICR5 = 65535; }
			else { TCCR5A |= (1 << WGM50); TCCR5B |= ((1 << CS51) | (1 << CS50)); }
		}
		else if (Pwm45onoff == HIGH) { TCCR5A &= ~(1 << COM5B1); }
		break;
	case 46:
		if (Pwm46onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR5A |= ((1 << COM5A1) | (1 << WGM51)); TCCR5B |= ((1 << WGM53) | (1 << WGM52) | (1 << CS50)); ICR5 = 65535; }
			else { TCCR5A |= (1 << WGM50); TCCR5B |= ((1 << CS51) | (1 << CS50)); }
		}
		else if (Pwm46onoff == HIGH) { TCCR5A &= ~(1 << COM5A1); }
		break;

	}
	if (val <= 0)
	{
		digitalWrite(pin, LOW);
	}
	else
	{
		switch (pin)
		{
		case 2:			//2pin OC3B
			// connect pwm to pin on timer 3, channel B
			if (Pwm2onoff == LOW) {
				sbi(TCCR3A, COM3B1);
				OCR3B = val; // set pwm duty
			}
			break;

		case 3:			//3pin OC3C
			// connect pwm to pin on timer 3, channel C
			if (Pwm3onoff == LOW) {
				sbi(TCCR3A, COM3C1);
				OCR3C = val; // set pwm duty
			}
			break;

		case 5:			//5pin OC3A
			// connect pwm to pin on timer 3, channel A
			if (Pwm5onoff == LOW) {
				sbi(TCCR3A, COM3A1);
				OCR3A = val; // set pwm duty
			}
			break;

		case 6:			//6pin OC4A
			// connect pwm to pin on timer 4, channel A
			if (Pwm6onoff == LOW) {
				sbi(TCCR4A, COM4A1);
				OCR4A = val; // set pwm duty
			}
			break;

		case 7:			//7pin OC4B
			// connect pwm to pin on timer 4, channel B
			if (Pwm7onoff == LOW) {
				sbi(TCCR4A, COM4B1);
				OCR4B = val; // set pwm duty
			}
			break;

		case 8:			//8pin OC4C
			// connect pwm to pin on timer 4, channel C
			if (Pwm8onoff == LOW) {
				sbi(TCCR4A, COM4C1);
				OCR4C = val; // set pwm duty
			}
			break;

		case 11:		//11pin OC1A
			// connect pwm to pin on timer 1, channel A
			if (Pwm11onoff == LOW) {
				sbi(TCCR1A, COM1A1);
				OCR1A = val; // set pwm duty
			}
			break;

		case 12:		//12pin OC1B
			// connect pwm to pin on timer 1, channel B
			if (Pwm12onoff == LOW) {
				sbi(TCCR1A, COM1B1);
				OCR1B = val; // set pwm duty
			}
			break;

		case 13:		//13pin OC1C
			// connect pwm to pin on timer 1, channel C
			if (Pwm13onoff == LOW) {
				sbi(TCCR1A, COM1C1);
				OCR1C = val; // set pwm duty
			}
			break;

		case 44:		//44pin OC5A
			// connect pwm to pin on timer 5, channel C
			if (Pwm44onoff == LOW) {
				sbi(TCCR5A, COM5C1);
				OCR5C = val; // set pwm duty
			}
			break;

		case 45:		//45pin OC5B
			// connect pwm to pin on timer 5, channel B
			if (Pwm45onoff == LOW) {
				sbi(TCCR5A, COM5B1);
				OCR5B = val; // set pwm duty
			}
			break;

		case 46:		//46pin OC5A
			// connect pwm to pin on timer 5, channel A
			if (Pwm46onoff == LOW) {
				sbi(TCCR5A, COM5A1);
				OCR5A = val; // set pwm duty
			}
			break;

		default:
			break;
		}
	}
}
uint16_t TimerCount1A = 0;
uint16_t TimerCount1B = 0;
uint16_t TimerCount1C = 0;

uint16_t TimerCount3A = 0;
uint16_t TimerCount3B = 0;
uint16_t TimerCount3C = 0;

uint16_t TimerCount4A = 0;
uint16_t TimerCount4B = 0;
uint16_t TimerCount4C = 0;

uint16_t TimerCount5A = 0;
uint16_t TimerCount5B = 0;
uint16_t TimerCount5C = 0;

uint32_t npwmtop = 0;
uint32_t npwmval = 0;

uint32_t N1A = 0;
uint32_t N1B = 0;
uint32_t N1C = 0;

uint32_t N3A = 0;
uint32_t N3B = 0;
uint32_t N3C = 0;

uint32_t N4A = 0;
uint32_t N4B = 0;
uint32_t N4C = 0;

uint32_t N5A = 0;
uint32_t N5B = 0;
uint32_t N5C = 0;


bool npwmTccrb = 0;

bool npwmTccrb1A = 0;
bool npwmTccrb1B = 0;
bool npwmTccrb1C = 0;

bool npwmTccrb3A = 0;
bool npwmTccrb3B = 0;
bool npwmTccrb3C = 0;

bool npwmTccrb4A = 0;
bool npwmTccrb4B = 0;
bool npwmTccrb4C = 0;

bool npwmTccrb5A = 0;
bool npwmTccrb5B = 0;
bool npwmTccrb5C = 0;

bool npwmON2 = 0, npwmON3 = 0, npwmON5 = 0, npwmON6 = 0, npwmON7 = 0, npwmON8 = 0, npwmON11 = 0, npwmON12 = 0, npwmON13 = 0, npwmON44 = 0, npwmON45 = 0, npwmON46 = 0;
bool npwm2ON = 0, npwm3ON = 0, npwm5ON = 0, npwm6ON = 0, npwm7ON = 0, npwm8ON = 0, npwm11ON = 0, npwm12ON = 0, npwm13ON = 0, npwm44ON = 0, npwm45ON = 0, npwm46ON = 0;
void PWM_RESET() {
	TCCR1A = 0x00;
	TCCR1B = 0x00;
	TCNT1 = 0x00;

	TCCR3A = 0x00;
	TCCR3B = 0x00;
	TCNT3 = 0x00;

	TCCR4A = 0x00;
	TCCR4B = 0x00;
	TCNT4 = 0x00;

	TCCR5A = 0x00;
	TCCR5B = 0x00;
	TCNT5 = 0x00;
}
void NPWM_BEGIN(uint8_t pin, uint32_t intHz, float Duty, uint32_t N) {
	if ((pin == 44) && (pin44change46 == 1)) {
		pin = 46;
		pin44change46 = 0;
	}if ((pin == 46) && (pin44change46 == 1)) {
		pin = 44;
		pin44change46 = 0;
	}
	pin44change46 = 1;

	if (intHz > 2000000) {
		npwmtop = (float)(F_CPU) / (float)2000000 - 1.0;
		npwmval = (float)(Duty / 100.0) * (float)npwmtop;//int(TOP * (Duty / 100) - 1);
		npwmTccrb = 0;
	}
	else if (intHz <= 2000000 && intHz >= 300) {
		npwmtop = (float)(F_CPU) / (float)intHz - 1.0;
		npwmval = (float)(Duty / 100.0) * (float)npwmtop;//int(TOP * (Duty / 100) - 1);
		npwmTccrb = 0;
	}
	else if (intHz < 300 && intHz >= 1) {
		npwmtop = (float)(15625) / (float)intHz - 1.0;
		npwmval = (float)(Duty / 100.0) * (float)npwmtop;//int(TOP * (Duty / 100) - 1);
		npwmTccrb = 1;
	}
	else {
		npwmtop = (float)(15625) - 1.0;
		npwmval = (float)(Duty / 100.0) * (float)npwmtop;//int(TOP * (Duty / 100) - 1);
		npwmTccrb = 1;
	}
	pinMode(pin, OUTPUT);
	switch (pin)
	{
	case 11://A		//TIMER1
		TIMSK1 |= (1 << TOIE1);
		TCCR1A |= (1 << WGM11);
		TCCR1B |= (1 << WGM13) | (1 << WGM12);
		ICR1 = npwmtop;
		OCR1A = npwmtop - npwmval;
		N1A = N;
		npwmTccrb1A = npwmTccrb;
		TimerCount1A = 0;
		npwm11ON = 1;
		break;
	case 12://B
		TIMSK1 |= (1 << TOIE1);
		TCCR1A |= (1 << WGM11);
		TCCR1B |= (1 << WGM13) | (1 << WGM12);
		ICR1 = npwmtop;
		OCR1B = npwmtop - npwmval;
		N1B = N;
		npwmTccrb1B = npwmTccrb;
		TimerCount1B = 0;
		npwm12ON = 1;
		break;
	case 13://C
		TIMSK1 |= (1 << TOIE1);
		TCCR1A |= (1 << WGM11);
		TCCR1B |= (1 << WGM13) | (1 << WGM12);
		ICR1 = npwmtop;
		OCR1C = npwmtop - npwmval;
		N1C = N;
		npwmTccrb1C = npwmTccrb;
		TimerCount1C = 0;
		npwm13ON = 1;
		break;
	case 2://B		//TIMER3
		TIMSK3 |= (1 << TOIE3);
		TCCR3A |= (1 << WGM31);
		TCCR3B |= (1 << WGM33) | (1 << WGM32);
		ICR3 = npwmtop;
		OCR3B = npwmtop - npwmval;
		N3B = N;
		npwmTccrb3B = npwmTccrb;
		TimerCount3B = 0;
		npwm2ON = 1;
		break;
	case 3://C
		TIMSK3 |= (1 << TOIE3);
		TCCR3A |= (1 << WGM31);
		TCCR3B |= (1 << WGM33) | (1 << WGM32);
		ICR3 = npwmtop;
		OCR3C = npwmtop - npwmval;
		N3C = N;
		npwmTccrb3C = npwmTccrb;
		TimerCount3C = 0;
		npwm3ON = 1;
		break;
	case 5://A
		TIMSK3 |= (1 << TOIE3);
		TCCR3A |= (1 << WGM31);
		TCCR3B |= (1 << WGM33) | (1 << WGM32);
		ICR3 = npwmtop;
		OCR3A = npwmtop - npwmval;
		N3A = N;
		npwmTccrb3A = npwmTccrb;
		TimerCount3A = 0;
		npwm5ON = 1;
		break;
	case 6://A		//TIMER4
		TIMSK4 |= (1 << TOIE4);
		TCCR4A |= (1 << WGM41);
		TCCR4B |= (1 << WGM43) | (1 << WGM42);
		ICR4 = npwmtop;
		OCR4A = npwmtop - npwmval;
		N4A = N;
		npwmTccrb4A = npwmTccrb;
		TimerCount4A = 0;
		npwm6ON = 1;
		break;
	case 7://B
		TIMSK4 |= (1 << TOIE4);
		TCCR4A |= (1 << WGM41);
		TCCR4B |= (1 << WGM43) | (1 << WGM42);
		ICR4 = npwmtop;
		OCR4B = npwmtop - npwmval;
		N4B = N;
		npwmTccrb4B = npwmTccrb;
		TimerCount4B = 0;
		npwm7ON = 1;
		break;
	case 8://C
		TIMSK4 |= (1 << TOIE4);
		TCCR4A |= (1 << WGM41);
		TCCR4B |= (1 << WGM43) | (1 << WGM42);
		ICR4 = npwmtop;
		OCR4C = npwmtop - npwmval;
		N4C = N;
		npwmTccrb4C = npwmTccrb;
		TimerCount4C = 0;
		npwm8ON = 1;
		break;
	case 44://C		//TIMER5
		TIMSK5 |= (1 << TOIE5);
		TCCR5A |= (1 << WGM51);
		TCCR5B |= (1 << WGM53) | (1 << WGM52);
		ICR5 = npwmtop;
		OCR5C = npwmtop - npwmval;
		N5C = N;
		npwmTccrb5C = npwmTccrb;
		TimerCount5C = 0;
		npwm44ON = 1;
		break;
	case 45://B
		TIMSK5 |= (1 << TOIE5);
		TCCR5A |= (1 << WGM51);
		TCCR5B |= (1 << WGM53) | (1 << WGM52);
		ICR5 = npwmtop;
		OCR5B = npwmtop - npwmval;
		N5B = N;
		npwmTccrb5B = npwmTccrb;
		TimerCount5B = 0;
		npwm45ON = 1;
		break;
	case 46://A
		TIMSK5 |= (1 << TOIE5);
		TCCR5A |= (1 << WGM51);
		TCCR5B |= (1 << WGM53) | (1 << WGM52);
		ICR5 = npwmtop;
		OCR5A = npwmtop - npwmval;
		N5A = N;
		npwmTccrb5A = npwmTccrb;
		TimerCount5A = 0;
		npwm46ON = 1;
		break;
	default:
		break;
		//		delayMicroseconds((1.0/(float)intHz)/(1.0/ 1000000.0));
	}
}

bool oneplus2 = 1, oneplus3 = 1, oneplus5 = 1;
bool oneplus6 = 1, oneplus7 = 1, oneplus8 = 1;
bool oneplus11 = 1, oneplus12 = 1, oneplus13 = 1;
bool oneplus44 = 1, oneplus45 = 1, oneplus46 = 1;

void NPWM(uint8_t pin) {
	if ((pin == 44) && (pin44change46 == 1)) {
		pin = 46;
		pin44change46 = 0;
	}if ((pin == 46) && (pin44change46 == 1)) {
		pin = 44;
		pin44change46 = 0;
	}
	pin44change46 = 1;

	if (npwm2ON == 1) {
		if (pin == 2) {
			if (oneplus2 == 1) {
				if (TimerCount3B == 0) {
					//pinMode(2, INPUT);
					DDRE &= ~(1 << 4);
				}
				else {
					DDRE |= (1 << 4);
					oneplus2 = 0;
					N3B++;
				}
				npwmON2 = 1;
				TCCR3A |= (1 << COM3B1) | (1 << COM3B0);
				if (npwmTccrb3B == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3B == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			if (TimerCount3B < N3B) {
				npwmON2 = 1;
				TCCR3A |= (1 << COM3B1) | (1 << COM3B0);
				if (npwmTccrb3B == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3B == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			else if (TimerCount3B >= N3B) {
				TCCR3A &= ~((1 << COM3B1) | (1 << COM3B0));
				TCCR3B &= ~((1 << CS32) | (1 << CS30));
				npwmON2 = 0;
				npwm2ON = 0;
				if ((npwm3ON == 1) | (npwm5ON == 1)) {}
				else {
					TCNT3 = 0x00;
				}
			}
		}
	}if (npwm3ON == 1) {
		if (pin == 3) {
			if (oneplus3 == 1) {
				if (TimerCount3C == 0) {
					//pinMode(2, INPUT);
					DDRE &= ~(1 << 5);
				}
				else {
					DDRE |= (1 << 5);
					oneplus3 = 0;
					N3C++;
				}
				npwmON3 = 1;
				TCCR3A |= (1 << COM3C1) | (1 << COM3C0);
				if (npwmTccrb3C == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3C == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			if (TimerCount3C < N3C) {
				npwmON3 = 1;
				TCCR3A |= (1 << COM3C1) | (1 << COM3C0);
				if (npwmTccrb3C == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3C == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			else if (TimerCount3C >= N3C) {
				TCCR3A &= ~((1 << COM3C1) | (1 << COM3C0));
				TCCR3B &= ~((1 << CS32) | (1 << CS30));
				npwmON3 = 0;
				npwm3ON = 0;

				if ((npwm2ON == 1) | (npwm5ON == 1)) {}
				else {
					TCNT3 = 0x00;
				}
			}
		}
	}if (npwm5ON == 1) {
		if (pin == 5) {
			if (oneplus5 == 1) {
				if (TimerCount3A == 0) {
					//pinMode(2, INPUT);
					DDRE &= ~(1 << 3);
				}
				else {
					DDRE |= (1 << 3);
					oneplus5 = 0;
					N3A++;
				}
				npwmON5 = 1;
				TCCR3A |= (1 << COM3A1) | (1 << COM3A0);
				if (npwmTccrb3C == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3C == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			if (TimerCount3A < N3A) {
				npwmON5 = 1;
				TCCR3A |= (1 << COM3A1) | (1 << COM3A0);
				if (npwmTccrb3A == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3A == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			else if (TimerCount3A >= N3A) {
				TCCR3A &= ~((1 << COM3A1) | (1 << COM3A0));
				TCCR3B &= ~((1 << CS32) | (1 << CS30));
				npwmON5 = 0;
				npwm5ON = 0;

				if ((npwm2ON == 1) | (npwm3ON == 1)) {}
				else {
					TCNT3 = 0x00;
				}
			}
		}
	}if (npwm6ON == 1) {
		if (pin == 6) {
			if (oneplus6 == 1) {
				if (TimerCount4A == 0) {
					//pinMode(2, INPUT);
					DDRH &= ~(1 << 3);
				}
				else {
					DDRH |= (1 << 3);
					oneplus6 = 0;
					N4A++;
				}
				npwmON6 = 1;
				TCCR4A |= (1 << COM4A1) | (1 << COM4A0);
				if (npwmTccrb4A == 0) {
					TCCR4B |= (1 << CS40);
				}
				else if (npwmTccrb4A == 1) {
					TCCR4B |= (1 << CS42) | (1 << CS40);
				}
			}
			if (TimerCount4A < N4A) {
				npwmON6 = 1;
				TCCR4A |= (1 << COM4A1) | (1 << COM4A0);
				if (npwmTccrb4A == 0) {
					TCCR4B |= (1 << CS40);
				}
				else if (npwmTccrb4A == 1) {
					TCCR4B |= (1 << CS42) | (1 << CS40);
				}
			}
			else if (TimerCount4A >= N4A) {
				TCCR4A &= ~((1 << COM4A1) | (1 << COM4A0));
				TCCR4B &= ~((1 << CS42) | (1 << CS40));
				npwmON6 = 0;
				npwm6ON = 0;
				if ((npwm7ON == 1) | (npwm8ON == 1)) {}
				else {
					TCNT4 = 0x00;
				}
			}
		}
	}if (npwm7ON == 1) {
		if (pin == 7) {
			if (oneplus7 == 1) {
				if (TimerCount4B == 0) {
					//pinMode(2, INPUT);
					DDRH &= ~(1 << 4);
				}
				else {
					DDRH |= (1 << 4);
					oneplus7 = 0;
					N4B++;
				}
				npwmON7 = 1;
				TCCR4A |= (1 << COM4B1) | (1 << COM4B0);
				if (npwmTccrb4B == 0) {
					TCCR4B |= (1 << CS40);
				}
				else if (npwmTccrb4B == 1) {
					TCCR4B |= (1 << CS42) | (1 << CS40);
				}
			}
			if (TimerCount4B < N4B) {
				npwmON7 = 1;
				TCCR4A |= (1 << COM4B1) | (1 << COM4B0);
				if (npwmTccrb4B == 0) {
					TCCR4B |= (1 << CS40);
				}
				else if (npwmTccrb4B == 1) {
					TCCR4B |= (1 << CS42) | (1 << CS40);
				}
			}
			else if (TimerCount4B >= N4B) {
				TCCR4A &= ~((1 << COM4B1) | (1 << COM4B0));
				TCCR4B &= ~((1 << CS42) | (1 << CS40));
				npwmON7 = 0;
				npwm7ON = 0;
				if ((npwm6ON == 1) | (npwm8ON == 1)) {}
				else {
					TCNT4 = 0x00;
				}
			}
		}
	}if (npwm8ON == 1) {
		if (pin == 8) {
			if (oneplus8 == 1) {
				if (TimerCount4C == 0) {
					//pinMode(2, INPUT);
					DDRE &= ~(1 << 5);
				}
				else {
					DDRE |= (1 << 5);
					oneplus8 = 0;
					N4C++;
				}
				npwmON8 = 1;
				TCCR4A |= (1 << COM4C1) | (1 << COM4C0);
				if (npwmTccrb4C == 0) {
					TCCR4B |= (1 << CS40);
				}
				else if (npwmTccrb4C == 1) {
					TCCR4B |= (1 << CS42) | (1 << CS40);
				}
			}
			if (TimerCount4C < N4C) {
				npwmON8 = 1;
				TCCR4A |= (1 << COM4C1) | (1 << COM4C0);
				if (npwmTccrb4C == 0) {
					TCCR4B |= (1 << CS40);
				}
				else if (npwmTccrb4C == 1) {
					TCCR4B |= (1 << CS42) | (1 << CS40);
				}
			}
			else if (TimerCount4C >= N4C) {
				TCCR4A &= ~((1 << COM4C1) | (1 << COM4C0));
				TCCR4B &= ~((1 << CS42) | (1 << CS40));
				npwmON8 = 0;
				npwm8ON = 0;
				if ((npwm7ON == 1) | (npwm6ON == 1)) {}
				else {
					TCNT4 = 0x00;
				}
			}
		}
	}if (npwm11ON == 1) {//1A
		if (pin == 11) {
			if (oneplus11 == 1) {
				if (TimerCount1A == 0) {
					//pinMode(2, INPUT);
					DDRB &= ~(1 << 5);
				}
				else {
					DDRB |= (1 << 5);
					oneplus11 = 0;
					N1A++;
				}
				npwmON11 = 1;
				TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
				if (npwmTccrb1A == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1A == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			if (TimerCount1A < N1A) {
				npwmON11 = 1;
				TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
				if (npwmTccrb1A == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1A == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			else if (TimerCount1A >= N1A) {
				TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
				TCCR1B &= ~((1 << CS12) | (1 << CS10));
				npwmON11 = 0;
				npwm11ON = 0;
				if ((npwm12ON == 1) | (npwm13ON == 1)) {}
				else {
					TCNT1 = 0x00;
				}
			}
		}
	}if (npwm12ON == 1) {//12 //1B
		if (pin == 12) {
			if (oneplus12 == 1) {
				if (TimerCount1B == 0) {
					//pinMode(2, INPUT);
					DDRB &= ~(1 << 6);
				}
				else {
					DDRB |= (1 << 6);
					oneplus12 = 0;
					N1B++;
				}
				npwmON12 = 1;
				TCCR1A |= (1 << COM1B1) | (1 << COM1B0);
				if (npwmTccrb1B == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1B == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			if (TimerCount1B < N1B) {
				npwmON12 = 1;
				TCCR1A |= (1 << COM1B1) | (1 << COM1B0);
				if (npwmTccrb1B == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1B == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			else if (TimerCount1B >= N1B) {
				TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));
				TCCR1B &= ~((1 << CS12) | (1 << CS10));
				npwmON12 = 0;
				npwm12ON = 0;
				if ((npwm11ON == 1) | (npwm13ON == 1)) {}
				else {
					TCNT1 = 0x00;
				}
			}
		}
	}if (npwm13ON == 1) {//1C
		if (pin == 13) {
			if (oneplus13 == 1) {
				if (TimerCount1C == 0) {
					//pinMode(2, INPUT);
					DDRB &= ~(1 << 7);
				}
				else {
					DDRB |= (1 << 7);
					oneplus13 = 0;
					N1C++;
				}
				npwmON13 = 1;
				TCCR1A |= (1 << COM1C1) | (1 << COM1C0);
				if (npwmTccrb1C == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1C == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			if (TimerCount1C < N1C) {
				npwmON13 = 1;
				TCCR1A |= (1 << COM1C1) | (1 << COM1C0);
				if (npwmTccrb1C == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1C == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			else if (TimerCount1C >= N1C) {
				TCCR1A &= ~((1 << COM1C1) | (1 << COM1C0));
				TCCR1B &= ~((1 << CS12) | (1 << CS10));
				npwmON13 = 0;
				npwm13ON = 0;
				if ((npwm11ON == 1) | (npwm12ON == 1)) {}
				else {
					TCNT1 = 0x00;
				}
			}
		}
	}if (npwm44ON == 1) {//5C
		if (pin == 44) {
			if (oneplus44 == 1) {
				if (TimerCount5C == 0) {
					//pinMode(2, INPUT);
					DDRL &= ~(1 << 5);
				}
				else {
					DDRL |= (1 << 5);
					oneplus44 = 0;
					N5C++;
				}
				npwmON44 = 1;
				TCCR5A |= (1 << COM5C1) | (1 << COM5C0);
				if (npwmTccrb5C == 0) {
					TCCR5B |= (1 << CS50);
				}
				else if (npwmTccrb5C == 1) {
					TCCR5B |= (1 << CS52) | (1 << CS50);
				}
			}
			if (TimerCount5C < N5C) {
				npwmON44 = 1;
				TCCR5A |= (1 << COM5C1) | (1 << COM5C0);
				if (npwmTccrb5C == 0) {
					TCCR5B |= (1 << CS50);
				}
				else if (npwmTccrb5C == 1) {
					TCCR5B |= (1 << CS52) | (1 << CS50);
				}
			}
			else if (TimerCount5C >= N5C) {
				TCCR5A &= ~((1 << COM5C1) | (1 << COM5C0));
				TCCR5B &= ~((1 << CS52) | (1 << CS50));
				npwmON44 = 0;
				npwm44ON = 0;
				if ((npwm45ON == 1) | (npwm46ON == 1)) {}
				else {
					TCNT5 = 0x00;
				}
			}
		}
	}if (npwm45ON == 1) {//5B
		if (pin == 45) {
			if (oneplus45 == 1) {
				if (TimerCount5B == 0) {
					//pinMode(2, INPUT);
					DDRL &= ~(1 << 4);
				}
				else {
					DDRL |= (1 << 4);
					oneplus45 = 0;
					N5B++;
				}
				npwmON45 = 1;
				TCCR5A |= (1 << COM5B1) | (1 << COM5B0);
				if (npwmTccrb5B == 0) {
					TCCR5B |= (1 << CS50);
				}
				else if (npwmTccrb5B == 1) {
					TCCR5B |= (1 << CS52) | (1 << CS50);
				}
			}
			if (TimerCount5B < N5B) {
				npwmON45 = 1;
				TCCR5A |= (1 << COM5B1) | (1 << COM5B0);
				if (npwmTccrb5B == 0) {
					TCCR5B |= (1 << CS50);
				}
				else if (npwmTccrb5B == 1) {
					TCCR5B |= (1 << CS52) | (1 << CS50);
				}
			}
			else if (TimerCount5B >= N5B) {
				TCCR5A &= ~((1 << COM5B1) | (1 << COM5B0));
				TCCR5B &= ~((1 << CS52) | (1 << CS50));
				npwmON45 = 0;
				npwm45ON = 0;
				if ((npwm44ON == 1) | (npwm46ON == 1)) {}
				else {
					TCNT5 = 0x00;
				}
			}
		}
	}if (npwm46ON == 1) {//5A
		if (pin == 46) {
			if (oneplus46 == 1) {
				if (TimerCount5A == 0) {
					//pinMode(2, INPUT);
					DDRL &= ~(1 << 3);
				}
				else {
					DDRL |= (1 << 3);
					oneplus46 = 0;
					N5A++;
				}
				npwmON46 = 1;
				TCCR5A |= (1 << COM5A1) | (1 << COM5A0);
				if (npwmTccrb5A == 0) {
					TCCR5B |= (1 << CS50);
				}
				else if (npwmTccrb5A == 1) {
					TCCR5B |= (1 << CS52) | (1 << CS50);
				}
			}
			if (TimerCount5A < N5A) {
				npwmON46 = 1;
				TCCR5A |= (1 << COM5A1) | (1 << COM5A0);
				if (npwmTccrb5A == 0) {
					TCCR5B |= (1 << CS50);
				}
				else if (npwmTccrb5A == 1) {
					TCCR5B |= (1 << CS52) | (1 << CS50);
				}
			}
			else if (TimerCount5A >= N5A) {
				TCCR5A &= ~((1 << COM5A1) | (1 << COM5A0));
				TCCR5B &= ~((1 << CS52) | (1 << CS50));
				npwmON46 = 0;
				npwm46ON = 0;
				if ((npwm44ON == 1) | (npwm45ON == 1)) {}
				else {
					TCNT5 = 0x00;
				}
			}
		}
	}
}
//0~8*10^6              0~100
void FDPWM(uint8_t pin, int32_t intHz, float Duty)
{
	if ((pin == 44) && (pin44change46 == 1)) {
		pin = 46;
		pin44change46 = 0;
	}if ((pin == 46) && (pin44change46 == 1)) {
		pin = 44;
		pin44change46 = 0;
	}
	pin44change46 = 1;
	int val = 0;
	uint32_t TOP = 0;
	//	byte ChangebyteA = 0;
	byte ChangebyteB = 0;
	/*switch (pin)
	{
	case 5: //A
	case 6:
	case 11:
	case 46:
		ChangebyteA = 0x82;
		break;
	case 2: //B
	case 7:
	case 12:
	case 45:
		ChangebyteA = 0x22;
		break;
	case 3: //C
	case 8:
	case 13:
	case 44:
		ChangebyteA = 0x0A;
		break;
	default:
		break;
	}*/
	if (intHz > 2000000) {
		TOP = (float)(F_CPU) / (float)2000000 - 1.0;
		val = (float)(Duty / 100.0) * (float)TOP;//int(TOP * (Duty / 100) - 1);
		ChangebyteB = 0x19;
	}
	else if (intHz <= 2000000 && intHz >= 300) {
		TOP = (float)(F_CPU) / (float)intHz - 1.0;
		val = (float)(Duty / 100.0) * (float)TOP;//int(TOP * (Duty / 100) - 1);
		ChangebyteB = 0x19;
	}
	else if (intHz < 300 && intHz >= 1) {
		TOP = (float)(15625) / (float)intHz - 1.0;
		val = (float)(Duty / 100.0) * (float)TOP;//int(TOP * (Duty / 100) - 1);
		ChangebyteB = 0x1D;
	}
	else {
		TOP = (float)(15625) - 1.0;
		val = (float)(Duty / 100.0) * (float)TOP;//int(TOP * (Duty / 100) - 1);
		ChangebyteB = 0x1D;
	}
	pinMode(pin, OUTPUT);
	switch (pin)
	{
	case 2:
		if (Pwm2onoff == LOW) {			 //3 Timer B
			TCCR3A |= ((1 << COM3B1) | (1 << WGM31)); TCCR3B = ChangebyteB; ICR3 = TOP;
		}
		else if (Pwm2onoff == HIGH) { TCCR3A &= ~(1 << COM3B1); }
		break;
	case 3:
		if (Pwm3onoff == LOW) {			//3 Timer C
			TCCR3A |= ((1 << COM3C1) | (1 << WGM31)); TCCR3B = ChangebyteB; ICR3 = TOP;
		}
		else if (Pwm3onoff == HIGH) { TCCR3A &= ~(1 << COM3C1); }
		break;
	case 5:
		if (Pwm5onoff == LOW) {			//3 Timer A
			TCCR3A |= ((1 << COM3A1) | (1 << WGM31)); TCCR3B = ChangebyteB; ICR3 = TOP;
		}
		else if (Pwm5onoff == HIGH) { TCCR3A &= ~(1 << COM3A1); }
		break;
	case 6:
		if (Pwm6onoff == LOW) {			//4 Timer A
			TCCR4A |= ((1 << COM4A1) | (1 << WGM41)); TCCR4B = ChangebyteB; ICR4 = TOP;
		}
		else if (Pwm6onoff == HIGH) { TCCR4A &= ~(1 << COM4A1); }
		break;
	case 7:
		if (Pwm7onoff == LOW) {			//4 Timer B
			TCCR4A |= ((1 << COM4B1) | (1 << WGM41)); TCCR4B = ChangebyteB; ICR4 = TOP;
		}
		else if (Pwm7onoff == HIGH) { TCCR4A &= ~(1 << COM4B1); }
		break;
	case 8:
		if (Pwm8onoff == LOW) {			//4 Timer C
			TCCR4A |= ((1 << COM4C1) | (1 << WGM41)); TCCR4B = ChangebyteB; ICR4 = TOP;
		}
		else if (Pwm8onoff == HIGH) { TCCR4A &= ~(1 << COM4C1); }
		break;
	case 11:
		if (Pwm11onoff == LOW) {		//1 Timer A
			TCCR1A |= ((1 << COM1A1) | (1 << WGM11)); TCCR1B = ChangebyteB; ICR1 = TOP;
		}
		else if (Pwm11onoff == HIGH) { TCCR1A &= ~(1 << COM1A1); }
		break;
	case 12:
		if (Pwm12onoff == LOW) {		//1 Timer B
			TCCR1A |= ((1 << COM1B1) | (1 << WGM11)); TCCR1B = ChangebyteB; ICR1 = TOP;
		}
		else if (Pwm12onoff == HIGH) { TCCR1A &= ~(1 << COM1B1); }
		break;
	case 13:
		if (Pwm13onoff == LOW) {		//1 Timer C
			TCCR1A |= ((1 << COM1C1) | (1 << WGM11));; TCCR1B = ChangebyteB; ICR1 = TOP;
		}
		else if (Pwm13onoff == HIGH) { TCCR1A &= ~(1 << COM1C1); }
		break;
	case 44:
		if (Pwm44onoff == LOW) {		//5 Timer C
			TCCR5A |= ((1 << COM5C1) | (1 << WGM51)); TCCR5B = ChangebyteB; ICR5 = TOP;
		}
		else if (Pwm44onoff == HIGH) { TCCR5A &= ~(1 << COM5C1); }
		break;
	case 45:
		if (Pwm45onoff == LOW) {		//5 Timer B
			TCCR5A |= ((1 << COM5B1) | (1 << WGM51)); TCCR5B = ChangebyteB; ICR5 = TOP;
		}
		else if (Pwm45onoff == HIGH) { TCCR5A &= ~(1 << COM5B1); }
		break;
	case 46:
		if (Pwm46onoff == LOW) {		//5 Timer A
			TCCR5A |= ((1 << COM5A1) | (1 << WGM51)); TCCR5B = ChangebyteB; ICR5 = TOP;
		}
		else if (Pwm46onoff == HIGH) { TCCR5A &= ~(1 << COM5A1); }
		break;

	}
	if (intHz <= 0)
	{
		digitalWrite(pin, LOW);
	}
	else
	{
		switch (pin)
		{
		case 2:			//2pin OC3B
			// connect pwm to pin on timer 3, channel B
			if (Pwm2onoff == LOW) {
				sbi(TCCR3A, COM3B1);
				OCR3B = val; // set pwm duty
			}
			break;

		case 3:			//3pin OC3C
			// connect pwm to pin on timer 3, channel C
			if (Pwm3onoff == LOW) {
				sbi(TCCR3A, COM3C1);
				OCR3C = val; // set pwm duty
			}
			break;

		case 5:			//5pin OC3A
			// connect pwm to pin on timer 3, channel A
			if (Pwm5onoff == LOW) {
				sbi(TCCR3A, COM3A1);
				OCR3A = val; // set pwm duty
			}
			break;

		case 6:			//6pin OC4A
			// connect pwm to pin on timer 4, channel A
			if (Pwm6onoff == LOW) {
				sbi(TCCR4A, COM4A1);
				OCR4A = val; // set pwm duty
			}
			break;

		case 7:			//7pin OC4B
			// connect pwm to pin on timer 4, channel B
			if (Pwm7onoff == LOW) {
				sbi(TCCR4A, COM4B1);
				OCR4B = val; // set pwm duty
			}
			break;

		case 8:			//8pin OC4C
			// connect pwm to pin on timer 4, channel C
			if (Pwm8onoff == LOW) {
				sbi(TCCR4A, COM4C1);
				OCR4C = val; // set pwm duty
			}
			break;

		case 11:		//11pin OC1A
			// connect pwm to pin on timer 1, channel A
			if (Pwm11onoff == LOW) {
				sbi(TCCR1A, COM1A1);
				OCR1A = val; // set pwm duty
			}
			break;

		case 12:		//12pin OC1B
			// connect pwm to pin on timer 1, channel B
			if (Pwm12onoff == LOW) {
				sbi(TCCR1A, COM1B1);
				OCR1B = val; // set pwm duty
			}
			break;

		case 13:		//13pin OC1C
			// connect pwm to pin on timer 1, channel C
			if (Pwm13onoff == LOW) {
				sbi(TCCR1A, COM1C1);
				OCR1C = val; // set pwm duty
			}
			break;

		case 44:		//44pin OC5A
			// connect pwm to pin on timer 5, channel C
			if (Pwm44onoff == LOW) {
				sbi(TCCR5A, COM5C1);
				OCR5C = val; // set pwm duty
			}
			break;

		case 45:		//45pin OC5B
			// connect pwm to pin on timer 5, channel B
			if (Pwm45onoff == LOW) {
				sbi(TCCR5A, COM5B1);
				OCR5B = val; // set pwm duty
			}
			break;

		case 46:		//46pin OC5A
			// connect pwm to pin on timer 5, channel A
			if (Pwm46onoff == LOW) {
				sbi(TCCR5A, COM5A1);
				OCR5A = val; // set pwm duty
			}
			break;

		default:
			break;
		}
	}

}
uint16_t tcntPlus1 = 0, tcntPlus3 = 0, tcntPlus4 = 0, tcntPlus5 = 0;
bool tcntsetupon1 = 0, tcntsetupon3 = 0, tcntsetupon4 = 0, tcntsetupon5 = 0;
uint16_t TCNTOUT(uint8_t timernumber) {
	switch (timernumber)
	{
	case 1:
		return tcntPlus1;
		break;
	case 3:
		return tcntPlus3;
		break;
	case 4:
		return tcntPlus4;
		break;
	case 5:
		return tcntPlus5;
		break;
	default:
		break;
	}
}
int PSR(uint8_t channel)//PWM State Read  // pinNumber input
{
	if (channel == 2) {//3TIMER		B
		if ((COM3B1 == 1) || (COM3B0 == 1)) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 3) {//C
		if (COM3C1 == 1 || COM3C0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 5) {//A
		if (COM3A1 == 1 || COM3A0 == 1) {
			return 1;
		}
		else return 0;
	}


	else if (channel == 6) {//4TIMER	A
		if (COM4A1 == 1 || COM4A0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 7) {//B
		if (COM4B1 == 1 || COM4B0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 8) {//C
		if (COM4C1 == 1 || COM4C0 == 1) {
			return 1;
		}
		else return 0;
	}

	else if (channel == 11) {//1TIMER	A
		if (COM1A1 == 1 || COM1A0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 12) {//B
		if (COM1B1 == 1 || COM1B0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 13) {//C
		if (COM1C1 == 1 || COM1C0 == 1) {
			return 1;
		}
		else return 0;
	}

	else if (channel == 44) {//5TIMER	C
		if (COM5C1 == 1 || COM5C0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 45) {//B
		if (COM5B1 == 1 || COM5B0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 46) {//A
		if (COM5A1 == 1 || COM5A0 == 1) {
			return 1;
		}
		else return 0;
	}
}
void PSR_FREQ(uint8_t channel)//PSR_FREQ State Read
{
}
void PSR_DUTY(uint8_t channel)//PSR_DUTY State Read
{
}

void TCNTSETUP(uint8_t timerNumber, bool on32bit)
{
	switch (timerNumber)
	{
	case 1:
		TIMSK1 = 0x00; TCCR1A = 0x00; TCCR1B = 0x07; TCNT1 = 0x00;
		if (on32bit == HIGH) { TIMSK1 |= (1 << TOIE1);	tcntsetupon1 = 1; }
		break;

	case 3:
		TIMSK3 = 0x00; TCCR3A = 0x00; TCCR3B = 0x07; TCNT3 = 0x00;
		if (on32bit == HIGH) { TIMSK3 |= (1 << TOIE3);	tcntsetupon3 = 1; }
		break;

	case 4:
		TIMSK4 = 0x00; TCCR4A = 0x00; TCCR4B = 0x07; TCNT4 = 0x00;
		if (on32bit == HIGH) { TIMSK4 |= (1 << TOIE4);	tcntsetupon4 = 1; }
		break;

	case 5:
		TIMSK5 = 0x00; TCCR5A = 0x00; TCCR5B = 0x07; TCNT5 = 0x00;
		if (on32bit == HIGH) { TIMSK5 |= (1 << TOIE5);	tcntsetupon5 = 1; }
		break;

	default:
		break;
	}
}

#endif

#if PWM111 == 2

uint16_t tcntPlus1 = 0, tcntPlus3 = 0;
bool tcntsetupon1 = 0, tcntsetupon3 = 0;
bool Pwm21onoff = 0, Pwm22onoff = 0, Pwm23onoff = 0;
bool Pwm26onoff = 0, Pwm27onoff = 0, Pwm28onoff = 0;
void PWM_RESET() {
	TCCR1A = 0x00;
	TCCR1B = 0x00;
	TCNT1 = 0x00;
	TCCR3A = 0x00;
	TCCR3B = 0x00;
	TCNT3 = 0x00;
}
void PWMOFF(uint8_t pin, bool POff) {
	switch (pin)
	{
	case 21:
		if (POff == HIGH) { Pwm21onoff = 1; }
		else if (POff == LOW) { Pwm21onoff = 0; }
		break;
	case 22:
		if (POff == HIGH) { Pwm22onoff = 1; }
		else if (POff == LOW) { Pwm22onoff = 0; }
		break;
	case 23:
		if (POff == HIGH) { Pwm23onoff = 1; }
		else if (POff == LOW) { Pwm23onoff = 0; }
		break;
	case 26:
		if (POff == HIGH) { Pwm26onoff = 1; }
		else if (POff == LOW) { Pwm26onoff = 0; }
		break;
	case 27:
		if (POff == HIGH) { Pwm27onoff = 1; }
		else if (POff == LOW) { Pwm27onoff = 0; }
		break;
	case 28:
		if (POff == HIGH) { Pwm28onoff = 1; }
		else if (POff == LOW) { Pwm28onoff = 0; }
		break;
	}
}

void PWM(uint8_t pin, uint16_t val, bool onDutybit16)
{
	pinMode(pin, OUTPUT);
	switch (pin)
	{
	case 21:
		if (Pwm21onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR3A |= ((1 << COM3A1) | (1 << WGM31)); TCCR3B |= ((1 << WGM33) | (1 << WGM32) | (1 << CS30)); ICR3 = 65535; }
			else { TCCR3A |= (1 << WGM30); TCCR3B |= ((1 << CS31) | (1 << CS30)); }
		}
		else if (Pwm21onoff == HIGH) { TCCR3A &= ~(1 << COM3A1); }
		break;
	case 22:
		if (Pwm22onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR3A |= ((1 << COM3B1) | (1 << WGM31)); TCCR3B |= ((1 << WGM33) | (1 << WGM32) | (1 << CS30));	ICR3 = 65535; }
			else { TCCR3A |= (1 << WGM30); TCCR3B |= ((1 << CS31) | (1 << CS30)); }
		}
		else if (Pwm22onoff == HIGH) { TCCR3A &= ~(1 << COM3B1); }
		break;
	case 23:
		if (Pwm23onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR3A |= ((1 << COM3C1) | (1 << WGM31)); TCCR3B |= ((1 << WGM33) | (1 << WGM32) | (1 << CS30)); ICR3 = 65535; }
			else { TCCR3A |= (1 << WGM30); TCCR3B |= ((1 << CS31) | (1 << CS30)); }
		}
		else if (Pwm23onoff == HIGH) { TCCR3A &= ~(1 << COM3C1); }
		break;
	case 26:
		if (Pwm26onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR1A |= ((1 << COM1A1) | (1 << WGM11)); TCCR1B |= ((1 << WGM13) | (1 << WGM12) | (1 << CS10)); ICR1 = 65535; }
			else { TCCR1A |= (1 << WGM10); TCCR1B |= ((1 << CS11) | (1 << CS10)); }
		}
		else if (Pwm26onoff == HIGH) { TCCR1A &= ~(1 << COM1A1); }
		break;
	case 27:
		if (Pwm27onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR1A |= ((1 << COM1B1) | (1 << WGM11)); TCCR1B |= ((1 << WGM13) | (1 << WGM12) | (1 << CS10)); ICR1 = 65535; }
			else { TCCR1A |= (1 << WGM10); TCCR1B |= ((1 << CS11) | (1 << CS10)); }
		}
		else if (Pwm27onoff == HIGH) { TCCR1A &= ~(1 << COM1B1); }
		break;
	case 28:
		if (Pwm28onoff == LOW) {
			if (onDutybit16 == HIGH) { TCCR1A |= ((1 << COM1C1) | (1 << WGM11)); TCCR1B |= ((1 << WGM13) | (1 << WGM12) | (1 << CS10)); ICR1 = 65535; }
			else { TCCR1A |= (1 << WGM10); TCCR1B |= ((1 << CS11) | (1 << CS10)); }
		}
		else if (Pwm28onoff == HIGH) { TCCR1A &= ~(1 << COM1C1); }
		break;
	}
	if (val <= 0)
	{
		digitalWrite(pin, LOW);
	}
	else
	{
		switch (pin)
		{
		case 21:			//5pin OC3A
			// connect pwm to pin on timer 3, channel A
			if (Pwm21onoff == LOW) {
				sbi(TCCR3A, COM3A1);
				OCR3A = val; // set pwm duty
			}
			break;
		case 22:			//2pin OC3B
			// connect pwm to pin on timer 3, channel B
			if (Pwm22onoff == LOW) {
				sbi(TCCR3A, COM3B1);
				OCR3B = val; // set pwm duty
			}
			break;

		case 23:			//3pin OC3C
			// connect pwm to pin on timer 3, channel C
			if (Pwm23onoff == LOW) {
				sbi(TCCR3A, COM3C1);
				OCR3C = val; // set pwm duty
			}
			break;

		case 26:		//26pin OC1A
			// connect pwm to pin on timer 1, channel A
			if (Pwm26onoff == LOW) {
				sbi(TCCR1A, COM1A1);
				OCR1A = val; // set pwm duty
			}
			break;

		case 27:		//27pin OC1B
			// connect pwm to pin on timer 1, channel B
			if (Pwm27onoff == LOW) {
				sbi(TCCR1A, COM1B1);
				OCR1B = val; // set pwm duty
			}
			break;

		case 28:		//28pin OC1C
			// connect pwm to pin on timer 1, channel C
			if (Pwm28onoff == LOW) {
				sbi(TCCR1A, COM1C1);
				OCR1C = val; // set pwm duty
			}
			break;

		default:
			break;
		}
	}
}



bool npwmTccrb = 0;
uint32_t npwmtop = 0;
uint32_t npwmval = 0;
bool oneplus26 = 1, oneplus27 = 1, oneplus28 = 1;
bool npwmON26 = 0, npwmON27 = 0, npwmON28 = 0;
bool npwm26ON = 0, npwm27ON = 0, npwm28ON = 0;
uint32_t N1A = 0;
uint32_t N1B = 0;
uint32_t N1C = 0;
bool npwmTccrb1A = 0;
bool npwmTccrb1B = 0;
bool npwmTccrb1C = 0;
uint16_t TimerCount1A = 0;
uint16_t TimerCount1B = 0;
uint16_t TimerCount1C = 0;

bool oneplus21 = 1, oneplus22 = 1, oneplus23 = 1;
bool npwmON21 = 0, npwmON22 = 0, npwmON23 = 0;
bool npwm21ON = 0, npwm22ON = 0, npwm23ON = 0;
uint32_t N3A = 0;
uint32_t N3B = 0;
uint32_t N3C = 0;
bool npwmTccrb3A = 0;
bool npwmTccrb3B = 0;
bool npwmTccrb3C = 0;
uint16_t TimerCount3A = 0;
uint16_t TimerCount3B = 0;
uint16_t TimerCount3C = 0;

void NPWM_BEGIN(uint8_t pin, uint32_t intHz, float Duty, uint32_t N) {
	if (intHz > 2000000) {
		npwmtop = (float)(F_CPU) / (float)2000000 - 1.0;
		npwmval = (float)(Duty / 100.0) * (float)npwmtop;//int(TOP * (Duty / 100) - 1);
		npwmTccrb = 0;
	}
	else if (intHz <= 2000000 && intHz >= 300) {
		npwmtop = (float)(F_CPU) / (float)intHz - 1.0;
		npwmval = (float)(Duty / 100.0) * (float)npwmtop;//int(TOP * (Duty / 100) - 1);
		npwmTccrb = 0;
	}
	else if (intHz < 300 && intHz >= 1) {
		npwmtop = (float)(15625) / (float)intHz - 1.0;
		npwmval = (float)(Duty / 100.0) * (float)npwmtop;//int(TOP * (Duty / 100) - 1);
		npwmTccrb = 1;
	}
	else {
		npwmtop = (float)(15625) - 1.0;
		npwmval = (float)(Duty / 100.0) * (float)npwmtop;//int(TOP * (Duty / 100) - 1);
		npwmTccrb = 1;
	}
	pinMode(pin, OUTPUT);
	switch (pin)
	{
	case 26://A		//TIMER1
		TIMSK |= (1 << TOIE1);
		TCCR1A |= (1 << WGM11);
		TCCR1B |= (1 << WGM13) | (1 << WGM12);
		ICR1 = npwmtop;
		OCR1A = npwmtop - npwmval;
		N1A = N;
		npwmTccrb1A = npwmTccrb;
		TimerCount1A = 0;
		npwm26ON = 1;
		break;
	case 27://B
		TIMSK |= (1 << TOIE1);
		TCCR1A |= (1 << WGM11);
		TCCR1B |= (1 << WGM13) | (1 << WGM12);
		ICR1 = npwmtop;
		OCR1B = npwmtop - npwmval;
		N1B = N;
		npwmTccrb1B = npwmTccrb;
		TimerCount1B = 0;
		npwm27ON = 1;
		break;
	case 28://C
		TIMSK |= (1 << TOIE1);
		TCCR1A |= (1 << WGM11);
		TCCR1B |= (1 << WGM13) | (1 << WGM12);
		ICR1 = npwmtop;
		OCR1C = npwmtop - npwmval;
		N1C = N;
		npwmTccrb1C = npwmTccrb;
		TimerCount1C = 0;
		npwm28ON = 1;
		break;
	case 21://A
		ETIMSK |= (1 << TOIE3);
		TCCR3A |= (1 << WGM31);
		TCCR3B |= (1 << WGM33) | (1 << WGM32);
		ICR3 = npwmtop;
		OCR3A = npwmtop - npwmval;
		N3A = N;
		npwmTccrb3A = npwmTccrb;
		TimerCount3A = 0;
		npwm21ON = 1;
		break;
	case 22://B		//TIMER3
		ETIMSK |= (1 << TOIE3);
		TCCR3A |= (1 << WGM31);
		TCCR3B |= (1 << WGM33) | (1 << WGM32);
		ICR3 = npwmtop;
		OCR3B = npwmtop - npwmval;
		N3B = N;
		npwmTccrb3B = npwmTccrb;
		TimerCount3B = 0;
		npwm22ON = 1;
		break;
	case 23://C
		ETIMSK |= (1 << TOIE3);
		TCCR3A |= (1 << WGM31);
		TCCR3B |= (1 << WGM33) | (1 << WGM32);
		ICR3 = npwmtop;
		OCR3C = npwmtop - npwmval;
		N3C = N;
		npwmTccrb3C = npwmTccrb;
		TimerCount3C = 0;
		npwm23ON = 1;
		break;
	default:
		break;
	}
}

void NPWM(uint8_t pin) {

	if (npwm21ON == 1) {
		if (pin == 21) {
			if (oneplus21 == 1) {
				if (TimerCount3A == 0) {
					//pinMode(2, INPUT);
					DDRE &= ~(1 << 3);
				}
				else {
					DDRE |= (1 << 3);
					oneplus21 = 0;
					N3A++;
				}
				npwmON21 = 1;
				TCCR3A |= (1 << COM3A1) | (1 << COM3A0);
				if (npwmTccrb3C == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3C == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			if (TimerCount3A < N3A) {
				npwmON21 = 1;
				TCCR3A |= (1 << COM3A1) | (1 << COM3A0);
				if (npwmTccrb3A == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3A == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			else if (TimerCount3A >= N3A) {
				TCCR3A &= ~((1 << COM3A1) | (1 << COM3A0));
				TCCR3B &= ~((1 << CS32) | (1 << CS30));
				npwmON21 = 0;
				npwm21ON = 0;

				if ((npwm22ON == 1) | (npwm23ON == 1)) {}
				else {
					TCNT3 = 0x00;
				}
			}
		}
	}if (npwm22ON == 1) {
		if (pin == 22) {
			if (oneplus22 == 1) {
				if (TimerCount3B == 0) {
					//pinMode(2, INPUT);
					DDRE &= ~(1 << 4);
				}
				else {
					DDRE |= (1 << 4);
					oneplus22 = 0;
					N3B++;
				}
				npwmON22 = 1;
				TCCR3A |= (1 << COM3B1) | (1 << COM3B0);
				if (npwmTccrb3B == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3B == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			if (TimerCount3B < N3B) {
				npwmON22 = 1;
				TCCR3A |= (1 << COM3B1) | (1 << COM3B0);
				if (npwmTccrb3B == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3B == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			else if (TimerCount3B >= N3B) {
				TCCR3A &= ~((1 << COM3B1) | (1 << COM3B0));
				TCCR3B &= ~((1 << CS32) | (1 << CS30));
				npwmON22 = 0;
				npwm22ON = 0;
				if ((npwm23ON == 1) | (npwm21ON == 1)) {}
				else {
					TCNT3 = 0x00;
				}
			}
		}
	}if (npwm23ON == 1) {
		if (pin == 23) {
			if (oneplus23 == 1) {
				if (TimerCount3C == 0) {
					//pinMode(2, INPUT);
					DDRE &= ~(1 << 5);
				}
				else {
					DDRE |= (1 << 5);
					oneplus23 = 0;
					N3C++;
				}
				npwmON23 = 1;
				TCCR3A |= (1 << COM3C1) | (1 << COM3C0);
				if (npwmTccrb3C == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3C == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			if (TimerCount3C < N3C) {
				npwmON23 = 1;
				TCCR3A |= (1 << COM3C1) | (1 << COM3C0);
				if (npwmTccrb3C == 0) {
					TCCR3B |= (1 << CS30);
				}
				else if (npwmTccrb3C == 1) {
					TCCR3B |= (1 << CS32) | (1 << CS30);
				}
			}
			else if (TimerCount3C >= N3C) {
				TCCR3A &= ~((1 << COM3C1) | (1 << COM3C0));
				TCCR3B &= ~((1 << CS32) | (1 << CS30));
				npwmON23 = 0;
				npwm23ON = 0;

				if ((npwm21ON == 1) | (npwm22ON == 1)) {}
				else {
					TCNT3 = 0x00;
				}
			}
		}
	}if (npwm26ON == 1) {
		if (pin == 26) {
			if (oneplus26 == 1) {
				if (TimerCount1A == 0) {
					//pinMode(2, INPUT);
					DDRB &= ~(1 << 5);
				}
				else {
					DDRB |= (1 << 5);
					oneplus26 = 0;
					N1A++;
				}
				npwmON26 = 1;
				TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
				if (npwmTccrb1A == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1A == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			if (TimerCount1A < N1A) {
				npwmON26 = 1;
				TCCR1A |= (1 << COM1A1) | (1 << COM1A0);
				if (npwmTccrb1A == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1A == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			else if (TimerCount1A >= N1A) {
				TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0));
				TCCR1B &= ~((1 << CS12) | (1 << CS10));
				npwmON26 = 0;
				npwm26ON = 0;
				if ((npwm27ON == 1) | (npwm28ON == 1)) {}
				else {
					TCNT1 = 0x00;
				}
			}
		}
	}if (npwm27ON == 1) {
		if (pin == 27) {
			if (oneplus27 == 1) {
				if (TimerCount1B == 0) {
					//pinMode(2, INPUT);
					DDRB &= ~(1 << 6);
				}
				else {
					DDRB |= (1 << 6);
					oneplus27 = 0;
					N1B++;
				}
				npwmON27 = 1;
				TCCR1A |= (1 << COM1B1) | (1 << COM1B0);
				if (npwmTccrb1B == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1B == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			if (TimerCount1B < N1B) {
				npwmON27 = 1;
				TCCR1A |= (1 << COM1B1) | (1 << COM1B0);
				if (npwmTccrb1B == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1B == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			else if (TimerCount1B >= N1B) {
				TCCR1A &= ~((1 << COM1B1) | (1 << COM1B0));
				TCCR1B &= ~((1 << CS12) | (1 << CS10));
				npwmON27 = 0;
				npwm27ON = 0;
				if ((npwm26ON == 1) | (npwm28ON == 1)) {}
				else {
					TCNT1 = 0x00;
				}
			}
		}
	}if (npwm28ON == 1) {
		if (pin == 28) {
			if (oneplus28 == 1) {
				if (TimerCount1C == 0) {
					//pinMode(2, INPUT);
					DDRB &= ~(1 << 7);
				}
				else {
					DDRB |= (1 << 7);
					oneplus28 = 0;
					N1C++;
				}
				npwmON28 = 1;
				TCCR1A |= (1 << COM1C1) | (1 << COM1C0);
				if (npwmTccrb1C == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1C == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			if (TimerCount1C < N1C) {
				npwmON28 = 1;
				TCCR1A |= (1 << COM1C1) | (1 << COM1C0);
				if (npwmTccrb1C == 0) {
					TCCR1B |= (1 << CS10);
				}
				else if (npwmTccrb1C == 1) {
					TCCR1B |= (1 << CS12) | (1 << CS10);
				}
			}
			else if (TimerCount1C >= N1C) {
				TCCR1A &= ~((1 << COM1C1) | (1 << COM1C0));
				TCCR1B &= ~((1 << CS12) | (1 << CS10));
				npwmON28 = 0;
				npwm28ON = 0;
				if ((npwm27ON == 1) | (npwm26ON == 1)) {}
				else {
					TCNT1 = 0x00;
				}
			}
		}
	}
}


void FDPWM(uint8_t pin, int32_t intHz, float Duty)
{
	int val = 0;
	uint32_t TOP = 0;
	byte ChangebyteB = 0;
	if (intHz > 2000000) {
		TOP = (float)(F_CPU) / (float)2000000 - 1.0;
		val = (float)(Duty / 100.0) * (float)TOP;//int(TOP * (Duty / 100) - 1);
		ChangebyteB = 0x19;
	}
	else if (intHz <= 2000000 && intHz >= 300) {
		TOP = (float)(F_CPU) / (float)intHz - 1.0;
		val = (float)(Duty / 100.0) * (float)TOP;//int(TOP * (Duty / 100) - 1);
		ChangebyteB = 0x19;
	}
	else if (intHz < 300 && intHz >= 1) {
		TOP = (float)(15625) / (float)intHz - 1.0;
		val = (float)(Duty / 100.0) * (float)TOP;//int(TOP * (Duty / 100) - 1);
		ChangebyteB = 0x1D;
	}
	else {
		TOP = (float)(15625) - 1.0;
		val = (float)(Duty / 100.0) * (float)TOP;//int(TOP * (Duty / 100) - 1);
		ChangebyteB = 0x1D;
	}
	pinMode(pin, OUTPUT);
	switch (pin)
	{
	case 21:
		if (Pwm21onoff == LOW) {			//3 Timer A
			TCCR3A |= ((1 << COM3A1) | (1 << WGM31)); TCCR3B = ChangebyteB; ICR3 = TOP;
		}
		else if (Pwm21onoff == HIGH) { TCCR3A &= ~(1 << COM3A1); }
		break;
	case 22:
		if (Pwm22onoff == LOW) {			 //3 Timer B
			TCCR3A |= ((1 << COM3B1) | (1 << WGM31)); TCCR3B = ChangebyteB; ICR3 = TOP;
		}
		else if (Pwm22onoff == HIGH) { TCCR3A &= ~(1 << COM3B1); }
		break;
	case 23:
		if (Pwm23onoff == LOW) {			//3 Timer C
			TCCR3A |= ((1 << COM3C1) | (1 << WGM31)); TCCR3B = ChangebyteB; ICR3 = TOP;
		}
		else if (Pwm23onoff == HIGH) { TCCR3A &= ~(1 << COM3C1); }
		break;
	case 26:
		if (Pwm26onoff == LOW) {		//1 Timer A
			TCCR1A |= ((1 << COM1A1) | (1 << WGM11)); TCCR1B = ChangebyteB; ICR1 = TOP;
		}
		else if (Pwm26onoff == HIGH) { TCCR1A &= ~(1 << COM1A1); }
		break;
	case 27:
		if (Pwm27onoff == LOW) {		//1 Timer B
			TCCR1A |= ((1 << COM1B1) | (1 << WGM11)); TCCR1B = ChangebyteB; ICR1 = TOP;
		}
		else if (Pwm27onoff == HIGH) { TCCR1A &= ~(1 << COM1B1); }
		break;
	case 28:
		if (Pwm28onoff == LOW) {		//1 Timer C
			TCCR1A |= ((1 << COM1C1) | (1 << WGM11));; TCCR1B = ChangebyteB; ICR1 = TOP;
		}
		else if (Pwm28onoff == HIGH) { TCCR1A &= ~(1 << COM1C1); }
		break;
	}
	if (intHz <= 0)
	{
		digitalWrite(pin, LOW);
	}
	else
	{
		switch (pin)
		{
		case 21:			//5pin OC3A
			// connect pwm to pin on timer 3, channel A
			if (Pwm21onoff == LOW) {
				sbi(TCCR3A, COM3A1);
				OCR3A = val; // set pwm duty
			}
			break;

		case 22:			//2pin OC3B
			// connect pwm to pin on timer 3, channel B
			if (Pwm22onoff == LOW) {
				sbi(TCCR3A, COM3B1);
				OCR3B = val; // set pwm duty
			}
			break;

		case 23:			//3pin OC3C
			// connect pwm to pin on timer 3, channel C
			if (Pwm23onoff == LOW) {
				sbi(TCCR3A, COM3C1);
				OCR3C = val; // set pwm duty
			}
			break;

		case 26:		//11pin OC1A
			// connect pwm to pin on timer 1, channel A
			if (Pwm26onoff == LOW) {
				sbi(TCCR1A, COM1A1);
				OCR1A = val; // set pwm duty
			}
			break;

		case 27:		//12pin OC1B
			// connect pwm to pin on timer 1, channel B
			if (Pwm27onoff == LOW) {
				sbi(TCCR1A, COM1B1);
				OCR1B = val; // set pwm duty
			}
			break;

		case 28:		//13pin OC1C
			// connect pwm to pin on timer 1, channel C
			if (Pwm28onoff == LOW) {
				sbi(TCCR1A, COM1C1);
				OCR1C = val; // set pwm duty
			}
			break;

		default:
			break;
		}
	}

}
uint16_t TCNTOUT(uint8_t timernumber) {
	switch (timernumber)
	{
	case 1:
		return tcntPlus1;
		break;
	case 3:
		return tcntPlus3;
		break;
	default:
		break;
	}
}
int PSR(uint8_t channel)//PWM State Read  // pinNumber input
{
	if (channel == 21) {//A
		if (COM3A1 == 1 || COM3A0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 22) {//3TIMER		B
		if ((COM3B1 == 1) || (COM3B0 == 1)) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 23) {//C
		if (COM3C1 == 1 || COM3C0 == 1) {
			return 1;
		}
		else return 0;
	}
	
	else if (channel == 26) {//1TIMER	A
		if (COM1A1 == 1 || COM1A0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 27) {//B
		if (COM1B1 == 1 || COM1B0 == 1) {
			return 1;
		}
		else return 0;
	}
	else if (channel == 28) {//C
		if (COM1C1 == 1 || COM1C0 == 1) {
			return 1;
		}
		else return 0;
	}
}
void PSR_FREQ(uint8_t channel)//PSR_FREQ State Read
{
}
void PSR_DUTY(uint8_t channel)//PSR_DUTY State Read
{
}

void TCNTSETUP(uint8_t timerNumber, bool on32bit)
{
	switch (timerNumber)
	{
	case 3:
		ETIMSK = 0x00; TCCR3A = 0x00; TCCR3B = 0x07; TCNT3 = 0x00;
		if (on32bit == HIGH) { ETIMSK |= (1 << TOIE3);	tcntsetupon3 = 1; }
		break;

	default:
		break;
	}
}

#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////		IDAC			IDAC			IDAC	      ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#if IDAC1==1

IDAC::IDAC(uint8_t dacNum) {
	_dacNum = dacNum;
	
}
void IDAC::swap_maxmin(uint32_t *maxvalues, uint32_t *minvalues) {
	uint32_t temp;
	if (*minvalues > *maxvalues) {
		temp = *maxvalues;
		*maxvalues = *minvalues;
		*minvalues = temp;
	}
}
/*
1~4SELLECT
1:TIMER1 => 11, 12, 13
2:TIMER5 => 44, 45, 46
3:TIMER3 =>  2,  3,  5
4:TIMER4 =>  6,  7,  8
*/
void IDAC::INIT(uint8_t channel1, uint32_t maxvalue65, uint32_t minvalue00, bool Mode) {
	uint32_t _maxvalue = maxvalue65;
	uint32_t _minvalue = minvalue00;
	bool _Mode = Mode;
	uint8_t _channel = channel1;
	swap_maxmin(&_maxvalue, &_minvalue);
	switch (_dacNum)
	{
		//B5
	case 1:
		//		TCCR1A = 0xAA; TCCR1B = 0x1A; ICR1 = 65535;
		TCCR1A = 0;
		TCCR1A |= (1 << WGM11);
		TCCR1B = 0x19;
		ICR1 = 65535;
		switch (_channel)
		{
		case 0:
			_maxvalue11 = _maxvalue;
			_minvalue11 = _minvalue;
			_Mode11 = _Mode;
			pinMode(11, OUTPUT);
			break;
		case 1:
			_maxvalue12 = _maxvalue;
			_minvalue12 = _minvalue;
			_Mode12 = _Mode;
			pinMode(12, OUTPUT);
			break;
		case 2:
			_maxvalue13 = _maxvalue;
			_minvalue13 = _minvalue;
			_Mode13 = _Mode;
			pinMode(13, OUTPUT);
			break;
		default:
			break;
		}
		break;
	case 2:
		switch (_channel)
		{
		case 0:
			_maxvalue44 = _maxvalue;
			_minvalue44 = _minvalue;
			_Mode44 = _Mode;
			pinMode(44, OUTPUT);
			break;
		case 1:
			_maxvalue45 = _maxvalue;
			_minvalue45 = _minvalue;
			_Mode45 = _Mode;
			pinMode(45, OUTPUT);
			break;
		case 2:
			_maxvalue46 = _maxvalue;
			_minvalue46 = _minvalue;
			_Mode46 = _Mode;
			pinMode(46, OUTPUT);
			break;
		default:
			break;
		}
		//		TCCR5A = 0xAA; TCCR5B = 0x1A; ICR5 = 65535;
		TCCR5A = 0;
		TCCR5A |= (1 << WGM51);
		TCCR5B = 0x19;
		ICR5 = 65535;
		break;

		//B4
	case 3:
		switch (_channel)
		{
		case 1:
			_maxvalue2 = _maxvalue;
			_minvalue2 = _minvalue;
			_Mode2 = _Mode;
			pinMode(2, OUTPUT);
			break;
		case 2:
			_maxvalue3 = _maxvalue;
			_minvalue3 = _minvalue;
			_Mode3 = _Mode;
			pinMode(3, OUTPUT);
			break;
		case 0:
			_maxvalue5 = _maxvalue;
			_minvalue5 = _minvalue;
			_Mode5 = _Mode;
			pinMode(5, OUTPUT);
			break;
		default:
			break;
		}
		//		TCCR3A = 0xAA; TCCR3B = 0x1A; ICR3 = 65535;
		TCCR3A = 0;
		TCCR3A |= (1 << WGM31);
		TCCR3B = 0x19;
		ICR3 = 65535;
		break;
	case 4:
		switch (_channel)
		{
		case 0:
		case 6:
			_maxvalue6 = _maxvalue;
			_minvalue6 = _minvalue;
			_Mode6 = _Mode;
			pinMode(6, OUTPUT);
			break;
		case 1:
		case 7:
			_maxvalue7 = _maxvalue;
			_minvalue7 = _minvalue;
			_Mode7 = _Mode;
			pinMode(7, OUTPUT);
			break;
		case 2:
		case 8:
			_maxvalue8 = _maxvalue;
			_minvalue8 = _minvalue;
			_Mode8 = _Mode;
			pinMode(8, OUTPUT);
			break;
		default:
			break;
		}
		//		TCCR4A = 0xAA; TCCR4B = 0x1A; ICR4 = 65535;
		TCCR4A = 0;
		TCCR4A |= (1 << WGM41);
		TCCR4B = 0x19;
		ICR4 = 65535;
		break;
	default:
		break;
	}

}
/*
1~4SELLECT
1:TIMER1 => 11, 12, 13
2:TIMER5 => 44, 45, 46
3:TIMER3 =>  2,  3,  5
4:TIMER4 =>  6,  7,  8
*/
void IDAC::DACOUT(uint8_t channel2, uint32_t inputValue) {
	uint8_t __channel = channel2;
	//_val = (float)((inputValue-_minvalue)) * (float)(65535.0-0) / float(_maxvalue-_minvalue)+0;//0.5 반올림
	/*_val1 = map(inputValue, _minvalue, _maxvalue , 0, 65535);*/

	//_val = (float)((inputValue-_minvalue)) * (float)(52428.0) / float(_maxvalue-_minvalue)+13107.0;//0.5 반올림
	/*_val1 = map(inputValue, _minvalue, _maxvalue , 13107, 65535);*/

	
/*	long map(long x, long in_min, long in_max, long out_min, long out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}*/

	switch (_dacNum)
	{
	case 1:
		switch (__channel)
		{
			//dacNum 1
		case 0:
			sbi(TCCR1A, COM1A1);
			if (_Mode11 == 0) {
				OCR1A = (float)(inputValue - _minvalue11) * (float)(65535.0 - 0) / float(_maxvalue11 - _minvalue11) + 0; // set pwm duty
			}
			else
			{
				OCR1A = (float)(inputValue - _minvalue11) * (float)(65535.0 - 13107.0) / float(_maxvalue11 - _minvalue11) + 13107.0; // set pwm duty
			}
			break;
		case 1:
			sbi(TCCR1A, COM1B1);
			if (_Mode12 == 0) {
				OCR1B = (float)(inputValue - _minvalue12) * (float)(65535.0) / float(_maxvalue12 - _minvalue12) + 0; // set pwm duty
			}
			else
			{
				OCR1B = (float)(inputValue - _minvalue12) * (float)(65535.0 - 13107.0) / float(_maxvalue12 - _minvalue12) + 13107.0; // set pwm duty
			}
			break;
		case 2:
			sbi(TCCR1A, COM1C1);
			if (_Mode13 == 0) {
				OCR1C = (float)(inputValue - _minvalue13) * (float)(65535.0) / float(_maxvalue13 - _minvalue13) + 0; // set pwm duty
			}
			else
			{
				OCR1C = (float)(inputValue - _minvalue13) * (float)(65535.0 - 13107.0) / float(_maxvalue13 - _minvalue13) + 13107.0; // set pwm duty
			}
			break;
		default:
			break;
		}
		break;

	case 2:
		switch (__channel)
		{
			//dacNum 4
		case 0:
			sbi(TCCR5A, COM5A1);
			if (_Mode44 == 0) {
				OCR5A = (float)(inputValue - _minvalue46) * (float)(65535.0 - 0) / float(_maxvalue46 - _minvalue46) + 0; // set pwm duty
			}
			else
			{
				OCR5A = (float)(inputValue - _minvalue46) * (float)(65535.0 - 13107.0) / float(_maxvalue46 - _minvalue46) + 13107.0; // set pwm duty
			}
			break;
		case 1:
			sbi(TCCR5A, COM5B1);
			if (_Mode45 == 0) {
				OCR5B = (float)(inputValue - _minvalue45) * (float)(65535.0 - 0) / float(_maxvalue45 - _minvalue45) + 0; // set pwm duty
			}
			else
			{
				OCR5B = (float)(inputValue - _minvalue45) * (float)(65535.0 - 13107.0) / float(_maxvalue45 - _minvalue45) + 13107.0; // set pwm duty
			}
			break;
		case 2:
			sbi(TCCR5A, COM5C1);
			if (_Mode46 == 0) {
				OCR5C = (float)(inputValue - _minvalue44) * (float)(65535.0 - 0) / float(_maxvalue44 - _minvalue44) + 0; // set pwm duty
			}
			else
			{
				OCR5C = (float)(inputValue - _minvalue44) * (float)(65535.0 - 13107.0) / float(_maxvalue44 - _minvalue44) + 13107.0; // set pwm duty
			}
			break;
		default:
			break;
		}
		break;

	case 3:
		switch (__channel)
		{
			//dacNum 1
		case 0:
			sbi(TCCR3A, COM3A1);
			if (_Mode5 == 0) {
				OCR3A = (float)(inputValue - _minvalue5) * (float)(65535.0 - 0) / float(_maxvalue5 - _minvalue5) + 0; // set pwm duty
			}
			else
			{
				OCR3A = (float)(inputValue - _minvalue5) * (float)(65535.0 - 13107.0) / float(_maxvalue5 - _minvalue5) + 13107.0; // set pwm duty
			}
			break;
		case 1:
			sbi(TCCR3A, COM3B1);
			if (_Mode2 == 0) {
				OCR3B = (float)(inputValue - _minvalue2) * (float)(65535.0 - 0) / float(_maxvalue2 - _minvalue2) + 0; // set pwm duty
			}
			else
			{
				OCR3B = (float)(inputValue - _minvalue2) * (float)(65535.0 - 13107.0) / float(_maxvalue2 - _minvalue2) + 13107.0; // set pwm duty
			}
			break;
		case 2:
			sbi(TCCR3A, COM3C1);
			if (_Mode3 == 0) {
				OCR3C = (float)(inputValue - _minvalue3) * (float)(65535.0 - 0) / float(_maxvalue3 - _minvalue3) + 0; // set pwm duty
			}
			else
			{
				OCR3C = (float)(inputValue - _minvalue3) * (float)(65535.0 - 13107.0) / float(_maxvalue3 - _minvalue3) + 13107.0; // set pwm duty
			}
			break;
		default:
			break;
		}
		break;

	case 4:
		switch (__channel)
		{
			//dacNum 4
		case 0:
		case 6:
			sbi(TCCR4A, COM4A1);
			if (_Mode6 == 0) {
				OCR4A = (float)(inputValue - _minvalue6) * (float)(65535.0 - 0) / float(_maxvalue6 - _minvalue6) + 0; // set pwm duty
			}
			else
			{
				OCR4A = (float)(inputValue - _minvalue6) * (float)(65535.0 - 13107.0) / float(_maxvalue6 - _minvalue6) + 13107.0; // set pwm duty
			}
			break;
		case 1:
		case 7:
			sbi(TCCR4A, COM4B1);
			if (_Mode7 == 0) {
				OCR4B = (float)(inputValue - _minvalue7) * (float)(65535.0 - 0) / float(_maxvalue7 - _minvalue7) + 0; // set pwm duty
			}
			else
			{
				OCR4B = (float)(inputValue - _minvalue7) * (float)(65535.0 - 13107.0) / float(_maxvalue7 - _minvalue7) + 13107.0; // set pwm duty
			}
			break;
		case 2:
			sbi(TCCR4A, COM4C1);
			if (_Mode8 == 0) {
				OCR4C = (float)(inputValue - _minvalue8) * (float)(65535.0 - 0) / float(_maxvalue8 - _minvalue8) + 0; // set pwm duty
			}
			else
			{
				OCR4C = (float)(inputValue - _minvalue8) * (float)(65535.0 - 13107.0) / float(_maxvalue8 - _minvalue8) + 13107.0; // set pwm duty
			}
			break;
		default:
			break;
		}
		break;

	default:
		break;
	}
	
}


#endif // IDAC1

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////		IADC			IADC			IADC	      ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#if IADC111==1

/**
 * Constructor of the class
 * @param io_pin_cs a byte indicating the pin to be use as the chip select pin (CS)
 */
IADC::IADC(uint8_t io_pin_cs) {
	boardModel = 0;
	if (io_pin_cs >= 40) {
		cs = io_pin_cs;
	}
	switch (io_pin_cs)
	{
	case 0:
		boardModel = 1;
		break;
	case 1:
		cs = 53;
		break;
	case 2:
		cs = 39;
		break;
	case 3:
		cs = 40;
		break;
	case 4:
		cs = 41;
		break;
	case 5:
		cs = 42;
		break;

	default:
		break;
	}
}						///< This method initialize the SPI port and the config register        



void IADC::PTbegin() {
	//pinMode(MISO, OUTPUT);
	//pinMode(MOSI, INPUT);
	//pinMode(SCK, OUTPUT);
	pinMode(cs, OUTPUT);
	digitalWrite(cs, HIGH);
	SPI.begin();
	SPI.beginTransaction(SPISettings(SCLK, MSBFIRST, SPI_MODE1));
	SPI.setClockDivider(SPI_CLOCK_DIV8);

}

int IADC::IPT100( uint8_t Fchannel) {
	uint8_t Fch = Fchannel;
	//Serial.begin(115200);
	byte RxData[10];
	byte RxCnt;
	int data;
	uint8_t delayMicroVal = 5;
	//----------------------------------//
	digitalWrite(cs, LOW);
	RxData[0] = SPI.transfer(0xA1);
	delayMicroseconds(delayMicroVal);
	RxData[1] = SPI.transfer(0xA2);
	delayMicroseconds(delayMicroVal);
	RxData[2] = SPI.transfer(0xB1);
	delayMicroseconds(delayMicroVal);
	RxData[3] = SPI.transfer(0xB2);
	delayMicroseconds(delayMicroVal);
	RxData[4] = SPI.transfer(0xC1);
	delayMicroseconds(delayMicroVal);
	RxData[5] = SPI.transfer(0xC2);
	delayMicroseconds(delayMicroVal);
	RxData[6] = SPI.transfer(0xD1);
	delayMicroseconds(delayMicroVal);
	RxData[7] = SPI.transfer(0xD2);
	delayMicroseconds(delayMicroVal);
	digitalWrite(cs, HIGH);

	PTdata[0] = (RxData[1] << 8 | RxData[0]);
	PTdata[1] = (RxData[3] << 8 | RxData[2]);
	PTdata[2] = (RxData[5] << 8 | RxData[4]);
	PTdata[3] = (RxData[7] << 8 | RxData[6]);
	delayMicroseconds(500);
	if (Fch == 0) {
		return PTdata[0];
	}
	else if (Fch == 1) {
		return PTdata[1];
	}
	else if (Fch == 2) {
		return PTdata[2];
	}
	else if (Fch == 3) {
		return PTdata[3];
	}
	return -2;
}
/**
 * This method initialize the SPI port and the config register
 */
void IADC::begin() {

	//analogReference(EXTERNAL);
	pinMode(cs, OUTPUT);
	digitalWrite(cs, HIGH);
	SPI.begin();
	SPI.beginTransaction(SPISettings(SCLK, MSBFIRST, SPI_MODE1));
	//Serial.println(SLCK);
	Serial.println(MSBFIRST);
	Serial.println(SPI_MODE1);

	configRegister.bits = { RESERVED, VALID_CFG, DOUT_NO_PULLUP, ADC_MODE, RATE_8SPS, SINGLE_SHOT, FSR_2048, AIN_0, START_NOW }; //Default values    
//	DEBUG_BEGIN(configRegister); //Debug this method: print the config register in the Serial port

}						///< This method initialize the SPI port and the config register        
/*
	const uint8_t RATE_8SPS = 0b000;  ///< 8 samples/s, Tconv=125ms
	const uint8_t RATE_16SPS = 0b001;  ///< 16 samples/s, Tconv=62.5ms
	const uint8_t RATE_32SPS = 0b010;  ///< 32 samples/s, Tconv=31.25ms
	const uint8_t RATE_64SPS = 0b011;  ///< 64 samples/s, Tconv=15.625ms
	const uint8_t RATE_128SPS = 0b100;  ///< 128 samples/s, Tconv=7.8125ms
	const uint8_t RATE_250SPS = 0b101;  ///< 250 samples/s, Tconv=4ms
	const uint8_t RATE_475SPS = 0b110;  ///< 475 samples/s, Tconv=2.105ms
	const uint8_t RATE_860SPS = 0b111;  ///< 860 samples/s, Tconv=1.163ms	
	*/
void IADC::INIT(int32_t maxvalue, int32_t minvalue, uint8_t rate_value) {
	if (boardModel == 1) {
		analogReference(EXTERNAL);
	}
	else {
		begin();

		switch (rate_value)
		{
		case 1:
			setSamplingRate(RATE_8SPS);
			break;
		case 2:
			setSamplingRate(RATE_16SPS);
			break;
		case 3:
			setSamplingRate(RATE_32SPS);
			break;
		case 4:
			setSamplingRate(RATE_64SPS);
			break;
		case 5:
			setSamplingRate(RATE_128SPS);
			break;
		case 6:
			setSamplingRate(RATE_250SPS);
			break;
		case 7:
			setSamplingRate(RATE_475SPS);
			break;
		case 8:
			setSamplingRate(RATE_860SPS);
			break;
		default:
			break;
		}

		getMilliVolts();
	}
	if (maxvalue == 0) {
		tempOn = 1;
		_maxvalue_adc = 32767;
		_minvalue_adc = 0;
	}
	if (maxvalue > minvalue) {
		_maxvalue_adc = maxvalue;
		_minvalue_adc = minvalue;
	}
	else if (maxvalue < minvalue) {
		tempOn = 0;
		_maxvalue_adc = minvalue;
		_minvalue_adc = maxvalue;
	}
}
/**
 * Getting a sample from the specified input
 * @param inputs Sets the input of the ADC: Diferential inputs: DIFF_0_1, DIFF_0_3, DIFF_1_3, DIFF_2_3. Single ended input: AIN_0, AIN_1, AIN_2, AIN_3
 * @return A word containing the ADC value
 */
long IADC::GET_ADC(uint8_t inputs, int8_t minus1_4) {
	long value;
	long value1;
	if (boardModel == 1) {
		if (tempOn == 1) {
			value = analogRead(inputs);
			return value;
		}
		if (minus1_4 == 1 || minus1_4 == -1 || minus1_4 == -4) {
			value = analogRead(inputs);
			if (value <= 180) {
				return -500;
			}
			else {
				value1 = (float)(value - 204) * (float)(_maxvalue_adc - _minvalue_adc) / (float)(1023 - 204) + _minvalue_adc;
				return value1;
			}
		}
		value = analogRead(inputs);
		value1 = (float)(value) * (float)(_maxvalue_adc - _minvalue_adc) / (float)(1023) + _minvalue_adc;
		return value1;
	}
	else {
		byte dataMSB, dataLSB, configMSB, configLSB, count = 0;
		if (lastSensorMode == ADC_MODE)  //Lucky you! We don't have to read twice the sensor
			count = 1;
		else
			configRegister.bits.sensorMode = ADC_MODE; //Sorry but we will have to read twice the sensor
		switch (inputs)
		{
		case 0:
			configRegister.bits.mux = 0b100;
			break;
		case 1:
			configRegister.bits.mux = 0b101;
			break;
		case 2:
			configRegister.bits.mux = 0b110;
			break;
		case 3:
			configRegister.bits.mux = 0b111;
			break;
		default:
			break;
		}
		do {
			digitalWrite(cs, LOW);
			dataMSB = SPI.transfer(configRegister.byte.msb);
			dataLSB = SPI.transfer(configRegister.byte.lsb);
			configMSB = SPI.transfer(configRegister.byte.msb);
			configLSB = SPI.transfer(configRegister.byte.lsb);
			digitalWrite(cs, HIGH);
			
			for (int i = 0; i < CONV_TIME[configRegister.bits.rate]; i++) //Lets wait the conversion time
			{
				delayMicroseconds(1003);
			}
			count++;
		} while (count <= 1);  //We make two readings because the second reading is the ADC conversion.	
//		DEBUG_GETADCVALUE(configRegister);  //Debug this method: print the config register in the Serial port
		value = (dataMSB << 8) | (dataLSB);
		//Serial.println(value);
		if (tempOn == 0) {
			if (minus1_4 == 1 || minus1_4 == -1 || minus1_4 == -4) {
				if (value <= 6250) {
					return -32768;
				}
				else {
					value1 = (float)(value - 6553) * (float)(_maxvalue_adc - _minvalue_adc) / (float)(32767 - 6553) + _minvalue_adc;
					return value1;
				}
			}
			else {
				value1 = (float)value * (float)(_maxvalue_adc - _minvalue_adc) / (float)(32767) + _minvalue_adc;
				//value1 = (float)value * (float)(_maxvalue_adc - _minvalue_adc) / (float)(32767) + _minvalue_adc;
				return value1;
			}
		}
		return value;
	}
}

int IADC::GET_ADCVALUE(uint8_t inputs, int8_t minus1_4) {
	int32_t value;
	if (boardModel == 1) {
		if (tempOn == 1) {
			value = analogRead(inputs);
			return value;
		}
		if (minus1_4 == 1 || minus1_4 == -1 || minus1_4 == -4) {
			value = analogRead(inputs);
			if (value <= 180) {
				return -500;
			}
			else {
				value = (float)(value - 204) * (float)(_maxvalue_adc - _minvalue_adc) / (float)(1023 - 204) + _minvalue_adc;
				return value;
			}
		}
		value = analogRead(inputs);
		value = (float)(value) * (float)(_maxvalue_adc - _minvalue_adc) / (float)(1023) + _minvalue_adc;
		return value;
	}
	else {
		byte dataMSB, dataLSB, configMSB, configLSB, count = 0;
		if (lastSensorMode == ADC_MODE)  //Lucky you! We don't have to read twice the sensor
			count = 1;
		else
			configRegister.bits.sensorMode = ADC_MODE; //Sorry but we will have to read twice the sensor
		switch (inputs)
		{
		case 0:
			configRegister.bits.mux = 0b100;
			break;
		case 1:
			configRegister.bits.mux = 0b101;
			break;
		case 2:
			configRegister.bits.mux = 0b110;
			break;
		case 3:
			configRegister.bits.mux = 0b111;
			break;
		default:
			break;
		}
		do {
			digitalWrite(cs, LOW);
			dataMSB = SPI.transfer(configRegister.byte.msb);
			dataLSB = SPI.transfer(configRegister.byte.lsb);
			configMSB = SPI.transfer(configRegister.byte.msb);
			configLSB = SPI.transfer(configRegister.byte.lsb);
			digitalWrite(cs, HIGH);

			while (digitalRead(50) != 1) {}
			for (int i = 0; i < CONV_TIME[configRegister.bits.rate]; i++) //Lets wait the conversion time
			{
				delayMicroseconds(1003);
			}
			count++;
		} while (count <= 1);
//		DEBUG_GETADCVALUE(configRegister);  //Debug this method: print the config register in the Serial port
		value = (dataMSB << 8) | (dataLSB);
		if (tempOn == 0) {
			if (minus1_4 == 1 || minus1_4 == -1 || minus1_4 == -4) {
				if (value <= 6250) {
					return -32768;
				}
				else {
					value = (float)(value - 6553) * (float)(_maxvalue_adc - _minvalue_adc) / (float)(32767 - 6553) + _minvalue_adc;
					return value;
				}
			}
			else {
				value = (float)value * (float)(_maxvalue_adc - _minvalue_adc) / (float)(32767) + _minvalue_adc;
				return value;
			}
		}
		return value;
	}
}
/**
 * Getting the millivolts from the specified inputs
 * @param inputs Sets the inputs to be adquired. Diferential inputs: DIFF_0_1, DIFF_0_3, DIFF_1_3, DIFF_2_3. Single ended input: AIN_0, AIN_1, AIN_2, AIN_3
 * @return A double (32bits) containing the ADC value in millivolts
 */
double IADC::getMilliVolts(uint8_t inputs) {
	float volts;
	float fsr = pgaFSR[configRegister.bits.pga];
	uint16_t value;
	value = GET_ADC(inputs);
	if (value >= 0x8000) {
		value = ((~value) + 1); //Applying binary twos complement format
		volts = ((float)(value * fsr / 32767) * -1);
	}
	else {
		volts = (float)(value * fsr / 32767);
	}
	return volts * 1000;
}


/**
 * Getting the millivolts from the settled inputs
 * @return A double (32bits) containing the ADC value in millivolts
 */
double IADC::getMilliVolts() {
	float volts;
	float fsr = pgaFSR[configRegister.bits.pga];
	uint16_t value;
	value = GET_ADC(configRegister.bits.mux);
	if (value >= 0x8000) {
		value = ((~value) + 1); //Applying binary twos complement format
		volts = ((float)(value * fsr / 32767) * -1);
	}
	else {
		volts = (float)(value * fsr / 32767);
	}
	return volts * 1000;
}



/**
 * Setting the sampling rate specified in the config register
 * @param samplingRate It's the sampling rate: RATE_8SPS, RATE_16SPS, RATE_32SPS, RATE_64SPS, RATE_128SPS, RATE_250SPS, RATE_475SPS, RATE_860SPS
 */
void IADC::setSamplingRate(uint8_t samplingRate) {
	configRegister.bits.rate = samplingRate;
}

/**
 * Setting the full scale range in the config register
 * @param fsr The full scale range: FSR_6144 (±6.144V)*, FSR_4096(±4.096V)*, FSR_2048(±2.048V), FSR_1024(±1.024V), FSR_0512(±0.512V), FSR_0256(±0.256V). (*) No more than VDD + 0.3 V must be applied to this device.
 */
void IADC::setFullScaleRange(uint8_t fsr) {
	configRegister.bits.pga = fsr;
}

/**
 * Setting the inputs to be adquired in the config register.
 * @param input The input selected: Diferential inputs: DIFF_0_1, DIFF_0_3, DIFF_1_3, DIFF_2_3. Single ended input: AIN_0, AIN_1, AIN_2, AIN_3
 */
void IADC::setInputSelected(uint8_t input) {
	configRegister.bits.mux = input;
}

/**
 * Setting to continuous adquisition mode
 */
void IADC::setContinuousMode() {
	configRegister.bits.operatingMode = CONTINUOUS;
}

/**
 * Setting to single shot adquisition and power down mode
 */
void IADC::setSingleShotMode() {
	configRegister.bits.operatingMode = SINGLE_SHOT;
}

/**
 * Disabling the internal pull-up resistor of the DOUT pin
 */
void IADC::disablePullup() {
	configRegister.bits.operatingMode = DOUT_NO_PULLUP;
}

/**
 * Enabling the internal pull-up resistor of the DOUT pin
 */
void IADC::enablePullup() {
	configRegister.bits.operatingMode = DOUT_PULLUP;
}

/**
 * Decoding a configRegister structure and then print it out to the Serial port
 * @param configRegister The config register in "union Config" format
 */
void IADC::decodeConfigRegister(union Config1 configRegister) {
	String mensaje = String();
	switch (configRegister.bits.singleStart) {
	case 0: mensaje = "NOINI"; break;
	case 1: mensaje = "START"; break;
	}
	mensaje += " ";
	switch (configRegister.bits.mux) {
	case 0: mensaje += "A0-A1"; break;
	case 1: mensaje += "A0-A3"; break;
	case 2: mensaje += "A1-A3"; break;
	case 3: mensaje += "A2-A3"; break;
	case 4: mensaje += "A0-GD"; break;
	case 5: mensaje += "A1-GD"; break;
	case 6: mensaje += "A2-GD"; break;
	case 7: mensaje += "A3-GD"; break;
	}
	mensaje += " ";
	switch (configRegister.bits.pga) {
	case 0: mensaje += "6.144"; break;
	case 1: mensaje += "4.096"; break;
	case 2: mensaje += "2.048"; break;
	case 3: mensaje += "1.024"; break;
	case 4: mensaje += "0.512"; break;
	case 5: mensaje += "0.256"; break;
	case 6: mensaje += "0.256"; break;
	case 7: mensaje += "0.256"; break;
	}
	mensaje += " ";
	switch (configRegister.bits.operatingMode) {
	case 0: mensaje += "CONT."; break;
	case 1: mensaje += "SSHOT"; break;
	}
	mensaje += " ";
	switch (configRegister.bits.rate) {
	case 0: mensaje += "8 SPS"; break;
	case 1: mensaje += "16SPS"; break;
	case 2: mensaje += "32SPS"; break;
	case 3: mensaje += "64SPS"; break;
	case 4: mensaje += "128SP"; break;
	case 5: mensaje += "250SP"; break;
	case 6: mensaje += "475SP"; break;
	case 7: mensaje += "860SP"; break;
	}
	mensaje += " ";
	switch (configRegister.bits.sensorMode) {
	case 0: mensaje += "ADC_M"; break;
	case 1: mensaje += "TMP_M"; break;
	}
	mensaje += " ";
	switch (configRegister.bits.pullUp) {
	case 0: mensaje += "DISAB"; break;
	case 1: mensaje += "ENABL"; break;
	}
	mensaje += " ";
	switch (configRegister.bits.noOperation) {
	case 0: mensaje += "INVAL"; break;
	case 1: mensaje += "VALID"; break;
	case 2: mensaje += "INVAL"; break;
	case 3: mensaje += "INVAL"; break;
	}
	mensaje += " ";
	switch (configRegister.bits.reserved) {
	case 0: mensaje += "RSRV0"; break;
	case 1: mensaje += "RSRV1"; break;
	}
	//Serial.println("\nSTART MXSEL PGASL MODES RATES ADTMP PLLUP NOOPE RESER");
	//Serial.println(mensaje);
}


#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////		ITIMER			ITIMER			ITIMER	      ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#if ITIMER1==1

/*
  MsTimer2.h - Using timer2 with 1ms resolution
  Javier Valencia <javiervalencia80@gmail.com>

  https://github.com/PaulStoffregen/MsTimer2

  History:
	6/Jun/14  - V0.7 added support for Teensy 3.0 & 3.1
	29/Dec/11 - V0.6 added support for ATmega32u4, AT90USB646, AT90USB1286 (paul@pjrc.com)
		some improvements added by Bill Perry
		note: uses timer4 on Atmega32u4
	29/May/09 - V0.5 added support for Atmega1280 (thanks to Manuel Negri)
	19/Mar/09 - V0.4 added support for ATmega328P (thanks to Jerome Despatis)
	11/Jun/08 - V0.3
		changes to allow working with different CPU frequencies
		added support for ATMega128 (using timer2)
		compatible with ATMega48/88/168/8
	10/May/08 - V0.2 added some security tests and volatile keywords
	9/May/08 - V0.1 released working on ATMEGA168 only


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/*
//unsigned long ITIMER::msecs2;
//void (*ITIMER::func2)();
//volatile unsigned long ITIMER::count2;
//volatile char ITIMER::overflowing2;
//volatile unsigned int ITIMER::tcnt2;
//bool Timer2On = 0;
*/
volatile unsigned long ITIMER::count1;
volatile unsigned int ITIMER::tcnt1;
volatile char ITIMER::overflowing1;
unsigned long ITIMER::msecs1;
void (*ITIMER::func1)();
bool Timer1On = 0;

volatile unsigned long ITIMER::count3;
volatile unsigned int ITIMER::tcnt3;
volatile char ITIMER::overflowing3;
unsigned long ITIMER::msecs3;
void (*ITIMER::func3)();
bool Timer3On = 0;

volatile unsigned long ITIMER::count4;
volatile unsigned int ITIMER::tcnt4;
volatile char ITIMER::overflowing4;
unsigned long ITIMER::msecs4;
void (*ITIMER::func4)();
bool Timer4On = 0;

volatile unsigned long ITIMER::count5;
volatile unsigned int ITIMER::tcnt5;
volatile char ITIMER::overflowing5;
unsigned long ITIMER::msecs5;
void (*ITIMER::func5)();
bool Timer5On = 0;

unsigned int Numtimer;

void ITIMER::SET(uint8_t Ntimer, unsigned long ms, void (*f)()) {
	float prescaler = 0.0;
	Numtimer = Ntimer;
	if (Ntimer == 1) {
		Timer1On = true;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TIMSK1 &= ~(1 << TOIE1);
		TCCR1A = 0x00;
		TCCR1B = 0x03;
		TIMSK1 &= ~(1 << OCIE1A);
#endif
		tcnt1 = 65285;
		if (ms == 0)
			msecs1 = 1;
		else
			msecs1 = ms;
		func1 = f;
	}
	else if (Ntimer == 3) {
		Timer3On = true;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TIMSK3 &= ~(1 << TOIE3);
		TCCR3A = 0x00;
		TCCR3B = 0x03;
		TIMSK3 &= ~(1 << OCIE3A);
#endif
		tcnt3 = 65285;
		if (ms == 0)
			msecs3 = 1;
		else
			msecs3 = ms;
		func3 = f;
	}
	else if (Ntimer == 4) {
		Timer4On = true;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TIMSK4 &= ~(1 << TOIE4);
		TCCR4A = 0x00;
		TCCR4B = 0x03;
		TIMSK4 &= ~(1 << OCIE4A);
#endif
		tcnt4 = 65285;
		if (ms == 0)
			msecs4 = 1;
		else
			msecs4 = ms;
		func4 = f;
	}
	else if (Ntimer == 5) {
		Timer5On = true;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TIMSK5 &= ~(1 << TOIE5);
		TCCR5A = 0x00;
		TCCR5B = 0x03;
		TIMSK5 &= ~(1 << OCIE5A);
#endif
		tcnt5 = 65285;
		if (ms == 0)
			msecs5 = 1;
		else
			msecs5 = ms;
		func5 = f;
	}
}
void ITIMER::START(uint8_t Numstart) {
	if (((Timer1On == true) && (Numstart == 10)) || ((Timer1On == true) && (Numstart == 1))) {
		count1 = 0;
		overflowing1 = 0;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TCNT1 = tcnt1;
		TIMSK1 |= (1 << TOIE1);
#endif
	}
	/*	if (((Timer2On == true) && (Numstart == 10)) || ((Timer2On == true) && (Numstart == 2))) {
			count2 = 0;
			overflowing2 = 0;
	#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
			TCNT2 = tcnt2;
			TIMSK2 |= (1 << TOIE2);
	#endif
		}*/
	if (((Timer3On == true) && (Numstart == 10)) || ((Timer3On == true) && (Numstart == 3))) {
		count3 = 0;
		overflowing3 = 0;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TCNT3 = tcnt3;
		TIMSK3 |= (1 << TOIE3);
#endif
	}
	if (((Timer4On == true) && (Numstart == 10)) || ((Timer4On == true) && (Numstart == 4))) {
		count4 = 0;
		overflowing4 = 0;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TCNT4 = tcnt4;
		TIMSK4 |= (1 << TOIE4);
#endif
	}
	if (((Timer5On == true) && (Numstart == 10)) || ((Timer5On == true) && (Numstart == 5))) {
		count5 = 0;
		overflowing5 = 0;
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TCNT5 = tcnt5;
		TIMSK5 |= (1 << TOIE5);
#endif
	}
}

void ITIMER::STOP(uint8_t Numstop) {
	if (((Timer1On == true) && (Numstop == 1)) || ((Timer1On == true) && (Numstop == 10))) {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TIMSK1 &= ~(1 << TOIE1);
#endif
	}
	/*	if (((Timer2On == true) && (Numstop == 2)) || ((Timer2On == true) && (Numstop == 10))) {
	#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
			TIMSK2 &= ~(1 << TOIE2);
	#endif
		}*/
	if (((Timer3On == true) && (Numstop == 3)) || ((Timer3On == true) && (Numstop == 10))) {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TIMSK3 &= ~(1 << TOIE3);
#endif
	}
	if (((Timer4On == true) && (Numstop == 4)) || ((Timer4On == true) && (Numstop == 10))) {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TIMSK4 &= ~(1 << TOIE4);
#endif
	}
	if (((Timer5On == true) && (Numstop == 5)) || ((Timer5On == true) && (Numstop == 10))) {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
		TIMSK5 &= ~(1 << TOIE5);
#endif
	}
}
void ITIMER::_overflow1() {
	count1 += 1;
	if (count1 >= msecs1 && !overflowing1) {
		overflowing1 = 1;
		count1 = 0;
		(*func1)();
		overflowing1 = 0;

	}
}
/*
void ITIMER::_overflow2() {
	count2 += 1;

	if (count2 >= msecs2 && !overflowing2) {
		overflowing2 = 1;
		count2 = 0;
		(*func2)();
		overflowing2 = 0;
	}
}*/
void ITIMER::_overflow3() {
	count3 += 1;
	if (count3 >= msecs3 && !overflowing3) {
		overflowing3 = 1;
		count3 = 0;
		(*func3)();
		overflowing3 = 0;

	}
}

void ITIMER::_overflow4() {
	count4 += 1;

	if (count4 >= msecs4 && !overflowing4) {
		overflowing4 = 1;
		count4 = 0;
		(*func4)();
		overflowing4 = 0;
	}
}
void ITIMER::_overflow5() {
	count5 += 1;

	if (count5 >= msecs5 && !overflowing5) {
		overflowing5 = 1;
		count5 = 0;
		(*func5)();
		overflowing5 = 0;
	}
}


//ISR(TIMER1_OVF_vect) at bottom

/*
#if !defined (MPAINO_8A8R) && !defined (MPAINO_8A8RX) && !defined (MPAINO_16A16R) && !defined (MPAINO_16A16RX) && !defined (MPAINO_16A32R) && !defined (MPAINO_16A32RX) && !defined (MPAINO_32A16R) && !defined (MPINO_8A8R) && !defined (MPINO_16A8R) && !defined (MPINO_16A8R8T)
ISR(TIMER2_OVF_vect) {
#if defined (__AVR_ATmega168__) || defined (__AVR_ATmega48__) || defined (__AVR_ATmega88__) || defined (__AVR_ATmega328P__) || (__AVR_ATmega1280__) || defined (__AVR_ATmega2560__)
	TCNT2 = ITIMER::tcnt2;
#elif defined (__AVR_ATmega128__)
	TCNT2 = ITIMER::tcnt2;
#elif defined (__AVR_ATmega8__)
	TCNT2 = ITIMER::tcnt2;
#endif
	ITIMER::_overflow2();
}
#endif
*/




#elif ITIMER1 == 2
volatile unsigned long ITIMER::count1;
volatile unsigned int ITIMER::tcnt1;
volatile char ITIMER::overflowing1;
unsigned long ITIMER::msecs1;
void (*ITIMER::func1)();
bool Timer1On = 0;

volatile unsigned long ITIMER::count3;
volatile unsigned int ITIMER::tcnt3;
volatile char ITIMER::overflowing3;
unsigned long ITIMER::msecs3;
void (*ITIMER::func3)();
bool Timer3On = 0;

unsigned int Numtimer;

void ITIMER::SET(uint8_t Ntimer, unsigned long ms, void (*f)()) {
	float prescaler = 0.0;
	Numtimer = Ntimer;
	if (Ntimer == 1) {
		Timer1On = true;
		TIMSK &= ~(1 << TOIE1);
		TCCR1A = 0x00;
		TCCR1B = 0x03;
		TIMSK &= ~(1 << OCIE1A);
#if F_CPU == 16000000L
		tcnt1 = 65285;
#elif F_CPU == 14745600L
		tcnt1 = 64383;
#endif // F_CPU == 16000000L

		if (ms == 0)
			msecs1 = 1;
		else
			msecs1 = ms;
		func1 = f;
	}
	if (Ntimer == 3) {
		Timer3On = true;
		ETIMSK &= ~(1 << TOIE3);
		TCCR3A = 0x00;
		TCCR3B = 0x03;
		ETIMSK &= ~(1 << OCIE3A);
#if F_CPU == 16000000L
		tcnt3 = 65285;
#elif F_CPU == 14745600L
		tcnt3 = 64383;
#endif // F_CPU == 16000000L
		if (ms == 0)
			msecs3 = 1;
		else
			msecs3 = ms;
		func3 = f;
	}
}
void ITIMER::START(uint8_t Numstart) {
	if (((Timer1On == true) && (Numstart == 10)) || ((Timer1On == true) && (Numstart == 1))) {
		count1 = 0;
		overflowing1 = 0;
		TCNT1 = tcnt1;
		TIMSK |= (1 << TOIE1);
	}
	if (((Timer3On == true) && (Numstart == 10)) || ((Timer3On == true) && (Numstart == 3))) {
		count3 = 0;
		overflowing3 = 0;
		TCNT3 = tcnt3;
		ETIMSK |= (1 << TOIE3);
	}
}

void ITIMER::STOP(uint8_t Numstop) {
	if (((Timer1On == true) && (Numstop == 1)) || ((Timer1On == true) && (Numstop == 10))) {
		TIMSK &= ~(1 << TOIE1);
	}
	if (((Timer3On == true) && (Numstop == 3)) || ((Timer3On == true) && (Numstop == 10))) {
		ETIMSK &= ~(1 << TOIE3);
	}
}
void ITIMER::_overflow1() {
	count1 += 1;
#if F_CPU == 16000000L
#elif F_CPU == 14745600L
	msecs1 /= 5.0;
#endif // F_CPU == 16000000L
	if (count1 >= msecs1 && !overflowing1) {
		overflowing1 = 1;
		count1 = 0;
		(*func1)();
		overflowing1 = 0;

	}
}
void ITIMER::_overflow3() {
	count3 += 1;
#if F_CPU == 16000000L
#elif F_CPU == 14745600L
	msecs3 /= 5.0;
#endif // F_CPU == 16000000L
	if (count3 >= msecs3 && !overflowing3) {
		overflowing3 = 1;
		count3 = 0;
		(*func3)();
		overflowing3 = 0;

	}
}

#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////		MODBUSMASTER			MODBUSMASTER	      ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MODBUSMASTER1 1;
#ifdef MODBUSMASTER1

/**
@file
Arduino library for communicating with Modbus slaves over RS232/485 (via RTU protocol).
*/
/*

  ModbusMaster.cpp - Arduino library for communicating with Modbus slaves
  over RS232/485 (via RTU protocol).

  Library:: ModbusMaster

  Copyright:: 2009-2016 Doc Walker

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

	  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.

*/

/* _____PROJECT INCLUDES_____________________________________________________ */
//#include "ModbusMaster.h"

/* _____GLOBAL VARIABLES_____________________________________________________ */

/* _____PUBLIC FUNCTIONS_____________________________________________________ */
/**
Constructor.

Creates class object; initialize it using ModbusMaster::begin().

@ingroup setup
*/
/* _____REVISION HISTORY_____________________________________________________ */
/**

void ModbusMaster::begin(uint8_t slave, Stream &serial)
↓
void ModbusMaster::begin(uint8_t slave, HardwareSerial *SerialPort, unsigned long baud, byte serialbit)


*/

ModbusMaster::ModbusMaster(void)
{
	_idle = 0;
	_preTransmission = 0;
	_postTransmission = 0;
}

/**
Initialize class object.

@param slave Modbus slave ID (1..255)
@param &serial reference to serial port object (Serial, Serial1, ... Serial3)
*/
void ModbusMaster::begin(uint8_t slave, HardwareSerial *SerialPort, unsigned long baud, byte serialbit)
{
	//  txBuffer = (uint16_t*) calloc(ku8MaxBufferSize, sizeof(uint16_t));
	_u8MBSlave = slave;
	_serial = SerialPort;
	_u8TransmitBufferIndex = 0;
	u16TransmitBufferLength = 0;
	(*SerialPort).begin(baud, serialbit);
#if __MODBUSMASTER_DEBUG__
	pinMode(__MODBUSMASTER_DEBUG_PIN_A__, OUTPUT);
	pinMode(__MODBUSMASTER_DEBUG_PIN_B__, OUTPUT);
#endif
	
}


void ModbusMaster::beginTransmission(uint16_t u16Address)
{
	_u16WriteAddress = u16Address;
	_u8TransmitBufferIndex = 0;
	u16TransmitBufferLength = 0;
}

// eliminate this function in favor of using existing MB request functions
uint8_t ModbusMaster::requestFrom(uint16_t address, uint16_t quantity)
{
	uint8_t read;
	// clamp to buffer length
	if (quantity > ku8MaxBufferSize)
	{
		quantity = ku8MaxBufferSize;
	}
	// set rx buffer iterator vars
	_u8ResponseBufferIndex = 0;
	_u8ResponseBufferLength = read;

	return read;
}


void ModbusMaster::sendBit(bool data)
{
	uint8_t txBitIndex = u16TransmitBufferLength % 16;
	if ((u16TransmitBufferLength >> 4) < ku8MaxBufferSize)
	{
		if (0 == txBitIndex)
		{
			_u16TransmitBuffer[_u8TransmitBufferIndex] = 0;
		}
		bitWrite(_u16TransmitBuffer[_u8TransmitBufferIndex], txBitIndex, data);
		u16TransmitBufferLength++;
		_u8TransmitBufferIndex = u16TransmitBufferLength >> 4;
	}
}


void ModbusMaster::send(uint16_t data)
{
	if (_u8TransmitBufferIndex < ku8MaxBufferSize)
	{
		_u16TransmitBuffer[_u8TransmitBufferIndex++] = data;
		u16TransmitBufferLength = _u8TransmitBufferIndex << 4;
	}
}

void ModbusMaster::send(uint32_t data)
{
	send(lowWord(data));
	send(highWord(data));
}

void ModbusMaster::send(uint8_t data)
{
	send(word(data));
}


uint8_t ModbusMaster::available(void)
{
	return _u8ResponseBufferLength - _u8ResponseBufferIndex;
}

uint16_t ModbusMaster::RECEIVE(void)
{
	if (_u8ResponseBufferIndex < _u8ResponseBufferLength)
	{
		return _u16ResponseBuffer[_u8ResponseBufferIndex++];
	}
	else
	{
		return 0xFFFF;
	}
}
uint16_t ModbusMaster::receive(void)
{
	if (_u8ResponseBufferIndex < _u8ResponseBufferLength)
	{
		return _u16ResponseBuffer[_u8ResponseBufferIndex++];
	}
	else
	{
		return 0xFFFF;
	}
}

/**
Set idle time callback function (cooperative multitasking).
*/
void ModbusMaster::idle(void (*idle)())
{
	_idle = idle;
}

/**
Set pre-transmission callback function.

*/
void ModbusMaster::preTransmission(void (*preTransmission)())
{
	_preTransmission = preTransmission;
}

/**
Set post-transmission callback function.

*/
void ModbusMaster::postTransmission(void (*postTransmission)())
{
	_postTransmission = postTransmission;
}


/**
Retrieve data from response buffer.
*/
uint16_t ModbusMaster::GET_RESPONSE_BUFFER(uint8_t u8Index)
{
	if (u8Index < ku8MaxBufferSize)
	{
		return _u16ResponseBuffer[u8Index];
	}
	else
	{
		return 0xFFFF;
	}
}
uint16_t ModbusMaster::getResponseBuffer(uint8_t u8Index)
{
	if (u8Index < ku8MaxBufferSize)
	{
		return _u16ResponseBuffer[u8Index];
	}
	else
	{
		return 0xFFFF;
	}
}


/**
Clear Modbus response buffer.
*/
void ModbusMaster::CLAER_RESPONSE_BUFFER() {
	uint8_t i;

	for (i = 0; i < ku8MaxBufferSize; i++)
	{
		_u16ResponseBuffer[i] = 0;
	}
}
void ModbusMaster::clearResponseBuffer()
{
	uint8_t i;

	for (i = 0; i < ku8MaxBufferSize; i++)
	{
		_u16ResponseBuffer[i] = 0;
	}
}


/**
Place data in transmit buffer.
*/
uint8_t ModbusMaster::SET_TRANSMIT_BUFFER(uint8_t u8Index, uint16_t u16Value) {
	setTransmitBuffer( u8Index, u16Value);
}
uint8_t ModbusMaster::setTransmitBuffer(uint8_t u8Index, uint16_t u16Value)
{
	if (u8Index < ku8MaxBufferSize)
	{
		_u16TransmitBuffer[u8Index] = u16Value;
		return ku8MBSuccess;
	}
	else
	{
		return ku8MBIllegalDataAddress;
	}
}


/**
Clear Modbus transmit buffer.
*/
void ModbusMaster::CLEAR_TRANSMIT_BUFFER() {
	uint8_t i;

	for (i = 0; i < ku8MaxBufferSize; i++)
	{
		_u16TransmitBuffer[i] = 0;
	}
}
void ModbusMaster::clearTransmitBuffer()
{
	uint8_t i;

	for (i = 0; i < ku8MaxBufferSize; i++)
	{
		_u16TransmitBuffer[i] = 0;
	}
}


/**
Modbus function 0x01 Read Coils.
*/
uint8_t ModbusMaster::READ_COILS(uint16_t u16ReadAddress, uint16_t u16BitQty) {
	readCoils(u16ReadAddress, u16BitQty);
}
uint8_t ModbusMaster::readCoils(uint16_t u16ReadAddress, uint16_t u16BitQty)
{
	_u16ReadAddress = u16ReadAddress;
	_u16ReadQty = u16BitQty;
	return ModbusMasterTransaction(ku8MBReadCoils);
}


/**
Modbus function 0x02 Read Discrete Inputs.
*/
uint8_t ModbusMaster::READ_DISCRETE_INPUTS(uint16_t u16ReadAddress,
	uint16_t u16BitQty) {
	readDiscreteInputs(u16ReadAddress, u16BitQty);
}

uint8_t ModbusMaster::readDiscreteInputs(uint16_t u16ReadAddress,
	uint16_t u16BitQty)
{
	_u16ReadAddress = u16ReadAddress;
	_u16ReadQty = u16BitQty;
	return ModbusMasterTransaction(ku8MBReadDiscreteInputs);
}


/**
Modbus function 0x03 Read Holding Registers.
*/
uint8_t ModbusMaster::READ_HOLDING_REGISTERS(uint16_t u16ReadAddress,
	uint16_t u16ReadQty) {
	readHoldingRegisters(u16ReadAddress, u16ReadQty);
}
uint8_t ModbusMaster::readHoldingRegisters(uint16_t u16ReadAddress,
	uint16_t u16ReadQty)
{
	_u16ReadAddress = u16ReadAddress;
	_u16ReadQty = u16ReadQty;
	return ModbusMasterTransaction(ku8MBReadHoldingRegisters);
}


/**
Modbus function 0x04 Read Input Registers.
*/
uint8_t ModbusMaster::READ_INPUT_REGISTERS(uint16_t u16ReadAddress,
	uint8_t u16ReadQty) {
	readInputRegisters( u16ReadAddress,	u16ReadQty);
}
uint8_t ModbusMaster::readInputRegisters(uint16_t u16ReadAddress,
		uint8_t u16ReadQty)
{
	_u16ReadAddress = u16ReadAddress;
	_u16ReadQty = u16ReadQty;
	return ModbusMasterTransaction(ku8MBReadInputRegisters);
}

/**
Modbus function 0x05 Write Single Coil.
*/
uint8_t ModbusMaster::WRITE_SINGLE_COIL(uint16_t u16WriteAddress, uint8_t u8State)
{
	writeSingleCoil(u16WriteAddress, u8State);
}
uint8_t ModbusMaster::writeSingleCoil(uint16_t u16WriteAddress, uint8_t u8State)
{
	_u16WriteAddress = u16WriteAddress;
	_u16WriteQty = (u8State ? 0xFF00 : 0x0000);
	return ModbusMasterTransaction(ku8MBWriteSingleCoil);
}


/**
Modbus function 0x06 Write Single Register.
*/
uint8_t ModbusMaster::WRITE_SINGLE_REGISTER(uint16_t u16WriteAddress,
	uint16_t u16WriteValue) {
	writeSingleRegister(u16WriteAddress, u16WriteValue);
}
uint8_t ModbusMaster::writeSingleRegister(uint16_t u16WriteAddress,
	uint16_t u16WriteValue)
{
	_u16WriteAddress = u16WriteAddress;
	_u16WriteQty = 0;
	_u16TransmitBuffer[0] = u16WriteValue;
	return ModbusMasterTransaction(ku8MBWriteSingleRegister);
}


/**
Modbus function 0x0F Write Multiple Coils.
*/
uint8_t ModbusMaster::WRITE_MULTIPLE_COILS(uint16_t u16WriteAddress,
	uint16_t u16BitQty) {
	writeMultipleCoils(u16WriteAddress, u16BitQty);
}
uint8_t ModbusMaster::writeMultipleCoils(uint16_t u16WriteAddress,
	uint16_t u16BitQty)
{
	_u16WriteAddress = u16WriteAddress;
	_u16WriteQty = u16BitQty;
	return ModbusMasterTransaction(ku8MBWriteMultipleCoils);
}

uint8_t ModbusMaster::WRITE_MULTIPLE_COILS()
{
	writeMultipleCoils();
}
uint8_t ModbusMaster::writeMultipleCoils()
{
	_u16WriteQty = u16TransmitBufferLength;
	return ModbusMasterTransaction(ku8MBWriteMultipleCoils);
}


/**
Modbus function 0x10 Write Multiple Registers.
*/
uint8_t ModbusMaster::WRITE_MULTIPLE_REGISTERS(uint16_t u16WriteAddress,
	uint16_t u16WriteQty) {
	writeMultipleRegisters(u16WriteAddress, u16WriteQty);
}
uint8_t ModbusMaster::writeMultipleRegisters(uint16_t u16WriteAddress,
		uint16_t u16WriteQty)
{
	_u16WriteAddress = u16WriteAddress;
	_u16WriteQty = u16WriteQty;
	return ModbusMasterTransaction(ku8MBWriteMultipleRegisters);
}

// new version based on Wire.h
uint8_t ModbusMaster::WRITE_MULTIPLE_REGISTERS() {
	writeMultipleRegisters();
}
uint8_t ModbusMaster::writeMultipleRegisters()
{
	_u16WriteQty = _u8TransmitBufferIndex;
	return ModbusMasterTransaction(ku8MBWriteMultipleRegisters);
}


/**
Modbus function 0x16 Mask Write Register.
*/
uint8_t ModbusMaster::MASK_WRITE_REGISTER(uint16_t u16WriteAddress,
	uint16_t u16AndMask, uint16_t u16OrMask)
{
	maskWriteRegister(u16WriteAddress, u16AndMask, u16OrMask);
}
uint8_t ModbusMaster::maskWriteRegister(uint16_t u16WriteAddress,
	uint16_t u16AndMask, uint16_t u16OrMask)
{
	_u16WriteAddress = u16WriteAddress;
	_u16TransmitBuffer[0] = u16AndMask;
	_u16TransmitBuffer[1] = u16OrMask;
	return ModbusMasterTransaction(ku8MBMaskWriteRegister);
}


/**
Modbus function 0x17 Read Write Multiple Registers.
*/
uint8_t ModbusMaster::READ_WRITE_MULTIPLE_REGISTERS(uint16_t u16ReadAddress,
	uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty)
{
	readWriteMultipleRegisters(u16ReadAddress, u16ReadQty, u16WriteAddress, u16WriteQty);
}
uint8_t ModbusMaster::readWriteMultipleRegisters(uint16_t u16ReadAddress,
	uint16_t u16ReadQty, uint16_t u16WriteAddress, uint16_t u16WriteQty)
{
	_u16ReadAddress = u16ReadAddress;
	_u16ReadQty = u16ReadQty;
	_u16WriteAddress = u16WriteAddress;
	_u16WriteQty = u16WriteQty;
	return ModbusMasterTransaction(ku8MBReadWriteMultipleRegisters);
}

uint8_t ModbusMaster::READ_WRITE_MULTIPLE_REGISTERS(uint16_t u16ReadAddress,
	uint16_t u16ReadQty) {
	readWriteMultipleRegisters(u16ReadAddress, u16ReadQty);
}
uint8_t ModbusMaster::readWriteMultipleRegisters(uint16_t u16ReadAddress,
	uint16_t u16ReadQty)
{
	_u16ReadAddress = u16ReadAddress;
	_u16ReadQty = u16ReadQty;
	_u16WriteQty = _u8TransmitBufferIndex;
	return ModbusMasterTransaction(ku8MBReadWriteMultipleRegisters);
}

uint8_t ModbusMaster::ModbusMasterTransaction(uint8_t u8MBFunction)
{
	uint8_t u8ModbusADU[256];
	uint8_t u8ModbusADUSize = 0;
	uint8_t i, u8Qty;
	uint16_t u16CRC;
	uint32_t u32StartTime;
	uint8_t u8BytesLeft = 8;
	uint8_t u8MBStatus = ku8MBSuccess;

	// assemble Modbus Request Application Data Unit
	u8ModbusADU[u8ModbusADUSize++] = _u8MBSlave;
	u8ModbusADU[u8ModbusADUSize++] = u8MBFunction;

	switch (u8MBFunction)
	{
	case ku8MBReadCoils:
	case ku8MBReadDiscreteInputs:
	case ku8MBReadInputRegisters:
	case ku8MBReadHoldingRegisters:
	case ku8MBReadWriteMultipleRegisters:
		u8ModbusADU[u8ModbusADUSize++] = highByte(_u16ReadAddress);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16ReadAddress);
		u8ModbusADU[u8ModbusADUSize++] = highByte(_u16ReadQty);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16ReadQty);
		break;
	}

	switch (u8MBFunction)
	{
	case ku8MBWriteSingleCoil:
	case ku8MBMaskWriteRegister:
	case ku8MBWriteMultipleCoils:
	case ku8MBWriteSingleRegister:
	case ku8MBWriteMultipleRegisters:
	case ku8MBReadWriteMultipleRegisters:
		u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteAddress);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteAddress);
		break;
	}

	switch (u8MBFunction)
	{
	case ku8MBWriteSingleCoil:
		u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
		break;

	case ku8MBWriteSingleRegister:
		u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[0]);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[0]);
		break;

	case ku8MBWriteMultipleCoils:
		u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
		u8Qty = (_u16WriteQty % 8) ? ((_u16WriteQty >> 3) + 1) : (_u16WriteQty >> 3);
		u8ModbusADU[u8ModbusADUSize++] = u8Qty;
		for (i = 0; i < u8Qty; i++)
		{
			switch (i % 2)
			{
			case 0: // i is even
				u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[i >> 1]);
				break;

			case 1: // i is odd
				u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[i >> 1]);
				break;
			}
		}
		break;

	case ku8MBWriteMultipleRegisters:
	case ku8MBReadWriteMultipleRegisters:
		u8ModbusADU[u8ModbusADUSize++] = highByte(_u16WriteQty);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16WriteQty << 1);

		for (i = 0; i < lowByte(_u16WriteQty); i++)
		{
			u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[i]);
			u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[i]);
		}
		break;

	case ku8MBMaskWriteRegister:
		u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[0]);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[0]);
		u8ModbusADU[u8ModbusADUSize++] = highByte(_u16TransmitBuffer[1]);
		u8ModbusADU[u8ModbusADUSize++] = lowByte(_u16TransmitBuffer[1]);
		break;
	}

	// append CRC
	u16CRC = 0xFFFF;
	for (i = 0; i < u8ModbusADUSize; i++)
	{
		u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
	}
	u8ModbusADU[u8ModbusADUSize++] = lowByte(u16CRC);
	u8ModbusADU[u8ModbusADUSize++] = highByte(u16CRC);
	u8ModbusADU[u8ModbusADUSize] = 0;

	// flush receive buffer before transmitting request
	while (_serial->read() != -1);

	// transmit request
	if (_preTransmission)
	{
		_preTransmission();
	}
	for (i = 0; i < u8ModbusADUSize; i++)
	{
		_serial->write(u8ModbusADU[i]);
	}

	u8ModbusADUSize = 0;
	_serial->flush();    // flush transmit buffer
	if (_postTransmission)
	{
		_postTransmission();
	}

	// loop until we run out of time or bytes, or an error occurs
	u32StartTime = millis();
	while (u8BytesLeft && !u8MBStatus)
	{
		if (_serial->available())
		{
#if __MODBUSMASTER_DEBUG__
			digitalWrite(__MODBUSMASTER_DEBUG_PIN_A__, true);
#endif
			u8ModbusADU[u8ModbusADUSize++] = _serial->read();
			u8BytesLeft--;
#if __MODBUSMASTER_DEBUG__
			digitalWrite(__MODBUSMASTER_DEBUG_PIN_A__, false);
#endif
		}
		else
		{
#if __MODBUSMASTER_DEBUG__
			digitalWrite(__MODBUSMASTER_DEBUG_PIN_B__, true);
#endif
			if (_idle)
			{
				_idle();
			}
#if __MODBUSMASTER_DEBUG__
			digitalWrite(__MODBUSMASTER_DEBUG_PIN_B__, false);
#endif
		}

		// evaluate slave ID, function code once enough bytes have been read
		if (u8ModbusADUSize == 5)
		{
			// verify response is for correct Modbus slave
			if (u8ModbusADU[0] != _u8MBSlave)
			{
				u8MBStatus = ku8MBInvalidSlaveID;
				break;
			}

			// verify response is for correct Modbus function code (mask exception bit 7)
			if ((u8ModbusADU[1] & 0x7F) != u8MBFunction)
			{
				u8MBStatus = ku8MBInvalidFunction;
				break;
			}

			// check whether Modbus exception occurred; return Modbus Exception Code
			if (bitRead(u8ModbusADU[1], 7))
			{
				u8MBStatus = u8ModbusADU[2];
				break;
			}

			// evaluate returned Modbus function code
			switch (u8ModbusADU[1])
			{
			case ku8MBReadCoils:
			case ku8MBReadDiscreteInputs:
			case ku8MBReadInputRegisters:
			case ku8MBReadHoldingRegisters:
			case ku8MBReadWriteMultipleRegisters:
				u8BytesLeft = u8ModbusADU[2];
				break;

			case ku8MBWriteSingleCoil:
			case ku8MBWriteMultipleCoils:
			case ku8MBWriteSingleRegister:
			case ku8MBWriteMultipleRegisters:
				u8BytesLeft = 3;
				break;

			case ku8MBMaskWriteRegister:
				u8BytesLeft = 5;
				break;
			}
		}
		if ((millis() - u32StartTime) > ku16MBResponseTimeout)
		{
			u8MBStatus = ku8MBResponseTimedOut;
		}
	}

	// verify response is large enough to inspect further
	if (!u8MBStatus && u8ModbusADUSize >= 5)
	{
		// calculate CRC
		u16CRC = 0xFFFF;
		for (i = 0; i < (u8ModbusADUSize - 2); i++)
		{
			u16CRC = crc16_update(u16CRC, u8ModbusADU[i]);
		}

		// verify CRC
		if (!u8MBStatus && (lowByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 2] ||
			highByte(u16CRC) != u8ModbusADU[u8ModbusADUSize - 1]))
		{
			u8MBStatus = ku8MBInvalidCRC;
		}
	}

	// disassemble ADU into words
	if (!u8MBStatus)
	{
		// evaluate returned Modbus function code
		switch (u8ModbusADU[1])
		{
		case ku8MBReadCoils:
		case ku8MBReadDiscreteInputs:
			// load bytes into word; response bytes are ordered L, H, L, H, ...
			for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
			{
				if (i < ku8MaxBufferSize)
				{
					_u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 4], u8ModbusADU[2 * i + 3]);
				}

				_u8ResponseBufferLength = i;
			}

			// in the event of an odd number of bytes, load last byte into zero-padded word
			if (u8ModbusADU[2] % 2)
			{
				if (i < ku8MaxBufferSize)
				{
					_u16ResponseBuffer[i] = word(0, u8ModbusADU[2 * i + 3]);
				}

				_u8ResponseBufferLength = i + 1;
			}
			break;

		case ku8MBReadInputRegisters:
		case ku8MBReadHoldingRegisters:
		case ku8MBReadWriteMultipleRegisters:
			// load bytes into word; response bytes are ordered H, L, H, L, ...
			for (i = 0; i < (u8ModbusADU[2] >> 1); i++)
			{
				if (i < ku8MaxBufferSize)
				{
					_u16ResponseBuffer[i] = word(u8ModbusADU[2 * i + 3], u8ModbusADU[2 * i + 4]);
				}

				_u8ResponseBufferLength = i;
			}
			break;
		}
	}

	_u8TransmitBufferIndex = 0;
	u16TransmitBufferLength = 0;
	_u8ResponseBufferIndex = 0;
	return u8MBStatus;
}
#endif


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////		MODBUSSLAVE				MODBUSSLAVE		      ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MODBUSSLAVE1 1;
#ifdef MODBUSSLAVE1 



ModbusRTUSlave::ModbusRTUSlave(byte slaveAddress, HardwareSerial* serialport, unsigned long baud, byte serialbit)
{
	slave = slaveAddress;
	ser = serialport;
	(*serialport).begin(baud, serialbit);

	words = new LinkedList<ModbusRTUSlaveWordAddress*>();
	bits = new LinkedList<ModbusRTUSlaveBitAddress*>();
	ResCnt = 0;
}


boolean ModbusRTUSlave::addWordArea(u16 Address, u16* values, int cnt)
{
	if (getWordAddress(Address) == NULL)
	{
		words->add(new ModbusRTUSlaveWordAddress(Address, values, cnt));
		return true;
	}
	return false;
}

boolean ModbusRTUSlave::addBitArea(u16 Address, u8* values, int cnt)
{
	if (getBitAddress(Address) == NULL)
	{
		bits->add(new ModbusRTUSlaveBitAddress(Address, values, cnt));
		return true;
	}
	return false;
}

ModbusRTUSlaveWordAddress* ModbusRTUSlave::getWordAddress(u16 Addr)
{
	ModbusRTUSlaveWordAddress* ret = NULL;
	for (int i = 0; i < words->size(); i++)
	{
		ModbusRTUSlaveWordAddress* a = words->get(i);
		if (a != NULL && Addr >= a->addr && Addr < a->addr + a->len) ret = a;
	}
	return ret;
}
ModbusRTUSlaveBitAddress* ModbusRTUSlave::getBitAddress(u16 Addr)
{
	ModbusRTUSlaveBitAddress* ret = NULL;
	for (int i = 0; i < bits->size(); i++)
	{
		ModbusRTUSlaveBitAddress* a = bits->get(i);
		if (a != NULL && Addr >= a->addr && Addr < a->addr + (a->len * 8)) ret = a;
	}
	return ret;
}

ModbusRTUSlaveWordAddress* ModbusRTUSlave::getWordAddress(u16 Addr, u16 Len)
{
	ModbusRTUSlaveWordAddress* ret = NULL;
	for (int i = 0; i < words->size(); i++)
	{
		ModbusRTUSlaveWordAddress* a = words->get(i);
		if (a != NULL && Addr >= a->addr && Addr + Len <= a->addr + a->len) ret = a;
	}
	return ret;
}
ModbusRTUSlaveBitAddress* ModbusRTUSlave::getBitAddress(u16 Addr, u16 Len)
{
	ModbusRTUSlaveBitAddress* ret = NULL;
	for (int i = 0; i < bits->size(); i++)
	{
		ModbusRTUSlaveBitAddress* a = bits->get(i);
		if (a != NULL && Addr >= a->addr && Addr + Len <= a->addr + (a->len * 8)) ret = a;
	}
	return ret;
}
int data123;


void ModbusRTUSlave::process()
{
	/*if (Serial2.available() > 0) {
		data123 = Serial2.read();

		Serial.println(data123, HEX);
		//    Serial.println(data);
	}
	if (Serial.available() > 0) {
		Serial2.write(Serial.read());
	}*/
	bool bvalid = true;
	while (ser->available())
	{
		byte d = ser->read();
		lstResponse[ResCnt++] = d;


		if (ResCnt >= 1) {
			if (ResCnt >= 4)
			{
				byte Slave = lstResponse[0];
				if (Slave == slave)
				{
					byte Function = lstResponse[1];
					u16 Address = (lstResponse[2] << 8) | lstResponse[3];
					switch (Function)
					{
						//Serial.println();
					case 1:		//BitRead
					case 2:
						if (ResCnt >= 8)
						{
							u16 Length = (lstResponse[4] << 8) | lstResponse[5];
							byte hi = 0xFF, lo = 0xFF;
							getCRC(lstResponse, 300, 0, 6, &hi, &lo);
							ModbusRTUSlaveBitAddress* a = getBitAddress(Address, Length);
							if (Length > 0 && a != NULL && lstResponse[6] == hi && lstResponse[7] == lo)
							{
								u16 stidx = (Address - a->addr) / 8;
								u16 nlen = ((Length - 1) / 8) + 1;

								byte dat[nlen];
								memset(dat, 0, nlen);

								int ng = (Address - a->addr) % 8;
								int ns = stidx;
								for (int i = 0; i < nlen; i++)
								{
									byte val = 0;
									for (int j = 0; j < 8; j++)
									{
										if (bitRead(a->values[ns], ng++)) bitSet(val, j);
										if (ng == 8) { ns++; ng = 0; }
									}
									dat[i] = val;
								}

								byte ret[3 + nlen + 2];
								ret[0] = Slave;	ret[1] = Function;	ret[2] = nlen;
								for (int i = 0; i < nlen; i++) ret[3 + i] = dat[i];
								byte hi = 0xFF, lo = 0xFF;
								getCRC(ret, 3 + nlen + 2, 0, 3 + nlen, &hi, &lo);
								ret[3 + nlen] = hi;
								ret[3 + nlen + 1] = lo;
								ser->write(ret, 3 + nlen + 2);

								ResCnt = 0;
							}
							else bvalid = false;
						}
						break;
					case 3:		//WordRead	
					case 4:
						if (ResCnt >= 8)
						{
							u16 Length = (lstResponse[4] << 8) | lstResponse[5];
							byte hi = 0xFF, lo = 0xFF;
							getCRC(lstResponse, 300, 0, 6, &hi, &lo);
							ModbusRTUSlaveWordAddress* a = getWordAddress(Address, Length);
							if (Length > 0 && a != NULL && lstResponse[6] == hi && lstResponse[7] == lo)
							{
								u16 stidx = Address - a->addr;
								u16 nlen = Length * 2;

								byte ret[3 + nlen + 2];
								ret[0] = Slave;	ret[1] = Function;	ret[2] = nlen;
								for (int i = stidx; i < stidx + Length; i++)
								{
									ret[3 + ((i - stidx) * 2) + 0] = ((a->values[i] & 0xFF00) >> 8);
									ret[3 + ((i - stidx) * 2) + 1] = ((a->values[i] & 0xFF));
								}
								byte hi = 0xFF, lo = 0xFF;
								getCRC(ret, 3 + nlen + 2, 0, 3 + nlen, &hi, &lo);
								ret[3 + nlen] = hi;
								ret[3 + nlen + 1] = lo;
								ser->write(ret, 3 + nlen + 2);
								ResCnt = 0;
							}
							else bvalid = false;
						}
						break;
					case 5:		//BitWrite
						if (ResCnt >= 8)
						{
							u16 Data = (lstResponse[4] << 8) | lstResponse[5];
							byte hi = 0xFF, lo = 0xFF;
							getCRC(lstResponse, 300, 0, 6, &hi, &lo);
							ModbusRTUSlaveBitAddress* a = getBitAddress(Address);
							if (a != NULL && lstResponse[6] == hi && lstResponse[7] == lo)
							{
								u16 stidx = (Address - a->addr) / 8;

								bitWrite(a->values[stidx], (Address - a->addr) % 8, Data == 0xFF00);

								byte ret[8];
								ret[0] = Slave;
								ret[1] = Function;
								ret[2] = ((Address & 0xFF00) >> 8);
								ret[3] = ((Address & 0x00FF));
								ret[4] = ((Data & 0xFF00) >> 8);
								ret[5] = ((Data & 0x00FF));
								byte hi = 0xFF, lo = 0xFF;
								getCRC(ret, 8, 0, 6, &hi, &lo);
								ret[6] = hi;
								ret[7] = lo;
								ser->write(ret, 8);

								ResCnt = 0;
							}
							else bvalid = false;
						}
						break;
					case 6:		//WordWrite
						if (ResCnt >= 8)
						{
							u16 Data = (lstResponse[4] << 8) | lstResponse[5];
							byte hi = 0xFF, lo = 0xFF;
							getCRC(lstResponse, 300, 0, 6, &hi, &lo);
							ModbusRTUSlaveWordAddress* a = getWordAddress(Address);
							if (a != NULL && lstResponse[6] == hi && lstResponse[7] == lo)
							{
								u16 stidx = Address - a->addr;

								a->values[stidx] = Data;

								byte ret[8];
								ret[0] = Slave;
								ret[1] = Function;
								ret[2] = ((Address & 0xFF00) >> 8);
								ret[3] = ((Address & 0x00FF));
								ret[4] = ((Data & 0xFF00) >> 8);
								ret[5] = ((Data & 0x00FF));
								byte hi = 0xFF, lo = 0xFF;
								getCRC(ret, 8, 0, 6, &hi, &lo);
								ret[6] = hi;
								ret[7] = lo;
								ser->write(ret, 8);

								ResCnt = 0;
							}
							else bvalid = false;
						}
						break;
					case 15:	//MultiBitWrite
						if (ResCnt >= 7)
						{
							u16 Length = (lstResponse[4] << 8) | lstResponse[5];
							u8 ByteCount = lstResponse[6];
							if (ResCnt >= 9 + ByteCount)
							{
								byte hi = 0xFF, lo = 0xFF;
								getCRC(lstResponse, 300, 0, 7 + ByteCount, &hi, &lo);
								if (lstResponse[(9 + ByteCount - 2)] == hi && lstResponse[(9 + ByteCount - 1)] == lo)
								{
									ModbusRTUSlaveBitAddress* a = getBitAddress(Address, Length);
									if (a != NULL)
									{
										u16 stidx = (Address - a->addr) / 8;
										int ng = (Address - a->addr) % 8;
										int ns = stidx;

										for (int i = 7; i < 7 + ByteCount; i++)
										{
											byte val = lstResponse[i];
											for (int j = 0; j < 8; j++)
											{
												bitWrite(a->values[ns], ng++, bitRead(val, j));
												if (ng == 8) { ns++; ng = 0; }
											}
										}

										if (bvalid)
										{
											byte ret[8];
											ret[0] = Slave;
											ret[1] = Function;
											ret[2] = ((Address & 0xFF00) >> 8);
											ret[3] = ((Address & 0x00FF));
											ret[4] = ((Length & 0xFF00) >> 8);
											ret[5] = ((Length & 0x00FF));
											byte hi = 0xFF, lo = 0xFF;
											getCRC(ret, 8, 0, 6, &hi, &lo);
											ret[6] = hi;
											ret[7] = lo;
											ser->write(ret, 8);

											ResCnt = 0;
										}
									}
								}
								else bvalid = false;
							}
						}
						break;
					case 16:	//MultiWordWrite
						if (ResCnt >= 7)
						{
							u16 Length = (lstResponse[4] << 8) | lstResponse[5];
							u8 ByteCount = lstResponse[6];
							if (ResCnt >= 9 + ByteCount)
							{
								byte hi = 0xFF, lo = 0xFF;
								getCRC(lstResponse, 300, 0, 7 + ByteCount, &hi, &lo);
								if (lstResponse[(9 + ByteCount - 2)] == hi && lstResponse[(9 + ByteCount - 1)] == lo)
								{
									for (int i = 7; i < 7 + ByteCount; i += 2)
									{
										u16 data = lstResponse[i] << 8 | lstResponse[i + 1];
										ModbusRTUSlaveWordAddress* a = getWordAddress(Address + ((i - 7) / 2));
										if (a != NULL) { a->values[(Address + ((i - 7) / 2)) - a->addr] = data; }
										else { bvalid = false; break; }
									}
									if (bvalid)
									{
										byte ret[8];
										ret[0] = Slave;
										ret[1] = Function;
										ret[2] = ((Address & 0xFF00) >> 8);
										ret[3] = ((Address & 0x00FF));
										ret[4] = ((Length & 0xFF00) >> 8);
										ret[5] = ((Length & 0x00FF));
										byte hi = 0xFF, lo = 0xFF;
										getCRC(ret, 8, 0, 6, &hi, &lo);
										ret[6] = hi;
										ret[7] = lo;
										ser->write(ret, 8);

										ResCnt = 0;
									}
								}
								else bvalid = false;
							}
						}
						break;
					}
				}
				else bvalid = false;
			}
			lastrecv = micros();
		}
	}

	if (ResCnt > 0)
	{
		unsigned long aaa = micros();
		if (aaa > lastrecv)
		{
			if (aaa - lastrecv > 1500) {
				ResCnt = 0;
				while (ser->available() > 0) { ser->read(); }
			}

		}
		else if (aaa < lastrecv)
		{
			if ((lastrecv - aaa) < (4294967295 - 1500))
			{
				ResCnt = 0;
				while (ser->available() > 0) { ser->read(); }
			}
		}
	}
	if (!bvalid && ResCnt > 0) {
		ResCnt = 0;
		while (ser->available() > 0) { ser->read(); }
	}
}

/*
void ModbusRTUSlave::getCRC(LinkedList<byte>* pby, int startindex, int nSize, byte* byFirstReturn, byte* bySecondReturn)
{
	int uIndex;
	byte uchCRCHi = 0xff;
	byte uchCRCLo = 0xff;
	for (int i = startindex; i < startindex + nSize && i<pby->size(); i++)
	{
		uIndex = uchCRCHi ^ pby->get(i);
		uchCRCHi = uchCRCLo ^ auchCRCHi1[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	(*byFirstReturn) = uchCRCHi;
	(*bySecondReturn) = uchCRCLo;
}
*/
void ModbusRTUSlave::getCRC(byte* pby, int arsize, int startindex, int nSize, byte* byFirstReturn, byte* bySecondReturn)
{
	int uIndex;
	byte uchCRCHi = 0xff;
	byte uchCRCLo = 0xff;
	for (int i = startindex; i < startindex + nSize && i < arsize; i++)
	{
		uIndex = uchCRCHi ^ pby[i];
		uchCRCHi = uchCRCLo ^ auchCRCHi1[uIndex];
		uchCRCLo = auchCRCLo1[uIndex];
	}
	(*byFirstReturn) = uchCRCHi;
	(*bySecondReturn) = uchCRCLo;
}

boolean getBit(u8* area, int index)
{
	u16 stidx = index / 8;
	return bitRead(area[stidx], index % 8);
}

void setBit(u8* area, int index, bool value)
{
	u16 stidx = index / 8;
	bitWrite(area[stidx], index % 8, value);
}

ModbusRTUSlaveBitAddress::ModbusRTUSlaveBitAddress(u16 Address, u8* value, int cnt)
{
	addr = Address;
	values = value;
	len = cnt;
}

ModbusRTUSlaveWordAddress::ModbusRTUSlaveWordAddress(u16 Address, u16* value, int cnt)
{
	addr = Address;
	values = value;
	len = cnt;
}


#endif


#if MODTIMER1 == 1

unsigned long MODTIMER::msecs1_m;
void (*MODTIMER::func1_m)();
volatile unsigned long MODTIMER::count1_m;
volatile char MODTIMER::overflowing1_m;
volatile unsigned int MODTIMER::tcnt1_m;

uint8_t Ntimer = 1;
unsigned long ms = 1;
bool Timer1On_m = 0;

void MODTIMER::SET(void (*f)()) {
	float prescaler = 0.0;
	if (Ntimer == 1) {
		Timer1On_m = true;
		TIMSK1 &= ~(1 << TOIE1);
		TCCR1A = 0x00;
		TCCR1B = 0x03;
		TIMSK1 &= ~(1 << OCIE1A);

		//		tcnt1_m = 64286;    //5ms
		//		tcnt1_m = 64536;	//4ms
		//		tcnt1_m = 64786;	//3ms
		//		tcnt1_m = 65036;	//2ms     
		//		tcnt1_m = 65286;    //1ms
//		tcnt1_m = 65411;    //0.5ms
		tcnt1_m = 65436;    //0.4ms
//		tcnt1_m = 65461;    //0.3ms
//		tcnt1_m = 65486;    //0.2ms
//		tcnt1_m = 65511;    //0.1ms
		if (ms == 0)
			msecs1_m = 1;
		else
			msecs1_m = ms;

		func1_m = f;
	}
	count1_m = 0;
	overflowing1_m = 0;
	TCNT1 = tcnt1_m;
	TIMSK1 |= (1 << TOIE1);
	MODTIMER::START();
}
void MODTIMER::START(uint8_t Numstart) {
	if (((Timer1On_m == true) && (Numstart == 10)) || ((Timer1On_m == true) && (Numstart == 1))) {
		count1_m = 0;
		overflowing1_m = 0;
		TCNT1 = tcnt1_m;
		TIMSK1 |= (1 << TOIE1);
	}
}

void MODTIMER::STOP(uint8_t Numstop) {
	if (((Timer1On_m == true) && (Numstop == 1)) || ((Timer1On_m == true) && (Numstop == 10))) {
		TIMSK1 &= ~(1 << TOIE1);
	}
}
void MODTIMER::_overflow1_m() {
	count1_m += 1;
	if (count1_m >= msecs1_m && !overflowing1_m) {
		overflowing1_m = 1;
		count1_m = 0;
		(*func1_m)();
		overflowing1_m = 0;
	}
}
#endif
#if MODTIMER1 == 2

unsigned long MODTIMER::msecs1_m;
void (*MODTIMER::func1_m)();
volatile unsigned long MODTIMER::count1_m;
volatile char MODTIMER::overflowing1_m;
volatile unsigned int MODTIMER::tcnt1_m;

uint8_t Ntimer = 1;
unsigned long ms = 1;
bool Timer1On_m = 0;

void MODTIMER::SET(void (*f)()) {
	float prescaler = 0.0;
	if (Ntimer == 1) {
		Timer1On_m = true;
		TIMSK &= ~(1 << TOIE1);
		TCCR1A = 0x00;
		TCCR1B = 0x03;
		TIMSK &= ~(1 << OCIE1A);

		//		tcnt1_m = 64286;    //5ms
		//		tcnt1_m = 64536;	//4ms
		//		tcnt1_m = 64786;	//3ms
		//		tcnt1_m = 65036;	//2ms     
		//		tcnt1_m = 65286;    //1ms
//		tcnt1_m = 65411;    //0.5ms
		tcnt1_m = 65436;    //0.4ms
//		tcnt1_m = 65461;    //0.3ms
//		tcnt1_m = 65486;    //0.2ms
//		tcnt1_m = 65511;    //0.1ms
		if (ms == 0)
			msecs1_m = 1;
		else
			msecs1_m = ms;

		func1_m = f;
	}
	count1_m = 0;
	overflowing1_m = 0;
	TCNT1 = tcnt1_m;
	TIMSK |= (1 << TOIE1);
	MODTIMER::START();
}
void MODTIMER::START(uint8_t Numstart) {
	if (((Timer1On_m == true) && (Numstart == 10)) || ((Timer1On_m == true) && (Numstart == 1))) {
		count1_m = 0;
		overflowing1_m = 0;
		TCNT1 = tcnt1_m;
		TIMSK |= (1 << TOIE1);
	}
}

void MODTIMER::STOP(uint8_t Numstop) {
	if (((Timer1On_m == true) && (Numstop == 1)) || ((Timer1On_m == true) && (Numstop == 10))) {
		TIMSK &= ~(1 << TOIE1);
	}
}
void MODTIMER::_overflow1_m() {
	count1_m += 1;
	if (count1_m >= msecs1_m && !overflowing1_m) {
		overflowing1_m = 1;
		count1_m = 0;
		(*func1_m)();
		overflowing1_m = 0;
	}
}
#endif // MODTIMER1

#if TIMER_OVF == 1

ISR(TIMER1_OVF_vect) {
	if (Timer1On == true) {
		TCNT1 = ITIMER::tcnt1;
		ITIMER::_overflow1();
	}
	if (Timer1On_m == true) {
		TCNT1 = MODTIMER::tcnt1_m;
		MODTIMER::_overflow1_m();
	}
	if (tcntsetupon1 == 1) {
		tcntPlus1++;
		TCNT1 = 0;
	}
	if (npwmON11 == 1) {
		TimerCount1A++;
	}
	if (npwmON12 == 1) {
		TimerCount1B++;
	}
	if (npwmON13 == 1) {
		TimerCount1C++;
	}
	if (TIMSK1 == 0x01 && TCCR1A == 0x00 && TCCR1B == 0x07) {
		OCR1A++;
	}
}
#if defined(TIMER3_OVF_vect)
#else
ISR(TIMER3_OVF_vect) {
	if (Timer3On == true) {
		TCNT3 = ITIMER::tcnt3;
		ITIMER::_overflow3();
	}
	if (tcntsetupon3 == 1) {
		tcntPlus3++;
	}
	if (npwmON2 == 1) {
		TimerCount3B++;
	}
	if (npwmON3 == 1) {
		TimerCount3C++;
	}
	if (npwmON5 == 1) {
		TimerCount3A++;
	}
}

#endif
#if defined(TIMER4_OVF_vect)
#else

ISR(TIMER4_OVF_vect) {
	if (Timer4On == true) {
		TCNT4 = ITIMER::tcnt4;
		ITIMER::_overflow4();
	}
	if (tcntsetupon4 == 1) {
		tcntPlus4++;
		TCNT4 = 0;
	}
	if (npwmON6 == 1) {
		TimerCount4A++;
	}
	if (npwmON7 == 1) {
		TimerCount4B++;
	}
	if (npwmON8 == 1) {
		TimerCount4C++;
	}
}


#endif
#if defined(TIMER5_OVF_vect)
#else

ISR(TIMER5_OVF_vect) {
	if (Timer5On == true) {
		TCNT5 = ITIMER::tcnt5;
		ITIMER::_overflow5();
	}
	if (tcntsetupon5 == 1) {
		tcntPlus5++;
		TCNT5 = 0;
	}
	if (npwmON44 == 1) {
		TimerCount5C++;
	}
	if (npwmON45 == 1) {
		TimerCount5B++;
	}
	if (npwmON46 == 1) {
		TimerCount5A++;
	}

}
#endif 

#endif

#if TIMER_OVF == 2


#if defined(TIMER1_OVF_vect)
#else
ISR(TIMER1_OVF_vect) {
	if (Timer1On == true) {
		TCNT1 = ITIMER::tcnt1;
		ITIMER::_overflow1();
	}
	if (Timer1On_m == true) {
		TCNT1 = MODTIMER::tcnt1_m;
		MODTIMER::_overflow1_m();
	}
	if (tcntsetupon1 == 1) {
		tcntPlus1++;
		TCNT1 = 0;
	}
	if (npwmON26 == 1) {
		TimerCount1A++;
	}
	if (npwmON27 == 1) {
		TimerCount1B++;
	}
	if (npwmON28 == 1) {
		TimerCount1C++;
	}
}
#endif
#if defined(TIMER3_OVF_vect)
#else
ISR(TIMER3_OVF_vect) {
	if (Timer3On == true) {
		TCNT3 = ITIMER::tcnt3;
		ITIMER::_overflow3();
	}
	if (tcntsetupon3 == 1) {
		tcntPlus3++;
		TCNT3 = 0;
	}
	if (npwmON21 == 1) {
		TimerCount3A++;
	}
	if (npwmON22 == 1) {
		TimerCount3B++;
	}
	if (npwmON23 == 1) {
		TimerCount3C++;
	}
}

#endif
#endif