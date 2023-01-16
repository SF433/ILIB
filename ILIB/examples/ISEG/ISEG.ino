#include "ILIB.h"
/*2560PLC*/
#ifdef __AVR_ATmega2560__
#define CLK 13//clk_pinNumber
#define DIO 12//dio_pinNumber
#endif

/*128PLC*/
#ifdef MPINO128
#define CLK 24//clk_pinNumber
#define DIO 25//dio_pinNumber
#endif
// The amount of time (in milliseconds) between tests
#define TEST_DELAY   2000

const uint8_t SEG_DONE[] = {
	SEG_B | SEG_C | SEG_D | SEG_E | SEG_G,           // d
	SEG_A | SEG_B | SEG_C | SEG_D | SEG_E | SEG_F,   // O
	SEG_C | SEG_E | SEG_G,                           // n
	SEG_A | SEG_D | SEG_E | SEG_F | SEG_G            // E
	};

ISEG iseg(CLK, DIO);

void setup()
{
}

void loop()
{
  int k;
  uint8_t data[] = { 0xff, 0xff, 0xff, 0xff };
  uint8_t blank[] = { 0x00, 0x00, 0x00, 0x00 };
  iseg.SET_BRIGHT(0x0f);

  // All segments on
  iseg.SET_SEG(data);
  delay(TEST_DELAY);

  // Selectively set different digits
  data[0] = iseg.encodeDigit(0);
  data[1] = iseg.encodeDigit(1);
  data[2] = iseg.encodeDigit(2);
  data[3] = iseg.encodeDigit(3);
  iseg.SET_SEG(data);
  delay(TEST_DELAY);

  /*
  for(k = 3; k >= 0; k--) {
	iseg.SET_SEG(data, 1, k);
	delay(TEST_DELAY);
	}
  */

  iseg.CLEAR();
  iseg.SET_SEG(data+2, 2, 2);
  delay(TEST_DELAY);

  iseg.CLEAR();
  iseg.SET_SEG(data+2, 2, 1);
  delay(TEST_DELAY);

  iseg.CLEAR();
  iseg.SET_SEG(data+1, 3, 1);
  delay(TEST_DELAY);


  // Show decimal numbers with/without leading zeros
  iseg.NUM_DEC(0, false); // Expect: ___0
  delay(TEST_DELAY);
  iseg.NUM_DEC(0, true);  // Expect: 0000
  delay(TEST_DELAY);
	iseg.NUM_DEC(1, false); // Expect: ___1
	delay(TEST_DELAY);
  iseg.NUM_DEC(1, true);  // Expect: 0001
  delay(TEST_DELAY);
  iseg.NUM_DEC(301, false); // Expect: _301
  delay(TEST_DELAY);
  iseg.NUM_DEC(301, true); // Expect: 0301
  delay(TEST_DELAY);
  iseg.CLEAR();
  iseg.NUM_DEC(14, false, 2, 1); // Expect: _14_
  delay(TEST_DELAY);
  iseg.CLEAR();
  iseg.NUM_DEC(4, true, 2, 0);  // Expect: 04__
  delay(TEST_DELAY);
  iseg.NUM_DEC(-1, false);  // Expect: __-1
  delay(TEST_DELAY);
  iseg.NUM_DEC(-12);        // Expect: _-12
  delay(TEST_DELAY);
  iseg.NUM_DEC(-999);       // Expect: -999
  delay(TEST_DELAY);
  iseg.CLEAR();
  iseg.NUM_DEC(-5, false, 3, 0); // Expect: _-5_
  delay(TEST_DELAY);
  iseg.NUM_HEX_EX(0xf1af);        // Expect: f1Af
  delay(TEST_DELAY);
  iseg.NUM_HEX_EX(0x2c);          // Expect: __2C
  delay(TEST_DELAY);
  iseg.NUM_HEX_EX(0xd1, 0, true); // Expect: 00d1
  delay(TEST_DELAY);
  iseg.CLEAR();
  iseg.NUM_HEX_EX(0xd1, 0, true, 2); // Expect: d1__
  delay(TEST_DELAY);
  
	// Run through all the dots
	for(k=0; k <= 4; k++) {
		iseg.NUM_DEC_EX(0, (0x80 >> k), true);
		delay(TEST_DELAY);
	}

  // Brightness Test
  for(k = 0; k < 4; k++)
	data[k] = 0xff;
  for(k = 0; k < 7; k++) {
    iseg.SET_BRIGHT(k);
    iseg.SET_SEG(data);
    delay(TEST_DELAY);
  }
  
  // On/Off test
  for(k = 0; k < 4; k++) {
    iseg.SET_BRIGHT(7, false);  // Turn off
    iseg.SET_SEG(data);
    delay(TEST_DELAY);
    iseg.SET_BRIGHT(7, true); // Turn on
    iseg.SET_SEG(data);
    delay(TEST_DELAY);  
  }

 
  // Done!
  iseg.SET_SEG(SEG_DONE);

  while(1);
}
