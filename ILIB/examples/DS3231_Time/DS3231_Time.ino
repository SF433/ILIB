#include "ILIB.h"

DS3231 time1;
RTClib myRTC;

char weekDay[4];
int day;

void setup() {
 // Start the I2C interface
  Wire.begin();
  // Start the serial interface
  Serial.begin(9600);
  //(day of weeks, year, month, day, hour, minute, second);
  time1.SET_ADJUST(5,21,4,1,11,21,0);
  //day of weeks ex) -> 1:Sun 2:Mon 3:Tue 4:Wen 5:Thu 6:Fri 7:Sat 
  time1.SET_CLOCK_MODE(0);
  day = time1.GET_DOW();
  switch (day) {
    case 1:
      strcpy(weekDay, "Sun");
      break;
    case 2:
      strcpy(weekDay, "Mon");
      break;
    case 3:
      strcpy(weekDay, "Tue");
      break;
    case 4:
      strcpy(weekDay, "Wed");
      break;
    case 5:
      strcpy(weekDay, "Thu");
      break;
    case 6:
      strcpy(weekDay, "Fri");
      break;
    case 7:
      strcpy(weekDay, "Sat");
      break;
  }
}

void loop() {
    DATE_TIME NOW = myRTC.NOW();
    
    Serial.print(NOW.YEAR(), DEC);
    Serial.print('/');
    Serial.print(NOW.MONTH(), DEC);
    Serial.print('/');
    Serial.print(NOW.DAY(), DEC);
    Serial.print(' ');
    Serial.print(weekDay);
    Serial.print(' ');
    Serial.print(NOW.HOUR(), DEC);
    Serial.print(':');
    Serial.print(NOW.MINUTE(), DEC);
    Serial.print(':');
    Serial.print(NOW.SECOND(), DEC);
    Serial.println();
   delay(1000);
}
