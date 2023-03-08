#include "ILIB.h"
void setup() {
  Serial.begin(115200);
  Serial.println("start");
  /*Atmega128 cpu WDTO list 
   * 통신채널개수가 1개인 제품
  WDTO_14MS
  WDTO_28MS
  WDTO_56MS
  WDTO_110MS
  WDTO_220MS
  WDTO_450MS
  WDTO_900MS
  WDTO_1D8S
  */
  /*Atmega2560 cpu WDTO list
   * 통신채널개수가 3개인 제품
  WDTO_16MS
  WDTO_32MS
  WDTO_64MS
  WDTO_125MS
  WDTO_250MS
  WDTO_500MS
  WDTO_1S
  WDTO_2S
  WDTO_4S
  WDTO_8S
  */
  WDT_ENABLE(WDTO_8S);
  //WDT_DISENABLE();
}

void loop() {
  byte a = 0;
  Serial.print("Time: ");
  Serial.println(millis());
  if (Serial.available()) {
    a = Serial.read();
  }
  if (a == '1') {
    Serial.println("WDT reset");
    WDT_RESET();
  }
}
