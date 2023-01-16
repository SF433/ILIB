/*2560PLC
**2, 3, 5 => 3번 타이머 사용
**6, 7, 8 => 4번 타이머 사용
**11, 12, 13 => 1번 타이머 사용
**44, 45, 46 => 5번 타이머 사용*/

/*128PLC
**21, 22, 23 => 3번 타이머 사용
**26, 27, 28 => 1번 타이머 사용*/

/*
**같은 타이머를 사용할 시 주파수를 동일하게 하여야 정상동작합니다. 
**ex) PWM(21,10000,1); PWM(22,32767,1); => 정상작동 
**    PWM(21,127,0); PWM(22,32767,1); => 비정상작동
*/
#include "ILIB.h"
int i = 0;
void setup() {
  PWM_RESET();
}
void loop() {
  //PWM(uint8_t pin, int val, uint8_t onDutybit16);
  //onDutybit16값에 따라 val 출력범위가 달라집니다.
  //onDutybit16값이 0값일 경우 val 범위 0~255   490Hz
  //onDutybit16값이 1값일 경우 val 범위 0~65535  244Hz
  
  PWM(11,127,0);
  PWM(44,20000,1);
  PWM(5,30000,1);
  PWM(6,40000,1);
}
