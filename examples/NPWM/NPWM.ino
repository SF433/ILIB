/*2560PLC
**2, 3, 5 => 3번 타이머 사용
**6, 7, 8 => 4번 타이머 사용
**11, 12, 13 => 1번 타이머 사용
**44, 45, 46 => 5번 타이머 사용*/

/*128PLC
**21, 22, 23 => 3번 타이머 사용
**26, 27, 28 => 1번 타이머 사용*/
/*
**같은 타이머를 사용할 시 주파수를 동일하게 하여야
**정상동작합니다. ex) 21번 주파수를 3000으로 하고 22번 주파수를 2000으로 할 수는 없습니다.
*/

#include "ILIB.h"
int SerialInput = 0;
void setup() {
  Serial.begin(115200);
  PWM_RESET();
}


void loop() {
  if (Serial.available() > 0) {
    SerialInput = Serial.read();
  }
  
  //시리얼 모니터에 1번 입력시
  if (SerialInput == 49) {
         //1번모듈 PWM0번핀 200Hz, Duty비 30%, 4번 출력
  NPWM_BEGIN(5, 200, 30, 4);
    SerialInput = 0;
  }
     
  //시리얼 모니터에 2번 입력시
 else if (SerialInput == 50) {
         //1번모듈 PWM3번핀 2kHz, Duty비 70%, 7번 출력
  NPWM_BEGIN(44, 2000, 70, 7);
         //1번모듈 PWM4번핀 2kHz, Duty비 40%, 5번 출력
  NPWM_BEGIN(45, 2000, 40, 5);
    SerialInput = 0;
  }
    
  //시리얼 모니터에 3번 입력시
  else if (SerialInput == 51) {
         //2번모듈 PWM0번핀 2kHz, Duty비 40%, 4번 출력
  NPWM_BEGIN(5, 2000, 40, 4);
         //2번모듈 PWM3번핀 2KHz, Duty비 60%, 2번 출력
  NPWM_BEGIN(6, 2000, 60, 2);
    SerialInput = 0;
  }
  NPWM(5);
  NPWM(6);
  NPWM(44);
  NPWM(45);
}
