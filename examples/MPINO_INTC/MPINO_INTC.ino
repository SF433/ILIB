
#include "ILIB.h"

void setup(void) {
  analogReference(EXTERNAL);
  Serial.begin(9600);
} 

void loop(void) {
//int INTC(int pin);
//INTC()함수는 int 정수형으로 반환됩니다.
//MPINO-16A8R8T제품기준 T(4)번핀은 4로 입력 T(5)번핀은 5로 입력
  Serial.println(INTC(4));       //출력값 225 => 22.5'C

//float INTC1(int32_t _ch, float _bValue);
//INTC1()함수는 float 소수형으로 반환됩니다.
//MPINO-16A8R8T제품기준 T(4)번핀은 4로 입력 T(5)번핀은 5로 입력
//_bValue값은 생략시 3950값으로 입력됩니다.
  
  Serial.println(INTC1(4));       //출력값 22.04
  Serial.println(INTC1(4),1);     //출력값 22.0
  Serial.println(INTC1(4),5);     //출력값 22.04025
  Serial.println((int)INTC1(4));  //출력값 22
}
