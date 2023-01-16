#include "ILIB.h"

//클래스 선언
IADC iadc1(1); //X1 : X모델 1개
IADC iadc2(2); //X2 : X모델 2개
IADC iadc3(3); //X3 : X모델 3개
IADC iadc4(4); //X4 : X모델 4개
IADC iadc5(5); //X5 : X모델 5개

int X1_ADC0 = 0, X1_ADC1 = 0, X1_ADC2 = 0, X1_ADC3 = 0;
int X2_ADC0 = 0, X2_ADC1 = 0, X2_ADC2 = 0, X2_ADC3 = 0;
int X3_ADC0 = 0, X3_ADC1 = 0, X3_ADC2 = 0, X3_ADC3 = 0;
int X4_ADC0 = 0, X4_ADC1 = 0, X4_ADC2 = 0, X4_ADC3 = 0;
int X5_ADC0 = 0, X5_ADC1 = 0, X5_ADC2 = 0, X5_ADC3 = 0;
int Temp0 = 0;
int Temp1 = 0;
int Temp2 = 0;
int Temp3 = 0;

void  setup()  {
  Serial.begin(115200);
  //NTC 모드로 사용할시 0 입력
  iadc1.INIT(0);  // IADC 함수 초기화
  iadc2.INIT(0);  // IADC 함수 초기화
  iadc3.INIT(0);  // IADC 함수 초기화
  iadc4.INIT(0);  // IADC 함수 초기화
  iadc5.INIT(0);  // IADC 함수 초기화
}


void loop()  {
  //첫번째 X모듈에 입력된 값을 변수 ADC0에 저장합니다.
  X1_ADC0 = iadc1.GET_ADC(0);
  X1_ADC1 = iadc1.GET_ADC(1);
  X1_ADC2 = iadc1.GET_ADC(2);
  X1_ADC3 = iadc1.GET_ADC(3);

  //온도값으로 변환합니다.
  Temp0 = INTC(X1_ADC0);
  Temp1 = INTC(X1_ADC1);
  Temp2 = INTC(X1_ADC2);
  Temp3 = INTC(X1_ADC3);
  //출력값 256  -> 25.6'C
/*  온도값으로 변환합니다.
  Temp0 = INTC1(X1_ADC0);
  Temp1 = INTC1(X1_ADC1);
  Temp2 = INTC1(X1_ADC2);
  Temp3 = INTC1(X1_ADC3);
  //출력값 25.67  -> 25.67'C
*/
  Serial.println();
  Serial.print("첫번째 X모듈 0CH 온도값");
  Serial.println(Temp0);
  Serial.print("첫번째 X모듈 1CH 온도값");
  Serial.println(Temp1);
  Serial.print("첫번째 X모듈 2CH 온도값");
  Serial.println(Temp2);
  Serial.print("첫번째 X모듈 3CH 온도값");
  Serial.println(Temp3);

  //두번째 X모듈에 입력된 값을 변수 ADC0에 저장합니다.

  X2_ADC0 = iadc2.GET_ADC(0);
  X2_ADC1 = iadc2.GET_ADC(1);
  X2_ADC2 = iadc2.GET_ADC(2);
  X2_ADC3 = iadc2.GET_ADC(3);

  //온도값으로 변환합니다.

  Temp0 = INTC(X2_ADC0);
  Temp1 = INTC(X2_ADC1);
  Temp2 = INTC(X2_ADC2);
  Temp3 = INTC(X2_ADC3);

  //출력값 256  -> 25.6'C
  Serial.println();
  Serial.print("두번째 X모듈 0CH 온도값");
  Serial.println(Temp0);
  Serial.print("두번째 X모듈 1CH 온도값");
  Serial.println(Temp1);
  Serial.print("두번째 X모듈 2CH 온도값");
  Serial.println(Temp2);
  Serial.print("두번째 X모듈 3CH 온도값");
  Serial.println(Temp3);

  //세번째 X모듈에 입력된 값을 변수 ADC0에 저장합니다.
  X3_ADC0 = iadc3.GET_ADC(0);
  X3_ADC1 = iadc3.GET_ADC(1);
  X3_ADC2 = iadc3.GET_ADC(2);
  X3_ADC3 = iadc3.GET_ADC(3);

  //온도값으로 변환합니다.
  Temp0 = INTC(X3_ADC0);
  Temp1 = INTC(X3_ADC1);
  Temp2 = INTC(X3_ADC2);
  Temp3 = INTC(X3_ADC3);

  //출력값 256  -> 25.6'C
  Serial.println();
  Serial.print("세번째 X모듈 0CH 온도값");
  Serial.println(Temp0);
  Serial.print("세번째 X모듈 1CH 온도값");
  Serial.println(Temp1);
  Serial.print("세번째 X모듈 2CH 온도값");
  Serial.println(Temp2);
  Serial.print("세번째 X모듈 3CH 온도값");
  Serial.println(Temp3);
  //네번째 X모듈에 입력된 값을 변수 ADC0에 저장합니다.
  X4_ADC0 = iadc4.GET_ADC(0);
  X4_ADC1 = iadc4.GET_ADC(1);
  X4_ADC2 = iadc4.GET_ADC(2);
  X4_ADC3 = iadc4.GET_ADC(3);

  //온도값으로 변환합니다.
  Temp0 = INTC(X4_ADC0);
  Temp1 = INTC(X4_ADC1);
  Temp2 = INTC(X4_ADC2);
  Temp3 = INTC(X4_ADC3);

  //출력값 256  -> 25.6'C
  Serial.println();
  Serial.print("네번째 X모듈 0CH 온도값");
  Serial.println(Temp0);
  Serial.print("네번째 X모듈 1CH 온도값");
  Serial.println(Temp1);
  Serial.print("네번째 X모듈 2CH 온도값");
  Serial.println(Temp2);
  Serial.print("네번째 X모듈 3CH 온도값");
  Serial.println(Temp3);

  //다섯번째 X모듈에 입력된 값을 변수 ADC0에 저장합니다.
  X5_ADC0 = iadc5.GET_ADC(0);
  X5_ADC1 = iadc5.GET_ADC(1);
  X5_ADC2 = iadc5.GET_ADC(2);
  X5_ADC3 = iadc5.GET_ADC(3);

  //온도값으로 변환합니다.
  Temp0 = INTC(X5_ADC0);
  Temp1 = INTC(X5_ADC1);
  Temp2 = INTC(X5_ADC2);
  Temp3 = INTC(X5_ADC3);

  //출력값 256  -> 25.6'C
  Serial.println();
  Serial.print("다섯번째 X모듈 0CH 온도값");
  Serial.println(Temp0);
  Serial.print("다섯번째 X모듈 1CH 온도값");
  Serial.println(Temp1);
  Serial.print("다섯번째 X모듈 2CH 온도값");
  Serial.println(Temp2);
  Serial.print("다섯번째 X모듈 3CH 온도값");
  Serial.println(Temp3);
}
