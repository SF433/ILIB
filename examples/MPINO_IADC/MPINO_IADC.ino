#include "ILIB.h"

//IADC iadc1(0);//MPINO 제품시리즈를 사용하신다면 0을 입력하시면 됩니다.
IADC iadc1(0);

void setup() {
  //  다운로드 포트의 보레이트를 115200으로 시작
  Serial.begin(115200);
  //  iadc1.INIT(최댓값, 최솟값);
  iadc1.INIT(50,0);
}

void loop() {
//MPINO-16A8R8T제품 기준 설명입니다.
//GET_ADC(입력핀, 1V~5V or 4~20mA 범위일 때 1)
//AI(0) 값 출력(1V~5V, 4~20mA)
  Serial.print("CH0: ");
  Serial.println(iadc1.GET_ADC(A0,1));
//GET_ADC(입력핀, 0V~5V or 0~20mA 범위일 때 0 또는 빈칸)
//AI(1) 값 출력(0V~5V, 0~20mA)
  Serial.print("CH1: ");
  Serial.println(iadc1.GET_ADC(A1));
  Serial.println();
  delay(100);
}
