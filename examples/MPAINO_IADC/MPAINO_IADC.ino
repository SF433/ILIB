#include "ILIB.h"
//IADC iadc1(모듈번호);
IADC iadc1(1);  
IADC iadc2(2);  
void  setup()  {
    Serial.begin(115200);  // 다운로드 포트의 보레이트를 115200으로 시작
//  iadc1.INIT(최댓값, 최솟값, SPS값(1~8));
/*    
SPS값1:   8 samples/s, Tconv=125ms
SPS값2:  16 samples/s, Tconv=62.5ms
SPS값3:  32 samples/s, Tconv=31.25ms
SPS값4:  64 samples/s, Tconv=15.625ms
SPS값5: 128 samples/s, Tconv=7.8125ms
SPS값6: 250 samples/s, Tconv=4ms
SPS값7: 475 samples/s, Tconv=2.105ms
SPS값8: 860 samples/s, Tconv=1.163ms
*/
    iadc1.INIT(30000, 0, 4);

//NTC로 설정시 0값만 입력
    iadc2.INIT(0);
}

void loop()  {
  Serial.print("GETADC : ");
//1번 모듈의 0CH 값 출력(0V~5V, 0~20mA)
  Serial.println(iadc1.GET_ADC(0));  
//1번 모듈의 1CH 값 출력(1V~5V, 4~20mA)
  Serial.println(iadc1.GET_ADC(1, 1));  

//2번 모듈의 0CH 값 출력(NTC)
  Serial.println(INTC(iadc2.GET_ADC(0)));  
}
