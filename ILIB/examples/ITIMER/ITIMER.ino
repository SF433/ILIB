#include "ILIB.h"
float i = 0,q=0;
int k = 0;
void flash()
{
  i += 0.5;
  Serial.println(i,1);
}
void flash1()
{
  q += 0.5;
  Serial.println(q,1);
}

void setup()
{
  Serial.begin(115200);
//ITIMER::SET(TimerNumber,  Interver_sec(ms), function_Name);
//          1,4,5 3개의 타이머 중 택1,반복 실행할 간격, 함수이름
  ITIMER::SET(1, 1000, flash); // 1000ms period
  ITIMER::SET(4, 1000, flash1); // 1000ms period

//ITIMER::START(TimerNumber)
//TimerNumber값이 빈칸이면 SET으로 설정된 타이머 전부 실행
//TimerNumber값이 있으면 TimerNumber값의 타이머 실행
//EX)ITIMER::START(1); 1번 타이머 실행
  ITIMER::START(1);
  delay(500);
  ITIMER::START(4);
}

void loop()
{
  if(i == 10.0){
//ITIMER::STOP(TimerNumber)
//TimerNumber값이 빈칸이면 SET으로 설정된 타이머 전부 멈춤
//TimerNumber값이 있으면 TimerNumber값의 타이머 멈춤
//EX)ITIMER::STOP(1); 1번 타이머 멈춤
    ITIMER::STOP(1);
  }

}
