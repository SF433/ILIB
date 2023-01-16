/*2560PLC
3번 타이머 
4번 타이머
1번 타이머
5번 타이머*/

/*128PLC
3번 타이머(24Pin) */

/*16bit
#include "ILIB.h"
unsigned int HCNT5;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  TCNTSETUP(5);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(TCNT5);
  HCNT5 = TCNT5; // 타이머5 카운트값을 HCNT5 변수에 저장
}
void hcntReset() {
  // 타이머5 카운트값을 0으로 리셋
  TCNT5 = 0;
}
*/
//32bit
#include "ILIB.h"

unsigned long HCNT5;

void setup(void){
  //TCNTSETUP(타이머번호, 32비트유무)
  //32비트를 사용하시려면 1, 사용하지 않으려면 0입력 또는 빈칸으로 씁니다.
  TCNTSETUP(5, 1);
  Serial.begin(115200);
}

void loop(void){

  //TCNTOUT(타이머번호);
  HCNT5 = (unsigned long)TCNTOUT(5) << 16 | TCNT5;
  Serial.println(HCNT5);
}
