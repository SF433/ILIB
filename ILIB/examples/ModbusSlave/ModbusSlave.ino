#include "ILIB.h"

u16 _D[100];
u8 _M[50];

//2560PLC
//ModbusRTUSlave rtu(SlaveNumber, &Serial1, baudrate);
//232통신일 경우 &Serial1, 485통신일 경우 &Serial2, UART 통신일 경우 &Serial3
#define SLAVE_N 1
#define BAUDRATE 115200
ModbusRTUSlave rtu(SLAVE_N, &Serial2, BAUDRATE);

//128PLC
//ModbusRTUSlave rtu(SlaveNumber, &Serial1, baudrate);
//485통신일 경우 &Serial1
//#define SLAVE_N 1
//#define BAUDRATE 115200
//ModbusRTUSlave rtu(SLAVE_N, &Serial1, BAUDRATE);

void rtu_process()
{
  rtu.process();
}
void setup() {
//  pinMode(62, OUTPUT);
  
  //Word주소 설정 (첫주소,데이터형, 데이터 개수)
  rtu.addWordArea(7000, _D, 100);
  //Bit주소 설정
  rtu.addBitArea(1000, _M, 50);
  _D[0] = 0x1111;
  _D[1] = 0x0457;
  _D[2] = 0x08AE;
  _D[3] = 0x0D05;
  //만약 ITIMER를 사용하신다면 
  //MODTIMER, void rtu_process()를 제거하시고 메인 Loop문에 rtu.process();를 입력하시거나
  //ITIMER의 1번타이머를 제외한 3번(128PLC인 경우), 4번, 5번(4,5 2560PLC인 경우)타이머를 사용하시면 됩니다.
  MODTIMER::SET(rtu_process); // 0.5ms period
}
void loop() {
  delay(1000);
  _D[1] += 0x0001;
//digitalWrite(62, getBit(_M,0));  //_M[0]값이 HIGH값이면 62번 핀 HIGH 출력합니다.
}
