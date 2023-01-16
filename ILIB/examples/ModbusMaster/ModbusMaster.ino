#include "ILIB.h"
// 모드버스마스터 클래스 선언
ModbusMaster node;
void setup()
{
  //2560PLC
  // .begin(통신할 슬레이브주소, 통신시리얼, 보드레이트)
  //통신시리얼 Serial1은 232통신, Serial2는 485통신, Serial3은 UART통신입니다.
  node.begin(1, &Serial2, 115200);
  
  //128PLC
  // .begin(통신할 슬레이브주소, 통신시리얼, 보드레이트)
  //통신시리얼 Serial1은 485통신입니다.
  //node.begin(1, &Serial1, 115200);
  
}


void loop()
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[6];
  
  i++;
  
  //.setTransmitBuffer(index, 워드값)
  //여러개의 비트 또는 워드를 쓰기하기 위해 사용하는 버퍼에 값을 쓰는 함수입니다.
  //index는 0 ~ 63까지 사용할 수 있습니다. 62개의 워드 메모리입니다.
  //Return : byte (0은 성공, 2는 실패)​
  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  node.setTransmitBuffer(0, lowWord(i));
  // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  node.setTransmitBuffer(1, highWord(i));


  // .writeMultipleRegisters(데이터 어드레스, 워드개수)
  //슬레이브 디바이스의 데이터 어드레스부터 워드개수만큼 워드를 쓰기합니다.
  // slave: write TX buffer to (2) 16-bit registers starting at register 0x0000
  result = node.writeMultipleRegisters(0x0000, 1);


  // .readHoldingRegisters(데이터 어드레스, 워드개수)
  // 슬레이브 디바이스의 데이터 어드레스부터 워드개수만큼 워드를 읽어 옵니다.
  // receive() 명령어로 읽어온 워드를 확인할 수 있습니다.
  // Return : byte
  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(2, 6);

  // 데이터가 정상적으로 받았을 때
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 6; j++)
    {
      // .getResponseBuffer(index)
      // Read관련 명령어를 실행 후, 받은 데이터중 index번호에 해당하는 워드를 반환합니다.
      // Read관련 명령어 실행한 이후, 받은 데이터의 첫번째 워드 index는 0입니다
      data[j] = node.getResponseBuffer(j);
    }
  }
}
