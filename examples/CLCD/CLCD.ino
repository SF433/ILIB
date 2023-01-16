#include "ILIB.h"

CLCD lcd(0x27,20,4);  

void setup()
{
  //lcd 초기화
  lcd.init();
  //lcd 백라이트 ON
  lcd.backlight();
  delay( 3000 );
  //lcd 백라이트 OFF
  lcd.noBacklight();
  delay( 3000 );
  //lcd 백라이트 ON
  lcd.backlight();
  //lcd 커서위치 설정(x좌표, y좌표)
  lcd.setCursor( 1, 0 );
  lcd.print( "Hello, world!" );
  delay( 2000 );
  //lcd 화면 꺼짐
  lcd.noDisplay();
  delay( 2000 );
  //lcd 화면 켜짐
  lcd.display();
  delay( 2000 );
  lcd.setCursor( 19, 1 );
  //우에서 좌로 글쓰기 설정
  lcd.rightToLeft();
  lcd.print( "hi" );
  delay( 2000 );
  lcd.setCursor( 0, 1 );
  //좌에서 우로 글쓰기 설정
  lcd.leftToRight();
  lcd.print( "hello" );
  delay( 2000 );
  //우측으로 1칸 이동하기
  lcd.scrollDisplayRight();
  delay( 1000 );
  //좌측으로 1칸 이동하기
  lcd.scrollDisplayLeft();
  delay( 1000 );
  //lcd 내용 지우기
  lcd.clear();
  //출력내용을 자동으로 우에서 좌로 스크롤
  lcd.autoscroll();
  lcd.print( "0123456789abcd" );
  delay( 5000 );
  //lcd 커서 ON
  lcd.cursor_on();
  //lcd 깜빡임 ON
  lcd.blink_on();
}

void loop()
{
}
