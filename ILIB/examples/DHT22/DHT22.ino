#include <ILIB.h>
#define DHTPIN 13        // GPIO 핀의 설정
#define DHTTYPE DHT22   // DHT22 (AM2302) 센서종류 설정

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200); 
  Serial.println("DHT22 TEST !!!");
  
 dht.BEGIN();
}
void loop(){
  // 센서의 온도와 습도를 읽어온다.
  float h = dht.READ_HUMI();
  float t = dht.READ_TEMP();
  
  if (isnan(t) || isnan(h)) {
    //값 읽기 실패시 시리얼 모니터 출력
    Serial.println("Failed to read from DHT");
    }else{
    //온도, 습도 표시 시리얼 모니터 출력
    
    Serial.print("Humidity: "); 
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: "); 
    Serial.print(t);
    Serial.println(" *C");
    
  }
  delay(2000);
}
