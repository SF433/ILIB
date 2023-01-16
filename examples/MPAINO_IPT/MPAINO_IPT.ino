#include "ILIB.h"
//PTname(moduleNumber);
IADC PT1(1);
IADC PT2(2);
IADC PT3(3);
IADC PT4(4);
IADC PT5(5);
void setup() {    
  Serial.begin(115200);
  PT1.PTbegin();
  PT2.PTbegin();
  PT3.PTbegin();
  PT4.PTbegin();
  PT5.PTbegin();
}
void loop() {
               //PT.IPT100(ch)
  Serial.println("****1111****");
  Serial.println(PT1.IPT100(0));
  Serial.println(PT1.IPT100(1));
  Serial.println(PT1.IPT100(2));
  Serial.println(PT1.IPT100(3));
  Serial.println("****2222****");
  Serial.println(PT2.IPT100(0));
  Serial.println(PT2.IPT100(1));
  Serial.println(PT2.IPT100(2));
  Serial.println(PT2.IPT100(3));
  Serial.println("****3333****");
  Serial.println(PT3.IPT100(0));
  Serial.println(PT3.IPT100(1));
  Serial.println(PT3.IPT100(2));
  Serial.println(PT3.IPT100(3));
  Serial.println("****4444****");
  Serial.println(PT4.IPT100(0));
  Serial.println(PT4.IPT100(1));
  Serial.println(PT4.IPT100(2));
  Serial.println(PT4.IPT100(3));
  Serial.println("****5555****");
  Serial.println(PT5.IPT100(0));
  Serial.println(PT5.IPT100(1));
  Serial.println(PT5.IPT100(2));
  Serial.println(PT5.IPT100(3));
  Serial.println();
}
