#include "ILIB.h"

IPID pid;

void setup() {
  Serial.begin(115200); // 디버깅 시리얼을 115,200 보레이트로 초기화
  PWM_RESET();
  //모터출력값만큼 설정하기위해 스케일 범위를 설정한다.
  pid.OUTPUT_SCALE(65535,0);
  //pid 입력 최대값을 100도로 설정, 입력 최소값을 0도로 설정
  //setValue(목표설정값, 설정최댓값, 설정최솟값, 샘플링시간10 = 0.01초)
  //PID의 Setting Value를 50.0도로 설정
  pid.SET_DATA(50.0, 100.0, 0.0, 10);
  //I값에 대한 제한 선 설정 해당 구문이 없을 시 출력최댓값-출력최솟값을 최댓값으로 설정 
  //최솟값은 -1 * 최댓값으로 설정됩니다.
  pid.ITERM_LIMIT(1000,-1000);
}

void loop() {
  // PID의 비례매개변수 P값을 설정
  //( P값이 클수록 Setting Value까지 빠르게 올라간다. 그만큼 제어값이 Setting Value를 초과하여 헌팅이 될 가능성이 높다)
  // PID의 적분매개변수 I값을 설정
  //( I값이 클수록 Setting Value까지 도달하는 시간에 대학 누적값이 커져서 빠르게 도달되지만 Setting Value를 초과하여 헌팅이 될 가능성이 높다)
  // PID의 미분매개변수 D값을 설정
  //( D값이 클수록 출력값 mv를 부드럽게 제어하여 헌팅을 줄일 수 있다)
  int input_data = 5;//ex) analogRead(A0);
                          //pid.outputPID(INPUT_DATA ,    KP,    KI,    KD);
  int output_data = pid.PID_OUTPUT(input_data, 5.0, 0.5, 1.0);
  PWM(12, output_data, 1);
}
