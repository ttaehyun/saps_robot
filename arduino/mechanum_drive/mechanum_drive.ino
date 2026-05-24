#include <EnableInterrupt.h>
#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);
PS2X ps2x;

// --- 핀 정의 (사용자 검증 완료) ---
const int encA[4] = {8, 6, 2, 4}; // M1(RL), M2(RR), M3(FR), M4(FL)
const int encB[4] = {9, 7, 3, 5}; 
const int motorFlip[4] = {1, 1, 1, 1}; 

// --- PID 제어 변수 ---
volatile long positions[4] = {0, 0, 0, 0};
long prevPositions[4] = {0, 0, 0, 0};
double currentVel[4] = {0, 0, 0, 0};
double targetVel[4] = {0, 0, 0, 0};
float outputPWM[4] = {0, 0, 0, 0};

// PID 게인 설정 (이 값들을 조금씩 깎으면서 최적값을 찾으세요)
float Kp = 10.0;  // 비례: 오차에 반응하는 강도
float Ki = 8.0;   // 적분: 누적 오차(바퀴 편차)를 해결

float integral[4] = {0, 0, 0, 0};

unsigned long lastTime = 0;
const int interval = 50; // 50ms 주기
String inputBuffer = ""; // 시리얼 데이터를 담을 임시 버퍼
int startBtnCounter = 0;     // 버튼 노이즈 방지용 카운터
bool isAutoMode = false; // 현재 모드 상태 (false: 조종기, true: 젯슨)

// --- 인터럽트 서비스 루틴 (사용자 방향성 검증 완료) ---
void isrM1() { (digitalRead(encA[0]) == digitalRead(encB[0])) ? positions[0]-- : positions[0]++; }
void isrM2() { (digitalRead(encA[1]) == digitalRead(encB[1])) ? positions[1]++ : positions[1]--; }
void isrM3() { (digitalRead(encA[2]) == digitalRead(encB[2])) ? positions[2]-- : positions[2]++; }
void isrM4() { (digitalRead(encA[3]) == digitalRead(encB[3])) ? positions[3]++ : positions[3]--; }

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(60);
  ps2x.config_gamepad(13, 11, 10, 12, true, true);

  for(int i=0; i<4; i++) {
    pinMode(encA[i], INPUT_PULLUP);
    pinMode(encB[i], INPUT_PULLUP);
  }
  
  enableInterrupt(encA[0], isrM1, CHANGE);
  enableInterrupt(encA[1], isrM2, CHANGE);
  enableInterrupt(encA[2], isrM3, CHANGE);
  enableInterrupt(encA[3], isrM4, CHANGE);
//  Serial.print("Start!!!!");
}

void loop() {
  ps2x.read_gamepad(false, 0);

  // 1. [수정] 모드 전환 로직 강화 (START 버튼을 약 0.3초간 꾹 눌러야 전환)
  if (ps2x.Button(PSB_START)) {
    startBtnCounter++;
    if (startBtnCounter > 15) { // 루프 주기 고려 약 0.3초
      isAutoMode = !isAutoMode;
      startBtnCounter = -50;    // 한번 바뀌면 1초 동안 재전환 방지 (쿨타임)
      stopAll();
      for(int i=0; i<4; i++) { targetVel[i] = 0; integral[i] = 0; }
//      Serial.println(isAutoMode ? ">>> MODE:AUTO (LOCKED)" : ">>> MODE:MANUAL");
    }
  } else {
    if (startBtnCounter > 0) startBtnCounter = 0;
    else if (startBtnCounter < 0) startBtnCounter++; // 쿨타임 회복
  }

  if (!isAutoMode) {
    // 1. 조종기 입력을 "목표 속도"로 변환
    int LY = -(ps2x.Analog(PSS_LY) - 128);
    int LX = (ps2x.Analog(PSS_LX) - 128);
    int RX = (ps2x.Analog(PSS_RX) - 128);

    // 데드존 및 정지 처리 (정지 시 PID 누적값 초기화 필수)
    if (abs(LY) > 15 || abs(LX) > 15 || abs(RX) > 15) {
      // 메카넘 역기구학 목표 속도 매핑 (단위: Ticks per 50ms)
      // 조종기 값(127)을 엔코더 최대 속도(약 100) 근처로 스케일링
      targetVel[0] = (LY - LX + RX); // M1
      targetVel[1] = (LY + LX - RX); // M2
      targetVel[2] = (LY - LX - RX); // M3
      targetVel[3] = (LY + LX + RX); // M4
      
    } 
    else {
      for(int i=0; i<4; i++) {
        targetVel[i] = 0;
        integral[i] = 0;
        outputPWM[i] = 0;
      }
      stopAll();
    }
  }
  else {
    // [자율 모드] 젯슨으로부터 시리얼 수신
    while (Serial.available() > 0) {
      char c = Serial.read(); // 한 글자씩 읽기
      
      if (c == '\n') { // 줄바꿈 문자를 만나면 한 문장이 끝난 것임
        if (inputBuffer.startsWith("A:")) {
          parseJetsonCommand(inputBuffer.substring(2));
        }
        inputBuffer = ""; // 파싱 후 버퍼 비우기
      } else {
        inputBuffer += c; // 문장이 안 끝났으면 버퍼에 추가
      }
    }
  }
  
  // 2. 50ms 마다 PID 계산 및 모터 출력 업데이트
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    noInterrupts(); // 카운트 복사 시 데이터 무결성 보장
    for (int i = 0; i < 4; i++) {
      currentVel[i] = positions[i] - prevPositions[i];
      prevPositions[i] = positions[i];
    }
    interrupts();

    for (int i = 0; i < 4; i++) {
      // PID 계산 루틴
      float error = targetVel[i] - currentVel[i];
      integral[i] += error;
      integral[i] = constrain(integral[i], -100, 100); // 윈드업 방지
      
      // 최종 PWM 계산 (기존 출력 + 보정량)
      outputPWM[i] = (targetVel[i]) + (error * Kp) + (integral[i] * Ki);

      // 출력 제한 (PCA9685 범위: 0~4095)
      outputPWM[i] = constrain(outputPWM[i], -4095, 4095);
    }


    
    // [피드백] 젯슨에게 현재 엔코더 누적 위치 전송
    Serial.print("E:");
    for(int i=0; i<4; i++) { Serial.print(positions[i]); if(i<3) Serial.print(","); }
    Serial.println();
    
    
    
    lastTime = currentTime;
  }
  // 3. 모터 드라이버에 명령 하달 (핀 번호 준수)
    setMotor(8, 9,   outputPWM[0] * motorFlip[0]); // M1 (뒤 왼쪽)
    setMotor(10, 11, outputPWM[1] * motorFlip[1]); // M2 (뒤 오른쪽)
    setMotor(14, 15, outputPWM[2] * motorFlip[2]); // M3 (앞 오른쪽)
    setMotor(12, 13, outputPWM[3] * motorFlip[3]); // M4 (앞 왼쪽)

    // 디버깅 출력 (M1 기준 - 잘 따라오는지 확인)
    // Serial.print("T:"); Serial.print(targetVel[0]);
    // Serial.print(" V:"); Serial.println(currentVel[0]);
}

// 젯슨 명령 "m1,m2,m3,m4" 파싱 함수
void parseJetsonCommand(String cmd) {
  int comma1 = cmd.indexOf(',');
  int comma2 = cmd.indexOf(',', comma1 + 1);
  int comma3 = cmd.indexOf(',', comma2 + 1);

  if (comma1 != -1 && comma2 != -1 && comma3 != -1) {
    targetVel[0] = cmd.substring(0, comma1).toInt();
    targetVel[1] = cmd.substring(comma1 + 1, comma2).toInt();
    targetVel[2] = cmd.substring(comma2 + 1, comma3).toInt();
    targetVel[3] = cmd.substring(comma3 + 1).toInt();
  }
}

void setMotor(int pinA, int pinB, int speed) {
  int val = abs((int)speed);
  if (speed > 0) { pwm.setPWM(pinA, 0, val); pwm.setPWM(pinB, 0, 0); }
  else if (speed < 0) { pwm.setPWM(pinA, 0, 0); pwm.setPWM(pinB, 0, val); }
  else { pwm.setPWM(pinA, 0, 0); pwm.setPWM(pinB, 0, 0); }
}

void stopAll() { 
  setMotor(8, 9,   0); // M1 (뒤 왼쪽)
  setMotor(10, 11, 0); // M2 (뒤 오른쪽)
  setMotor(14, 15, 0); // M3 (앞 오른쪽)
  setMotor(12, 13, 0); // M4 (앞 왼쪽) 
}
