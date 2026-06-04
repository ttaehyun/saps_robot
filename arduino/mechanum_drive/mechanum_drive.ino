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

// 모터 제원표
// 모델명 : gm37-520 
// 12v333rpm/1:30
// 12ppr/AB Phase

// PID 게인 설정 (이 값들을 조금씩 깎으면서 최적값을 찾으세요)
float Kp = 8.0;    // 비례 게인 (Kf가 베이스를 잡아주므로 기존보다 낮춰도 반응이 빠릅니다)
float Ki = 1.0;    // 적분 게인
float Kf = 10.0;   // [모터 사양 기반 계산된 피드포워드 상수] 10.23에 마찰 마진 반영

float integral[4] = {0, 0, 0, 0};

unsigned long lastTime = 0;
const int interval = 50; // 50ms 주기
String inputBuffer = ""; // 시리얼 데이터를 담을 임시 버퍼
unsigned long lastSerialTime = 0; // 젯슨 통신 타임아웃용
int startBtnCounter = 0;     // 버튼 노이즈 방지용 카운터
bool isAutoMode = false; // 현재 모드 상태 (false: 조종기, true: 젯슨)

// 최근 5개 샘플을 저장할 버퍼 배열 초기화 (가만히 있을 때의 스틱 값 128)
int buffer_lx[5] = {128, 128, 128, 128, 128};
int buffer_ly[5] = {128, 128, 128, 128, 128};
int buffer_rx[5] = {128, 128, 128, 128, 128};

// 5개 배열 요소 중 중간값을 찾기 위한 가벼운 정렬 함수
int getMedianOf5(int* array) {
  int sorted[5];
  // 원본 버퍼 오염을 막기 위해 가상 배열에 복사
  for (int i = 0; i < 5; i++) sorted[i] = array[i];
  
  // 단순 버블 정렬
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (sorted[i] > sorted[j]) {
        int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  return sorted[2]; // 크기 순 정렬 후 정확히 정중앙(인덱스 2) 값 반환
}

// 버퍼 배열을 한 칸씩 뒤로 밀고 맨 앞에 새 값을 넣는 함수
void updateBuffer(int newValue, int* array) {
  for (int i = 4; i > 0; i--) {
    array[i] = array[i-1];
  }
  array[0] = newValue;
}

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
      lastSerialTime = millis(); // 오토 모드 진입 시 통신 시간 초기화
      for(int i=0; i<4; i++) { targetVel[i] = 0; integral[i] = 0; }
//      Serial.println(isAutoMode ? ">>> MODE:AUTO (LOCKED)" : ">>> MODE:MANUAL");
    }
  } else {
    if (startBtnCounter > 0) startBtnCounter = 0;
    else if (startBtnCounter < 0) startBtnCounter++; // 쿨타임 회복
  }

  if (!isAutoMode) {
    // [Step 1] 조종기로부터 날것의 데이터(0 ~ 255) 읽기
    int raw_ly = ps2x.Analog(PSS_LY);
    int raw_lx = ps2x.Analog(PSS_LX);
    int raw_rx = ps2x.Analog(PSS_RX);
    
    // 2. 버퍼 업데이트
    updateBuffer(raw_ly, buffer_ly);
    updateBuffer(raw_lx, buffer_lx);
    updateBuffer(raw_rx, buffer_rx);
    
    // 3. 5샘플 중간값 필터 통과 (연속 2회 에러 완벽 차단)
    int filtered_ly = getMedianOf5(buffer_ly);
    int filtered_lx = getMedianOf5(buffer_lx);
    int filtered_rx = getMedianOf5(buffer_rx);
    // 1. 조종기 입력을 "목표 속도"로 변환
    int LY = -(filtered_ly - 128);
    int LX = (filtered_lx - 128);
    int RX = (filtered_rx - 128);
    //Serial.print("LY:"); Serial.print(LY);
    //Serial.print(" LX:"); Serial.print(LX);
    //Serial.print(" RX:"); Serial.println(RX);
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
      // 스틱을 놓았을 때: 목표 속도만 0으로 주고 PID가 능동 브레이크를 잡게 둡니다.
      for(int i=0; i<4; i++) {
        targetVel[i] = 0;
      }
    }
  }
  else {
    // [자율 모드] 젯슨으로부터 시리얼 수신
    while (Serial.available() > 0) {
      char c = Serial.read(); // 한 글자씩 읽기
      
      if (c == '\n') { // 줄바꿈 문자를 만나면 한 문장이 끝난 것임
        if (inputBuffer.startsWith("A:")) {
          parseJetsonCommand(inputBuffer.substring(2));
          lastSerialTime = millis(); // 정상 수신 시 시간 갱신
        }
        inputBuffer = ""; // 파싱 후 버퍼 비우기
      } else {
        inputBuffer += c; // 문장이 안 끝났으면 버퍼에 추가
      }
    }

    // 젯슨 통신 두절 대비 페일세이프 (500ms 동안 명령이 없으면 자동 정지)
    if (millis() - lastSerialTime > 500) {
      for(int i=0; i<4; i++) { targetVel[i] = 0; }
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
      float error = targetVel[i] - currentVel[i];
  
      // 목표 속도가 0일 때는 누적 오차를 서서히 감쇄시켜 정지 안정성 확보
      if (targetVel[i] == 0) {
        integral[i] *= 0.5;
        if(abs(currentVel[i]) <= 2) integral[i] = 0; // 완전히 멈추면 0 초기화
      } else {
        integral[i] += error;
        // 50ms 주기당 물리 한계 틱수가 400이므로 윈드업 제한도 그에 맞춰 상향
        integral[i] = constrain(integral[i], -400, 400); 
      }
      
      // 최종 PWM 계산 (사양 기반 피드포워드 수식 적용)
      outputPWM[i] = (targetVel[i] * Kf) + (error * Kp) + (integral[i] * Ki);
    
      // 출력 제한 및 Deadband 처리
      if (targetVel[i] == 0 && abs(currentVel[i]) <= 2) {
        outputPWM[i] = 0;
      } else {
        outputPWM[i] = constrain(outputPWM[i], -4095, 4095);
      }
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
    //Serial.print("T0:"); Serial.print(targetVel[0]);
    //Serial.print(" T1:"); Serial.print(targetVel[1]);
    //Serial.print(" T2:"); Serial.print(targetVel[2]);
    //Serial.print(" T3:"); Serial.println(targetVel[3]);
    //Serial.print(" V0:"); Serial.println(currentVel[0]);
    //Serial.print(" V1:"); Serial.print(currentVel[1]);
    //Serial.print(" V2:"); Serial.print(currentVel[2]);
    //Serial.print(" V3:"); Serial.println(currentVel[3]);
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
