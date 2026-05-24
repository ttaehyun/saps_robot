import serial
import threading
import time
import sys

try:
    # Jetson의 8번, 10번 핀 포트인 /dev/ttyTHS1을 엽니다.
    uwb = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=0.5)
except serial.SerialException as e:
    print(f"시리얼 포트 연결 실패: {e}")
    sys.exit(1)

# 공식 문서 8페이지의 입력 버퍼 메커니즘 반영
input_buffer = ""

def read_from_uwb():
    global input_buffer
    while True:
        try:
            if uwb.in_waiting:
                # 1바이트씩 읽어서 버퍼에 채우는 공식 매뉴얼 방식
                data = uwb.read().decode(errors='ignore')
                if data == '\n':
                    line = input_buffer.strip()
                    if line:
                        # UWB 응답 출력
                        print(f"[UWB 응답] {line}")
                    input_buffer = ""  # 버퍼 초기화
                else:
                    input_buffer += data
        except Exception as e:
            pass

def write_to_uwb():
    # 쉘을 깨우기 위해 처음에 엔터(\r) 전송
    uwb.write(b'\r')
    time.sleep(0.1)
    
    while True:
        try:
            cmd = input(">>> ")
            if cmd.strip():
                # 매뉴얼 6페이지 기준 명령어 뒤에 '\r'을 붙여서 전송
                uwb.write((cmd + '\r').encode())
                uwb.flush()
        except KeyboardInterrupt:
            print("\n종료합니다.")
            break

if __name__ == "__main__":
    print("GrowSpace UWB GPIO 중계 시작 (/dev/ttyTHS1)")
    threading.Thread(target=read_from_uwb, daemon=True).start()
    write_to_uwb()