import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)  # /dev/spidev0.0

spi.mode = 3
spi.max_speed_hz = 1_200_000
spi.bits_per_word = 8
spi.lsbfirst = False
spi.cshigh = False

# 가능한 경우 루프백과 수동 CS 비활성화
try:
    spi.loop = False
    spi.no_cs = False
    spi.threewire = False
except AttributeError:
    pass


def transfer(name, tx):
    # xfer2 전송 전 값을 따로 보존
    tx_expected = list(tx)

    # 혹시 라이브러리가 리스트를 변경해도 출력에는 영향 없게 복사본 사용
    rx = spi.xfer2(list(tx))

    print(
        f"{name}: "
        f"TX_EXPECTED={tx_expected}, "
        f"TX_HEX={[f'0x{x:02X}' for x in tx_expected]}, "
        f"RX={rx}, "
        f"RX_HEX={[f'0x{x:02X}' for x in rx]}"
    )

    return rx


def write_reg(address, value):
    command = 0x8000 | (address & 0x7FFF)

    tx = [
        (command >> 8) & 0xFF,
        command & 0xFF,
        value & 0xFF
    ]

    transfer(f"WRITE 0x{address:04X}", tx)


def read_reg(address):
    command = address & 0x7FFF

    # 주소 2바이트 + 데이터를 받기 위한 클럭 1바이트
    tx = [
        (command >> 8) & 0xFF,
        command & 0xFF,
        0x00
    ]

    rx = transfer(f"READ  0x{address:04X}", tx)

    # 주소 다음 바이트가 실제 읽은 데이터
    return rx[2]


try:
    # Register page 0 선택
    write_reg(0x7FFF, 0x00)
    time.sleep(0.01)

    device_id = read_reg(0x0000)
    revision_id = read_reg(0x0001)

    print(f"\nDevice ID   = 0x{device_id:02X}")
    print(f"Revision ID = 0x{revision_id:02X}")

finally:
    spi.close()