import spidev

spi = spidev.SpiDev()
spi.open(0, 0)

spi.mode = 3
spi.max_speed_hz = 1_200_000
# spi.max_speed_hz = 10_000
spi.bits_per_word = 8
spi.lsbfirst = False
spi.cshigh = False

patterns = [
    [0x00, 0xFF, 0x55, 0xAA],
    [0x12, 0x34, 0x56, 0x78],
    [0xFF, 0x00, 0xAA, 0x55],
]

error_count = 0

try:
    for i in range(1000):
        for tx in patterns:
            rx = spi.xfer2(tx.copy())

            if rx != tx:
                error_count += 1
                print(f"FAIL {i}: TX={tx}, RX={rx}")

                if error_count >= 20:
                    raise RuntimeError("오류가 너무 많이 발생함")

    print(f"테스트 완료: 오류 {error_count}회")

finally:
    spi.close()