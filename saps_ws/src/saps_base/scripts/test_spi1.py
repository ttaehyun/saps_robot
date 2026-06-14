import spidev

spi = spidev.SpiDev()
spi.open(0, 0)
spi.mode = 3
spi.max_speed_hz = 2_000_000

patterns = [
    [0x00, 0xFF, 0x55, 0xAA],
    [0x12, 0x34, 0x56, 0x78],
]

for tx in patterns:
    rx = spi.xfer2(tx.copy())
    print("TX:", tx)
    print("RX:", rx)

spi.close()