# FUKUSHIMA ArduPilot Flight Controllers

Professional flight controller configurations for ArduPilot.

## Available Models

| Model | MCU | Features | Price |
|-------|-----|----------|-------|
| FUKUSHIMA_F7 | STM32F7 | Cost-effective | $170 |
| FUKUSHIMA_F7-Agri | STM32F7 | Agricultural optimized | $330 |
| FUKUSHIMA_H7 | STM32H743 @ 480MHz | Dual IMU, High performance | $500 |
| FUKUSHIMA_H7_Anti-Jamming | STM32H743 + SX1280 | Anti-jamming, Frequency hopping | $750 |

## Quick Start

1. Download the .apj firmware file for your board
2. Connect flight controller via USB
3. Open Mission Planner
4. Go to "Initial Setup" -> "Install Firmware" -> "Load custom firmware"
5. Select the downloaded .apj file

## Features by Model

### FUKUSHIMA_H7_Anti-Jamming (Premium) - $750
- SX1280 2.4GHz LoRa transceiver
- Frequency hopping spread spectrum (FHSS)
- Up to 10km range
- -137dBm sensitivity
- Military-grade anti-jamming
- GPS-denied operation capable

### FUKUSHIMA_H7 (Standard) - $500
- Dual IMU (ICM-42688-P + BMI270)
- BMP388 barometer
- 16MB DataFlash
- OSD support

### FUKUSHIMA_F7-Agri (Agricultural) - $330
- Optimized for spraying drones
- Cost-effective
- Reliable performance

### FUKUSHIMA_F7 (Entry) - $170
- Budget-friendly
- Full ArduPilot support
- Hobbyist-friendly

## Documentation

- [Build Guide](docs/BUILD.md)
- [SITL Guide](docs/SITL.md)

## License

- Hardware: Open Source Hardware (OSHW)
- Firmware: ArduPilot GPL v3.0
- hwdef files: MIT License

## Contact

- GitHub: https://github.com/FUKUSHIMA-UAV
- Email: contact@fukushima-uav.com
