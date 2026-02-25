# FUKUSHIMA ArduPilot Flight Controllers

**Website:** https://fukushima-uav.super.site/

Professional flight controller configurations for ArduPilot, designed for reliability in electronic warfare environments.

---

## 📖 Publications

### 🇯🇵 日本語版
**耐電子戦フライトコントローラー設計の全記録**
IMU三重冗長化・FHSS・適応型スペクトラム制御・LoRaフォールバックを全て解説。

👉 https://www.amazon.co.jp/dp/B0GPMYT7VJ

### 🇺🇸 English Edition
**Design Records of an EW-Resistant Flight Controller: Survival Strategies for Drones in Modern Warfare**
Triple IMU redundancy, SHA-256 encrypted FHSS, adaptive spectrum control, and LoRa SF12 fallback — all implemented on ArduPilot + STM32H743.

👉 https://www.amazon.com/dp/B0GPW9Y8LY

---

## Available Models

| Model | MCU | Features | Price |
|-------|-----|----------|-------|
| FUKUSHIMA_F7 | STM32F7 | Cost-effective | $170 |
| FUKUSHIMA_F7-Agri | STM32F7 | Agricultural optimized | $330 |
| FUKUSHIMA_H7 | STM32H743 @ 480MHz | Dual IMU, High performance | $500 |
| FUKUSHIMA_H7_Anti-Jamming | STM32H743 + SX1280 | Anti-jamming, Frequency hopping | $850 |

---

## Quick Start

1. Download the `.apj` firmware file for your board
2. Connect flight controller via USB
3. Open Mission Planner
4. Go to "Initial Setup" → "Install Firmware" → "Load custom firmware"
5. Select the downloaded `.apj` file

---

## Features by Model

### FUKUSHIMA_H7_Anti-Jamming (Premium) — $850

- STM32H743 @ 480MHz, ARM Cortex-M7
- **Triple IMU redundancy** — ICM-42688-P + IIM-42652 + BMI270
- SX1280 2.4GHz transceiver — FHSS across 40 channels at 200 hops/sec
- **SHA-256 encrypted hopping pattern** — unpredictable to any interceptor
- **Adaptive spectrum control (Phase 2)** — real-time channel blacklisting
- **LoRa SF12 fallback (Phase 3)** — −137 dBm sensitivity, 1,600× more sensitive than FLRC
- Staged failsafe: LINK_WARN → hover → RTH → land
- EMP and surge protection (TVS diodes, polyfuse)
- GPS-denied operation via EKF3 inertial navigation
- Up to 10km range

### FUKUSHIMA_H7 (Standard) — $500

- Dual IMU (ICM-42688-P + BMI270)
- BMP388 barometer
- 16MB DataFlash
- OSD support

### FUKUSHIMA_F7-Agri (Agricultural) — $330

- Optimized for spraying drones
- Cost-effective
- Reliable performance

### FUKUSHIMA_F7 (Entry) — $170

- Budget-friendly
- Full ArduPilot support
- Hobbyist-friendly

---

## Anti-Jamming Architecture

```
Phase 1: FHSS 200Hz → jamming detected → 400Hz
Phase 2: Scan all 40ch every 100ms → blacklist jammed channels
Phase 3: Clean channels < 5 → switch FLRC → LoRa SF12
         Sensitivity: −105dBm → −137dBm (1,600× improvement)
```

All code is open source. See `libraries/AP_SX1280_AJ/` for the full driver implementation.

---

## Documentation

- [Build Guide](./docs/build_guide.md)
- [SITL Guide](./docs/sitl_guide.md)

---

## License

- Hardware: Open Source Hardware (OSHW)
- Firmware: ArduPilot GPL v3.0
- hwdef files: MIT License

---

## Contact

- Website: https://fukushima-uav.super.site/
- GitHub: https://github.com/FUKUSHIMA-UAV
