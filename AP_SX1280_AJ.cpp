#include "AP_SX1280_AJ.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// ─────────────────────────────────────────────────────────────────────────────
// SHA-256 minimal implementation for seed derivation
// ─────────────────────────────────────────────────────────────────────────────
static const uint32_t K[64] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,
    0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,
    0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,
    0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,
    0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,
    0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,
    0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,
    0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,
    0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
};

static inline uint32_t rotr32(uint32_t x, int n) { return (x >> n) | (x << (32-n)); }

static void sha256_block(const uint8_t *msg, uint32_t *h) {
    uint32_t w[64];
    for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)msg[i*4]<<24)|((uint32_t)msg[i*4+1]<<16)|
               ((uint32_t)msg[i*4+2]<<8)|(uint32_t)msg[i*4+3];
    }
    for (int i = 16; i < 64; i++) {
        uint32_t s0 = rotr32(w[i-15],7)^rotr32(w[i-15],18)^(w[i-15]>>3);
        uint32_t s1 = rotr32(w[i-2],17)^rotr32(w[i-2],19)^(w[i-2]>>10);
        w[i] = w[i-16]+s0+w[i-7]+s1;
    }
    uint32_t a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7];
    for (int i = 0; i < 64; i++) {
        uint32_t S1 = rotr32(e,6)^rotr32(e,11)^rotr32(e,25);
        uint32_t ch = (e&f)^(~e&g);
        uint32_t t1 = hh+S1+ch+K[i]+w[i];
        uint32_t S0 = rotr32(a,2)^rotr32(a,13)^rotr32(a,22);
        uint32_t maj= (a&b)^(a&c)^(b&c);
        uint32_t t2 = S0+maj;
        hh=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
    }
    h[0]+=a; h[1]+=b; h[2]+=c; h[3]+=d;
    h[4]+=e; h[5]+=f; h[6]+=g; h[7]+=hh;
}

// SHA-256 of 36 bytes (32-byte master seed + 4-byte counter)
static void sha256_36(const uint8_t *in36, uint8_t *out32) {
    uint32_t h[8] = {
        0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
        0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19
    };
    // Pad to 64 bytes: 36 data bytes + 1 byte 0x80 + zeros + length
    uint8_t block[64] = {};
    memcpy(block, in36, 36);
    block[36] = 0x80;
    // Length in bits = 36*8 = 288 = 0x120
    block[62] = 0x01;
    block[63] = 0x20;
    sha256_block(block, h);
    for (int i = 0; i < 8; i++) {
        out32[i*4+0] = (h[i]>>24)&0xFF;
        out32[i*4+1] = (h[i]>>16)&0xFF;
        out32[i*4+2] = (h[i]>> 8)&0xFF;
        out32[i*4+3] =  h[i]     &0xFF;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// AP_SX1280_AJ implementation
// ─────────────────────────────────────────────────────────────────────────────

AP_SX1280_AJ::AP_SX1280_AJ() {}

bool AP_SX1280_AJ::init()
{
    // Get SPI device (defined in hwdef.dat as "sx1280" on SPI6)
    _dev = std::move(hal.spi->get_device("sx1280"));
    if (!_dev) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "SX1280: SPI device not found");
        return false;
    }

    // GPIO pins from hwdef.dat
    _pin_rst  = hal.gpio->channel(HAL_GPIO_SX1280_RST);
    _pin_busy = hal.gpio->channel(HAL_GPIO_SX1280_BUSY);

    // Hardware reset
    _pin_rst->mode(HAL_GPIO_OUTPUT);
    _pin_rst->write(0);
    hal.scheduler->delay_microseconds(200);
    _pin_rst->write(1);
    hal.scheduler->delay_microseconds(5000);

    // Configure SX1280
    _set_standby();
    _set_packet_type_flrc();
    _set_modulation_params_flrc();
    _set_packet_params_flrc();
    _set_tx_power(13);       // +12.5dBm (max)
    _configure_dio_irq();

    // Derive initial FHSS seed from master seed stored in parameters
    // For now use a fixed seed; replace with TRNG + flash read in production
    _fhss_init(0xACE1);

    // Start on first channel in RX mode
    _fhss_hop();
    _set_rx_mode();

    _initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SX1280: Anti-Jamming FHSS initialised");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Main update — call at 400Hz
// ─────────────────────────────────────────────────────────────────────────────

void AP_SX1280_AJ::update()
{
    if (!_initialised) return;

    uint32_t now_us = AP_HAL::micros();

    // ── 1. Handle received packet if IRQ asserted ────────────────────
    _handle_irq();

    // ── 2. FHSS hop timer ────────────────────────────────────────────
    if ((now_us - _last_hop_us) >= _hop_interval_us) {
        _fhss_hop();
        _set_rx_mode();
        _last_hop_us = now_us;
    }

    // ── 3. Link timeout detection ─────────────────────────────────────
    uint32_t lost_ms = (now_us - _last_rx_us) / 1000UL;
    LinkState new_state;

    if (lost_ms < 500) {
        new_state = LinkState::LINK_OK;
    } else if (lost_ms < 1000) {
        new_state = LinkState::LINK_WARN;
    } else if (lost_ms < 5000) {
        new_state = LinkState::LINK_LOST_1S;
    } else if (lost_ms < 30000) {
        new_state = LinkState::LINK_LOST_5S;
    } else {
        new_state = LinkState::LINK_LOST_30S;
    }

    _apply_link_state(new_state);
}

// ─────────────────────────────────────────────────────────────────────────────
// IRQ handler — read packet status when RX_DONE asserted
// ─────────────────────────────────────────────────────────────────────────────

void AP_SX1280_AJ::_handle_irq()
{
    uint8_t irq_buf[3];
    _read_command(SX1280_CMD_GET_IRQ_STATUS, irq_buf, 3);
    uint16_t irq = ((uint16_t)irq_buf[1] << 8) | irq_buf[2];

    if (irq & SX1280_IRQ_RX_DONE) {
        // Clear IRQ
        uint8_t clr[2] = {(uint8_t)(irq>>8), (uint8_t)irq};
        _write_command(SX1280_CMD_CLR_IRQ_STATUS, clr, 2);

        bool crc_error = (irq & SX1280_IRQ_CRC_ERROR) != 0;
        _read_packet_status();
        _update_jam_detector(_rssi_dbm, _snr_db, crc_error);

        if (!crc_error) {
            _last_rx_us = AP_HAL::micros();
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Read RSSI and SNR from SX1280
// ─────────────────────────────────────────────────────────────────────────────

void AP_SX1280_AJ::_read_packet_status()
{
    uint8_t buf[6];
    _read_command(SX1280_CMD_GET_PACKET_STATUS, buf, 6);
    // FLRC: buf[1]=RssiSync, buf[2]=errors, buf[3]=status, buf[4]=sync
    // RSSI = -buf[1]/2 dBm
    _rssi_dbm = -(float)buf[1] / 2.0f;
    // SNR not directly available in FLRC; estimate from RSSI vs noise floor
    // Use a fixed noise floor of -105dBm (receiver sensitivity)
    _snr_db = _rssi_dbm - (-105.0f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Jamming detection
// ─────────────────────────────────────────────────────────────────────────────

void AP_SX1280_AJ::_update_jam_detector(float rssi, float snr, bool lost)
{
    _rssi_history[_history_idx] = rssi;
    _snr_history [_history_idx] = snr;
    _loss_history[_history_idx] = lost ? 1 : 0;
    _history_idx = (_history_idx + 1) % JAM_DETECTION_WINDOW;

    // Calculate loss rate and average SNR over the window
    uint8_t loss_count = 0;
    float   snr_sum    = 0.0f;
    for (int i = 0; i < JAM_DETECTION_WINDOW; i++) {
        loss_count += _loss_history[i];
        snr_sum    += _snr_history[i];
    }
    float loss_rate = (float)loss_count / JAM_DETECTION_WINDOW;
    float avg_snr   = snr_sum / JAM_DETECTION_WINDOW;

    bool suspected = (loss_rate > JAM_LOSS_THRESHOLD) ||
                     (avg_snr   < JAM_SNR_THRESHOLD);

    if (suspected) {
        if (++_jam_confirm_count >= JAM_CONFIRM_COUNT) {
            if (!_jamming_detected) {
                _jamming_detected = true;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "SX1280: Jamming detected! RSSI=%.0f SNR=%.0f loss=%.0f%%",
                              (double)rssi, (double)avg_snr,
                              (double)(loss_rate * 100.0f));
            }
        }
    } else {
        _jam_confirm_count = 0;
        if (_jamming_detected) {
            _jamming_detected = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SX1280: Jamming cleared");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Link state machine — adjusts hop rate and triggers failsafe
// ─────────────────────────────────────────────────────────────────────────────

void AP_SX1280_AJ::_apply_link_state(LinkState new_state)
{
    if (new_state == _link_state) return;
    _link_state = new_state;

    switch (new_state) {
    case LinkState::LINK_WARN:
        // Double hop rate to outrun the jammer
        _hop_interval_us = 1000000UL / FHSS_HOP_RATE_JAMMING_HZ;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "SX1280: Link degraded — hop rate doubled to %dHz",
                      FHSS_HOP_RATE_JAMMING_HZ);
        break;

    case LinkState::LINK_OK:
        // Restore normal hop rate
        _hop_interval_us = 1000000UL / FHSS_HOP_RATE_NORMAL_HZ;
        break;

    case LinkState::LINK_LOST_1S:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SX1280: Link lost 1s — hover");
        // ArduPilot failsafe will handle mode change via RC_Channels
        break;

    case LinkState::LINK_LOST_5S:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "SX1280: Link lost 5s — RTH");
        break;

    case LinkState::LINK_LOST_30S:
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "SX1280: Link lost 30s — LAND");
        break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FHSS — Galois LFSR + channel mapping
// ─────────────────────────────────────────────────────────────────────────────

void AP_SX1280_AJ::_fhss_init(uint16_t seed)
{
    _lfsr_state = seed ? seed : 0xACE1;
}

// 16-bit Galois LFSR  polynomial: x^16 + x^15 + x^13 + x^4 + 1
uint16_t AP_SX1280_AJ::_lfsr_next()
{
    uint8_t lsb    = _lfsr_state & 1;
    _lfsr_state  >>= 1;
    if (lsb) _lfsr_state ^= 0xB400;
    return _lfsr_state;
}

uint8_t AP_SX1280_AJ::_fhss_next_channel()
{
    return (uint8_t)(_lfsr_next() % FHSS_NUM_CHANNELS);
}

uint32_t AP_SX1280_AJ::_channel_to_freq_hz(uint8_t ch)
{
    // 2402MHz + ch*2MHz  (channels 0-39 → 2402-2480MHz)
    return (uint32_t)(FHSS_START_FREQ_MHZ + ch * FHSS_CHANNEL_STEP_MHZ) * 1000000UL;
}

void AP_SX1280_AJ::_fhss_hop()
{
    _current_channel = _fhss_next_channel();
    _set_frequency(_channel_to_freq_hz(_current_channel));
}

// ─────────────────────────────────────────────────────────────────────────────
// Crypto seed derivation  (master_seed[32] + flight_counter → 16-bit seed)
// ─────────────────────────────────────────────────────────────────────────────

uint16_t AP_SX1280_AJ::_derive_flight_seed(const uint8_t *master_seed_32,
                                             uint32_t flight_counter)
{
    uint8_t input[36];
    memcpy(input, master_seed_32, 32);
    input[32] = (flight_counter >> 24) & 0xFF;
    input[33] = (flight_counter >> 16) & 0xFF;
    input[34] = (flight_counter >>  8) & 0xFF;
    input[35] =  flight_counter        & 0xFF;

    uint8_t hash[32];
    sha256_36(input, hash);

    uint16_t seed = ((uint16_t)hash[0] << 8) | hash[1];
    return seed ? seed : 0xACE1;
}

// ─────────────────────────────────────────────────────────────────────────────
// SX1280 low-level SPI commands
// ─────────────────────────────────────────────────────────────────────────────

void AP_SX1280_AJ::_wait_busy()
{
    // Poll BUSY pin — HIGH means IC is processing
    if (!_pin_busy) return;
    uint32_t timeout = AP_HAL::millis() + 100;
    while (_pin_busy->read() && AP_HAL::millis() < timeout) {
        hal.scheduler->delay_microseconds(10);
    }
}

void AP_SX1280_AJ::_write_command(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    _wait_busy();
    WITH_SEMAPHORE(_dev->get_semaphore());
    uint8_t buf[len + 1];
    buf[0] = cmd;
    memcpy(&buf[1], data, len);
    _dev->transfer(buf, nullptr, len + 1);
}

void AP_SX1280_AJ::_read_command(uint8_t cmd, uint8_t *data, uint8_t len)
{
    _wait_busy();
    WITH_SEMAPHORE(_dev->get_semaphore());
    // SX1280 read: send cmd + 1 NOP status byte, then read data
    uint8_t tx[len + 2];
    uint8_t rx[len + 2];
    memset(tx, 0, sizeof(tx));
    tx[0] = cmd;
    _dev->transfer(tx, rx, len + 2);
    memcpy(data, &rx[2], len);
}

void AP_SX1280_AJ::_set_standby()
{
    uint8_t d = 0x00; // STDBY_RC
    _write_command(SX1280_CMD_SET_STANDBY, &d, 1);
    hal.scheduler->delay_microseconds(1000);
}

void AP_SX1280_AJ::_set_packet_type_flrc()
{
    uint8_t d = SX1280_PACKET_TYPE_FLRC;
    _write_command(SX1280_CMD_SET_PACKET_TYPE, &d, 1);
}

void AP_SX1280_AJ::_set_tx_power(int8_t dbm)
{
    // TX power: 0x00=-18dBm … 0x1F=+12.5dBm  (step ~1dBm)
    // Ramp time: 0x E0 = 20us
    uint8_t d[2] = {(uint8_t)(dbm + 18), 0xE0};
    _write_command(SX1280_CMD_SET_TX_PARAMS, d, 2);
}

void AP_SX1280_AJ::_set_modulation_params_flrc()
{
    // BR_1_300_BW_1_2 | CR_1_0 | BT_0_5
    uint8_t d[3] = {0x45, 0x01, 0x10};
    _write_command(SX1280_CMD_SET_MODULATION_PARAMS, d, 3);
}

void AP_SX1280_AJ::_set_packet_params_flrc()
{
    // PreambleLength=8, SyncWordLength=4, SyncWordMatch=1,
    // PacketType=VARIABLE, PayloadLength=64, CrcLength=3, Whitening=OFF
    uint8_t d[7] = {0x10, 0x04, 0x04, 0x20, 0x40, 0x24, 0x08};
    _write_command(SX1280_CMD_SET_PACKET_PARAMS, d, 7);
}

void AP_SX1280_AJ::_configure_dio_irq()
{
    // Enable RX_DONE and CRC_ERROR on DIO1
    uint8_t d[8] = {
        0x00, (uint8_t)((SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR) >> 8),
              (uint8_t) (SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR),
        0x00, (uint8_t)((SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR) >> 8),
              (uint8_t) (SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR),
        0x00, 0x00
    };
    _write_command(SX1280_CMD_SET_DIO_IRQ_PARAMS, d, 8);
}

void AP_SX1280_AJ::_set_frequency(uint32_t freq_hz)
{
    // PLL step = 52e6 / 2^18 ≈ 198.364 Hz
    uint32_t pll = (uint32_t)((double)freq_hz / 198.3642578125);
    uint8_t d[3] = {
        (uint8_t)(pll >> 16),
        (uint8_t)(pll >>  8),
        (uint8_t) pll
    };
    _write_command(SX1280_CMD_SET_RF_FREQUENCY, d, 3);
}

void AP_SX1280_AJ::_set_rx_mode()
{
    // Continuous RX (timeout = 0xFFFFFF)
    uint8_t d[3] = {0xFF, 0xFF, 0xFF};
    _write_command(SX1280_CMD_SET_RX, d, 3);
}

void AP_SX1280_AJ::_set_tx_mode()
{
    uint8_t d[3] = {0x00, 0x00, 0x00};
    _write_command(SX1280_CMD_SET_TX, d, 3);
}
