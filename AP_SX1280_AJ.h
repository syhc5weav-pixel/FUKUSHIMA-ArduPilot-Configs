#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>

// SX1280 Register / Command definitions
#define SX1280_CMD_GET_STATUS           0xC0
#define SX1280_CMD_WRITE_REGISTER       0x18
#define SX1280_CMD_READ_REGISTER        0x19
#define SX1280_CMD_SET_STANDBY          0x80
#define SX1280_CMD_SET_PACKET_TYPE      0x8A
#define SX1280_CMD_SET_RF_FREQUENCY     0x86
#define SX1280_CMD_SET_TX_PARAMS        0x8E
#define SX1280_CMD_SET_BUFFER_BASE_ADDR 0x8F
#define SX1280_CMD_SET_MODULATION_PARAMS 0x8B
#define SX1280_CMD_SET_PACKET_PARAMS    0x8C
#define SX1280_CMD_SET_DIO_IRQ_PARAMS   0x8D
#define SX1280_CMD_SET_RX              0x82
#define SX1280_CMD_SET_TX              0x83
#define SX1280_CMD_GET_IRQ_STATUS       0x15
#define SX1280_CMD_CLR_IRQ_STATUS       0x97
#define SX1280_CMD_GET_RX_BUFFER_STATUS 0x17
#define SX1280_CMD_GET_PACKET_STATUS    0x1D
#define SX1280_CMD_SET_FS              0xC1

// Packet types
#define SX1280_PACKET_TYPE_FLRC        0x03

// IRQ masks
#define SX1280_IRQ_TX_DONE             (1 << 0)
#define SX1280_IRQ_RX_DONE             (1 << 1)
#define SX1280_IRQ_SYNC_WORD_VALID     (1 << 2)
#define SX1280_IRQ_CRC_ERROR           (1 << 6)

// FHSS configuration
#define FHSS_NUM_CHANNELS              40
#define FHSS_HOP_RATE_NORMAL_HZ        200   // 200 hops/sec = 5ms per channel
#define FHSS_HOP_RATE_JAMMING_HZ       400   // doubled when jamming detected
#define FHSS_START_FREQ_MHZ            2402
#define FHSS_CHANNEL_STEP_MHZ          2

// Jamming detection thresholds
#define JAM_DETECTION_WINDOW           20    // packets to monitor
#define JAM_LOSS_THRESHOLD             0.5f  // 50% loss rate triggers warning
#define JAM_SNR_THRESHOLD             -5.0f  // SNR below this triggers warning
#define JAM_CONFIRM_COUNT              3     // consecutive detections to confirm

// Link state machine
enum class LinkState {
    LINK_OK       = 0,
    LINK_WARN     = 1,   // quality degraded → double hop rate
    LINK_LOST_1S  = 2,   // 1s lost  → hover
    LINK_LOST_5S  = 3,   // 5s lost  → RTH / alt hold
    LINK_LOST_30S = 4,   // 30s lost → land
};

class AP_SX1280_AJ {
public:
    AP_SX1280_AJ();

    // Initialise hardware and FHSS state
    bool init();

    // Call at 400Hz from RC/main thread
    void update();

    // Accessors
    LinkState get_link_state() const { return _link_state; }
    float     get_rssi()       const { return _rssi_dbm; }
    float     get_snr()        const { return _snr_db; }
    bool      is_jamming()     const { return _jamming_detected; }

private:
    // ── SPI helpers ──────────────────────────────────────────────────
    bool     _spi_init();
    void     _write_command(uint8_t cmd, const uint8_t *data, uint8_t len);
    void     _read_command (uint8_t cmd, uint8_t *data, uint8_t len);
    void     _wait_busy();

    // ── SX1280 configuration ─────────────────────────────────────────
    void     _set_standby();
    void     _set_packet_type_flrc();
    void     _set_tx_power(int8_t dbm);
    void     _set_modulation_params_flrc();
    void     _set_packet_params_flrc();
    void     _configure_dio_irq();
    void     _set_frequency(uint32_t freq_hz);
    void     _set_rx_mode();
    void     _set_tx_mode();

    // ── FHSS ─────────────────────────────────────────────────────────
    void     _fhss_init(uint16_t seed);
    uint8_t  _fhss_next_channel();
    void     _fhss_hop();
    uint16_t _lfsr_next();
    uint32_t _channel_to_freq_hz(uint8_t ch);

    // ── Crypto seed ──────────────────────────────────────────────────
    uint16_t _derive_flight_seed(const uint8_t *master_seed_32,
                                  uint32_t flight_counter);

    // ── Jamming detection ─────────────────────────────────────────────
    void     _update_jam_detector(float rssi, float snr, bool lost);
    void     _apply_link_state(LinkState new_state);

    // ── IRQ / packet handling ─────────────────────────────────────────
    void     _handle_irq();
    void     _read_packet_status();

    // ── State ─────────────────────────────────────────────────────────
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;
    AP_HAL::DigitalSource *_pin_rst  = nullptr;
    AP_HAL::DigitalSource *_pin_busy = nullptr;
    AP_HAL::DigitalSource *_pin_dio1 = nullptr;

    bool      _initialised           = false;

    // FHSS
    uint16_t  _lfsr_state            = 0xACE1;
    uint8_t   _current_channel       = 0;
    uint32_t  _hop_interval_us       = 5000;  // 5ms = 200Hz
    uint32_t  _last_hop_us           = 0;

    // Link quality
    float     _rssi_dbm              = -100.0f;
    float     _snr_db                = 0.0f;
    uint32_t  _last_rx_us            = 0;
    LinkState _link_state            = LinkState::LINK_OK;

    // Jam detection ring buffers
    float    _rssi_history[JAM_DETECTION_WINDOW] = {};
    float    _snr_history [JAM_DETECTION_WINDOW] = {};
    uint8_t  _loss_history[JAM_DETECTION_WINDOW] = {};
    uint8_t  _history_idx            = 0;
    uint8_t  _jam_confirm_count      = 0;
    bool     _jamming_detected       = false;
};
