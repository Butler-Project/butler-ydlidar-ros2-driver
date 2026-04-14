#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace tmini {

// YDLidar protocol constants
namespace proto {
    // Sync bytes
    inline constexpr uint16_t kPacketHeader    = 0x55AA;
    inline constexpr uint8_t  kCmdPrefix       = 0xA5;
    inline constexpr uint8_t  kRespPrefix1     = 0xA5;
    inline constexpr uint8_t  kRespPrefix2     = 0x5A;

    // Commands
    inline constexpr uint8_t kCmdScan          = 0x60;
    inline constexpr uint8_t kCmdForceScan     = 0x61;
    inline constexpr uint8_t kCmdStop          = 0x65;
    inline constexpr uint8_t kCmdForceStop     = 0x00;
    inline constexpr uint8_t kCmdReset         = 0x80;
    inline constexpr uint8_t kCmdGetDeviceInfo  = 0x90;
    inline constexpr uint8_t kCmdGetHealth     = 0x92;

    // Response types
    inline constexpr uint8_t kRespDeviceInfo   = 0x04;
    inline constexpr uint8_t kRespHealth       = 0x06;
    inline constexpr uint8_t kRespMeasurement  = 0x81;

    // Packet constants
    inline constexpr int kPacketHeaderSize     = 10;
    inline constexpr int kMaxSamplesPerPacket  = 128;

    // Model codes
    inline constexpr uint8_t kModelTmini       = 140;
    inline constexpr uint8_t kModelTminiPro    = 150;
    inline constexpr uint8_t kModelTminiPlus   = 151;

    // CT byte flags
    inline constexpr uint8_t kCtRingStart      = 0x01;

    // Angle encoding
    inline constexpr uint8_t kAngleCheckBit    = 0x01;
    inline constexpr float   kAngleScale       = 1.0f / 64.0f;  // raw angle units to degrees
    inline constexpr float   kFullRotation     = 360.0f * 64.0f; // 23040 in raw units

    // Device info response size
    inline constexpr int kDeviceInfoSize       = 20;
    inline constexpr int kHealthSize           = 3;
    inline constexpr int kResponseHeaderSize   = 7;

    // Health status
    inline constexpr uint8_t kHealthOk         = 0x00;
    inline constexpr uint8_t kHealthWarning    = 0x01;
    inline constexpr uint8_t kHealthError      = 0x02;

    // Triangulation correction constant (for non-TOF models)
    inline constexpr float kTriCorrK           = 21.8f;
    inline constexpr float kTriCorrB           = 155.3f;

    struct __attribute__((packed)) DeviceInfo {
        uint8_t  model;
        uint16_t firmware_version;
        uint8_t  hardware_version;
        uint8_t  serial_number[16];
    };

    struct __attribute__((packed)) HealthInfo {
        uint8_t  status;
        uint16_t error_code;
    };
}

// Logging callback for diagnostics (set by node, called from reader thread)
using LogCallback = std::function<void(int level, const char* msg)>;
inline constexpr int kLogDebug = 0;
inline constexpr int kLogWarn  = 1;
inline constexpr int kLogError = 2;

struct Config {
    std::string port          = "/dev/lidar";
    uint32_t    baud_rate     = 230400;
    float       target_freq   = 10.0f;
    bool        motor_pid     = false;
    LogCallback log_cb;
};

struct Sample {
    float angle_deg;     // 0..360
    float distance_m;    // meters (0 = invalid)
    float intensity;     // 0-255 (triangle) or 0-65535 (TOF)
};

struct Scan {
    std::vector<Sample> samples;
    std::chrono::steady_clock::time_point stamp_start;
    std::chrono::steady_clock::time_point stamp_end;
    float    scan_freq_hz    = 0.0f;
    uint8_t  model_code      = 0;
    bool     intensity_16bit = false;
};

class TminiLidar {
public:
    using ScanCallback = std::function<void(Scan&&)>;

    explicit TminiLidar(Config config);
    ~TminiLidar();

    TminiLidar(const TminiLidar&) = delete;
    TminiLidar& operator=(const TminiLidar&) = delete;

    bool start(ScanCallback cb);
    void stop();

    uint8_t model_code() const { return model_code_; }
    bool is_tof() const;

private:
    // Serial I/O
    bool    open_serial();
    void    close_serial();
    bool    read_exact(uint8_t* buf, size_t len, std::chrono::milliseconds timeout);
    void    write_bytes(std::span<const uint8_t> data);
    void    flush_serial();
    void    set_dtr(bool high);

    // Protocol
    bool send_command(uint8_t cmd, std::span<const uint8_t> payload = {});
    bool read_response_header(uint8_t& type, uint32_t& payload_size);
    bool get_device_info();
    bool get_device_health();
    bool start_scan_cmd();
    bool stop_scan_cmd();

    // Reader thread
    void reader_loop(std::stop_token stoken);
    bool read_packet_diag(std::stop_token& stoken,
                          uint32_t& pkt_ok, uint32_t& pkt_cs_fail,
                          uint32_t& pkt_sync_fail, uint32_t& scans_sent);

    // Angle math
    int  bytes_per_sample() const;
    void parse_samples(const uint8_t* data, int count,
                       uint16_t first_angle_raw, uint16_t last_angle_raw,
                       std::vector<Sample>& out);
    float triangle_angle_correction(float distance_mm) const;

    void log(int level, const char* fmt, ...) const;

    Config       config_;
    ScanCallback callback_;
    int          fd_ = -1;
    std::jthread reader_thread_;

    // Device state (set during init)
    uint8_t model_code_      = 0;
    bool    intensity_16bit_ = false;

    // Current scan accumulator (used only in reader thread)
    Scan current_scan_;
    bool have_first_scan_ = false;
};

} // namespace tmini
