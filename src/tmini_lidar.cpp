#include "tmini_driver_node/tmini_lidar.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <stdexcept>

namespace tmini {

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

// ── Construction / destruction ──────────────────────────────────────

TminiLidar::TminiLidar(Config config)
    : config_(std::move(config)) {}

TminiLidar::~TminiLidar() { stop(); }

bool TminiLidar::is_tof() const {
    // TMini Plus/Pro are physically ToF sensors (direct distance, no angle correction)
    // but they ALL use the triangle packet format (3 bytes/sample with intensity)
    return model_code_ == proto::kModelTminiPro ||
           model_code_ == proto::kModelTminiPlus;
}

int TminiLidar::bytes_per_sample() const {
    // All TMini family models use triangle packet format: 3 bytes/sample
    // (uint8 qual + uint16 dist) — regardless of being physically ToF or not
    return 3;
}

void TminiLidar::log(int level, const char* fmt, ...) const {
    if (!config_.log_cb) return;
    char buf[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    config_.log_cb(level, buf);
}

// ── Serial I/O ──────────────────────────────────────────────────────

bool TminiLidar::open_serial() {
    fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    // Clear non-blocking for subsequent reads
    int flags = ::fcntl(fd_, F_GETFL, 0);
    ::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tio{};
    if (::tcgetattr(fd_, &tio) != 0) {
        close_serial();
        return false;
    }

    // Raw mode, 8N1, no flow control
    ::cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~(CSTOPB | CRTSCTS);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1; // 100ms inter-byte timeout

    speed_t baud;
    switch (config_.baud_rate) {
        case 115200: baud = B115200; break;
        case 230400: baud = B230400; break;
        case 460800: baud = B460800; break;
        default:     baud = B230400; break;
    }
    ::cfsetispeed(&tio, baud);
    ::cfsetospeed(&tio, baud);

    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
        close_serial();
        return false;
    }

    ::tcflush(fd_, TCIOFLUSH);
    return true;
}

void TminiLidar::close_serial() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool TminiLidar::read_exact(uint8_t* buf, size_t len,
                             std::chrono::milliseconds timeout) {
    size_t total = 0;
    auto deadline = Clock::now() + timeout;

    while (total < len) {
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - Clock::now());
        if (remaining.count() <= 0) return false;

        // Use select for timeout
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        struct timeval tv;
        tv.tv_sec = remaining.count() / 1000;
        tv.tv_usec = (remaining.count() % 1000) * 1000;

        int sel = ::select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (sel <= 0) return false;

        ssize_t n = ::read(fd_, buf + total, len - total);
        if (n <= 0) return false;
        total += static_cast<size_t>(n);
    }
    return true;
}

void TminiLidar::write_bytes(std::span<const uint8_t> data) {
    size_t written = 0;
    while (written < data.size()) {
        ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        written += static_cast<size_t>(n);
    }
}

void TminiLidar::flush_serial() {
    ::tcflush(fd_, TCIOFLUSH);
    // Drain any buffered data
    uint8_t discard[256];
    while (true) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        struct timeval tv{0, 30000}; // 30ms
        if (::select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) break;
        if (::read(fd_, discard, sizeof(discard)) <= 0) break;
    }
}

void TminiLidar::set_dtr(bool high) {
    int status;
    ::ioctl(fd_, TIOCMGET, &status);
    if (high)
        status |= TIOCM_DTR;
    else
        status &= ~TIOCM_DTR;
    ::ioctl(fd_, TIOCMSET, &status);
}

// ── Protocol commands ───────────────────────────────────────────────

bool TminiLidar::send_command(uint8_t cmd, std::span<const uint8_t> payload) {
    if (payload.empty()) {
        std::array<uint8_t, 2> pkt = {proto::kCmdPrefix, cmd};
        write_bytes(pkt);
    } else {
        std::vector<uint8_t> pkt;
        pkt.reserve(4 + payload.size());
        pkt.push_back(proto::kCmdPrefix);
        pkt.push_back(cmd | 0x80); // LIDAR_CMDFLAG_HAS_PAYLOAD
        pkt.push_back(static_cast<uint8_t>(payload.size()));
        pkt.insert(pkt.end(), payload.begin(), payload.end());
        // XOR checksum over everything
        uint8_t cs = 0;
        for (auto b : pkt) cs ^= b;
        pkt.push_back(cs);
        write_bytes(pkt);
    }
    return true;
}

bool TminiLidar::read_response_header(uint8_t& type, uint32_t& payload_size) {
    // Response: A5 5A [size:30|subtype:2](4 bytes LE) [type](1 byte)
    // Total 7 bytes
    std::array<uint8_t, proto::kResponseHeaderSize> hdr{};
    if (!read_exact(hdr.data(), hdr.size(), 1000ms)) return false;

    if (hdr[0] != proto::kRespPrefix1 || hdr[1] != proto::kRespPrefix2)
        return false;

    uint32_t size_field = hdr[2] | (hdr[3] << 8) | (hdr[4] << 16) | (hdr[5] << 24);
    payload_size = size_field & 0x3FFFFFFF;
    type = hdr[6];
    return true;
}

bool TminiLidar::get_device_info() {
    send_command(proto::kCmdGetDeviceInfo);

    uint8_t type = 0;
    uint32_t size = 0;
    if (!read_response_header(type, size)) return false;
    if (type != proto::kRespDeviceInfo) return false;

    proto::DeviceInfo info{};
    if (!read_exact(reinterpret_cast<uint8_t*>(&info), sizeof(info), 500ms))
        return false;

    model_code_ = info.model;

    // Auto-detect intensity format from model
    intensity_16bit_ = is_tof();

    return true;
}

bool TminiLidar::get_device_health() {
    send_command(proto::kCmdGetHealth);

    uint8_t type = 0;
    uint32_t size = 0;
    if (!read_response_header(type, size)) return false;
    if (type != proto::kRespHealth) return false;

    proto::HealthInfo health{};
    if (!read_exact(reinterpret_cast<uint8_t*>(&health), sizeof(health), 500ms))
        return false;

    return health.status != proto::kHealthError;
}

bool TminiLidar::start_scan_cmd() {
    send_command(proto::kCmdScan);

    uint8_t type = 0;
    uint32_t size = 0;
    if (!read_response_header(type, size)) return false;

    return type == proto::kRespMeasurement;
}

bool TminiLidar::stop_scan_cmd() {
    send_command(proto::kCmdStop);
    std::this_thread::sleep_for(10ms);
    send_command(proto::kCmdForceStop);
    std::this_thread::sleep_for(10ms);
    return true;
}

// ── High-level start/stop ───────────────────────────────────────────

bool TminiLidar::start(ScanCallback cb) {
    callback_ = std::move(cb);

    if (!open_serial()) return false;

    flush_serial();
    stop_scan_cmd();
    flush_serial();

    if (!get_device_info()) {
        close_serial();
        return false;
    }

    if (!get_device_health()) {
        close_serial();
        return false;
    }

    flush_serial();

    if (!start_scan_cmd()) {
        close_serial();
        return false;
    }

    // Start the reader thread
    have_first_scan_ = false;
    current_scan_ = Scan{};
    current_scan_.samples.reserve(500);

    reader_thread_ = std::jthread([this](std::stop_token st) {
        reader_loop(st);
    });

    return true;
}

void TminiLidar::stop() {
    if (reader_thread_.joinable()) {
        reader_thread_.request_stop();
        reader_thread_.join();
    }
    if (fd_ >= 0) {
        stop_scan_cmd();
        close_serial();
    }
}

// ── Packet reading & parsing ────────────────────────────────────────

void TminiLidar::reader_loop(std::stop_token stoken) {
    uint32_t pkt_ok = 0, pkt_cs_fail = 0, pkt_sync_fail = 0, scans_sent = 0;
    auto last_diag = Clock::now();

    while (!stoken.stop_requested()) {
        auto result = read_packet_diag(stoken, pkt_ok, pkt_cs_fail, pkt_sync_fail, scans_sent);
        if (!result) {
            std::this_thread::sleep_for(1ms);
        }

        // Periodic diagnostics every 5 seconds
        auto now = Clock::now();
        if (now - last_diag > 5s) {
            log(kLogDebug, "diag: ok=%u cs_fail=%u sync_fail=%u scans=%u bps=%d",
                pkt_ok, pkt_cs_fail, pkt_sync_fail, scans_sent, bytes_per_sample());
            last_diag = now;
        }
    }
}

bool TminiLidar::read_packet_diag(std::stop_token& stoken,
                                   uint32_t& pkt_ok, uint32_t& pkt_cs_fail,
                                   uint32_t& pkt_sync_fail, uint32_t& scans_sent) {
    // Phase 1: Sync to packet header 0xAA 0x55
    // Read header byte-by-byte to find sync
    uint8_t b;
    bool found_aa = false;
    auto sync_deadline = Clock::now() + 500ms;

    while (!stoken.stop_requested()) {
        if (Clock::now() > sync_deadline) return false;
        if (!read_exact(&b, 1, 50ms)) continue;

        if (!found_aa) {
            if (b == 0xAA) found_aa = true;
            continue;
        }
        // Previous byte was 0xAA
        if (b == 0x55) break;       // Found sync
        if (b == 0xAA) continue;    // Consecutive 0xAA
        found_aa = false;           // False start
    }
    if (stoken.stop_requested()) return false;

    // Phase 2: Read remaining 8 bytes of header (we already consumed 2)
    std::array<uint8_t, proto::kPacketHeaderSize> hdr{};
    hdr[0] = 0xAA;
    hdr[1] = 0x55;
    if (!read_exact(hdr.data() + 2, 8, 200ms)) {
        pkt_sync_fail++;
        return false;
    }

    uint8_t  ct          = hdr[2];
    uint8_t  count       = hdr[3];
    uint16_t first_angle = hdr[4] | (hdr[5] << 8);
    uint16_t last_angle  = hdr[6] | (hdr[7] << 8);
    uint16_t checksum    = hdr[8] | (hdr[9] << 8);

    // Validate angle check bits
    if (!(first_angle & proto::kAngleCheckBit)) return false;
    if (!(last_angle & proto::kAngleCheckBit)) return false;

    // Sanity check sample count
    if (count == 0 || count > proto::kMaxSamplesPerPacket) return false;

    bool is_ring_start = (ct & proto::kCtRingStart) != 0;

    // Phase 3: Read sample data
    int sample_bytes = count * bytes_per_sample();
    std::vector<uint8_t> sample_buf(sample_bytes);
    if (!read_exact(sample_buf.data(), sample_bytes, 500ms)) return false;

    // Phase 4: Compute & validate XOR checksum
    // Init with header word 0x55AA
    uint16_t cs_calc = 0x55AA;
    cs_calc ^= first_angle;

    // All TMini: triangle format (3 bytes/sample)
    // XOR each qual byte standalone, then each dist as uint16
    for (int i = 0; i < count; i++) {
        int off = i * 3;
        cs_calc ^= sample_buf[off]; // qual byte
        uint16_t dist16 = sample_buf[off + 1] | (sample_buf[off + 2] << 8);
        cs_calc ^= dist16;
    }

    uint16_t ct_count = hdr[2] | (hdr[3] << 8); // CT + count as uint16
    cs_calc ^= ct_count;
    cs_calc ^= last_angle; // raw last angle with checkbit

    if (cs_calc != checksum) {
        pkt_cs_fail++;
        if (pkt_cs_fail <= 5) {
            log(kLogWarn, "checksum fail: calc=0x%04X recv=0x%04X ct=%02X count=%d bps=%d",
                cs_calc, checksum, ct, count, bytes_per_sample());
        }
        return false;
    }
    pkt_ok++;

    // Phase 5: On ring start, deliver previous scan and start new one
    auto now = Clock::now();

    if (is_ring_start) {
        if (have_first_scan_ && !current_scan_.samples.empty()) {
            current_scan_.stamp_end = now;
            current_scan_.model_code = model_code_;
            current_scan_.intensity_16bit = intensity_16bit_;
            if (callback_) {
                scans_sent++;
                callback_(std::move(current_scan_));
            }
        }
        current_scan_ = Scan{};
        current_scan_.samples.reserve(500);
        current_scan_.stamp_start = now;
        current_scan_.scan_freq_hz = static_cast<float>(ct >> 1) * 0.1f;
        have_first_scan_ = true;
    }

    if (!have_first_scan_) return true; // Discard until first ring start

    // Phase 6: Parse samples and append to current scan
    uint16_t first_raw = first_angle >> 1;
    uint16_t last_raw  = last_angle >> 1;

    parse_samples(sample_buf.data(), count, first_raw, last_raw,
                  current_scan_.samples);

    return true;
}

void TminiLidar::parse_samples(const uint8_t* data, int count,
                                uint16_t first_angle_raw,
                                uint16_t last_angle_raw,
                                std::vector<Sample>& out) {
    // Compute angle interval between samples
    float interval = 0.0f;
    static float last_interval = 0.0f;

    if (count > 1) {
        if (last_angle_raw < first_angle_raw) {
            if (first_angle_raw > 270 * 64 && last_angle_raw < 90 * 64) {
                interval = static_cast<float>(
                    360 * 64 + last_angle_raw - first_angle_raw) /
                    static_cast<float>(count - 1);
                last_interval = interval;
            } else {
                interval = last_interval;
            }
        } else {
            interval = static_cast<float>(last_angle_raw - first_angle_raw) /
                       static_cast<float>(count - 1);
            last_interval = interval;
        }
    }

    int bps = bytes_per_sample();

    for (int i = 0; i < count; i++) {
        Sample s{};
        int off = i * bps;

        // All TMini models: triangle packet format (uint8 qual + uint16 dist)
        uint8_t qual_byte = data[off];
        uint16_t dist_raw = data[off + 1] | (data[off + 2] << 8);

        // 10-bit intensity: lower 2 bits of dist are MSBs of intensity
        float intensity = static_cast<float>(
            ((dist_raw & 0x03) << 8) | qual_byte);
        dist_raw &= 0xFFFC; // mask off intensity bits from distance

        // Triangle packet format: dist_raw is always in quarter-mm units
        // (applies to all TMini variants, including Plus/Pro)
        float distance_mm = static_cast<float>(dist_raw) / 4.0f;

        // Angle calculation
        float sample_angle_raw = static_cast<float>(first_angle_raw) +
                                 interval * static_cast<float>(i);

        // Normalize to 0..23040 range
        if (sample_angle_raw < 0)
            sample_angle_raw += 360.0f * 64.0f;
        else if (sample_angle_raw > 360.0f * 64.0f)
            sample_angle_raw -= 360.0f * 64.0f;

        float angle_deg = sample_angle_raw * proto::kAngleScale;

        s.angle_deg = angle_deg;
        s.distance_m = distance_mm / 1000.0f;
        s.intensity = intensity;
        out.push_back(s);
    }
}

float TminiLidar::triangle_angle_correction(float distance_mm) const {
    if (distance_mm < 1.0f) return 0.0f;
    float d = distance_mm / 4.0f;
    return std::atan((proto::kTriCorrK * (proto::kTriCorrB - d)) /
                     (proto::kTriCorrB * d)) * (180.0f / static_cast<float>(M_PI)) * 64.0f;
}

} // namespace tmini
