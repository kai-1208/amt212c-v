#include "amt212c_v.hpp"

using namespace anglelib;

constexpr float TICKS_TO_RAD = (2.0f * PI) / 16384.0f;

Amt212CV::Amt212CV(PinName tx, PinName rx, PinName dere, uint8_t address, int baud)
    : rs485(tx, rx, baud), de(dere), addr(address) {
    de = 0;
    rs485.set_blocking(false);
    timer.start();
}

bool Amt212CV::update() {
    Anglef best{1e9f};

    for (int i = 0; i < 3; ++i) {
        Anglef ang;
        if (!read_angle(ang)) return false;

        if (!initialized || (last_angle - ang).abs().rad() < (last_angle - best).abs().rad()) {
            best = ang;
        }
    }

    last_angle = best;
    angle = last_angle * scale + offset;
    initialized = true;
    return true;
}

Anglef Amt212CV::get_angle() const {
    return angle;
}

uint16_t Amt212CV::get_position() const {
    return ticks;
}

void Amt212CV::flush() {
    uint8_t buf;
    while (rs485.readable()) rs485.read(&buf, 1);
}

void Amt212CV::send(const void* data, size_t len) {
    de = 1;
    rs485.write(data, len);
    wait_us(5);  // 必要に応じて調整
    de = 0;
}

bool Amt212CV::recv(void* data, size_t len, std::chrono::microseconds timeout) {
    timer.reset();
    size_t read = 0;

    while (read < len) {
        if (rs485.readable()) {
            int ret = rs485.read((char*)data + read, len - read);
            if (ret > 0) read += ret;
        }
        if (timer.elapsed_time() > timeout) return false;
    }

    return true;
}

bool Amt212CV::read_angle(Anglef& result) {
    flush();
    send(&addr, 1);
    uint16_t raw;

    if (!recv(&raw, 2, 300us)) return false;
    if (!is_valid(raw)) return false;

    ticks = raw & 0x3FFF;
    result = Anglef::from_rad(ticks * TICKS_TO_RAD);
    return true;
}

bool Amt212CV::is_valid(uint16_t data) {
    auto b = [data](int pos) { return (data >> pos) & 1; };
    bool k1 = b(15);
    bool k0 = b(14);
    bool k1_calc = !(b(13) ^ b(11) ^ b(9) ^ b(7) ^ b(5) ^ b(3) ^ b(1));
    bool k0_calc = !(b(12) ^ b(10) ^ b(8) ^ b(6) ^ b(4) ^ b(2) ^ b(0));
    return (k1 == k1_calc) && (k0 == k0_calc);
}
