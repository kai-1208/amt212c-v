#include <mbed.h>
#include "amt212c_v.hpp"

int main() {
    BufferedSerial pc(USBTX, USBRX, 115200);
    Amt212CV encoder(PB_6, PA_10, D6, 0x54);  // TX, RX, DE/RE, address　適当に変えてください

    while (true) {
        if (encoder.update()) {
            auto angle = encoder.get_angle();
            auto ticks = encoder.get_position();
            printf("AMT212C-V ticks: %u, angle: %f [rad], %f [deg]\n", ticks, angle.rad(), angle.deg());
        } else {
            printf("Failed to read encoder\n");
        }

        ThisThread::sleep_for(100ms);
    }
}
