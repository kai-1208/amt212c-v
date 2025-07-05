#include <mbed.h>
#include "amt212c_v.hpp"

int main() {
    BufferedSerial pc(USBTX, USBRX, 115200);
    Amt212CV encoder(PB_6, PA_10, D6, 0x54);
    encoder.set_mode(Amt212CV::Mode::Continuous);  // Continuousモードで∞範囲に追従、Wrappedモードで0~2π範囲に追従

    while (true) {
        if (encoder.update()) {
            float deg = encoder.get_angle().deg();
            float rad = encoder.get_angle().rad();
            int32_t pos = encoder.get_position();

            printf("Ticks: %ld, Angle: %.3f [deg], %.3f [rad]\n", pos, deg, rad);
        } else {
            printf("Failed to read encoder\n");
        }
        ThisThread::sleep_for(100ms);
    }
}
