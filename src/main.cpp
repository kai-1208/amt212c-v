#include <mbed.h>
#include "amt212c_v.hpp"

int main() {
    BufferedSerial pc(USBTX, USBRX, 115200);
    // エンコーダーの数増やすなら、アドレスを別々に決めてください。
    Amt212CV encoder(PB_6, PA_10, D6, 0x54);
    // Amt212CV encoder2(PB_6, PA_10, D6, 0x50);
    encoder.set_mode(Amt212CV::Mode::Continuous);  // Continuousモードで∞範囲に追従、Wrappedモードで0~2π範囲に追従
    // encoder2.set_mode(Amt212CV::Mode::Continuous);

    while (true) {
        if (encoder.update()) {
            float deg = encoder.get_angle().deg();
            float rad = encoder.get_angle().rad();
            int32_t pos = encoder.get_position();

            printf("Encoder 1 - Ticks: %ld, Angle: %.3f [deg], %.3f [rad]\n", pos, deg, rad);
        } else {
            printf("Failed to read encoder\n");
        }
        // if (encoder2.update()) {
        //     float deg2 = encoder2.get_angle().deg();
        //     float rad2 = encoder2.get_angle().rad();
        //     int32_t pos2 = encoder2.get_position();

        //     printf("Encoder 2 - Ticks: %ld, Angle: %.3f [deg], %.3f [rad]\n", pos2, deg2, rad2);
        // } else {
        //     printf("Failed to read encoder 2\n");
        // }
        ThisThread::sleep_for(100ms);
    }
}
