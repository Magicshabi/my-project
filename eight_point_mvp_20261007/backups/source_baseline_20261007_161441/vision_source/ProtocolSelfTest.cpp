#include "CommProtocol.h"
#include <cstring>

// Exercises the actual parser without any UART, I2C or actuator commands.
bool protocol_self_test()
{
    CommProtocol_t a, b, encoder;
    a.begin(); b.begin(); encoder.begin();
    uint8_t pa[] = {10, 20, 30, 40};
    uint8_t pb[] = {1, 2, 3, 4, 5, 6};
    uint8_t fa[10], fb[12];
    encoder.tx_packet_complete(1, 0x60, pa, sizeof(pa));
    memcpy(fa, &encoder.tx_packet, sizeof(fa));
    encoder.tx_packet_complete(2, 0x68, pb, sizeof(pb));
    memcpy(fb, &encoder.tx_packet, sizeof(fb));
    int ca = 0, cb = 0;
    bool good = true;
    a.register_success_callback([&](PacketTypeDef* p) {
        ++ca;
        good &= p->elements.id == 1 && p->elements.length == 6 &&
                memcmp(p->elements.args, pa, sizeof(pa)) == 0;
    });
    b.register_success_callback([&](PacketTypeDef* p) {
        ++cb;
        good &= p->elements.id == 2 && p->elements.length == 8 &&
                memcmp(p->elements.args, pb, sizeof(pb)) == 0;
    });
    for (int i = 0; i < 1000; ++i) {
        a.parsing(fa, 6);       // A has received one payload byte.
        b.parsing(fb, 8);       // B has received three payload bytes.
        a.parsing(fa + 6, 4);   // A must retain its own payload index.
        b.parsing(fb + 8, 4);
    }
    if (!good || ca != 1000 || cb != 1000) return false;

    fa[9] ^= 1;
    a.parsing(fa, sizeof(fa));
    if (ca != 1000) return false;
    fa[9] ^= 1;
    a.parsing(fa, sizeof(fa));
    if (ca != 1001) return false;

    // Reentrant callback must not overwrite the outer callback's frame.
    a.begin();
    int depth = 0, nested_calls = 0;
    a.register_success_callback([&](PacketTypeDef* p) {
        ++nested_calls;
        if (depth == 0) {
            ++depth;
            a.parsing(fb, sizeof(fb));
            --depth;
            good &= p->elements.id == 1 && memcmp(p->elements.args, pa, sizeof(pa)) == 0;
        } else {
            good &= p->elements.id == 2;
        }
    });
    a.parsing(fa, sizeof(fa));
    if (!good || nested_calls != 2) return false;

    uint8_t big[250] = {};
    if (encoder.tx_packet_complete(1, 1, big, 250) != 0) return false;
    if (encoder.tx_packet_complete(1, 1, nullptr, 1) != 0) return false;
    return true;
}
