#include "AT32_OTA.h"
#include "Global.h"
#include "soc/rtc_cntl_reg.h"

void AT32_OTA::oled_msg(OLED_t &oled, const char *l0, const char *l1, const char *l2, const char *l3) {
    if (l0) oled.set_custom_text(0, String(l0));
    if (l1) oled.set_custom_text(1, String(l1));
    if (l2) oled.set_custom_text(2, String(l2));
    if (l3) oled.set_custom_text(3, String(l3));
    oled.show_custom();
}

static bool wait_reply(HardwareSerial &uart, uint8_t expect_cmd,
                       uint8_t *out_args, uint8_t *out_args_len, uint32_t timeout_ms)
{
    uint32_t start = millis();
    uint8_t state = 0;
    uint8_t id = 0, len = 0, cmd = 0;
    uint8_t args[16];
    uint8_t arg_idx = 0;
    uint16_t checksum = 0;
    uint8_t skip_remaining = 0;

    while (millis() - start < timeout_ms) {
        if (!uart.available()) { delay(1); continue; }
        uint8_t ch = uart.read();

        if (skip_remaining > 0) {
            skip_remaining--;
            continue;
        }

        switch (state) {
            case 0: state = (ch == 0xFF) ? 1 : 0; break;
            case 1: state = (ch == 0xFF) ? 2 : 0; break;
            case 2: id = ch; checksum = ch; state = 3; break;
            case 3:
                len = ch; checksum += ch; arg_idx = 0;
                if (len < 2) { state = 0; break; }
                if (len > 16) {
                    skip_remaining = len;
                    state = 0;
                    break;
                }
                state = 4; break;
            case 4:
                cmd = ch; checksum += ch;
                state = (len > 2) ? 5 : 6; break;
            case 5:
                if (arg_idx < 16) args[arg_idx] = ch;
                arg_idx++; checksum += ch;
                if (arg_idx >= len - 2) state = 6;
                break;
            case 6:
                state = 0;
                if (ch == (uint8_t)(~checksum) && cmd == expect_cmd) {
                    if (out_args && out_args_len) {
                        *out_args_len = (arg_idx > 16) ? 16 : arg_idx;
                        memcpy(out_args, args, *out_args_len);
                    }
                    return true;
                }
                break;
        }
    }
    return false;
}

bool AT32_OTA::query_version(CommProtocol_t &protocol, HardwareSerial &uart, uint8_t ver[3]) {
    for (int retry = 0; retry < 3; retry++) {
        while (uart.available()) uart.read();
        delay(50);
        while (uart.available()) uart.read();

        uint8_t len = protocol.tx_packet_complete(0xFF, CMD_FW_QUERY, nullptr, 0);
        uart.write((const uint8_t *)&protocol.tx_packet, len);
        uart.flush();

        uint8_t args[8];
        uint8_t args_len = 0;
        if (wait_reply(uart, CMD_FW_QUERY, args, &args_len, 500)) {
            if (args_len >= 3) {
                ver[0] = args[0]; ver[1] = args[1]; ver[2] = args[2];
                return true;
            }
        }
        delay(300);
    }
    return false;
}

bool AT32_OTA::check_and_update(CommProtocol_t &protocol, HardwareSerial &uart, OLED_t &oled) {
    Serial.printf("[OTA] Start: embedded=%d.%d.%d size=%d\n",
                  AT32_FW_VERSION_MAJOR, AT32_FW_VERSION_MINOR, AT32_FW_VERSION_PATCH, AT32_FW_SIZE);

    uint8_t cur_ver[3] = {0, 0, 0};
    bool got = query_version(protocol, uart, cur_ver);
    Serial.printf("[OTA] query: got=%d ver=%d.%d.%d\n", got, cur_ver[0], cur_ver[1], cur_ver[2]);

    char ver_now[24], ver_new[24];
    if (got) {
        snprintf(ver_now, sizeof(ver_now), "Now:  %d.%d.%d", cur_ver[0], cur_ver[1], cur_ver[2]);
    } else {
        snprintf(ver_now, sizeof(ver_now), "Now:  unknown");
    }
    snprintf(ver_new, sizeof(ver_new), "New:  %d.%d.%d", AT32_FW_VERSION_MAJOR, AT32_FW_VERSION_MINOR, AT32_FW_VERSION_PATCH);

    if (got) {
        uint32_t cur = (cur_ver[0] << 16) | (cur_ver[1] << 8) | cur_ver[2];
        uint32_t emb = (AT32_FW_VERSION_MAJOR << 16) | (AT32_FW_VERSION_MINOR << 8) | AT32_FW_VERSION_PATCH;
        if (cur >= emb) {
            Serial.println("[OTA] Up to date, skip.");
            return false;
        }
    }

    Serial.println("[OTA] Upgrading...");

    // 禁用 brownout detector，AT32 复位时电源瞬态可能触发 ESP32 重启
    uint32_t saved_rtc = REG_READ(RTC_CNTL_BROWN_OUT_REG);
    REG_WRITE(RTC_CNTL_BROWN_OUT_REG, 0);

    // Step 1: 发 CMD 90 让 AT32 进 Bootloader
    Serial.println("[OTA] Step1: CMD 90");
    Serial.flush();
    while (uart.available()) uart.read();
    {
        uint8_t len = protocol.tx_packet_complete(0xFF, CMD_FIRMWARE_UPDATE, nullptr, 0);
        uart.write((const uint8_t *)&protocol.tx_packet, len);
        uart.flush();
    }
    delay(3000);
    while (uart.available()) uart.read();
    Serial.println("[OTA] Wait done");

    oled_msg(oled, ver_now, ver_new, "Erasing...", "0%");

    // 确认 Bootloader 就绪
    Serial.println("[OTA] Checking bootloader...");
    {
        uint8_t args[8], args_len = 0;
        uint8_t len = protocol.tx_packet_complete(0xFF, CMD_FW_QUERY, nullptr, 0);
        uart.write((const uint8_t *)&protocol.tx_packet, len);
        uart.flush();
        bool bl_ok = wait_reply(uart, CMD_FW_QUERY, args, &args_len, 2000);
        Serial.printf("[OTA] Bootloader ready: %d\n", bl_ok);
        if (!bl_ok) {
            Serial.println("[OTA] FAIL: bootloader not responding");
            oled_msg(oled, "OTA FAILED", "Bootloader", "not responding", "");
            delay(3000);
            REG_WRITE(RTC_CNTL_BROWN_OUT_REG, saved_rtc);
            return false;
        }
    }

    // Step 2: 擦除
    Serial.println("[OTA] Step2: Erase");
    while (uart.available()) uart.read();
    {
        uint8_t len = protocol.tx_packet_complete(0xFF, CMD_FW_START, nullptr, 0);
        uart.write((const uint8_t *)&protocol.tx_packet, len);
        uart.flush();
        uint8_t args[8], args_len = 0;
        bool erase_ok = wait_reply(uart, CMD_FW_START, args, &args_len, 10000);
        Serial.printf("[OTA] Erase done: %d\n", erase_ok);
        if (!erase_ok) {
            Serial.println("[OTA] FAIL: erase timeout");
            oled_msg(oled, "OTA FAILED", "Erase timeout", "", "");
            delay(3000);
            REG_WRITE(RTC_CNTL_BROWN_OUT_REG, saved_rtc);
            return false;
        }
    }

    // Step 3: 写入
    Serial.println("[OTA] Step3: Write");
    const uint16_t PKT_SIZE = 128;
    uint32_t total = (AT32_FW_SIZE + PKT_SIZE - 1) / PKT_SIZE;

    for (uint32_t i = 0; i < total; i++) {
        uint32_t offset = i * PKT_SIZE;
        uint16_t chunk_len = PKT_SIZE;
        if (offset + chunk_len > AT32_FW_SIZE)
            chunk_len = AT32_FW_SIZE - offset;

        uint8_t padded = chunk_len;
        if (padded % 4) padded += 4 - (padded % 4);

        static uint8_t pkt[2 + 132];
        pkt[0] = i & 0xFF;
        pkt[1] = (i >> 8) & 0xFF;
        memcpy(&pkt[2], &at32_firmware[offset], chunk_len);
        for (uint8_t j = chunk_len; j < padded; j++)
            pkt[2 + j] = 0xFF;

        uint8_t tx_len = protocol.tx_packet_complete(0xFF, CMD_FW_DATA, pkt, 2 + padded);
        uart.write((const uint8_t *)&protocol.tx_packet, tx_len);
        uart.flush();
        delay(15);
        while (uart.available()) uart.read();

        if (i % 10 == 0 || i == total - 1) {
            uint8_t pct = (i + 1) * 100 / total;
            char progress[8];
            snprintf(progress, sizeof(progress), "%d%%", pct);
            oled_msg(oled, ver_now, ver_new, "Writing flash...", progress);
        }

        if (i % 50 == 0 || i == total - 1) {
            Serial.printf("[OTA] %d/%d\n", (int)(i+1), (int)total);
        }
    }

    // Step 4: 完成
    Serial.println("[OTA] Step4: FW_END");
    Serial.flush();
    while (uart.available()) uart.read();
    {
        uint8_t len = protocol.tx_packet_complete(0xFF, CMD_FW_END, nullptr, 0);
        uart.write((const uint8_t *)&protocol.tx_packet, len);
        uart.flush();
    }
    delay(3000);
    while (uart.available()) uart.read();

    // Step 5: 验证
    uint8_t new_ver[3] = {0, 0, 0};
    bool ok = query_version(protocol, uart, new_ver);
    Serial.printf("[OTA] Verify: ok=%d ver=%d.%d.%d\n", ok, new_ver[0], new_ver[1], new_ver[2]);

    if (ok) {
        char done_msg[24];
        snprintf(done_msg, sizeof(done_msg), "Done: %d.%d.%d", new_ver[0], new_ver[1], new_ver[2]);
        oled_msg(oled, ver_now, done_msg, "Success!", "100%");
    } else {
        oled_msg(oled, ver_now, ver_new, "Cannot verify", "100%");
    }

    Serial.println("[OTA] Done.");
    delay(2000);
    REG_WRITE(RTC_CNTL_BROWN_OUT_REG, saved_rtc);
    return true;
}
