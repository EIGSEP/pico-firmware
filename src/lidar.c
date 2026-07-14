#include "lidar.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "eigsep_command.h"
#include "cJSON.h"
#include "currentmon.h"
#include <stdio.h>
#include <string.h>

#define I2C_PORT i2c0
#define I2C_SDA 12
#define I2C_SCL 13
#define I2C_FREQ 100000
#define I2C_ADDR 0x66

// GRF-250 LWNX command protocol (Product Guide Rev 3, §9.3-9.4), used only
// for RFI standby. Opcode 50 "Laser firing": 0=off, 1=on (non-persistent).
#define GRF250_OP_LASER_FIRING 50
#define GRF250_I2C_TIMEOUT_US  2000
#define GRF250_LASER_RETRIES   3

static struct {
    float distance;
    bool last_op_ok;
    // RFI standby: when true the GRF-250 laser is disabled (opcode 50<-0) and
    // op() skips the distance read. currentmon keeps running (separate main.c
    // dispatch). In-RAM only; a reboot comes back with the laser on.
    bool standby;
    // Laser firing state reported in the standby status: the value read back
    // from opcode 50 when confirmable, else the commanded state.
    int laser_firing;
} lidar_data = {0};

static void init_i2c() {
    i2c_init(I2C_PORT, I2C_FREQ);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}


static void free_i2c_bus() {
    /* Temporarily bit-bang SCL as a GPIO to clock out any half-byte */
    gpio_set_function(I2C_SCL, GPIO_FUNC_SIO);
    gpio_set_dir(I2C_SCL, GPIO_OUT);
    gpio_set_function(I2C_SDA, GPIO_FUNC_SIO);
    gpio_set_dir(I2C_SDA, GPIO_IN);          // SDA as input (pull-up already on)

    for (int i = 0; i < 9; ++i) {        // 9 clocks releases most slaves
        gpio_put(I2C_SCL, 0); sleep_us(5);
        gpio_put(I2C_SCL, 1); sleep_us(5);
        if (gpio_get(I2C_SDA)) break;        // bus released
    }

    /* Restore pins to I²C mode */
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
}


void lidar_init(uint8_t app_id) {
    init_i2c();
    free_i2c_bus();
}


/* ------------------------------------------------------------------ */
/* GRF-250 LWNX command framing (for RFI standby only)                */
/* ------------------------------------------------------------------ */
/* Packet: 0xAA start, 16-bit flags (payload length in bits 15..6, write
   bit = bit 0), payload (opcode + data bytes), 16-bit CRC (little-endian)
   over every byte except the CRC. The plain 2-byte distance read in
   lidar_op() is a separate, unchanged transaction. */

// CRC-16 algorithm from the Product Guide §9.4 (verbatim).
static uint16_t grf250_crc(const uint8_t *data, uint16_t size) {
    uint16_t crc = 0;
    for (uint16_t i = 0; i < size; i++) {
        uint16_t code = crc >> 8;
        code ^= data[i];
        code ^= code >> 4;
        crc = crc << 8;
        crc ^= code;
        code = code << 5;
        crc ^= code;
        code = code << 7;
        crc ^= code;
    }
    return crc;
}

// Write one uint8 to a command opcode. Returns true if the I2C transaction
// ACKed. The GRF-250 sends no response to a write, so this confirms only the
// bus write, not that the command took effect (see grf250_read_u8).
static bool grf250_write_u8(uint8_t opcode, uint8_t value) {
    uint8_t pkt[7];
    uint16_t flags = (2u << 6) | 1u;      // payload len 2 (opcode+data), write
    pkt[0] = 0xAA;
    pkt[1] = (uint8_t)(flags & 0xFF);
    pkt[2] = (uint8_t)(flags >> 8);
    pkt[3] = opcode;
    pkt[4] = value;
    uint16_t crc = grf250_crc(pkt, 5);
    pkt[5] = (uint8_t)(crc & 0xFF);
    pkt[6] = (uint8_t)(crc >> 8);
    return i2c_write_timeout_us(I2C_PORT, I2C_ADDR, pkt, sizeof(pkt), false,
                                GRF250_I2C_TIMEOUT_US) == (int)sizeof(pkt);
}

// Read one uint8 back from a command opcode. Returns 0/1, or -1 if the
// request/response failed or the response frame/CRC was invalid. The CRC
// check means a framing mismatch degrades to -1, never a bogus value.
static int grf250_read_u8(uint8_t opcode) {
    uint8_t req[6];
    uint16_t flags = (1u << 6);           // payload len 1 (opcode only), read
    req[0] = 0xAA;
    req[1] = (uint8_t)(flags & 0xFF);
    req[2] = (uint8_t)(flags >> 8);
    req[3] = opcode;
    uint16_t crc = grf250_crc(req, 4);
    req[4] = (uint8_t)(crc & 0xFF);
    req[5] = (uint8_t)(crc >> 8);
    if (i2c_write_timeout_us(I2C_PORT, I2C_ADDR, req, sizeof(req), false,
                             GRF250_I2C_TIMEOUT_US) != (int)sizeof(req)) {
        return -1;
    }
    uint8_t resp[7];   // 0xAA + flags(2) + opcode + data + CRC(2)
    if (i2c_read_timeout_us(I2C_PORT, I2C_ADDR, resp, sizeof(resp), false,
                            GRF250_I2C_TIMEOUT_US) != (int)sizeof(resp)) {
        return -1;
    }
    if (resp[0] != 0xAA || resp[3] != opcode) {
        return -1;
    }
    uint16_t got = (uint16_t)(resp[5] | (resp[6] << 8));
    if (got != grf250_crc(resp, 5)) {
        return -1;
    }
    return resp[4];
}

static bool grf250_set_laser(bool on) {
    for (int i = 0; i < GRF250_LASER_RETRIES; i++) {
        if (grf250_write_u8(GRF250_OP_LASER_FIRING, on ? 1 : 0)) {
            return true;
        }
        sleep_ms(2);
    }
    return false;
}

static void lidar_enter_standby(void) {
    grf250_set_laser(false);
    // Confirm the laser is off via read-back where possible; fall back to the
    // commanded state (0) if the read cannot be validated.
    int rb = grf250_read_u8(GRF250_OP_LASER_FIRING);
    lidar_data.laser_firing = (rb >= 0) ? rb : 0;
    lidar_data.standby = true;
}

static void lidar_exit_standby(void) {
    lidar_data.standby = false;
    grf250_set_laser(true);
}

void lidar_server(uint8_t app_id, const char *json_str) {
    (void)app_id;
    // The only lidar commands are the universal RFI standby controls:
    // standby disables laser firing (opcode 50<-0), resume re-enables it.
    cJSON *root = cJSON_Parse(json_str);
    if (!root || !cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return;
    }
    cJSON *cmd = cJSON_GetObjectItem(root, "cmd");
    if (cJSON_IsString(cmd) && cmd->valuestring != NULL) {
        if (strcmp(cmd->valuestring, "standby") == 0) {
            lidar_enter_standby();
        } else if (strcmp(cmd->valuestring, "resume") == 0) {
            lidar_exit_standby();
        }
    }
    cJSON_Delete(root);
}

void lidar_status(uint8_t app_id) {
    if (lidar_data.standby) {
        /* Commanded-off reports status="error" (no valid distance), same as a
           fault, but with standby=true so the host can tell the two apart.
           laser_firing is the opcode-50 read-back: 0 = laser confirmed off;
           standby=true with laser_firing=1 means the write did not take. */
        send_json(6,
            KV_STR, "sensor_name", "lidar",
            KV_STR, "status", "error",
            KV_INT, "app_id", app_id,
            KV_FLOAT, "current_voltage", currentmon_voltage(),
            KV_INT, "laser_firing", lidar_data.laser_firing,
            KV_BOOL, "standby", true
        );
        return;
    }
    const char *status = lidar_data.last_op_ok ? "update" : "error";
    send_json(5,
        KV_STR, "sensor_name", "lidar",
        KV_STR, "status", status,
        KV_INT, "app_id", app_id,
        KV_FLOAT, "distance_m", lidar_data.distance,
        KV_FLOAT, "current_voltage", currentmon_voltage()
    );
    lidar_data.last_op_ok = false;
}


void lidar_reset(uint8_t app_id) {
    // Reset only the I2C peripheral. The co-located current monitor owns the
    // ADC (GP26/ADC0); deliberately leave it untouched so a lidar recovery
    // does not disturb the independent current reading.
    i2c_deinit(I2C_PORT);
    sleep_ms(50);
    lidar_init(app_id);
}

void lidar_op(uint8_t app_id) {
    if (lidar_data.standby) {
        return;  // laser off; currentmon_op() still runs (separate dispatch)
    }
    uint8_t buf[2];
    //lidar_init(app_id);
    if (i2c_read_timeout_us(I2C_PORT, I2C_ADDR, buf, 2, false, 1000) != 2) {
        lidar_reset(app_id);
        return;
    }

    uint16_t dist_cm = (uint16_t)(buf[0] << 8) | buf[1];
    if (dist_cm == 0) {                  // still not ready – ignore
        lidar_reset(app_id);
        return;
    }
    lidar_data.distance = dist_cm / 100.0;
    lidar_data.last_op_ok = true;
}


