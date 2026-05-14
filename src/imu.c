#include "imu.h"
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico_multi.h"

static ImuState imu;

/* ------------------------------------------------------------------ */
/* Hardware reset                                                     */
/* ------------------------------------------------------------------ */
/* Toggle the RST pin to force the BNO08x into a known state.
   Required so the sensor reliably enters RVC mode on power-up. */
static void imu_hardware_reset(void) {
    gpio_init(IMU_RST_GPIO);
    gpio_set_dir(IMU_RST_GPIO, GPIO_OUT);
    gpio_put(IMU_RST_GPIO, 0);
    sleep_ms(10);
    gpio_put(IMU_RST_GPIO, 1);
    sleep_ms(100);
}

/* ------------------------------------------------------------------ */
/* UART setup                                                         */
/* ------------------------------------------------------------------ */
static void imu_uart_init(void) {
    uart_init(IMU_UART, IMU_UART_BAUD);
    gpio_set_function(IMU_UART_RX_PIN, GPIO_FUNC_UART);
    /* Drain any stale bytes */
    while (uart_is_readable(IMU_UART))
        uart_getc(IMU_UART);
}

/* ------------------------------------------------------------------ */
/* RVC packet parsing                                                 */
/* ------------------------------------------------------------------ */

/* Verify checksum: sum of bytes 2..17 must equal byte 18 (mod 256). */
static bool rvc_checksum_ok(const uint8_t *pkt) {
    uint8_t sum = 0;
    for (int i = 2; i < RVC_PACKET_SIZE - 1; i++)
        sum += pkt[i];
    return sum == pkt[RVC_PACKET_SIZE - 1];
}

static int16_t le16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void rvc_parse(const uint8_t *pkt, RvcData *out) {
    /* Bytes 3-8: yaw, pitch, roll in centidegrees */
    out->yaw   = le16(&pkt[3])  / RVC_ANGLE_SCALE;
    out->pitch = le16(&pkt[5])  / RVC_ANGLE_SCALE;
    out->roll  = le16(&pkt[7])  / RVC_ANGLE_SCALE;
    /* Bytes 9-14: x, y, z acceleration in milli-g */
    out->accel_x = le16(&pkt[9])  / RVC_ACCEL_SCALE * GRAVITY_EARTH;
    out->accel_y = le16(&pkt[11]) / RVC_ACCEL_SCALE * GRAVITY_EARTH;
    out->accel_z = le16(&pkt[13]) / RVC_ACCEL_SCALE * GRAVITY_EARTH;
}

/* Feed one byte at a time; returns true when a complete valid packet
   has been parsed into imu.data. */
static bool rvc_feed_byte(ImuState *st, uint8_t byte) {
    /* Sync on 0xAA 0xAA header */
    if (st->rx_pos == 0) {
        if (byte == RVC_HEADER_BYTE) st->rx_pos = 1;
        return false;
    }
    if (st->rx_pos == 1) {
        if (byte == RVC_HEADER_BYTE) {
            st->rx_buf[0] = RVC_HEADER_BYTE;
            st->rx_buf[1] = RVC_HEADER_BYTE;
            st->rx_pos = 2;
        } else {
            st->rx_pos = 0;
        }
        return false;
    }

    st->rx_buf[st->rx_pos++] = byte;

    if (st->rx_pos < RVC_PACKET_SIZE)
        return false;

    /* Full packet received — validate and parse */
    st->rx_pos = 0;
    if (!rvc_checksum_ok(st->rx_buf))
        return false;

    rvc_parse(st->rx_buf, &st->data);
    return true;
}

/* ------------------------------------------------------------------ */
/* App interface                                                      */
/* ------------------------------------------------------------------ */

void imu_init(uint8_t app_id) {
    if (imu.is_initialized) return;

    if (app_id == APP_IMU_EL) {
        strncpy(imu.name, "imu_el", IMU_NAME_LEN - 1);
    } else {
        strncpy(imu.name, "imu_az", IMU_NAME_LEN - 1);
    }
    imu.name[IMU_NAME_LEN - 1] = '\0';
    imu.rx_pos = 0;
    memset(&imu.data, 0, sizeof(imu.data));

    imu_hardware_reset();
    imu_uart_init();

    imu.last_event_time = to_ms_since_boot(get_absolute_time());
    imu.is_initialized = true;
}

void imu_server(uint8_t app_id, const char *json_str) {
    (void)app_id;
    (void)json_str;
    /* RVC mode: no commands supported */
}

void imu_op(uint8_t app_id) {
    /* No-op while healthy (guarded by is_initialized check inside
       imu_init).  After an event timeout sets is_initialized = false,
       this re-runs the full init sequence to recover the sensor. */
    imu_init(app_id);
    if (!imu.is_initialized) return;

    uint32_t now = to_ms_since_boot(get_absolute_time());
    bool got_packet = false;

    /* Drain all available UART bytes */
    while (uart_is_readable(IMU_UART)) {
        uint8_t byte = uart_getc(IMU_UART);
        if (rvc_feed_byte(&imu, byte))
            got_packet = true;
    }

    if (got_packet) {
        imu.last_event_time = now;
        imu.got_packet_this_cycle = true;
    } else if ((now - imu.last_event_time) > IMU_EVENT_TIMEOUT_MS) {
        imu.is_initialized = false;
    }
}

void imu_status(uint8_t app_id) {
    const char *status = imu.got_packet_this_cycle ? "update" : "error";

    send_json(9,
        KV_STR,   "sensor_name", imu.name,
        KV_STR,   "status",      status,
        KV_INT,   "app_id",      app_id,
        KV_FLOAT, "yaw",         imu.data.yaw,
        KV_FLOAT, "pitch",       imu.data.pitch,
        KV_FLOAT, "roll",        imu.data.roll,
        KV_FLOAT, "accel_x",     imu.data.accel_x,
        KV_FLOAT, "accel_y",     imu.data.accel_y,
        KV_FLOAT, "accel_z",     imu.data.accel_z
    );
    imu.got_packet_this_cycle = false;
}
