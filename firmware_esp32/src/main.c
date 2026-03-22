#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/kernel.h>
#include <string.h>

/* ── CAN Protocol IDs ─────────────────────────────────────────────────────── */
/* TX: ESP32 → RPi4B (via shared CAN bus), 0x100~0x10A, 8 bytes each */
#define CAN_ID_TARGET_GPS      0x100  /* lat×1e7, lon×1e7 (int32 LE) */
#define CAN_ID_TARGET_STATUS   0x101  /* kf_alt(int16) + rssi(int16) + gps_fix + sats_used + utc_h + utc_m */
#define CAN_ID_BALLOON_UTC     0x102  /* utc_sec + utc_day + utc_mon + sats_view + utc_year(u16 LE) + status_flags(u16 LE) */
#define CAN_ID_BALLOON_ACCEL   0x103  /* accel_x/y/z(int16 ×100) + gyro_x(int16 ×1000) */
#define CAN_ID_BALLOON_GYROMAG 0x104  /* gyro_y/z(int16) + mag_x/y(int16 ×10) */
#define CAN_ID_BALLOON_ORIENT  0x105  /* mag_z(int16) + kf_roll/pitch(int16 ×100) + press_alt(int16) */
#define CAN_ID_BALLOON_ENV     0x106  /* board_temp + ext_temp + sht31_temp(int16 ×100) + sht31_rh(u16 ×100) */
#define CAN_ID_BALLOON_PRESS   0x107  /* ms5611_press(u32 LE, Pa) + ms5611_temp(int16 ×100) + co2(u16 ppm) */
#define CAN_ID_BALLOON_AIR     0x108  /* pm1 + pm25 + pm10(u16 LE) + ozone(int16 ppb) */
#define CAN_ID_BALLOON_SYS     0x109  /* gdk101(u16 ×100) + bat_mv(u16) + bat_temp(int16 ×100) + heater_bat + heater_board */
#define CAN_ID_BALLOON_META    0x10A  /* seq(u16 LE) + uptime_ms(u32 LE) + 0x00 + 0x00 */

/* ── LoRa Configuration ───────────────────────────────────────────────────── */
/*
 * Space Balloon 송신부(telemetry_rx_main.c) 설정과 일치:
 *   주파수: 915 MHz, SF11, BW125, CR4/5
 *
 * ⚠ 한국 ISM 대역은 920~923 MHz. 운용 전 규정 확인 필요.
 */
#define LORA_FREQ_HZ         915000000  /* 송신부와 동일 */
#define LORA_TX_POWER        17         /* dBm (PA_BOOST, RX 모드에선 미사용) */

/* ── telemetry_frame_t (Space Balloon 송신 포맷과 동일) ──────────────────── */
/* #pragma pack 이 Zephyr/GCC 에서 동작하므로 그대로 사용 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint16_t status_flags;
    uint16_t co2_ppm;
    int32_t  accel_mps2_x1000[3];
    int32_t  gyro_rads_x1000[3];
    float    mag_uT[3];
    int16_t  board_temp_c_x100;
    int16_t  external_temp_c_x100;
    int16_t  sht31_temp_c_x100;
    int16_t  bat_temp_c_x100;
    int32_t  gps_lat_deg_e7;
    int32_t  gps_lon_deg_e7;
    float    gps_alt_m;
    uint8_t  gps_fix;
    uint8_t  gps_sats_used;
    uint8_t  gps_sats_in_view_total;
    uint8_t  gps_sats_in_view_gps;
    uint8_t  gps_sats_in_view_glonass;
    uint8_t  gps_sats_in_view_galileo;
    uint8_t  gps_sats_in_view_beidou;
    uint8_t  gps_utc_hour;
    uint8_t  gps_utc_min;
    uint8_t  gps_utc_sec;
    uint8_t  gps_utc_day;
    uint8_t  gps_utc_month;
    uint16_t gps_utc_year;
    uint16_t bat_mv;
    uint16_t pm1_ugm3;
    uint16_t pm25_ugm3;
    uint16_t pm10_ugm3;
    int16_t  ozone_ppb;
    uint16_t sht31_rh_x100;
    uint32_t ms5611_press_pa;
    int16_t  ms5611_temp_c_x100;
    uint16_t gdk101_usvh_x100;
    uint8_t  heater_bat_duty_percent;
    uint8_t  heater_board_duty_percent;
    float    press_alt_m;
    float    kf_alt_m;
    float    kf_roll_deg;
    float    kf_pitch_deg;
} telemetry_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t            magic[2];      /* 0xA5, 0x5A */
    uint8_t            version;
    uint8_t            msg_type;
    uint16_t           payload_len;
    uint16_t           seq;
    uint32_t           timestamp_ms;
    telemetry_payload_t payload;
    uint16_t           crc16;
} telemetry_frame_t;

#pragma GCC diagnostic pop

#define TELEM_FRAME_SIZE  sizeof(telemetry_frame_t)
#define TELEM_MAGIC0      0xA5u
#define TELEM_MAGIC1      0x5Au

/* ── Devices ──────────────────────────────────────────────────────────────── */
static const struct device *can_dev  = DEVICE_DT_GET(DT_NODELABEL(twai));
static const struct device *lora_dev = DEVICE_DT_GET(DT_NODELABEL(lora0));

/* ── CRC16-CCITT-FALSE (seed=0xFFFF, poly=0x1021) ────────────────────────── */
static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u)
                  ? (uint16_t)((crc << 1) ^ 0x1021u)
                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ── CAN TX Helper ────────────────────────────────────────────────────────── */
static void can_send_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    struct can_frame frame = {0};
    frame.id  = id;
    frame.dlc = dlc;
    memcpy(frame.data, data, dlc);
    can_send(can_dev, &frame, K_MSEC(5), NULL, NULL);
}

/* ── LoRa → CAN Bridge ────────────────────────────────────────────────────── */
static void process_lora_packet(const uint8_t *buf, int len, int16_t rssi)
{
    /* 최소 크기 확인 */
    if (len < (int)TELEM_FRAME_SIZE) {
        printk("LoRa: short packet (%d/%zu bytes), discarded\n",
               len, TELEM_FRAME_SIZE);
        return;
    }

    const telemetry_frame_t *f = (const telemetry_frame_t *)buf;

    /* Magic 검증 */
    if (f->magic[0] != TELEM_MAGIC0 || f->magic[1] != TELEM_MAGIC1) {
        printk("LoRa: bad magic 0x%02X 0x%02X, discarded\n",
               f->magic[0], f->magic[1]);
        return;
    }

    /* CRC16 검증 (헤더 + payload, CRC 필드 제외) */
    size_t crc_len = 12u + f->payload_len;   /* header(12) + payload */
    if (crc_len + 2u > (size_t)len) {
        printk("LoRa: payload_len mismatch, discarded\n");
        return;
    }
    uint16_t calc_crc = crc16_ccitt_false(buf, crc_len);
    if (calc_crc != f->crc16) {
        printk("LoRa: CRC error (calc=0x%04X recv=0x%04X), discarded\n",
               calc_crc, f->crc16);
        return;
    }

    /* ── 변환 중간값 ──────────────────────────────────────────────────────── */
    int32_t lat    = f->payload.gps_lat_deg_e7;
    int32_t lon    = f->payload.gps_lon_deg_e7;
    int16_t kf_alt = (int16_t)f->payload.kf_alt_m;

    int16_t accel_x  = (int16_t)(f->payload.accel_mps2_x1000[0] / 10);
    int16_t accel_y  = (int16_t)(f->payload.accel_mps2_x1000[1] / 10);
    int16_t accel_z  = (int16_t)(f->payload.accel_mps2_x1000[2] / 10);
    int16_t gyro_x   = (int16_t)(f->payload.gyro_rads_x1000[0]);
    int16_t gyro_y   = (int16_t)(f->payload.gyro_rads_x1000[1]);
    int16_t gyro_z   = (int16_t)(f->payload.gyro_rads_x1000[2]);
    int16_t mag_x    = (int16_t)(f->payload.mag_uT[0] * 10.0f);
    int16_t mag_y    = (int16_t)(f->payload.mag_uT[1] * 10.0f);
    int16_t mag_z    = (int16_t)(f->payload.mag_uT[2] * 10.0f);
    int16_t kf_roll  = (int16_t)(f->payload.kf_roll_deg  * 100.0f);
    int16_t kf_pitch = (int16_t)(f->payload.kf_pitch_deg * 100.0f);
    int16_t press_alt = (int16_t)(f->payload.press_alt_m);

    uint8_t d[8];

    /* 0x100: lat×1e7, lon×1e7 */
    memcpy(d,     &lat, 4);
    memcpy(d + 4, &lon, 4);
    can_send_frame(CAN_ID_TARGET_GPS, d, 8);

    /* 0x101: kf_alt(int16) + rssi(int16) + gps_fix(u8) + sats_used(u8) + utc_h(u8) + utc_m(u8) */
    memcpy(d,     &kf_alt, 2);
    memcpy(d + 2, &rssi,   2);
    d[4] = f->payload.gps_fix;
    d[5] = f->payload.gps_sats_used;
    d[6] = f->payload.gps_utc_hour;
    d[7] = f->payload.gps_utc_min;
    can_send_frame(CAN_ID_TARGET_STATUS, d, 8);

    /* 0x102: utc_sec + utc_day + utc_mon + sats_view + utc_year(u16 LE) + status_flags(u16 LE) */
    d[0] = f->payload.gps_utc_sec;
    d[1] = f->payload.gps_utc_day;
    d[2] = f->payload.gps_utc_month;
    d[3] = f->payload.gps_sats_in_view_total;
    memcpy(d + 4, &f->payload.gps_utc_year,   2);
    memcpy(d + 6, &f->payload.status_flags,    2);
    can_send_frame(CAN_ID_BALLOON_UTC, d, 8);

    /* 0x103: accel_x/y/z(int16 ×100) + gyro_x(int16 ×1000) */
    memcpy(d,     &accel_x, 2);
    memcpy(d + 2, &accel_y, 2);
    memcpy(d + 4, &accel_z, 2);
    memcpy(d + 6, &gyro_x,  2);
    can_send_frame(CAN_ID_BALLOON_ACCEL, d, 8);

    /* 0x104: gyro_y/z(int16 ×1000) + mag_x/y(int16 ×10) */
    memcpy(d,     &gyro_y, 2);
    memcpy(d + 2, &gyro_z, 2);
    memcpy(d + 4, &mag_x,  2);
    memcpy(d + 6, &mag_y,  2);
    can_send_frame(CAN_ID_BALLOON_GYROMAG, d, 8);

    /* 0x105: mag_z(int16) + kf_roll/pitch(int16 ×100) + press_alt(int16) */
    memcpy(d,     &mag_z,     2);
    memcpy(d + 2, &kf_roll,   2);
    memcpy(d + 4, &kf_pitch,  2);
    memcpy(d + 6, &press_alt, 2);
    can_send_frame(CAN_ID_BALLOON_ORIENT, d, 8);

    /* 0x106: board_temp + ext_temp + sht31_temp(int16 ×100) + sht31_rh(u16 ×100) */
    memcpy(d,     &f->payload.board_temp_c_x100,    2);
    memcpy(d + 2, &f->payload.external_temp_c_x100, 2);
    memcpy(d + 4, &f->payload.sht31_temp_c_x100,    2);
    memcpy(d + 6, &f->payload.sht31_rh_x100,        2);
    can_send_frame(CAN_ID_BALLOON_ENV, d, 8);

    /* 0x107: ms5611_press(u32 LE, Pa) + ms5611_temp(int16 ×100) + co2(u16 ppm) */
    memcpy(d,     &f->payload.ms5611_press_pa,    4);
    memcpy(d + 4, &f->payload.ms5611_temp_c_x100, 2);
    memcpy(d + 6, &f->payload.co2_ppm,            2);
    can_send_frame(CAN_ID_BALLOON_PRESS, d, 8);

    /* 0x108: pm1 + pm25 + pm10(u16 LE) + ozone(int16 ppb) */
    memcpy(d,     &f->payload.pm1_ugm3,  2);
    memcpy(d + 2, &f->payload.pm25_ugm3, 2);
    memcpy(d + 4, &f->payload.pm10_ugm3, 2);
    memcpy(d + 6, &f->payload.ozone_ppb, 2);
    can_send_frame(CAN_ID_BALLOON_AIR, d, 8);

    /* 0x109: gdk101(u16 ×100) + bat_mv(u16) + bat_temp(int16 ×100) + heater_bat(u8) + heater_board(u8) */
    memcpy(d,     &f->payload.gdk101_usvh_x100,           2);
    memcpy(d + 2, &f->payload.bat_mv,                     2);
    memcpy(d + 4, &f->payload.bat_temp_c_x100,            2);
    d[6] = f->payload.heater_bat_duty_percent;
    d[7] = f->payload.heater_board_duty_percent;
    can_send_frame(CAN_ID_BALLOON_SYS, d, 8);

    /* 0x10A: seq(u16 LE) + uptime_ms(u32 LE) + 0x00 + 0x00 */
    memcpy(d,     &f->seq,          2);
    memcpy(d + 2, &f->timestamp_ms, 4);
    d[6] = 0x00u;
    d[7] = 0x00u;
    can_send_frame(CAN_ID_BALLOON_META, d, 8);

    printk("LoRa RX → CAN(11fr): seq=%u lat=%d lon=%d kf_alt=%d rssi=%d fix=%u sats=%u\n",
           f->seq, lat, lon, (int)kf_alt, (int)rssi,
           f->payload.gps_fix, f->payload.gps_sats_used);
}

/* ── Main ─────────────────────────────────────────────────────────────────── */
int main(void)
{
    printk("ESP32 LoRa-CAN Bridge — Zephyr RTOS (TTGO LoRa32 V2.1)\n");
    printk("CAN: TWAI 500kbps (TX=GPIO32, RX=GPIO33)\n");
    printk("LoRa: SX1276 %.0fMHz SF11 BW125 CR4/5  frame=%zu bytes\n",
           LORA_FREQ_HZ / 1e6, TELEM_FRAME_SIZE);

    /* ── CAN init ────────────────────────────────────────────────────────── */
    if (!device_is_ready(can_dev)) {
        printk("TWAI CAN device not ready!\n");
        return -1;
    }
    can_start(can_dev);
    printk("TWAI CAN OK\n");

    /* ── LoRa init ───────────────────────────────────────────────────────── */
    if (!device_is_ready(lora_dev)) {
        printk("LoRa device not ready!\n");
        return -1;
    }

    struct lora_modem_config lora_cfg = {
        .frequency    = LORA_FREQ_HZ,
        .bandwidth    = BW_125_KHZ,
        .datarate     = SF_11,   /* 송신부(SF11)와 일치 */
        .preamble_len = 8,
        .coding_rate  = CR_4_5,
        .tx_power     = LORA_TX_POWER,
        .tx           = false,   /* RX 모드 */
    };
    if (lora_config(lora_dev, &lora_cfg) < 0) {
        printk("LoRa config failed!\n");
        return -1;
    }
    printk("LoRa OK — listening on %.0f MHz\n", LORA_FREQ_HZ / 1e6);

    /* ── RX Loop ─────────────────────────────────────────────────────────── */
    uint8_t buf[256];   /* telemetry_frame_t ~130 bytes に余裕 */
    int16_t rssi;
    int8_t  snr;

    while (1) {
        /* Blocking receive, 1s timeout → 계속 polling */
        int len = lora_recv(lora_dev, buf, sizeof(buf), K_MSEC(1000),
                            &rssi, &snr);
        if (len > 0) {
            process_lora_packet(buf, len, rssi);
        }
        /* timeout(len==0) or error(len<0): 재시도 */
    }

    return 0;
}
