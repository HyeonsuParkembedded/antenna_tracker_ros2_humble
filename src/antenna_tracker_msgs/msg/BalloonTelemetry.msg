std_msgs/Header header

# GPS
float64 latitude
float64 longitude
float32 kf_altitude_m
float32 press_altitude_m
uint8 gps_fix
uint8 gps_sats_used
uint8 gps_sats_in_view
uint8 utc_hour
uint8 utc_min
uint8 utc_sec
uint8 utc_day
uint8 utc_month
uint16 utc_year

# IMU (Space Balloon 온보드 센서)
float32 accel_x_mps2
float32 accel_y_mps2
float32 accel_z_mps2
float32 gyro_x_rads
float32 gyro_y_rads
float32 gyro_z_rads
float32 mag_x_ut
float32 mag_y_ut
float32 mag_z_ut
float32 kf_roll_deg
float32 kf_pitch_deg

# 환경
float32 board_temp_c
float32 external_temp_c
float32 sht31_temp_c
float32 sht31_rh_percent
uint32 ms5611_press_pa
float32 ms5611_temp_c

# 공기질
uint16 co2_ppm
uint16 pm1_ugm3
uint16 pm25_ugm3
uint16 pm10_ugm3
int16 ozone_ppb

# 방사선 / 배터리 / 히터
float32 gdk101_usvh
uint16 bat_mv
float32 bat_temp_c
uint8 heater_bat_duty_percent
uint8 heater_board_duty_percent

# 링크 품질
int16 rssi_dbm
uint16 status_flags

# 시스템
uint16 seq
uint32 uptime_ms
