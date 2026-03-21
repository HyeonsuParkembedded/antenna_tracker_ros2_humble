#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include <stdbool.h>
#include <stdint.h>

#define NMEA_MAX_LEN 128

typedef struct {
    double latitude;
    double longitude;
    double altitude;
    uint8_t fix_quality;
    uint8_t num_satellites;
    float hdop;
    bool valid;
    uint64_t last_fix_ms;
} gps_fix_t;

typedef struct {
    char buffer[NMEA_MAX_LEN];
    uint8_t index;
    bool receiving;
    gps_fix_t fix;
} gps_parser_t;

void gps_parser_init(gps_parser_t *parser);
void gps_parser_feed_char(gps_parser_t *parser, char c, uint64_t now_ms);
bool gps_parser_has_fix(const gps_parser_t *parser);
const gps_fix_t *gps_parser_get_fix(const gps_parser_t *parser);

#endif /* GPS_PARSER_H */
