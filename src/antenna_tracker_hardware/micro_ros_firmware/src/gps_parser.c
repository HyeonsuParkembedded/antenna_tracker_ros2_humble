#include "gps_parser.h"
#include <string.h>
#include <stdlib.h>

static double nmea_to_decimal(const char *raw, char direction)
{
    if (!raw || raw[0] == '\0') {
        return 0.0;
    }

    double value = atof(raw);
    int degrees = (int)(value / 100.0);
    double minutes = value - degrees * 100.0;
    double decimal = degrees + minutes / 60.0;

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

static char *get_field(char *sentence, int field_num)
{
    int count = 0;
    char *p = sentence;

    while (*p && count < field_num) {
        if (*p == ',') {
            count++;
        }
        p++;
    }

    return p;
}

static void parse_gga(gps_parser_t *parser, char *sentence, uint64_t now_ms)
{
    /* $GPGGA,hhmmss.ss,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,...*cs */
    char *lat_str = get_field(sentence, 2);
    char *lat_dir_str = get_field(sentence, 3);
    char *lon_str = get_field(sentence, 4);
    char *lon_dir_str = get_field(sentence, 5);
    char *quality_str = get_field(sentence, 6);
    char *num_sv_str = get_field(sentence, 7);
    char *hdop_str = get_field(sentence, 8);
    char *alt_str = get_field(sentence, 9);

    int quality = atoi(quality_str);

    if (quality > 0) {
        char lat_dir = lat_dir_str[0];
        char lon_dir = lon_dir_str[0];

        /* Null-terminate fields at commas */
        char *comma;
        comma = strchr(lat_str, ','); if (comma) *comma = '\0';
        comma = strchr(lon_str, ','); if (comma) *comma = '\0';
        comma = strchr(alt_str, ','); if (comma) *comma = '\0';
        comma = strchr(hdop_str, ','); if (comma) *comma = '\0';

        parser->fix.latitude = nmea_to_decimal(lat_str, lat_dir);
        parser->fix.longitude = nmea_to_decimal(lon_str, lon_dir);
        parser->fix.altitude = atof(alt_str);
        parser->fix.fix_quality = (uint8_t)quality;
        parser->fix.num_satellites = (uint8_t)atoi(num_sv_str);
        parser->fix.hdop = (float)atof(hdop_str);
        parser->fix.valid = true;
        parser->fix.last_fix_ms = now_ms;
    }
}

static bool verify_checksum(const char *sentence)
{
    if (sentence[0] != '$') {
        return false;
    }

    uint8_t checksum = 0;
    const char *p = sentence + 1;

    while (*p && *p != '*') {
        checksum ^= (uint8_t)*p;
        p++;
    }

    if (*p == '*') {
        uint8_t expected = (uint8_t)strtol(p + 1, NULL, 16);
        return checksum == expected;
    }

    return false;
}

void gps_parser_init(gps_parser_t *parser)
{
    memset(parser, 0, sizeof(gps_parser_t));
}

void gps_parser_feed_char(gps_parser_t *parser, char c, uint64_t now_ms)
{
    if (c == '$') {
        parser->index = 0;
        parser->receiving = true;
    }

    if (!parser->receiving) {
        return;
    }

    if (parser->index < NMEA_MAX_LEN - 1) {
        parser->buffer[parser->index++] = c;
    }

    if (c == '\n' || parser->index >= NMEA_MAX_LEN - 1) {
        parser->buffer[parser->index] = '\0';
        parser->receiving = false;

        if (verify_checksum(parser->buffer)) {
            if (strncmp(parser->buffer + 3, "GGA", 3) == 0) {
                parse_gga(parser, parser->buffer, now_ms);
            }
        }
    }
}

bool gps_parser_has_fix(const gps_parser_t *parser)
{
    return parser->fix.valid;
}

const gps_fix_t *gps_parser_get_fix(const gps_parser_t *parser)
{
    return &parser->fix;
}
