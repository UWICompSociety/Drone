#include <Arduino.h>

#include "pel_log.h"
#include "pel_msp.h"

size_t
pel_msp_send(uint8_t opcode, const uint8_t *data, size_t n)
{
    int bytes = 0;
    uint8_t checksum = 0;

    bytes = Naze32Serial.write((byte *) "$M<", 3);
    bytes += Naze32Serial.write(n);
    checksum ^= n;

    bytes += Naze32Serial.write(opcode);
    checksum ^= opcode;

    bytes += Naze32Serial.write(checksum);

    return bytes;
}

size_t
pel_msp_recv(uint8_t *buf, size_t n)
{
    int8_t b;
    size_t data_len;
    uint8_t command_code;
    uint8_t checksum;
    size_t offset;
    enum {
        sw_idle,
        sw_header_start,
        sw_header_m,
        sw_header_arrow,
        sw_header_size,
        sw_header_cmd
    } state;

    delay(60);

    state = sw_idle;

    while (Naze32Serial.available() > 0) {
        b = Naze32Serial.read();
        if (b == -1)
            continue;

        switch (state) {
            case sw_idle:
                switch (b) {
                    case '$':
                        state = sw_header_start;
                        break;

                    default:
                        state = sw_idle;
                        break;
                }

                break;

            case sw_header_start:
                switch (b) {
                    case 'M':
                        state = sw_header_m;
                        break;

                    default:
                        state = sw_idle;
                        break;
                }

                break;

            case sw_header_m:
                switch (b) {
                    case '>':
                        state = sw_header_arrow;
                        break;

                    default:
                        state = sw_idle;
                        break;
                }

                break;

            /* if sw_header_start begin message size check */
            case sw_header_arrow:
                data_len = b;
                
                /*
                 * Now we are expecting the payload size.  If the size indicated is
                 * larger than the buffer, dump the payload.
                 */
                if (data_len > n) {
                    state = sw_idle;
                    break;
                } else {
                    offset = 0;

                    checksum = 0;
                    checksum ^= (uint8_t) b;

                    state = sw_header_size;
                    break;
                }

                break;

            case sw_header_size:
                command_code = (uint8_t) b;
                checksum ^= (uint8_t) b;

                state = sw_header_cmd;
                break;

            case sw_header_cmd:
                if (offset < data_len) {
                    checksum ^= (uint8_t) b;
                    buf[offset++] = (uint8_t) b;
                    /* else command loops here until offset is no longer less that data_len */
                } else {
                    /*
                     * Verify that checksum and b are of the same value.  Then,
                     * assuming true, compare command_code against available
                     * commands in pel_msp.h and execute related command
                     */
                    if (checksum == b) {
                        /* TODO */
                    }

                    state = sw_idle;
                }

                break;
        }
    }

    return offset;
}

void
pel_msp_attitude(uint8_t *buf)
{
    int16_t angx = readint16(buf);
    int16_t angy = readint16(buf + sizeof (int16_t));
    int16_t heading = readint16(buf + 2 * sizeof (int16_t));

    pel_log_debug("Attitude Recieved: ");
    pel_log_debug(angx);
    pel_log_debug(", ");
    pel_log_debug(angy);
    pel_log_debug(", ");
    pel_log_debug(heading);
    pel_log_debug("\n");
}

void
pel_msp_raw_imu(uint8_t *buf)
{
    int16_t accx;
    int16_t accy;
    int16_t accz;
    int16_t gyrx;
    int16_t gyry;
    int16_t gyrz;
    int16_t magx;
    int16_t magy;
    int16_t magz;

    accx = readint16(buf);
    accy = readint16(buf + 2);
    accz = readint16(buf + 4);
    gyrx = readint16(buf + 6);
    gyry = readint16(buf + 8);
    gyrz = readint16(buf + 10);
    magx = readint16(buf + 12);
    magy = readint16(buf + 14);
    magz = readint16(buf + 16);
    
    pel_log_debug("Raw GPS Recieved: ");
    pel_log_debug(accx);
    pel_log_debug(", ");
    pel_log_debug(accy);
    pel_log_debug(", ");
    pel_log_debug(accz);
    pel_log_debug(", ");
    pel_log_debug(gyrx);
    pel_log_debug(", ");
    pel_log_debug(gyry);
    pel_log_debug(", ");
    pel_log_debug(gyrz);
    pel_log_debug(", ");
    pel_log_debug(magx);
    pel_log_debug(", ");
    pel_log_debug(magy);
    pel_log_debug(", ");
    pel_log_debug(magz);
    pel_log_debug("\n");
}

void
pel_msp_raw_gps(uint8_t *buf)
{
    uint8_t fix;
    uint8_t num_sat;
    uint32_t lat;
    uint32_t lng;
    uint16_t altitude;
    uint16_t pel_speed;
    uint16_t ground_course;

    fix = buf[0];
    num_sat = buf[1];
    lat = readuint32(buf + 2);
    lng = readuint32(buf + 6);
    altitude = readuint16(buf + 10);
    pel_speed = readuint16(buf + 12);
    ground_course = readuint16(buf + 14);

    pel_log_debug("Raw GPS Recieved: ");
    pel_log_debug(fix);
    pel_log_debug(", ");
    pel_log_debug(num_sat);
    pel_log_debug(", ");
    pel_log_debug(lat);
    pel_log_debug(", ");
    pel_log_debug(lng);
    pel_log_debug(", ");
    pel_log_debug(altitude);
    pel_log_debug(", ");
    pel_log_debug(pel_speed);
    pel_log_debug(", ");
    pel_log_debug(ground_course);
    pel_log_debug("\n");
}
void
pel_msp_altitude(uint8_t *buf)
{
    int32_t est_alt;
    int16_t vario;

    est_alt = readint16(buf);
    vario = readint16(buf + sizeof (int32_t));

    
    pel_log_debug("Altitude Recieved: ");
    pel_log_debug(est_alt);
    pel_log_debug(", ");
    pel_log_debug(vario);
    pel_log_debug("\n");
}

void
pel_msp_rc(uint8_t *buf)
{
    int c;
    uint16_t a[16];
    
    for (c = 0; c < 16; c++)
        a[c] = readuint16(buf + 2 * c);

    pel_log_debug("RC Values: ");
    
    for (c = 0; c < 16; c++) {
        pel_log_debug(a[c]);
        pel_log_debug(",");
    }

    pel_log_debug("\n");
}

void
pel_msp_ident(uint8_t *buf)
{
    uint8_t ver;
    uint8_t multitype;
    uint8_t msp_version;
    uint32_t capability;

    ver = buf[0];
    multitype = buf[1];
    msp_version = buf[2];
    capability = readuint32(buf + 3);

    pel_log_debug(ver);
    pel_log_debug(multitype);
    pel_log_debug(msp_version);
    pel_log_debug(capability);
    pel_log_debug("\n");
}

