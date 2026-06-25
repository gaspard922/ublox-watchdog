#include "ubx_protocol.h"

/* Extraction explicite little-endian, octet par octet : ne dépend pas de
 * l'endianness ou de l'alignement de la plateforme hôte. */

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int16_t read_i16_le(const uint8_t *p)
{
    return (int16_t)read_u16_le(p);
}

static int32_t read_i32_le(const uint8_t *p)
{
    return (int32_t)read_u32_le(p);
}

bool ubx_parse_nav_pvt(const uint8_t *payload, size_t len, ubx_nav_pvt_t *out)
{
    if (len != UBX_NAV_PVT_PAYLOAD_LEN) {
        return false;
    }

    out->i_tow = read_u32_le(&payload[0]);
    out->year = read_u16_le(&payload[4]);
    out->month = payload[6];
    out->day = payload[7];
    out->hour = payload[8];
    out->min = payload[9];
    out->sec = payload[10];

    uint8_t valid = payload[11];
    out->valid_date = (valid & 0x01) != 0;
    out->valid_time = (valid & 0x02) != 0;
    out->fully_resolved = (valid & 0x04) != 0;
    out->valid_mag = (valid & 0x08) != 0;

    out->t_acc = read_u32_le(&payload[12]);
    out->nano = read_i32_le(&payload[16]);

    out->fix_type = payload[20];

    uint8_t flags = payload[21];
    out->gnss_fix_ok = (flags & 0x01) != 0;
    out->diff_soln = (flags & 0x02) != 0;
    out->psm_state = (uint8_t)((flags >> 2) & 0x07);
    out->head_veh_valid = (flags & 0x20) != 0;
    out->carr_soln = (uint8_t)((flags >> 6) & 0x03);

    uint8_t flags2 = payload[22];
    out->confirmed_avai = (flags2 & 0x20) != 0;
    out->confirmed_date = (flags2 & 0x40) != 0;
    out->confirmed_time = (flags2 & 0x80) != 0;

    out->num_sv = payload[23];

    out->lon = read_i32_le(&payload[24]);
    out->lat = read_i32_le(&payload[28]);
    out->height = read_i32_le(&payload[32]);
    out->h_msl = read_i32_le(&payload[36]);
    out->h_acc = read_u32_le(&payload[40]);
    out->v_acc = read_u32_le(&payload[44]);

    out->vel_n = read_i32_le(&payload[48]);
    out->vel_e = read_i32_le(&payload[52]);
    out->vel_d = read_i32_le(&payload[56]);
    out->g_speed = read_i32_le(&payload[60]);
    out->head_mot = read_i32_le(&payload[64]);
    out->s_acc = read_u32_le(&payload[68]);
    out->head_acc = read_u32_le(&payload[72]);

    out->p_dop = read_u16_le(&payload[76]);

    uint16_t flags3 = read_u16_le(&payload[78]);
    out->invalid_llh = (flags3 & 0x01) != 0;
    out->last_correction_age = (uint8_t)((flags3 >> 4) & 0x0F);

    /* offset 80..83 : reserved0, ignoré */

    out->head_veh = read_i32_le(&payload[84]);
    out->mag_dec = read_i16_le(&payload[88]);
    out->mag_acc = read_u16_le(&payload[90]);

    return true;
}

bool ubx_parse_ack(const uint8_t *payload, size_t len, ubx_ack_t *out)
{
    if (len != UBX_ACK_PAYLOAD_LEN) {
        return false;
    }
    out->acked_class = payload[0];
    out->acked_id = payload[1];
    return true;
}

bool ubx_parse_esf_status(const uint8_t *payload, size_t len, ubx_esf_status_t *out)
{
    if (len < UBX_ESF_STATUS_FIXED_LEN) {
        return false;
    }

    uint8_t num_sens = payload[15];
    size_t expected_len = (size_t)UBX_ESF_STATUS_FIXED_LEN + (size_t)num_sens * 4u;
    if (len != expected_len) {
        return false;
    }
    if (num_sens > UBX_ESF_MAX_SENS) {
        return false;
    }

    out->i_tow = read_u32_le(&payload[0]);
    out->version = payload[4];
    /* offset 5..11 : reserved0, ignoré */
    out->fusion_mode = payload[12];
    /* offset 13..14 : reserved1, ignoré */
    out->num_sens = num_sens;

    for (uint8_t i = 0; i < num_sens; i++) {
        const uint8_t *sensor = &payload[UBX_ESF_STATUS_FIXED_LEN + (size_t)i * 4u];
        ubx_esf_sensor_t *out_sensor = &out->sensors[i];

        uint8_t sens_status1 = sensor[0];
        out_sensor->type = (uint8_t)(sens_status1 & 0x3F);
        out_sensor->used = (sens_status1 & 0x40) != 0;
        out_sensor->ready = (sens_status1 & 0x80) != 0;

        uint8_t sens_status2 = sensor[1];
        out_sensor->calib_status = (uint8_t)(sens_status2 & 0x03);
        out_sensor->time_status = (uint8_t)((sens_status2 >> 2) & 0x03);

        out_sensor->freq = sensor[2];

        uint8_t faults = sensor[3];
        out_sensor->bad_meas = (faults & 0x01) != 0;
        out_sensor->bad_ttag = (faults & 0x02) != 0;
        out_sensor->missing_meas = (faults & 0x04) != 0;
        out_sensor->noisy_meas = (faults & 0x08) != 0;
    }

    return true;
}
