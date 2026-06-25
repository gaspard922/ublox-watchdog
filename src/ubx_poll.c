#include "ubx_poll.h"

#include "ubx_checksum.h"
#include "ubx_parser.h" /* UBX_SYNC_CHAR_1 / UBX_SYNC_CHAR_2 */

bool ubx_build_poll_request(uint8_t *out_buf, size_t out_buf_size, uint8_t msg_class, uint8_t msg_id, size_t *out_len)
{
    const size_t frame_len = 8; /* sync(2)+class(1)+id(1)+length(2)+checksum(2), payload vide */

    if (out_buf_size < frame_len) {
        return false;
    }

    out_buf[0] = UBX_SYNC_CHAR_1;
    out_buf[1] = UBX_SYNC_CHAR_2;
    out_buf[2] = msg_class;
    out_buf[3] = msg_id;
    out_buf[4] = 0; /* longueur = 0 (LSB) */
    out_buf[5] = 0; /* longueur = 0 (MSB) */

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    ubx_checksum_update(&ck_a, &ck_b, msg_class);
    ubx_checksum_update(&ck_a, &ck_b, msg_id);
    ubx_checksum_update(&ck_a, &ck_b, 0);
    ubx_checksum_update(&ck_a, &ck_b, 0);

    out_buf[6] = ck_a;
    out_buf[7] = ck_b;

    *out_len = frame_len;
    return true;
}
