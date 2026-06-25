#include "ubx_cfg.h"

#include "ubx_checksum.h"
#include "ubx_parser.h" /* UBX_SYNC_CHAR_1 / UBX_SYNC_CHAR_2 */

bool ubx_build_cfg_valset(uint8_t *out_buf, size_t out_buf_size, uint8_t layers,
                          const ubx_cfg_kv_t *kvs, size_t num_kvs, size_t *out_len)
{
    /* payload = version(1) + layers(1) + transaction(1) + reserved(1)
     *         + num_kvs * (keyID(4) + value(1)) */
    size_t payload_len = 4u + num_kvs * 5u;
    size_t frame_len = 2u /* sync */ + 1u /* class */ + 1u /* id */ + 2u /* length */
                       + payload_len + 2u /* checksum */;

    if (out_buf_size < frame_len) {
        return false;
    }

    size_t i = 0;
    out_buf[i++] = UBX_SYNC_CHAR_1;
    out_buf[i++] = UBX_SYNC_CHAR_2;
    out_buf[i++] = UBX_CFG_CLASS;
    out_buf[i++] = UBX_CFG_VALSET_ID;
    out_buf[i++] = (uint8_t)(payload_len & 0xFFu);
    out_buf[i++] = (uint8_t)((payload_len >> 8) & 0xFFu);

    out_buf[i++] = 0; /* version */
    out_buf[i++] = layers;
    out_buf[i++] = UBX_CFG_TRANSACTION_NONE;
    out_buf[i++] = 0; /* reserved */

    for (size_t k = 0; k < num_kvs; k++) {
        uint32_t key = kvs[k].key_id;
        out_buf[i++] = (uint8_t)(key & 0xFFu);
        out_buf[i++] = (uint8_t)((key >> 8) & 0xFFu);
        out_buf[i++] = (uint8_t)((key >> 16) & 0xFFu);
        out_buf[i++] = (uint8_t)((key >> 24) & 0xFFu);
        out_buf[i++] = kvs[k].value;
    }

    /* Checksum calculé sur classe+ID+longueur+payload, jamais sur les
     * sync bytes : on saute les 2 premiers octets de out_buf. */
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t j = 2; j < i; j++) {
        ubx_checksum_update(&ck_a, &ck_b, out_buf[j]);
    }
    out_buf[i++] = ck_a;
    out_buf[i++] = ck_b;

    *out_len = i;
    return true;
}
