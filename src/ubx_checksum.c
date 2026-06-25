#include "ubx_checksum.h"

void ubx_checksum_update(uint8_t *ck_a, uint8_t *ck_b, uint8_t byte)
{
    *ck_a = (uint8_t)(*ck_a + byte);
    *ck_b = (uint8_t)(*ck_b + *ck_a);
}
