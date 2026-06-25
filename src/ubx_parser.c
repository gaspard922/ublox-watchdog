#include "ubx_parser.h"

#include "ubx_checksum.h"

void ubx_parser_init(ubx_parser_t *parser)
{
    parser->state = UBX_STATE_WAIT_SYNC1;
    parser->payload_index = 0;
    parser->ck_a_calc = 0;
    parser->ck_b_calc = 0;
}

int ubx_parser_feed(ubx_parser_t *parser, uint8_t byte, ubx_frame_t *out_frame)
{
    switch (parser->state) {

    case UBX_STATE_WAIT_SYNC1:
        if (byte == UBX_SYNC_CHAR_1) {
            parser->state = UBX_STATE_WAIT_SYNC2;
        }
        return 0;

    case UBX_STATE_WAIT_SYNC2:
        if (byte == UBX_SYNC_CHAR_2) {
            parser->ck_a_calc = 0;
            parser->ck_b_calc = 0;
            parser->state = UBX_STATE_WAIT_CLASS;
        } else if (byte != UBX_SYNC_CHAR_1) {
            parser->state = UBX_STATE_WAIT_SYNC1;
        }
        /* si byte == SYNC_CHAR_1, on reste en WAIT_SYNC2 (resynchronisation) */
        return 0;

    case UBX_STATE_WAIT_CLASS:
        parser->frame.msg_class = byte;
        ubx_checksum_update(&parser->ck_a_calc, &parser->ck_b_calc, byte);
        parser->state = UBX_STATE_WAIT_ID;
        return 0;

    case UBX_STATE_WAIT_ID:
        parser->frame.msg_id = byte;
        ubx_checksum_update(&parser->ck_a_calc, &parser->ck_b_calc, byte);
        parser->state = UBX_STATE_WAIT_LEN1;
        return 0;

    case UBX_STATE_WAIT_LEN1:
        parser->frame.length = byte;
        ubx_checksum_update(&parser->ck_a_calc, &parser->ck_b_calc, byte);
        parser->state = UBX_STATE_WAIT_LEN2;
        return 0;

    case UBX_STATE_WAIT_LEN2:
        parser->frame.length = (uint16_t)(parser->frame.length | ((uint16_t)byte << 8));
        ubx_checksum_update(&parser->ck_a_calc, &parser->ck_b_calc, byte);
        parser->payload_index = 0;
        if (parser->frame.length > UBX_MAX_PAYLOAD_LEN) {
            /* Trame trop grande pour le buffer fixe : on l'abandonne et on
             * se resynchronise plutôt que de débordder le buffer. */
            ubx_parser_init(parser);
            return -1;
        }
        parser->state = (parser->frame.length == 0) ? UBX_STATE_WAIT_CK_A : UBX_STATE_WAIT_PAYLOAD;
        return 0;

    case UBX_STATE_WAIT_PAYLOAD:
        parser->frame.payload[parser->payload_index++] = byte;
        ubx_checksum_update(&parser->ck_a_calc, &parser->ck_b_calc, byte);
        if (parser->payload_index == parser->frame.length) {
            parser->state = UBX_STATE_WAIT_CK_A;
        }
        return 0;

    case UBX_STATE_WAIT_CK_A:
        parser->frame.ck_a = byte;
        parser->state = UBX_STATE_WAIT_CK_B;
        return 0;

    case UBX_STATE_WAIT_CK_B:
        parser->frame.ck_b = byte;
        parser->frame.checksum_valid =
            (parser->frame.ck_a == parser->ck_a_calc) && (parser->frame.ck_b == parser->ck_b_calc);
        *out_frame = parser->frame;
        ubx_parser_init(parser);
        return 1;

    default:
        ubx_parser_init(parser);
        return -1;
    }
}
