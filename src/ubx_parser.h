#ifndef UBX_PARSER_H
#define UBX_PARSER_H

#include <stdint.h>

/* State machine de parsing de trame UBX générique.
 *
 * Scope volontairement limité à la structure de trame (sync bytes,
 * classe, ID, longueur, payload brut, checksum) : aucune interprétation
 * des champs spécifiques à un message (NAV-PVT, ESF-STATUS, etc.) n'est
 * faite ici. Le dispatch et le décodage des messages spécifiques viendront
 * dans un module séparé une fois les offsets exacts vérifiés dans la doc
 * u-blox. */

#define UBX_SYNC_CHAR_1 0xB5
#define UBX_SYNC_CHAR_2 0x62

/* Taille max de payload supportée (buffer fixe, pas de malloc).
 * Largement suffisante pour NAV-PVT (92 octets) et ESF-STATUS/ESF-MEAS. */
#define UBX_MAX_PAYLOAD_LEN 1024

enum ubx_parser_state {
    UBX_STATE_WAIT_SYNC1,
    UBX_STATE_WAIT_SYNC2,
    UBX_STATE_WAIT_CLASS,
    UBX_STATE_WAIT_ID,
    UBX_STATE_WAIT_LEN1,
    UBX_STATE_WAIT_LEN2,
    UBX_STATE_WAIT_PAYLOAD,
    UBX_STATE_WAIT_CK_A,
    UBX_STATE_WAIT_CK_B
};

/* Trame UBX brute (sans interprétation du payload). */
typedef struct {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
    uint8_t payload[UBX_MAX_PAYLOAD_LEN];
    uint8_t ck_a;
    uint8_t ck_b;
    int checksum_valid; /* 1 si ck_a/ck_b calculés correspondent à ceux reçus */
} ubx_frame_t;

/* Contexte du parser, à initialiser une seule fois par flux d'entrée
 * (un par port série ouvert). Pas d'allocation dynamique. */
typedef struct {
    enum ubx_parser_state state;
    ubx_frame_t frame;
    uint16_t payload_index;
    uint8_t ck_a_calc;
    uint8_t ck_b_calc;
} ubx_parser_t;

/* Initialise (ou réinitialise) le parser à l'état d'attente de synchro. */
void ubx_parser_init(ubx_parser_t *parser);

/* Fait avancer la state machine d'un octet.
 *
 * Retourne :
 *   1  si une trame complète a été reçue ; *out_frame est rempli
 *      (vérifier out_frame->checksum_valid avant d'utiliser le payload)
 *   0  si l'octet a été consommé mais qu'aucune trame n'est encore prête
 *  -1  si la trame en cours a été abandonnée (longueur annoncée supérieure
 *      à UBX_MAX_PAYLOAD_LEN) ; le parser s'est réinitialisé en attente
 *      de synchro, l'octet courant a été consommé
 */
int ubx_parser_feed(ubx_parser_t *parser, uint8_t byte, ubx_frame_t *out_frame);

#endif /* UBX_PARSER_H */
