#ifndef UBX_POLL_H
#define UBX_POLL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Construit une requête de poll UBX : une trame à payload vide qui fait
 * répondre le récepteur une fois avec l'état courant du message demandé
 * (même classe/ID que la trame normalement poussée). C'est ce qu'on
 * utilise pour NAV-PVT et ESF-STATUS, le push périodique via CFG-MSGOUT
 * ne produisant pas de sortie spontanée sur ce hardware. */

/* Construit une trame de poll (sync bytes, classe, ID, longueur=0,
 * checksum) dans out_buf. Retourne false si out_buf_size < 8. */
bool ubx_build_poll_request(uint8_t *out_buf, size_t out_buf_size, uint8_t msg_class, uint8_t msg_id, size_t *out_len);

#endif /* UBX_POLL_H */
