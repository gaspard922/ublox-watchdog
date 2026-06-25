#ifndef UBX_POLL_H
#define UBX_POLL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Construction d'une requête de poll UBX générique : trame à payload
 * vide qui déclenche une réponse unique du récepteur avec l'état actuel
 * du message demandé (classe/ID identiques à la trame normalement
 * poussée). Utilisé pour NAV-PVT et ESF-STATUS tant que le push
 * périodique via CFG-MSGOUT ne déclenche pas de sortie spontanée sur ce
 * hardware (cf investigation session précédente). */

/* Construit une trame de poll (sync bytes, classe, ID, longueur=0,
 * checksum) dans out_buf. Retourne false si out_buf_size < 8. */
bool ubx_build_poll_request(uint8_t *out_buf, size_t out_buf_size, uint8_t msg_class, uint8_t msg_id, size_t *out_len);

#endif /* UBX_POLL_H */
