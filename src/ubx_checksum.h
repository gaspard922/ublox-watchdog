#ifndef UBX_CHECKSUM_H
#define UBX_CHECKSUM_H

#include <stdint.h>

/* Algorithme Fletcher-8 du protocole UBX, calculé sur classe, ID,
 * longueur et payload (jamais sur les sync bytes). Partagé entre le
 * parser (vérification) et le builder de trames de configuration
 * (calcul à l'envoi). */

/* Met à jour l'accumulateur de checksum avec un octet supplémentaire.
 * Initialiser ck_a et ck_b à 0 avant le premier appel d'une trame. */
void ubx_checksum_update(uint8_t *ck_a, uint8_t *ck_b, uint8_t byte);

#endif /* UBX_CHECKSUM_H */
