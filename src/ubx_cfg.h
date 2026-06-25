#ifndef UBX_CFG_H
#define UBX_CFG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Construction de trames UBX-CFG-VALSET (classe 0x06, ID 0x8A), utilisées
 * pour configurer le récepteur (ex: activer la sortie périodique de
 * messages UBX spécifiques sur le port USB). */

#define UBX_CFG_CLASS 0x06
#define UBX_CFG_VALSET_ID 0x8A

/* Bitfield "layers" : peuvent être combinés par OR. */
#define UBX_CFG_LAYER_RAM 0x01
#define UBX_CFG_LAYER_BBR 0x02
#define UBX_CFG_LAYER_FLASH 0x04

/* "transaction" : on utilise toujours 0 (transactionless) dans ce projet. */
#define UBX_CFG_TRANSACTION_NONE 0x00

/* Key IDs confirmées (config DB u-blox, type U1 = taux d'envoi en
 * cycles de navigation, 1 = chaque cycle). */
#define UBX_CFG_KEY_MSGOUT_NAV_PVT_USB 0x20910009u
#define UBX_CFG_KEY_MSGOUT_ESF_STATUS_USB 0x20910108u

/* Couple clé/valeur pour une clé de type U1 (seul type utilisé ici). */
typedef struct {
    uint32_t key_id;
    uint8_t value;
} ubx_cfg_kv_t;

/* Construit une trame UBX-CFG-VALSET complète (sync bytes, classe, ID,
 * longueur, payload, checksum Fletcher-8) dans out_buf.
 *
 * layers : OR de UBX_CFG_LAYER_*.
 * transaction est fixé à UBX_CFG_TRANSACTION_NONE.
 *
 * Retourne false si out_buf_size est insuffisant pour contenir la trame
 * complète (rien n'est écrit dans ce cas). */
bool ubx_build_cfg_valset(uint8_t *out_buf, size_t out_buf_size, uint8_t layers,
                          const ubx_cfg_kv_t *kvs, size_t num_kvs, size_t *out_len);

#endif /* UBX_CFG_H */
