#ifndef UBX_FUSION_TRACKER_H
#define UBX_FUSION_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

#include "ubx_protocol.h"

/* State machine de suivi de la fusion GNSS/INS. Ne décode rien : prend
 * en entrée les structures déjà décodées par ubx_protocol.c
 * (UBX-NAV-PVT et UBX-ESF-STATUS) et déduit l'état de fusion avec une
 * logique temporelle propre (timeout de dégradation configurable). */

typedef enum {
    FUSION_STATE_UNKNOWN,
    FUSION_STATE_NOT_CALIBRATED,
    FUSION_STATE_CALIBRATING,
    FUSION_STATE_FUSION,
    FUSION_STATE_DEGRADED,
    FUSION_STATE_LOST
} fusion_state_t;

/* Nom lisible d'un état, pour le logging et la future sortie JSON
 * (output_stream). */
const char *fusion_state_name(fusion_state_t state);

typedef struct {
    fusion_state_t current_state;
    fusion_state_t previous_state;
    uint64_t state_entered_at_ms;
    uint64_t last_gnss_fix_ms;     /* dernière fois qu'un fix GNSS valide a été observé (0 si jamais) */
    uint64_t degraded_timeout_ms;  /* configurable à l'init, voir fusion_tracker_init() */

    /* Cache du dernier statut de fix connu (fixType>=3 ET gnssFixOK).
     * Nécessaire car fusion_tracker_update() peut être appelé avec
     * nav_pvt == NULL (ex: seul ESF-STATUS disponible ce cycle) : sans
     * ce cache, l'information de fix serait perdue entre deux appels. */
    bool has_gnss_fix;
} fusion_tracker_t;

/* Initialise le tracker à l'état UNKNOWN. degraded_timeout_ms : durée
 * max passée en DEGRADED avant de basculer en LOST (ex: 30000 pour
 * 30s, conforme au défaut documenté dans CLAUDE.md). */
void fusion_tracker_init(fusion_tracker_t *ft, uint64_t degraded_timeout_ms);

/* Fait avancer la state machine avec les dernières données disponibles.
 *
 * nav_pvt et esf_status peuvent être NULL indépendamment l'un de
 * l'autre si le message correspondant n'a pas encore été reçu à cet
 * appel (ils arrivent en trames UBX séparées) — la fonction ne plante
 * jamais dans ce cas, elle évalue seulement les transitions pour
 * lesquelles l'information nécessaire est disponible.
 *
 * now_ms : horloge monotone (ex: clock_gettime(CLOCK_MONOTONIC)
 * convertie en ms par l'appelant).
 *
 * Retourne true si une transition d'état a eu lieu pendant cet appel.
 * Si non NULL, *out_previous_state, *out_new_state et *out_reason sont
 * alors renseignés (état avant/après la transition, et raison sous
 * forme de chaîne statique en lecture seule — ne pas la libérer).
 *
 * On retourne un booléen accompagné de paramètres de sortie plutôt que
 * de passer par un callback enregistré à part : un test peut lire
 * directement la valeur de retour et les états avant/après, sans avoir
 * à bricoler un contexte de callback (le C n'a pas de fermetures). Le
 * logging vers stderr reste interne à la fonction quel que soit ce
 * choix. */
bool fusion_tracker_update(fusion_tracker_t *ft, const ubx_nav_pvt_t *nav_pvt,
                           const ubx_esf_status_t *esf_status, uint64_t now_ms,
                           fusion_state_t *out_previous_state, fusion_state_t *out_new_state,
                           const char **out_reason);

#endif /* UBX_FUSION_TRACKER_H */
