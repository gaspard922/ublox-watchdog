#include "ubx_fusion_tracker.h"

#include <stdio.h>

const char *fusion_state_name(fusion_state_t state)
{
    switch (state) {
    case FUSION_STATE_UNKNOWN: return "UNKNOWN";
    case FUSION_STATE_NOT_CALIBRATED: return "NOT_CALIBRATED";
    case FUSION_STATE_CALIBRATING: return "CALIBRATING";
    case FUSION_STATE_FUSION: return "FUSION";
    case FUSION_STATE_DEGRADED: return "DEGRADED";
    case FUSION_STATE_LOST: return "LOST";
    default: return "INVALID";
    }
}

void fusion_tracker_init(fusion_tracker_t *ft, uint64_t degraded_timeout_ms)
{
    ft->current_state = FUSION_STATE_UNKNOWN;
    ft->previous_state = FUSION_STATE_UNKNOWN;
    ft->state_entered_at_ms = 0;
    ft->last_gnss_fix_ms = 0;
    ft->degraded_timeout_ms = degraded_timeout_ms;
    ft->has_gnss_fix = false;
}

static void do_transition(fusion_tracker_t *ft, fusion_state_t new_state, uint64_t now_ms, const char *reason)
{
    fprintf(stderr, "[fusion_tracker] t=%lums %s -> %s (%s)\n",
            (unsigned long)now_ms, fusion_state_name(ft->current_state), fusion_state_name(new_state), reason);

    ft->previous_state = ft->current_state;
    ft->current_state = new_state;
    ft->state_entered_at_ms = now_ms;
}

bool fusion_tracker_update(fusion_tracker_t *ft, const ubx_nav_pvt_t *nav_pvt,
                           const ubx_esf_status_t *esf_status, uint64_t now_ms,
                           fusion_state_t *out_previous_state, fusion_state_t *out_new_state,
                           const char **out_reason)
{
    fusion_state_t state_before = ft->current_state;
    const char *reason = NULL;

    if (nav_pvt != NULL) {
        bool fix_now = (nav_pvt->fix_type >= UBX_FIX_3D) && nav_pvt->gnss_fix_ok;
        ft->has_gnss_fix = fix_now;
        if (fix_now) {
            ft->last_gnss_fix_ms = now_ms;
        }
    }

    switch (ft->current_state) {

    case FUSION_STATE_UNKNOWN:
        if (nav_pvt != NULL) {
            if (ft->has_gnss_fix) {
                reason = "premier fix GNSS valide à l'initialisation";
                do_transition(ft, FUSION_STATE_CALIBRATING, now_ms, reason);
            } else {
                reason = "première donnée NAV-PVT reçue, pas de fix";
                do_transition(ft, FUSION_STATE_NOT_CALIBRATED, now_ms, reason);
            }
        }
        break;

    case FUSION_STATE_NOT_CALIBRATED:
        if (nav_pvt != NULL && ft->has_gnss_fix) {
            reason = "fix GNSS valide détecté";
            do_transition(ft, FUSION_STATE_CALIBRATING, now_ms, reason);
        }
        break;

    case FUSION_STATE_CALIBRATING:
        if (esf_status != NULL && ft->has_gnss_fix && esf_status->fusion_mode == UBX_ESF_FUSION_FUSION) {
            reason = "fusionMode=FUSION atteint avec fix GNSS valide";
            do_transition(ft, FUSION_STATE_FUSION, now_ms, reason);
        }
        break;

    case FUSION_STATE_FUSION:
        if (nav_pvt != NULL && !ft->has_gnss_fix) {
            reason = "fix GNSS perdu (fixType<3 ou gnssFixOK=0)";
            do_transition(ft, FUSION_STATE_DEGRADED, now_ms, reason);
        }
        break;

    case FUSION_STATE_DEGRADED:
        if (nav_pvt != NULL && ft->has_gnss_fix) {
            reason = "fix GNSS redevenu valide avant timeout";
            do_transition(ft, FUSION_STATE_FUSION, now_ms, reason);
        } else if (now_ms - ft->state_entered_at_ms > ft->degraded_timeout_ms) {
            reason = "timeout dégradation dépassé";
            do_transition(ft, FUSION_STATE_LOST, now_ms, reason);
        }
        break;

    case FUSION_STATE_LOST:
        if (nav_pvt != NULL && ft->has_gnss_fix) {
            reason = "fix GNSS redevenu valide, recalibration nécessaire";
            do_transition(ft, FUSION_STATE_CALIBRATING, now_ms, reason);
        }
        break;
    }

    bool transition_occurred = (ft->current_state != state_before);
    if (out_previous_state != NULL) {
        *out_previous_state = state_before;
    }
    if (out_new_state != NULL) {
        *out_new_state = ft->current_state;
    }
    if (out_reason != NULL) {
        *out_reason = reason;
    }
    return transition_occurred;
}
