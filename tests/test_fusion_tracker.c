#include <stdio.h>
#include <string.h>

#include "ubx_fusion_tracker.h"
#include "ubx_protocol.h"

/* Tests unitaires sans dépendance hardware : toutes les données
 * NAV-PVT/ESF-STATUS sont construites en dur en mémoire, aucune trame
 * UBX n'est parsée ici. */

static int g_failures = 0;

#define CHECK(cond, msg)                                                              \
    do {                                                                              \
        if (!(cond)) {                                                                \
            fprintf(stderr, "FAIL (%s:%d): %s\n", __FILE__, __LINE__, msg);            \
            g_failures++;                                                             \
        }                                                                             \
    } while (0)

#define DEFAULT_DEGRADED_TIMEOUT_MS 30000

static ubx_nav_pvt_t make_nav_pvt(uint8_t fix_type, bool gnss_fix_ok)
{
    ubx_nav_pvt_t pvt;
    memset(&pvt, 0, sizeof(pvt));
    pvt.fix_type = fix_type;
    pvt.gnss_fix_ok = gnss_fix_ok;
    return pvt;
}

static ubx_esf_status_t make_esf_status(uint8_t fusion_mode)
{
    ubx_esf_status_t esf;
    memset(&esf, 0, sizeof(esf));
    esf.fusion_mode = fusion_mode;
    return esf;
}

/* Fait passer un tracker fraîchement initialisé par
 * UNKNOWN -> NOT_CALIBRATED -> CALIBRATING -> FUSION, en avançant
 * *now_ms de 1000ms à chaque étape. Réutilisé par les scénarios B, C, D. */
static void drive_to_fusion(fusion_tracker_t *ft, uint64_t *now_ms)
{
    ubx_nav_pvt_t no_fix = make_nav_pvt(0, false);
    ubx_nav_pvt_t good_fix = make_nav_pvt(UBX_FIX_3D, true);
    ubx_esf_status_t fusion_mode = make_esf_status(UBX_ESF_FUSION_FUSION);

    fusion_tracker_update(ft, &no_fix, NULL, *now_ms, NULL, NULL);
    *now_ms += 1000;
    fusion_tracker_update(ft, &good_fix, NULL, *now_ms, NULL, NULL);
    *now_ms += 1000;
    fusion_tracker_update(ft, NULL, &fusion_mode, *now_ms, NULL, NULL);
    *now_ms += 1000;
}

static void test_scenario_a_cold_start_to_fusion(void)
{
    fusion_tracker_t ft;
    fusion_tracker_init(&ft, DEFAULT_DEGRADED_TIMEOUT_MS);
    CHECK(ft.current_state == FUSION_STATE_UNKNOWN, "A: état initial attendu UNKNOWN");

    uint64_t now_ms = 0;
    fusion_state_t prev, next;
    bool transitioned;

    ubx_nav_pvt_t no_fix = make_nav_pvt(0, false);
    transitioned = fusion_tracker_update(&ft, &no_fix, NULL, now_ms, &prev, &next);
    CHECK(transitioned, "A: UNKNOWN -> NOT_CALIBRATED aurait dû se produire");
    CHECK(prev == FUSION_STATE_UNKNOWN && next == FUSION_STATE_NOT_CALIBRATED, "A: états avant/après incorrects (étape 1)");
    now_ms += 1000;

    ubx_nav_pvt_t good_fix = make_nav_pvt(UBX_FIX_3D, true);
    transitioned = fusion_tracker_update(&ft, &good_fix, NULL, now_ms, &prev, &next);
    CHECK(transitioned, "A: NOT_CALIBRATED -> CALIBRATING aurait dû se produire");
    CHECK(prev == FUSION_STATE_NOT_CALIBRATED && next == FUSION_STATE_CALIBRATING, "A: états avant/après incorrects (étape 2)");
    now_ms += 1000;

    ubx_esf_status_t fusion_mode = make_esf_status(UBX_ESF_FUSION_FUSION);
    transitioned = fusion_tracker_update(&ft, NULL, &fusion_mode, now_ms, &prev, &next);
    CHECK(transitioned, "A: CALIBRATING -> FUSION aurait dû se produire");
    CHECK(prev == FUSION_STATE_CALIBRATING && next == FUSION_STATE_FUSION, "A: états avant/après incorrects (étape 3)");
    CHECK(ft.current_state == FUSION_STATE_FUSION, "A: état final attendu FUSION");
}

static void test_scenario_b_fusion_to_degraded_on_fix_loss(void)
{
    fusion_tracker_t ft;
    fusion_tracker_init(&ft, DEFAULT_DEGRADED_TIMEOUT_MS);
    uint64_t now_ms = 0;
    drive_to_fusion(&ft, &now_ms);
    CHECK(ft.current_state == FUSION_STATE_FUSION, "B: précondition FUSION non atteinte");

    ubx_nav_pvt_t lost_fix = make_nav_pvt(0, false);
    fusion_state_t prev, next;
    bool transitioned = fusion_tracker_update(&ft, &lost_fix, NULL, now_ms, &prev, &next);

    CHECK(transitioned, "B: FUSION -> DEGRADED aurait dû se produire immédiatement sur perte de fix");
    CHECK(prev == FUSION_STATE_FUSION && next == FUSION_STATE_DEGRADED, "B: états avant/après incorrects");
    CHECK(ft.current_state == FUSION_STATE_DEGRADED, "B: état final attendu DEGRADED");
}

static void test_scenario_c_degraded_to_lost_after_timeout(void)
{
    fusion_tracker_t ft;
    fusion_tracker_init(&ft, DEFAULT_DEGRADED_TIMEOUT_MS);
    uint64_t now_ms = 0;
    drive_to_fusion(&ft, &now_ms);

    ubx_nav_pvt_t lost_fix = make_nav_pvt(0, false);
    fusion_tracker_update(&ft, &lost_fix, NULL, now_ms, NULL, NULL);
    CHECK(ft.current_state == FUSION_STATE_DEGRADED, "C: précondition DEGRADED non atteinte");
    uint64_t degraded_entered_ms = now_ms;

    /* Plusieurs appels successifs avec now_ms croissant, toujours sans
     * fix : on ne doit PAS basculer en LOST avant que le timeout soit
     * dépassé, même après plusieurs cycles. */
    for (int i = 1; i <= 5; i++) {
        now_ms = degraded_entered_ms + (uint64_t)i * 5000; /* 5s, 10s, 15s, 20s, 25s */
        bool transitioned = fusion_tracker_update(&ft, NULL, NULL, now_ms, NULL, NULL);
        CHECK(!transitioned, "C: transition prématurée vers LOST avant la fin du timeout");
        CHECK(ft.current_state == FUSION_STATE_DEGRADED, "C: devrait rester DEGRADED avant le timeout");
    }

    /* Dépasse maintenant le timeout de 30000ms depuis l'entrée en DEGRADED. */
    now_ms = degraded_entered_ms + DEFAULT_DEGRADED_TIMEOUT_MS + 1;
    fusion_state_t prev, next;
    bool transitioned = fusion_tracker_update(&ft, NULL, NULL, now_ms, &prev, &next);

    CHECK(transitioned, "C: DEGRADED -> LOST aurait dû se produire après dépassement du timeout");
    CHECK(prev == FUSION_STATE_DEGRADED && next == FUSION_STATE_LOST, "C: états avant/après incorrects");
    CHECK(ft.current_state == FUSION_STATE_LOST, "C: état final attendu LOST");
}

static void test_scenario_d_degraded_to_fusion_before_timeout(void)
{
    fusion_tracker_t ft;
    fusion_tracker_init(&ft, DEFAULT_DEGRADED_TIMEOUT_MS);
    uint64_t now_ms = 0;
    drive_to_fusion(&ft, &now_ms);

    ubx_nav_pvt_t lost_fix = make_nav_pvt(0, false);
    fusion_tracker_update(&ft, &lost_fix, NULL, now_ms, NULL, NULL);
    CHECK(ft.current_state == FUSION_STATE_DEGRADED, "D: précondition DEGRADED non atteinte");
    uint64_t degraded_entered_ms = now_ms;

    /* Le fix revient bien avant le timeout de 30000ms. */
    now_ms = degraded_entered_ms + 5000;
    ubx_nav_pvt_t recovered_fix = make_nav_pvt(UBX_FIX_3D, true);
    fusion_state_t prev, next;
    bool transitioned = fusion_tracker_update(&ft, &recovered_fix, NULL, now_ms, &prev, &next);

    CHECK(transitioned, "D: DEGRADED -> FUSION aurait dû se produire (fix revenu avant timeout)");
    CHECK(prev == FUSION_STATE_DEGRADED && next == FUSION_STATE_FUSION, "D: états avant/après incorrects (pas de passage par LOST attendu)");
    CHECK(ft.current_state == FUSION_STATE_FUSION, "D: état final attendu FUSION directement, sans passer par LOST");
}

static void test_scenario_e_lost_to_calibrating_on_fix_recovery(void)
{
    fusion_tracker_t ft;
    fusion_tracker_init(&ft, DEFAULT_DEGRADED_TIMEOUT_MS);
    uint64_t now_ms = 0;
    drive_to_fusion(&ft, &now_ms);

    ubx_nav_pvt_t lost_fix = make_nav_pvt(0, false);
    fusion_tracker_update(&ft, &lost_fix, NULL, now_ms, NULL, NULL);
    uint64_t degraded_entered_ms = now_ms;

    now_ms = degraded_entered_ms + DEFAULT_DEGRADED_TIMEOUT_MS + 1;
    fusion_tracker_update(&ft, NULL, NULL, now_ms, NULL, NULL);
    CHECK(ft.current_state == FUSION_STATE_LOST, "E: précondition LOST non atteinte");

    now_ms += 1000;
    ubx_nav_pvt_t recovered_fix = make_nav_pvt(UBX_FIX_3D, true);
    fusion_state_t prev, next;
    bool transitioned = fusion_tracker_update(&ft, &recovered_fix, NULL, now_ms, &prev, &next);

    CHECK(transitioned, "E: LOST -> CALIBRATING aurait dû se produire au retour du fix");
    CHECK(prev == FUSION_STATE_LOST && next == FUSION_STATE_CALIBRATING, "E: états avant/après incorrects");
    CHECK(ft.current_state == FUSION_STATE_CALIBRATING, "E: état final attendu CALIBRATING (pas FUSION directement, recalibration nécessaire)");
}

int main(void)
{
    test_scenario_a_cold_start_to_fusion();
    test_scenario_b_fusion_to_degraded_on_fix_loss();
    test_scenario_c_degraded_to_lost_after_timeout();
    test_scenario_d_degraded_to_fusion_before_timeout();
    test_scenario_e_lost_to_calibrating_on_fix_recovery();

    if (g_failures == 0) {
        printf("Tous les tests sont passés.\n");
        return 0;
    }
    printf("%d test(s) en échec.\n", g_failures);
    return g_failures;
}
