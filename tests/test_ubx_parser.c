#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ubx_cfg.h"
#include "ubx_parser.h"
#include "ubx_protocol.h"

/* Tests unitaires sans dépendance externe : chaque test incrémente
 * g_failures en cas d'échec, le programme retourne ce compteur comme
 * code de sortie (0 = tous les tests passent). */

static int g_failures = 0;

#define CHECK(cond, msg)                                                                         \
    do {                                                                                          \
        if (!(cond)) {                                                                            \
            fprintf(stderr, "FAIL (%s:%d): %s\n", __FILE__, __LINE__, msg);                       \
            g_failures++;                                                                         \
        }                                                                                          \
    } while (0)

static unsigned char *read_fixture(const char *path, long *out_len)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "impossible d'ouvrir la fixture %s (exécuter les tests depuis la racine du projet)\n", path);
        exit(1);
    }
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    unsigned char *buf = malloc((size_t)len);
    if (fread(buf, 1, (size_t)len, f) != (size_t)len) {
        fprintf(stderr, "lecture incomplète de la fixture %s\n", path);
        exit(1);
    }
    fclose(f);
    *out_len = len;
    return buf;
}

/* Fait avancer un parser sur tout un buffer, retourne la dernière trame
 * complète obtenue (le buffer ne doit contenir qu'une seule trame utile). */
static int feed_all(ubx_parser_t *parser, const unsigned char *data, long len, ubx_frame_t *out_frame)
{
    int last_result = 0;
    for (long i = 0; i < len; i++) {
        int r = ubx_parser_feed(parser, data[i], out_frame);
        if (r != 0) {
            last_result = r;
        }
    }
    return last_result;
}

static void test_generic_framing_round_trip(void)
{
    /* Construit une trame CFG-VALSET avec notre propre builder, la fait
     * passer dans le parser, et vérifie que la structure de trame
     * générique (classe/ID/longueur/checksum) est correctement extraite. */
    uint8_t buf[64];
    size_t len = 0;
    ubx_cfg_kv_t kvs[2] = {
        { UBX_CFG_KEY_MSGOUT_NAV_PVT_USB, 1 },
        { UBX_CFG_KEY_MSGOUT_ESF_STATUS_USB, 1 },
    };

    CHECK(ubx_build_cfg_valset(buf, sizeof(buf), UBX_CFG_LAYER_RAM, kvs, 2, &len), "ubx_build_cfg_valset a échoué");
    CHECK(len == 22, "longueur de trame CFG-VALSET inattendue");

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_frame_t frame;
    int result = feed_all(&parser, buf, (long)len, &frame);

    CHECK(result == 1, "la trame CFG-VALSET construite n'a pas été reconnue comme complète");
    CHECK(frame.checksum_valid, "checksum invalide sur une trame que nous venons de construire nous-mêmes");
    CHECK(frame.msg_class == UBX_CFG_CLASS, "classe incorrecte");
    CHECK(frame.msg_id == UBX_CFG_VALSET_ID, "ID incorrect");
    CHECK(frame.length == 14, "longueur de payload incorrecte");
}

static void test_corrupted_checksum_detected(void)
{
    uint8_t buf[64];
    size_t len = 0;
    ubx_cfg_kv_t kvs[1] = { { UBX_CFG_KEY_MSGOUT_NAV_PVT_USB, 1 } };

    CHECK(ubx_build_cfg_valset(buf, sizeof(buf), UBX_CFG_LAYER_RAM, kvs, 1, &len), "ubx_build_cfg_valset a échoué");

    /* On corrompt un octet du payload sans toucher au checksum : la
     * vérification doit détecter l'incohérence. */
    buf[len - 5] ^= 0xFF;

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_frame_t frame;
    int result = feed_all(&parser, buf, (long)len, &frame);

    CHECK(result == 1, "la trame corrompue aurait dû être reconnue comme complète (structure intacte)");
    CHECK(!frame.checksum_valid, "le checksum aurait dû être détecté comme invalide après corruption du payload");
}

static void test_oversized_length_drops_and_resyncs(void)
{
    /* Sync + classe + ID + longueur annoncée largement supérieure à
     * UBX_MAX_PAYLOAD_LEN : la trame doit être abandonnée (-1) et le
     * parser doit pouvoir resynchroniser sur la trame valide suivante. */
    uint8_t bogus[] = { UBX_SYNC_CHAR_1, UBX_SYNC_CHAR_2, 0x01, 0x07, 0xFF, 0xFF };

    uint8_t good[64];
    size_t good_len = 0;
    ubx_cfg_kv_t kvs[1] = { { UBX_CFG_KEY_MSGOUT_NAV_PVT_USB, 1 } };
    CHECK(ubx_build_cfg_valset(good, sizeof(good), UBX_CFG_LAYER_RAM, kvs, 1, &good_len), "ubx_build_cfg_valset a échoué");

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_frame_t frame;

    int saw_drop = 0;
    for (size_t i = 0; i < sizeof(bogus); i++) {
        int r = ubx_parser_feed(&parser, bogus[i], &frame);
        if (r == -1) {
            saw_drop = 1;
        }
    }
    CHECK(saw_drop, "la trame à longueur excessive aurait dû être abandonnée (retour -1)");

    int result = feed_all(&parser, good, (long)good_len, &frame);
    CHECK(result == 1, "le parser n'a pas resynchronisé correctement après une trame abandonnée");
    CHECK(frame.checksum_valid, "la trame valide après resynchronisation devrait avoir un checksum correct");
}

static void test_nav_pvt_real_fixture(void)
{
    /* Trame réelle capturée sur hardware (C100-F9K, indoor, sans
     * antenne) via poll UBX-NAV-PVT (classe 0x01, ID 0x07, payload vide). */
    long len;
    unsigned char *data = read_fixture("tests/fixtures/nav_pvt_real.bin", &len);

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_frame_t frame;
    int result = feed_all(&parser, data, len, &frame);
    free(data);

    CHECK(result == 1, "fixture NAV-PVT : trame non reconnue comme complète");
    CHECK(frame.checksum_valid, "fixture NAV-PVT : checksum invalide");
    CHECK(frame.msg_class == UBX_NAV_PVT_CLASS && frame.msg_id == UBX_NAV_PVT_ID, "fixture NAV-PVT : classe/ID incorrects");
    CHECK(frame.length == UBX_NAV_PVT_PAYLOAD_LEN, "fixture NAV-PVT : longueur de payload incorrecte");

    ubx_nav_pvt_t pvt;
    CHECK(ubx_parse_nav_pvt(frame.payload, frame.length, &pvt), "ubx_parse_nav_pvt a échoué sur la fixture réelle");

    /* Capture réalisée indoor sans antenne : pas de fix GNSS, valeurs
     * attendues en conséquence (vérifiées via decode manuel le 2026-06-23). */
    CHECK(pvt.fix_type == UBX_FIX_NO_FIX, "fixture NAV-PVT : fixType attendu = NoFix");
    CHECK(!pvt.gnss_fix_ok, "fixture NAV-PVT : gnssFixOK attendu = false");
    CHECK(pvt.num_sv == 0, "fixture NAV-PVT : numSV attendu = 0");
    CHECK(pvt.p_dop == 9999, "fixture NAV-PVT : pDOP attendu = 99.99 (valeur sentinelle sans fix)");
}

static void test_esf_status_real_fixture(void)
{
    /* Trame réelle capturée sur hardware via poll UBX-ESF-STATUS (classe
     * 0x10, ID 0x10, payload vide). */
    long len;
    unsigned char *data = read_fixture("tests/fixtures/esf_status_real.bin", &len);

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_frame_t frame;
    int result = feed_all(&parser, data, len, &frame);
    free(data);

    CHECK(result == 1, "fixture ESF-STATUS : trame non reconnue comme complète");
    CHECK(frame.checksum_valid, "fixture ESF-STATUS : checksum invalide");
    CHECK(frame.msg_class == UBX_ESF_STATUS_CLASS && frame.msg_id == UBX_ESF_STATUS_ID, "fixture ESF-STATUS : classe/ID incorrects");

    ubx_esf_status_t esf;
    CHECK(ubx_parse_esf_status(frame.payload, frame.length, &esf), "ubx_parse_esf_status a échoué sur la fixture réelle");

    CHECK(esf.version == 2, "fixture ESF-STATUS : version attendue = 2");
    CHECK(esf.fusion_mode == UBX_ESF_FUSION_INIT, "fixture ESF-STATUS : fusionMode attendu = INIT (pas de fix GNSS)");
    CHECK(esf.num_sens == 7, "fixture ESF-STATUS : numSens attendu = 7");

    /* Capteurs prêts mais non utilisés en solution (fusion en INIT, pas
     * de fix GNSS pour les combiner) — valeurs vérifiées via decode
     * manuel sur la même capture. */
    static const uint8_t expected_types[7] = { 5, 10, 13, 14, 16, 17, 18 };
    for (uint8_t i = 0; i < esf.num_sens; i++) {
        CHECK(esf.sensors[i].type == expected_types[i], "fixture ESF-STATUS : type de capteur inattendu");
        CHECK(!esf.sensors[i].used, "fixture ESF-STATUS : capteur ne devrait pas être 'used' (fusion INIT)");
        CHECK(esf.sensors[i].ready, "fixture ESF-STATUS : capteur attendu 'ready'");
        CHECK(esf.sensors[i].calib_status == UBX_ESF_CALIB_NOT_CALIBRATED, "fixture ESF-STATUS : calibStatus attendu = non calibré");
        CHECK(!esf.sensors[i].bad_meas && !esf.sensors[i].bad_ttag && !esf.sensors[i].missing_meas && !esf.sensors[i].noisy_meas,
              "fixture ESF-STATUS : aucun fault attendu");
    }
}

static void test_esf_status_rejects_wrong_length(void)
{
    /* Tronque la fixture réelle pour simuler une trame ESF-STATUS dont
     * la longueur ne correspond plus à 16 + numSens*4 : le décodage doit
     * être refusé plutôt que de lire hors limites. */
    long len;
    unsigned char *data = read_fixture("tests/fixtures/esf_status_real.bin", &len);

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_frame_t frame;
    feed_all(&parser, data, len, &frame);
    free(data);

    ubx_esf_status_t esf;
    CHECK(!ubx_parse_esf_status(frame.payload, (size_t)(frame.length - 4), &esf),
          "ubx_parse_esf_status aurait dû refuser une longueur tronquée (incohérente avec numSens)");
}

int main(void)
{
    test_generic_framing_round_trip();
    test_corrupted_checksum_detected();
    test_oversized_length_drops_and_resyncs();
    test_nav_pvt_real_fixture();
    test_esf_status_real_fixture();
    test_esf_status_rejects_wrong_length();

    if (g_failures == 0) {
        printf("Tous les tests sont passés.\n");
        return 0;
    }
    printf("%d test(s) en échec.\n", g_failures);
    return g_failures;
}
