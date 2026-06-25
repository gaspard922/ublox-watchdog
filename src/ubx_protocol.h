#ifndef UBX_PROTOCOL_H
#define UBX_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Structures et fonctions de décodage des messages UBX utilisés par ce
 * projet : UBX-NAV-PVT et UBX-ESF-STATUS uniquement.
 *
 * Offsets et tailles de champs tels que fournis (à vérifier contre
 * l'interface description u-blox ZED-F9K en cas de doute). Extraction
 * manuelle little-endian champ par champ, pas de memcpy sur struct
 * packée (portabilité, pas de dépendance à l'alignement mémoire). */

/* ===================== UBX-NAV-PVT (classe 0x01, id 0x07) ===================== */

#define UBX_NAV_PVT_CLASS 0x01
#define UBX_NAV_PVT_ID 0x07
#define UBX_NAV_PVT_PAYLOAD_LEN 92

/* fixType (offset 20) */
enum ubx_nav_pvt_fix_type {
    UBX_FIX_NO_FIX = 0,
    UBX_FIX_DEAD_RECKONING_ONLY = 1,
    UBX_FIX_2D = 2,
    UBX_FIX_3D = 3,
    UBX_FIX_GNSS_PLUS_DEAD_RECKONING = 4,
    UBX_FIX_TIME_ONLY = 5
};

/* carrSoln (flags bits 6-7) */
enum ubx_nav_pvt_carr_soln {
    UBX_CARR_SOLN_NONE = 0,
    UBX_CARR_SOLN_FLOAT = 1,
    UBX_CARR_SOLN_FIXED = 2
};

typedef struct {
    uint32_t i_tow;     /* ms */
    uint16_t year;
    uint8_t month;      /* 1-12 */
    uint8_t day;         /* 1-31 */
    uint8_t hour;        /* 0-23 */
    uint8_t min;         /* 0-59 */
    uint8_t sec;         /* 0-60 */

    /* bitfield "valid" (offset 11) */
    bool valid_date;
    bool valid_time;
    bool fully_resolved;
    bool valid_mag;

    uint32_t t_acc;      /* ns */
    int32_t nano;        /* ns, signé */

    uint8_t fix_type;    /* enum ubx_nav_pvt_fix_type */

    /* bitfield "flags" (offset 21) */
    bool gnss_fix_ok;
    bool diff_soln;
    uint8_t psm_state;   /* bits 2-4 */
    bool head_veh_valid;
    uint8_t carr_soln;   /* bits 6-7, enum ubx_nav_pvt_carr_soln */

    /* bitfield "flags2" (offset 22) */
    bool confirmed_avai;
    bool confirmed_date;
    bool confirmed_time;

    uint8_t num_sv;

    int32_t lon;         /* 1e-7 deg */
    int32_t lat;         /* 1e-7 deg */
    int32_t height;      /* mm, ellipsoïde WGS84 */
    int32_t h_msl;        /* mm, niveau moyen mer */
    uint32_t h_acc;       /* mm */
    uint32_t v_acc;       /* mm */

    int32_t vel_n;        /* mm/s */
    int32_t vel_e;        /* mm/s */
    int32_t vel_d;        /* mm/s */
    int32_t g_speed;      /* mm/s, vitesse sol 2D */
    int32_t head_mot;     /* 1e-5 deg */
    uint32_t s_acc;        /* mm/s */
    uint32_t head_acc;     /* 1e-5 deg */

    uint16_t p_dop;        /* x0.01 */

    /* bitfield "flags3" (offset 78) */
    bool invalid_llh;
    uint8_t last_correction_age; /* bits 4-7 */

    int32_t head_veh;      /* 1e-5 deg, valide seulement si head_veh_valid */
    int16_t mag_dec;       /* 1e-2 deg */
    uint16_t mag_acc;      /* 1e-2 deg */
} ubx_nav_pvt_t;

/* Décode un payload UBX-NAV-PVT brut.
 * Retourne false si len != UBX_NAV_PVT_PAYLOAD_LEN (92). */
bool ubx_parse_nav_pvt(const uint8_t *payload, size_t len, ubx_nav_pvt_t *out);

/* ===================== UBX-ESF-STATUS (classe 0x10, id 0x10) ===================== */

#define UBX_ESF_STATUS_CLASS 0x10
#define UBX_ESF_STATUS_ID 0x10
#define UBX_ESF_STATUS_FIXED_LEN 16 /* partie fixe avant le groupe répétitif */

/* Nombre max de capteurs supportés (buffer fixe, pas de malloc).
 * 32 est une valeur raisonnable largement supérieure au nombre de
 * capteurs IMU/wheel-tick attendus sur un C100-F9K. */
#define UBX_ESF_MAX_SENS 32

/* fusionMode (offset 12) */
enum ubx_esf_fusion_mode {
    UBX_ESF_FUSION_INIT = 0,
    UBX_ESF_FUSION_FUSION = 1,
    UBX_ESF_FUSION_SUSPENDED = 2,
    UBX_ESF_FUSION_DISABLED = 3
};

/* calibStatus (sensStatus2 bits 1-0) */
enum ubx_esf_calib_status {
    UBX_ESF_CALIB_NOT_CALIBRATED = 0,
    UBX_ESF_CALIB_CALIBRATING = 1,
    UBX_ESF_CALIB_CALIBRATED_2 = 2, /* doc : 10 et 11 signifient tous deux "calibré" */
    UBX_ESF_CALIB_CALIBRATED_3 = 3
};

/* timeStatus (sensStatus2 bits 3-2) */
enum ubx_esf_time_status {
    UBX_ESF_TIME_NO_DATA = 0,
    UBX_ESF_TIME_FIRST_BYTE = 1,
    UBX_ESF_TIME_EVENT_INPUT = 2,
    UBX_ESF_TIME_TAG_PROVIDED = 3
};

typedef struct {
    uint8_t type;          /* sensStatus1 bits 5-0, identifiant type capteur */
    bool used;              /* sensStatus1 bit 6 */
    bool ready;              /* sensStatus1 bit 7 */

    uint8_t calib_status;    /* sensStatus2 bits 1-0, enum ubx_esf_calib_status */
    uint8_t time_status;     /* sensStatus2 bits 3-2, enum ubx_esf_time_status */

    uint8_t freq;             /* Hz */

    /* faults */
    bool bad_meas;
    bool bad_ttag;
    bool missing_meas;
    bool noisy_meas;
} ubx_esf_sensor_t;

typedef struct {
    uint32_t i_tow;          /* ms */
    uint8_t version;
    uint8_t fusion_mode;     /* enum ubx_esf_fusion_mode */
    uint8_t num_sens;         /* nombre de capteurs réellement présents dans sensors[] */
    ubx_esf_sensor_t sensors[UBX_ESF_MAX_SENS];
} ubx_esf_status_t;

/* Décode un payload UBX-ESF-STATUS brut.
 * La longueur attendue est variable (16 + numSens*4) : numSens est lu à
 * l'offset 15 puis comparé à len avant de parser le groupe répétitif.
 * Retourne false si len ne correspond pas à la longueur attendue, ou si
 * numSens dépasse UBX_ESF_MAX_SENS. */
bool ubx_parse_esf_status(const uint8_t *payload, size_t len, ubx_esf_status_t *out);

/* ===================== UBX-ACK-ACK / UBX-ACK-NAK (classe 0x05) ===================== */

#define UBX_ACK_CLASS 0x05
#define UBX_ACK_ACK_ID 0x01
#define UBX_ACK_NAK_ID 0x00
#define UBX_ACK_PAYLOAD_LEN 2

/* Payload identique pour ACK-ACK et ACK-NAK : classe/ID du message
 * acquitté ou rejeté. C'est le msg_id de la trame ACK elle-même
 * (UBX_ACK_ACK_ID ou UBX_ACK_NAK_ID) qui indique s'il s'agit d'un
 * acquittement ou d'un rejet. */
typedef struct {
    uint8_t acked_class;
    uint8_t acked_id;
} ubx_ack_t;

/* Décode un payload UBX-ACK-ACK ou UBX-ACK-NAK brut.
 * Retourne false si len != UBX_ACK_PAYLOAD_LEN (2). */
bool ubx_parse_ack(const uint8_t *payload, size_t len, ubx_ack_t *out);

#endif /* UBX_PROTOCOL_H */
