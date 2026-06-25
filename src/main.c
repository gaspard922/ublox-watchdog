/* _POSIX_C_SOURCE : nécessaire pour poll() et clock_gettime() en mode
 * strict -std=c11 (sinon non déclarés par les en-têtes glibc). */
#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "serial_port.h"
#include "ubx_fusion_tracker.h"
#include "ubx_parser.h"
#include "ubx_poll.h"
#include "ubx_protocol.h"

/* Programme minimal de validation hardware : ouvre le port série du
 * C100-F9K et interroge activement NAV-PVT et ESF-STATUS à intervalle
 * régulier (poll request à payload vide, classe/ID identiques au
 * message normalement poussé). On a constaté sur ce hardware que le
 * push périodique configuré via CFG-MSGOUT (cf module setup_messages,
 * conservé mais non utilisé ici) ne déclenche pas de sortie spontanée
 * tant qu'aucun fix GNSS n'est obtenu — le poll actif fonctionne dans
 * tous les cas, c'est le mécanisme retenu pour cette étape.
 *
 * Affiche en hex chaque trame UBX détectée, décode en clair NAV-PVT et
 * ESF-STATUS, et fait avancer fusion_tracker à chaque message décodé.
 * drift_estimator et output_stream ne sont pas encore branchés. */

#define POLL_INTERVAL_MS 1000
#define DEFAULT_DEGRADED_TIMEOUT_MS 30000

static volatile sig_atomic_t g_stop_requested = 0;

static void handle_sigint(int signum)
{
    (void)signum;
    g_stop_requested = 1;
}

/* Regroupe tout l'état mutable de la boucle principale (tracker de
 * fusion + dernières données décodées de chaque message), passé par
 * pointeur plutôt que stocké en variables globales. */
typedef struct {
    fusion_tracker_t tracker;
    ubx_nav_pvt_t last_nav_pvt;
    bool have_nav_pvt;
    ubx_esf_status_t last_esf_status;
    bool have_esf_status;
} watchdog_state_t;

static void print_frame_hex(const ubx_frame_t *frame)
{
    printf("UBX class=0x%02X id=0x%02X len=%u checksum=%s payload=",
           frame->msg_class, frame->msg_id, frame->length,
           frame->checksum_valid ? "OK" : "INVALID");

    for (uint16_t i = 0; i < frame->length; i++) {
        printf("%02X ", frame->payload[i]);
    }
    printf("\n");
    fflush(stdout);
}

static void print_nav_pvt(const ubx_nav_pvt_t *pvt)
{
    printf("  NAV-PVT: %04u-%02u-%02u %02u:%02u:%02u fixType=%u gnssFixOK=%d numSV=%u "
           "lat=%.7f lon=%.7f hMSL=%.3fm gSpeed=%.3fm/s carrSoln=%u\n",
           pvt->year, pvt->month, pvt->day, pvt->hour, pvt->min, pvt->sec,
           pvt->fix_type, pvt->gnss_fix_ok ? 1 : 0, pvt->num_sv,
           pvt->lat * 1e-7, pvt->lon * 1e-7, pvt->h_msl / 1000.0, pvt->g_speed / 1000.0,
           pvt->carr_soln);
    fflush(stdout);
}

static const char *fusion_mode_name(uint8_t mode)
{
    switch (mode) {
    case UBX_ESF_FUSION_INIT: return "INIT";
    case UBX_ESF_FUSION_FUSION: return "FUSION";
    case UBX_ESF_FUSION_SUSPENDED: return "SUSPENDED";
    case UBX_ESF_FUSION_DISABLED: return "DISABLED";
    default: return "UNKNOWN";
    }
}

static void print_esf_status(const ubx_esf_status_t *esf)
{
    printf("  ESF-STATUS: fusionMode=%s numSens=%u\n", fusion_mode_name(esf->fusion_mode), esf->num_sens);
    for (uint8_t i = 0; i < esf->num_sens; i++) {
        const ubx_esf_sensor_t *s = &esf->sensors[i];
        printf("    sensor[%u]: type=%u used=%d ready=%d calibStatus=%u freq=%uHz "
               "badMeas=%d badTTag=%d missingMeas=%d noisyMeas=%d\n",
               i, s->type, s->used ? 1 : 0, s->ready ? 1 : 0, s->calib_status, s->freq,
               s->bad_meas ? 1 : 0, s->bad_ttag ? 1 : 0, s->missing_meas ? 1 : 0, s->noisy_meas ? 1 : 0);
    }
    fflush(stdout);
}

static void print_fusion_status(const watchdog_state_t *state, uint64_t now_ms)
{
    uint64_t state_duration_ms = now_ms - state->tracker.state_entered_at_ms;
    uint32_t num_sv = state->have_nav_pvt ? state->last_nav_pvt.num_sv : 0;

    printf("[fusion_state: %s | depuis %llus | fix=%s numSV=%u]\n",
           fusion_state_name(state->tracker.current_state),
           (unsigned long long)(state_duration_ms / 1000),
           state->tracker.has_gnss_fix ? "valid" : "invalid",
           num_sv);
    fflush(stdout);
}

static void dispatch_frame(const ubx_frame_t *frame, watchdog_state_t *state, uint64_t now_ms)
{
    print_frame_hex(frame);

    if (!frame->checksum_valid) {
        return;
    }

    const ubx_nav_pvt_t *nav_pvt_for_tracker = NULL;
    const ubx_esf_status_t *esf_status_for_tracker = NULL;

    if (frame->msg_class == UBX_NAV_PVT_CLASS && frame->msg_id == UBX_NAV_PVT_ID) {
        if (ubx_parse_nav_pvt(frame->payload, frame->length, &state->last_nav_pvt)) {
            state->have_nav_pvt = true;
            print_nav_pvt(&state->last_nav_pvt);
            nav_pvt_for_tracker = &state->last_nav_pvt;
        }
    } else if (frame->msg_class == UBX_ESF_STATUS_CLASS && frame->msg_id == UBX_ESF_STATUS_ID) {
        if (ubx_parse_esf_status(frame->payload, frame->length, &state->last_esf_status)) {
            state->have_esf_status = true;
            print_esf_status(&state->last_esf_status);
            esf_status_for_tracker = &state->last_esf_status;
        }
    } else {
        return; /* message dont fusion_tracker n'a pas besoin */
    }

    fusion_state_t prev_state, new_state;
    const char *reason = NULL;
    bool transitioned = fusion_tracker_update(&state->tracker, nav_pvt_for_tracker, esf_status_for_tracker,
                                              now_ms, &prev_state, &new_state, &reason);

    print_fusion_status(state, now_ms);

    if (transitioned) {
        printf(">>> TRANSITION: %s -> %s (raison: %s)\n",
               fusion_state_name(prev_state), fusion_state_name(new_state),
               reason != NULL ? reason : "inconnue");
        fflush(stdout);
    }
}

static double elapsed_ms_since(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)(now.tv_sec - start->tv_sec) * 1000.0
           + (double)(now.tv_nsec - start->tv_nsec) / 1e6;
}

static uint64_t monotonic_now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000);
}

static void send_poll_requests(int fd)
{
    uint8_t buf[8];
    size_t len = 0;

    if (ubx_build_poll_request(buf, sizeof(buf), UBX_NAV_PVT_CLASS, UBX_NAV_PVT_ID, &len)) {
        if (write(fd, buf, len) != (ssize_t)len) {
            fprintf(stderr, "main: échec d'écriture du poll NAV-PVT\n");
        }
    }
    if (ubx_build_poll_request(buf, sizeof(buf), UBX_ESF_STATUS_CLASS, UBX_ESF_STATUS_ID, &len)) {
        if (write(fd, buf, len) != (ssize_t)len) {
            fprintf(stderr, "main: échec d'écriture du poll ESF-STATUS\n");
        }
    }
}

/* Parse les arguments de ligne de commande : un chemin de périphérique
 * optionnel (positionnel) et --degraded-timeout=<ms> optionnel (dans
 * n'importe quel ordre). */
static void parse_args(int argc, char *argv[], const char **out_device, uint64_t *out_degraded_timeout_ms)
{
    static const char prefix[] = "--degraded-timeout=";
    size_t prefix_len = sizeof(prefix) - 1;

    *out_device = "/dev/ttyACM0";
    *out_degraded_timeout_ms = DEFAULT_DEGRADED_TIMEOUT_MS;

    for (int i = 1; i < argc; i++) {
        if (strncmp(argv[i], prefix, prefix_len) == 0) {
            *out_degraded_timeout_ms = strtoull(argv[i] + prefix_len, NULL, 10);
        } else {
            *out_device = argv[i];
        }
    }
}

int main(int argc, char *argv[])
{
    const char *device;
    uint64_t degraded_timeout_ms;
    parse_args(argc, argv, &device, &degraded_timeout_ms);

    if (signal(SIGINT, handle_sigint) == SIG_ERR) {
        fprintf(stderr, "main: impossible d'installer le handler SIGINT\n");
        return 1;
    }

    int fd = serial_port_open(device, B9600);
    if (fd == SERIAL_PORT_INVALID_FD) {
        fprintf(stderr, "main: impossible d'ouvrir le port série %s\n", device);
        return 1;
    }

    printf("Port série %s ouvert (9600 bauds). Poll actif NAV-PVT/ESF-STATUS toutes les %dms. "
           "degraded_timeout=%llums.\n",
           device, POLL_INTERVAL_MS, (unsigned long long)degraded_timeout_ms);
    fflush(stdout);

    watchdog_state_t state;
    memset(&state, 0, sizeof(state));
    fusion_tracker_init(&state.tracker, degraded_timeout_ms);

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_frame_t frame;

    struct timespec last_poll;
    clock_gettime(CLOCK_MONOTONIC, &last_poll);
    send_poll_requests(fd);

    while (!g_stop_requested) {
        double since_last_poll = elapsed_ms_since(&last_poll);
        double wait_ms = POLL_INTERVAL_MS - since_last_poll;
        if (wait_ms < 0.0) {
            wait_ms = 0.0;
        }

        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        int poll_result = poll(&pfd, 1, (int)wait_ms);
        if (poll_result < 0) {
            if (errno == EINTR) {
                continue; /* probablement notre propre SIGINT, re-vérifié en haut de boucle */
            }
            fprintf(stderr, "main: erreur poll() sur le port série\n");
            break;
        }

        if (elapsed_ms_since(&last_poll) >= POLL_INTERVAL_MS) {
            send_poll_requests(fd);
            clock_gettime(CLOCK_MONOTONIC, &last_poll);
        }

        if (poll_result == 0) {
            continue; /* timeout : pas de données, on retente le cycle (déclenchera le prochain poll) */
        }

        uint8_t byte;
        ssize_t n = read(fd, &byte, 1);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            fprintf(stderr, "main: erreur de lecture sur le port série\n");
            break;
        }
        if (n == 0) {
            continue;
        }

        int result = ubx_parser_feed(&parser, byte, &frame);
        if (result == 1) {
            dispatch_frame(&frame, &state, monotonic_now_ms());
        } else if (result == -1) {
            fprintf(stderr, "main: trame abandonnée (longueur hors limites)\n");
        }
    }

    if (g_stop_requested) {
        printf("\nArrêt demandé (SIGINT), fermeture du port série...\n");
        fflush(stdout);
    }

    serial_port_close(fd);
    return 0;
}
