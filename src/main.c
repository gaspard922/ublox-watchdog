/* _POSIX_C_SOURCE : nécessaire pour poll() et clock_gettime() en mode
 * strict -std=c11 (sinon non déclarés par les en-têtes glibc). */
#define _POSIX_C_SOURCE 200809L

#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "serial_port.h"
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
 * Affiche en hex chaque trame UBX détectée et décode en clair NAV-PVT et
 * ESF-STATUS pour validation visuelle. fusion_tracker, drift_estimator et
 * output_stream ne sont pas encore branchés. */

#define POLL_INTERVAL_MS 1000

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

static void dispatch_frame(const ubx_frame_t *frame)
{
    print_frame_hex(frame);

    if (!frame->checksum_valid) {
        return;
    }

    if (frame->msg_class == UBX_NAV_PVT_CLASS && frame->msg_id == UBX_NAV_PVT_ID) {
        ubx_nav_pvt_t pvt;
        if (ubx_parse_nav_pvt(frame->payload, frame->length, &pvt)) {
            print_nav_pvt(&pvt);
        }
    } else if (frame->msg_class == UBX_ESF_STATUS_CLASS && frame->msg_id == UBX_ESF_STATUS_ID) {
        ubx_esf_status_t esf;
        if (ubx_parse_esf_status(frame->payload, frame->length, &esf)) {
            print_esf_status(&esf);
        }
    }
}

static double elapsed_ms_since(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)(now.tv_sec - start->tv_sec) * 1000.0
           + (double)(now.tv_nsec - start->tv_nsec) / 1e6;
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

int main(int argc, char *argv[])
{
    const char *device = (argc > 1) ? argv[1] : "/dev/ttyACM0";

    int fd = serial_port_open(device, B9600);
    if (fd == SERIAL_PORT_INVALID_FD) {
        fprintf(stderr, "main: impossible d'ouvrir le port série %s\n", device);
        return 1;
    }

    printf("Port série %s ouvert (9600 bauds). Poll actif NAV-PVT/ESF-STATUS toutes les %dms.\n",
           device, POLL_INTERVAL_MS);
    fflush(stdout);

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_frame_t frame;

    struct timespec last_poll;
    clock_gettime(CLOCK_MONOTONIC, &last_poll);
    send_poll_requests(fd);

    for (;;) {
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
            fprintf(stderr, "main: erreur de lecture sur le port série\n");
            break;
        }
        if (n == 0) {
            continue;
        }

        int result = ubx_parser_feed(&parser, byte, &frame);
        if (result == 1) {
            dispatch_frame(&frame);
        } else if (result == -1) {
            fprintf(stderr, "main: trame abandonnée (longueur hors limites)\n");
        }
    }

    serial_port_close(fd);
    return 0;
}
