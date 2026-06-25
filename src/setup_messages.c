/* _POSIX_C_SOURCE : nécessaire pour poll() et clock_gettime() en mode
 * strict -std=c11 (sinon non déclarés par les en-têtes glibc). */
#define _POSIX_C_SOURCE 200809L

#include "setup_messages.h"

#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "ubx_cfg.h"
#include "ubx_parser.h"
#include "ubx_protocol.h"

#define SETUP_ACK_TIMEOUT_MS 1000

static void print_hex_buf(const char *label, const uint8_t *buf, size_t len)
{
    printf("%s (%zu octets): ", label, len);
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\n");
    fflush(stdout);
}

static double elapsed_ms_since(const struct timespec *start)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)(now.tv_sec - start->tv_sec) * 1000.0
           + (double)(now.tv_nsec - start->tv_nsec) / 1e6;
}

bool setup_enable_nav_and_esf_messages(int fd)
{
    uint8_t frame_buf[64];
    size_t frame_len = 0;

    ubx_cfg_kv_t kvs[2] = {
        { UBX_CFG_KEY_MSGOUT_NAV_PVT_USB, 1 },
        { UBX_CFG_KEY_MSGOUT_ESF_STATUS_USB, 1 },
    };

    if (!ubx_build_cfg_valset(frame_buf, sizeof(frame_buf), UBX_CFG_LAYER_RAM, kvs, 2, &frame_len)) {
        fprintf(stderr, "setup_enable_nav_and_esf_messages: échec de construction de la trame CFG-VALSET\n");
        return false;
    }

    print_hex_buf("CFG-VALSET envoyé", frame_buf, frame_len);

    ssize_t written = write(fd, frame_buf, frame_len);
    if (written < 0) {
        fprintf(stderr, "setup_enable_nav_and_esf_messages: write() a échoué: %s\n", strerror(errno));
        return false;
    }
    if ((size_t)written != frame_len) {
        fprintf(stderr, "setup_enable_nav_and_esf_messages: écriture partielle (%zd/%zu octets)\n", written, frame_len);
        return false;
    }

    ubx_parser_t parser;
    ubx_parser_init(&parser);
    ubx_frame_t frame;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (;;) {
        double remaining_ms = SETUP_ACK_TIMEOUT_MS - elapsed_ms_since(&start);
        if (remaining_ms <= 0.0) {
            fprintf(stderr, "setup_enable_nav_and_esf_messages: timeout (%dms) en attente de l'ACK/NAK\n",
                    SETUP_ACK_TIMEOUT_MS);
            return false;
        }

        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        int poll_result = poll(&pfd, 1, (int)remaining_ms);
        if (poll_result < 0) {
            fprintf(stderr, "setup_enable_nav_and_esf_messages: poll() a échoué: %s\n", strerror(errno));
            return false;
        }
        if (poll_result == 0) {
            fprintf(stderr, "setup_enable_nav_and_esf_messages: timeout (%dms) en attente de l'ACK/NAK\n",
                    SETUP_ACK_TIMEOUT_MS);
            return false;
        }

        uint8_t byte;
        ssize_t n = read(fd, &byte, 1);
        if (n <= 0) {
            fprintf(stderr, "setup_enable_nav_and_esf_messages: erreur de lecture sur le port série\n");
            return false;
        }

        int result = ubx_parser_feed(&parser, byte, &frame);
        if (result != 1) {
            continue;
        }
        if (!frame.checksum_valid) {
            continue;
        }

        if (frame.msg_class != UBX_ACK_CLASS) {
            continue; /* trame UBX sans rapport (improbable à ce stade) */
        }
        if (frame.msg_id != UBX_ACK_ACK_ID && frame.msg_id != UBX_ACK_NAK_ID) {
            continue;
        }

        ubx_ack_t ack;
        if (!ubx_parse_ack(frame.payload, frame.length, &ack)) {
            continue;
        }
        if (ack.acked_class != UBX_CFG_CLASS || ack.acked_id != UBX_CFG_VALSET_ID) {
            continue; /* ACK/NAK pour une autre commande que la nôtre */
        }

        print_hex_buf(frame.msg_id == UBX_ACK_ACK_ID ? "ACK reçu" : "NAK reçu",
                       frame.payload, frame.length);

        if (frame.msg_id == UBX_ACK_ACK_ID) {
            printf("CFG-VALSET acquittée (ACK) pour classe=0x%02X id=0x%02X\n",
                   ack.acked_class, ack.acked_id);
            fflush(stdout);
            return true;
        }

        fprintf(stderr, "setup_enable_nav_and_esf_messages: NAK reçu — configuration refusée par le récepteur "
                         "(classe=0x%02X id=0x%02X)\n", ack.acked_class, ack.acked_id);
        return false;
    }
}
