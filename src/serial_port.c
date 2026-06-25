/* _DEFAULT_SOURCE : nécessaire pour cfmakeraw() et CRTSCTS (extensions
 * glibc/BSD), masquées par défaut en mode strict -std=c11. */
#define _DEFAULT_SOURCE

#include "serial_port.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

int serial_port_open(const char *device, int baudrate)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == SERIAL_PORT_INVALID_FD) {
        fprintf(stderr, "serial_port_open: open(%s) failed: %s\n", device, strerror(errno));
        return SERIAL_PORT_INVALID_FD;
    }

    /* Repasser en mode bloquant pour les lectures : O_NONBLOCK n'était
     * utile que pour éviter un open() bloquant sur certains pilotes USB CDC-ACM. */
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
        fprintf(stderr, "serial_port_open: fcntl(F_GETFL) failed: %s\n", strerror(errno));
        close(fd);
        return SERIAL_PORT_INVALID_FD;
    }
    if (fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) == -1) {
        fprintf(stderr, "serial_port_open: fcntl(F_SETFL) failed: %s\n", strerror(errno));
        close(fd);
        return SERIAL_PORT_INVALID_FD;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "serial_port_open: tcgetattr failed: %s\n", strerror(errno));
        close(fd);
        return SERIAL_PORT_INVALID_FD;
    }

    if (cfsetispeed(&tty, (speed_t)baudrate) != 0) {
        fprintf(stderr, "serial_port_open: cfsetispeed failed: %s\n", strerror(errno));
        close(fd);
        return SERIAL_PORT_INVALID_FD;
    }
    if (cfsetospeed(&tty, (speed_t)baudrate) != 0) {
        fprintf(stderr, "serial_port_open: cfsetospeed failed: %s\n", strerror(errno));
        close(fd);
        return SERIAL_PORT_INVALID_FD;
    }

    /* Mode raw : pas de traitement canonique, pas d'écho, pas de
     * traduction de caractères de contrôle. Indispensable pour un
     * protocole binaire comme UBX. */
    cfmakeraw(&tty);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     /* 8 bits de données */
    tty.c_cflag &= ~PARENB; /* pas de parité */
    tty.c_cflag &= ~CSTOPB; /* 1 bit de stop */
    tty.c_cflag &= ~CRTSCTS; /* pas de contrôle de flux matériel */

    /* read() retourne dès qu'au moins 1 octet est disponible, sans
     * timeout : on laisse le bouclage applicatif gérer le rythme. */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "serial_port_open: tcsetattr failed: %s\n", strerror(errno));
        close(fd);
        return SERIAL_PORT_INVALID_FD;
    }

    if (tcflush(fd, TCIOFLUSH) != 0) {
        fprintf(stderr, "serial_port_open: tcflush failed: %s\n", strerror(errno));
        close(fd);
        return SERIAL_PORT_INVALID_FD;
    }

    return fd;
}

int serial_port_close(int fd)
{
    if (fd == SERIAL_PORT_INVALID_FD) {
        return -1;
    }
    if (close(fd) != 0) {
        fprintf(stderr, "serial_port_close: close failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}
