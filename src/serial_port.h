#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

/* Ouverture et configuration d'un port série USB en mode raw (termios).
 * Cible typique : récepteur u-blox C100-F9K, apparaissant sous Linux
 * comme /dev/ttyACM0, vitesse de boot par défaut 9600 bauds. */

/* Descripteur de fichier invalide retourné en cas d'échec. */
#define SERIAL_PORT_INVALID_FD (-1)

/* Ouvre le port série, le configure en mode raw au baudrate demandé.
 * device : chemin du périphérique (ex: "/dev/ttyACM0")
 * baudrate : valeur POSIX termios (ex: B9600, B115200, depuis termios.h)
 * Retourne le file descriptor ouvert, ou SERIAL_PORT_INVALID_FD en cas
 * d'erreur (détails écrits sur stderr via perror). */
int serial_port_open(const char *device, int baudrate);

/* Ferme le port série ouvert par serial_port_open().
 * Retourne 0 en cas de succès, -1 en cas d'erreur. */
int serial_port_close(int fd);

#endif /* SERIAL_PORT_H */
