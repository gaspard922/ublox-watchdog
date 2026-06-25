#ifndef SETUP_MESSAGES_H
#define SETUP_MESSAGES_H

#include <stdbool.h>

/* Envoie une commande UBX-CFG-VALSET (layer RAM) pour activer la sortie
 * périodique de UBX-NAV-PVT et UBX-ESF-STATUS sur le port USB, puis
 * attend l'ACK/NAK correspondant avec un timeout.
 *
 * fd : descripteur du port série déjà ouvert et configuré.
 *
 * Retourne true si un ACK a été reçu avant le timeout. Retourne false
 * si un NAK a été reçu, si le timeout a expiré, ou en cas d'erreur
 * d'écriture/lecture sur le port (détails écrits sur stderr dans tous
 * les cas d'échec). */
bool setup_enable_nav_and_esf_messages(int fd);

#endif /* SETUP_MESSAGES_H */
