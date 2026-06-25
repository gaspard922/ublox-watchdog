# ublox-watchdog — Contexte projet pour Claude Code

## Vue d'ensemble

Bibliothèque et démon en **C pur (C11), zéro dépendance externe**, qui se connecte à un récepteur GNSS/INS u-blox **ZED-F9K** (carte d'évaluation **C100-F9K**) via port série USB, décode le protocole binaire **UBX**, et fournit une couche de **supervision de la fusion GNSS/INS** absente des outils existants (u-center, ubxlib officiel, libs Arduino).

Le projet cible une cible Linux embarqué (testé sur NVIDIA Jetson), mais ne doit dépendre d'aucune lib spécifique Jetson — uniquement POSIX standard (termios, fichiers, sockets Unix).

## Pourquoi ce projet existe (ne pas dévier de ça)

Les outils existants (u-center, ubxlib, SparkFun Arduino lib) savent déjà parser et **afficher** les messages UBX-ESF-STATUS / UBX-NAV-PVT / UBX-ESF-MEAS. Ce n'est PAS la valeur ajoutée de ce projet.

La valeur ajoutée est une **couche de décision programmatique** au-dessus du parsing brut :
1. Une state machine qui suit les transitions de fusion GNSS/INS (calibration, fusion, dégradation, perte) avec une logique temporelle propre (timeouts configurables)
2. Un estimateur de dérive qui intègre les données de vitesse/IMU pendant les phases de dégradation GNSS, pour quantifier "depuis combien de mètres on navigue en dead-reckoning pur"
3. Une sortie structurée (JSON sur stdout ou socket Unix) pensée pour être consommée par un autre process (ex: un node ROS2), PAS pour être lue par un humain dans un GUI

Si une fonctionnalité proposée se résume à "parser un message UBX et l'afficher", ce n'est pas dans le scope — ça existe déjà ailleurs.

## Stack technique imposée

- **Langage** : C11 strict, compilé avec `-Wall -Wextra -Wpedantic -std=c11`
- **Zéro dépendance externe** : pas de libubx, pas de libserialport. Utiliser directement `termios.h` pour le port série
- **POSIX uniquement** : `unistd.h`, `termios.h`, `time.h` (`clock_gettime` avec `CLOCK_MONOTONIC`), `sys/socket.h` pour le socket Unix de sortie
- **Pas de malloc/free en continu dans le hot path** : préférer des buffers de taille fixe alloués une fois au démarrage
- **Build** : Makefile simple, pas de CMake (pas nécessaire pour ce volume de code)

## Architecture cible

```
ublox-watchdog/
├── src/
│   ├── ubx_protocol.h       # structures des messages UBX utilisés (NAV-PVT, ESF-STATUS, ESF-MEAS)
│   ├── ubx_parser.c/h       # state machine de parsing trame UBX (sync bytes, classe/ID, longueur, payload, checksum Fletcher-8)
│   ├── serial_port.c/h      # ouverture/configuration port série via termios (baudrate, raw mode)
│   ├── fusion_tracker.c/h   # state machine de suivi fusion GNSS/INS (cf section dédiée)
│   ├── drift_estimator.c/h # calcul distance parcourue en dead-reckoning pendant dégradation
│   ├── output_stream.c/h   # sérialisation JSON + écriture stdout/socket Unix
│   └── main.c               # boucle principale : lecture port série -> parsing -> update tracker -> output
├── tests/
│   ├── test_ubx_parser.c    # tests sur trames UBX fixtures (valides et corrompues)
│   ├── test_fusion_tracker.c # tests des transitions d'état avec timestamps simulés
│   └── fixtures/             # captures de trames UBX réelles ou générées pour les tests
├── examples/
│   └── ros2_bridge_demo/     # exemple minimal montrant la lecture du socket Unix depuis un node ROS2 (Python, juste pour la démo d'intégration)
├── docs/
│   └── test_protocol.md      # protocole de test terrain (coupure GNSS volontaire, mesure dérive réelle)
├── Makefile
├── README.md
└── CLAUDE.md
```

## La state machine de fusion (fusion_tracker.c/h)

États : `UNKNOWN`, `NOT_CALIBRATED`, `CALIBRATING`, `FUSION`, `DEGRADED`, `LOST`

Logique de transition principale :
- `FUSION -> DEGRADED` : dès que `UBX-NAV-PVT.fixType < 3` ou flag `GNSS_FIX_OK` absent
- `DEGRADED -> FUSION` : dès que le fix GNSS redevient valide
- `DEGRADED -> LOST` : si `DEGRADED` dure plus de `DEGRADED_TIMEOUT_MS` (configurable, défaut 30000ms)
- `LOST -> CALIBRATING` : dès que le fix GNSS redevient valide (recalibration nécessaire)

Chaque transition déclenche un callback `on_state_transition()` pour logging + mise à jour du timestamp d'entrée d'état.

Source de vérité pour les champs UBX exacts (fixType, flags, fusionMode) : se référer aux interface description u-blox (UBX-23010294 ou la dernière révision disponible pour ZED-F9K) — **vérifier les offsets exacts dans la doc avant d'implémenter, ne pas deviner les structures**.

## L'estimateur de dérive (drift_estimator.c/h)

V1 (à implémenter en premier) : intégration simple de la vitesse sol (`groundSpeed` de NAV-PVT, en mm/s) sur le temps écoulé, uniquement pendant l'état `DEGRADED`. Reset du compteur à chaque entrée dans `DEGRADED`.

```c
accumulated_drift_m += (ground_speed_mm_s / 1000.0) * dt_seconds;
```

V2 (optionnelle, après validation terrain) : enveloppe d'incertitude basée sur un taux de dérive empirique mesuré via le protocole de test terrain (voir docs/test_protocol.md).

**Important** : ne jamais présenter `accumulated_drift_m` comme une mesure d'erreur réelle — c'est une distance parcourue en dead-reckoning, pas une erreur de position vérifiée. Le README et les commentaires de code doivent être rigoureux sur cette distinction.

## Format de sortie JSON

```json
{
  "timestamp_ms": 1718978234123,
  "fusion_state": "DEGRADED",
  "state_duration_ms": 4500,
  "accumulated_drift_m": 12.4,
  "gnss_last_fix_ms_ago": 4500
}
```

Une ligne JSON par mise à jour d'état significative (pas nécessairement à chaque trame UBX reçue, pour éviter de noyer le consommateur).

## Conventions de code

- snake_case partout (fonctions, variables, fichiers)
- Chaque fichier `.c` a son `.h` correspondant avec include guards (`#ifndef UBX_PARSER_H` style, pas `#pragma once`)
- Pas de variables globales sauf si explicitement justifié en commentaire
- Gestion d'erreur systématique sur les appels POSIX (vérifier les codes retour, pas d'erreurs silencieuses)
- Commentaires en français acceptés dans le code, mais noms de fonctions/variables en anglais (convention dominante du domaine)

## Étapes de développement attendues (par priorité)

1. `serial_port.c` — ouverture et configuration termios du port série USB du C100-F9K
2. `ubx_parser.c` — state machine de parsing de trame UBX générique (sync, longueur, checksum) + dispatch par classe/ID
3. Structures et parsing spécifique pour `UBX-NAV-PVT` et `UBX-ESF-STATUS` uniquement (pas tous les messages UBX, seulement ceux nécessaires)
4. `fusion_tracker.c` — state machine de fusion avec tests unitaires sur transitions
5. `drift_estimator.c` — V1 intégration simple
6. `output_stream.c` — sortie JSON stdout, puis socket Unix
7. `main.c` — assemblage de la boucle complète
8. Tests d'intégration avec fixtures de trames réelles capturées sur le hardware

## Ce qu'il ne faut PAS faire

- Ne pas réimplémenter un parser UBX générique pour tous les messages existants — seulement NAV-PVT et ESF-STATUS sont nécessaires pour le scope V1
- Ne pas ajouter de dépendance externe (pas de cJSON, écrire la sérialisation JSON minimale à la main, c'est trivial pour ce volume de champs)
- Ne pas confondre dérive estimée et erreur réelle dans la doc/code
- Ne pas optimiser prématurément (SIMD, lock-free) — ce n'est pas le goulot d'étranglement ici, la priorité est la justesse du parsing et de la state machine