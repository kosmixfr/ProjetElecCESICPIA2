#!/usr/bin/env bash
#
# Téléverse le firmware ACAB sur l'Arduino Uno.
#
# Usage :
#   ./upload.sh                     # env uno, port auto-détecté
#   ./upload.sh uno_sans_rtc        # variante sans RTC
#   ./upload.sh -e uno_sans_rtc     # idem
#   ./upload.sh -p /dev/ttyACM0     # forcer le port
#   ./upload.sh -m                  # ouvrir le moniteur série (9600 bauds) après upload
#
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

ENV_NAME="uno"
PORT=""
MONITOR=0

usage() {
    sed -n '2,11p' "$0" | sed 's/^# \{0,1\}//'
}

die() {
    echo "Erreur : $*" >&2
    exit 1
}

# Environnements déclarés dans platformio.ini
available_envs() {
    sed -n 's/^\[env:\([^]]*\)\].*/\1/p' "$PROJECT_DIR/platformio.ini"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -e|--env)
            [[ $# -ge 2 ]] || die "valeur manquante pour $1"
            ENV_NAME="$2"
            shift 2
            ;;
        -p|--port)
            [[ $# -ge 2 ]] || die "valeur manquante pour $1"
            PORT="$2"
            shift 2
            ;;
        -m|--monitor)
            MONITOR=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        -*)
            die "option inconnue : $1"
            ;;
        *)
            ENV_NAME="$1"
            shift
            ;;
    esac
done

# Validation de l'environnement
if ! available_envs | grep -qx "$ENV_NAME"; then
    die "environnement '$ENV_NAME' introuvable. Disponibles : $(available_envs | tr '\n' ' ')"
fi

# Localisation de PlatformIO (hors PATH possible)
PIO="$(command -v pio || true)"
if [[ -z "$PIO" && -x "$HOME/.platformio/penv/bin/pio" ]]; then
    PIO="$HOME/.platformio/penv/bin/pio"
fi
[[ -n "$PIO" ]] || die "PlatformIO (pio) introuvable. Installez-le ou ajoutez ~/.platformio/penv/bin au PATH."

# Détection du port série
if [[ -z "$PORT" ]]; then
    mapfile -t PORTS < <(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true)
    case "${#PORTS[@]}" in
        0)
            echo "Aucun port série détecté, PlatformIO tentera une auto-détection."
            ;;
        1)
            PORT="${PORTS[0]}"
            echo "Port détecté : $PORT"
            ;;
        *)
            echo "Plusieurs ports série détectés :"
            select CHOICE in "${PORTS[@]}"; do
                [[ -n "$CHOICE" ]] || { echo "Choix invalide." >&2; continue; }
                PORT="$CHOICE"
                break
            done
            ;;
    esac
fi

[[ -z "$PORT" ]] || [[ -e "$PORT" ]] || die "port '$PORT' introuvable."

UPLOAD_ARGS=(-e "$ENV_NAME" -t upload)
[[ -z "$PORT" ]] || UPLOAD_ARGS+=(--upload-port "$PORT")

echo "Téléversement : env=$ENV_NAME${PORT:+ port=$PORT}"
"$PIO" run "${UPLOAD_ARGS[@]}"

echo "Téléversement terminé."

if [[ "$MONITOR" -eq 1 ]]; then
    MONITOR_ARGS=(--baud 9600)
    [[ -z "$PORT" ]] || MONITOR_ARGS+=(--port "$PORT")
    exec "$PIO" device monitor "${MONITOR_ARGS[@]}"
fi
