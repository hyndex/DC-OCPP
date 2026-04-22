#!/usr/bin/env bash
set -euo pipefail
printf '%s dc-ocpp disabled for module bench testing\n' "$(date -Iseconds)" >&2
exec /bin/sleep infinity
