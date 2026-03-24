#!/usr/bin/env bash
# Renew Tailscale-issued TLS certs for navi (Let's Encrypt via tailscale cert).
# Usage:
#   ./deploy/renew-tailscale-cert.sh [fqdn]
# If fqdn is omitted, uses this node's MagicDNS name from `tailscale status --json`.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
CERT="${REPO_ROOT}/certs/robot.pem"
KEY="${REPO_ROOT}/certs/robot-key.pem"

run_cert() {
  if [[ "${EUID:-0}" -eq 0 ]]; then
    tailscale cert "$@"
  else
    sudo tailscale cert "$@"
  fi
}

if ! command -v tailscale >/dev/null 2>&1; then
  echo "error: tailscale CLI not found" >&2
  exit 1
fi

if [[ "${1:-}" ]]; then
  FQDN="$1"
else
  if ! command -v jq >/dev/null 2>&1; then
    echo "error: jq is required to read MagicDNS name (sudo apt install jq)" >&2
    exit 1
  fi
  FQDN="$(tailscale status --json | jq -r '.Self.DNSName' | sed 's/\.$//')"
  if [[ -z "$FQDN" || "$FQDN" == "null" ]]; then
    echo "error: could not determine FQDN; pass it as the first argument" >&2
    exit 1
  fi
fi

mkdir -p "$(dirname "$CERT")"
echo "Writing TLS cert for ${FQDN} -> ${CERT} ${KEY}"
run_cert --cert-file "$CERT" --key-file "$KEY" "$FQDN"
echo "Done. Restart navi (e.g. sudo systemctl restart link.service) to load new certs."
