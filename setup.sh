#!/usr/bin/env bash
# =============================================================================
# Per-machine bootstrap. Run once after cloning, on every new computer.
#
# Writes a .env into each compose project with THIS machine's UID/GID. That is
# the only thing in the repo that cannot be committed: the container user is
# built to match the host user so that files written into the bind-mounted
# workspace/ come out owned by you instead of root. Hardcoding 1000 works on
# exactly the machines where you happen to be 1000.
#
# Does NOT install udev rules or the NVIDIA toolkit -- those need sudo and are
# per-container concerns. See middle-arm/host-setup/setup-host.sh.
# =============================================================================
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")"

uid="$(id -u)"
gid="$(id -g)"

for project in manip-arm middle-arm; do
  [ -d "$project" ] || continue
  printf 'UID=%s\nGID=%s\n' "$uid" "$gid" > "$project/.env"
  echo "wrote $project/.env  (UID=$uid GID=$gid)"
done

echo
echo "Next:"
echo "  middle arm (wx250s + camera):"
echo "    ./middle-arm/host-setup/setup-host.sh    # udev rules; NVIDIA toolkit if using ZED"
echo "    cd middle-arm && docker compose build && docker compose up -d"
echo
echo "  manipulator arms (wxai_v0 over Ethernet):"
echo "    cd manip-arm && docker compose build && docker compose up -d"
echo
echo "See README.md for what does and does not transfer between machines."
