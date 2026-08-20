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

# Discovered, not listed. A hardcoded list goes stale silently every time a
# project is added or removed, and the failure it produces is genuinely nasty:
# a project with no .env builds with UID 1000 instead of yours, and then Fast
# DDS cannot move data between that container and the others -- its shared
# memory segments are 0644 and a DDS writer has to WRITE into the reader's
# segment. Discovery still works, so you get a full `ros2 topic list` and not
# one message. Ask the filesystem instead.
for project in $(find . -maxdepth 2 -name docker-compose.yml -printf '%h\n' \
                | sed 's|^\./||' | grep -v '^\.$' | sort); do
  [ -d "$project" ] || continue
  printf 'UID=%s\nGID=%s\n' "$uid" "$gid" > "$project/.env"
  echo "wrote $project/.env  (UID=$uid GID=$gid)"
done

echo
echo "Next:"
echo "  everything at once:"
echo "    docker compose build && docker compose up -d      # from this directory"
echo
echo "  the two host steps that cannot live in an image:"
echo "    ./middle-arm/host-setup/setup-host.sh     # udev for the U2D2"
echo "    ./slate-base/host-setup/setup-host.sh     # udev for the base; removes brltty"
echo
echo "  per-project builds still work on their own:"
echo "    cd slate-base && docker compose up -d"
echo
echo "See README.md, and docs/topic-contract.md for how the containers talk."
