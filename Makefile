# =============================================================================
# The whole rig, from ~/Trossen.
#
#   make            build anything missing, then start everything
#   make up         start everything (no build)
#   make down       stop everything
#   make status     what is running, and every topic with its rate
#   make help       all targets
#
# Wraps docker compose so there is one place to run things from. Running compose
# by hand still works; this exists so that "start the rig" is one word and so the
# common mistakes are impossible to make -- in particular it always runs from the
# repo root, which is what stops the container-name conflicts you get by running
# `up` inside a subdirectory.
#
# DOCKER=sudo docker  if your user is not in the docker group:
#   make up DOCKER="sudo docker"
# =============================================================================

DOCKER  ?= docker
COMPOSE := $(DOCKER) compose
SVC     ?=

# Every recipe runs from the directory holding this Makefile, whatever the
# caller's cwd. Compose resolves include: paths relative to the root file, so
# this is what keeps one project name and one set of container names.
MAKEFILE_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

.DEFAULT_GOAL := all
.PHONY: all up build rebuild down restart status ps topics logs watch \
        drive-test drive torque arms arm-go arm-stop key jog tmux shell env check dash \
        clean fresh help home start save-pose sim rig-urdf kill orphans

## all: build what is missing, then start everything
all: build up

## up: start every container; all nodes and topics come up with them
up:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) up -d $(SVC)
	@echo
	@echo "  up. Every topic should be live -- check with:  make status"

## build: build images (only what changed)
build:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) build $(SVC)

## rebuild: force a rebuild and replace the containers
rebuild:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) up -d --build --force-recreate $(SVC)

## down: stop and remove every container
down:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) down

## restart: restart nodes after editing a script in a workspace/ folder
restart:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) restart $(SVC)

## status: containers, then one line per subsystem
status: ps dash

## dash: one line per subsystem -- the readable summary
dash:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor bash -lc './watch.py --dash --once' 

## ps: which containers are up
ps:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) ps --format 'table {{.Name}}\t{{.Status}}'

## topics: one snapshot of every topic, its rate and latest value
topics:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor bash -lc './watch.py --once'

## watch: live topic table, updating until Ctrl-C
watch:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec monitor ./watch.py

## logs: follow container output   (make logs SVC=slate-base)
logs:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) logs -f $(SVC)

## drive-test: DRY RUN of the base velocity command -- moves nothing
drive-test:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor bash -lc './drive_test.py'

## drive: ACTUALLY drive the base forward 3 s, then stop
drive:
	@echo "  This MOVES THE BASE. Ctrl-C within 2 s to abort."
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor bash -lc './drive_test.py --execute'

## arms: where every arm is, and what poses it knows
arms:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor ./arm_ctl.py state
	@echo "  poses:"
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor ./arm_ctl.py list

## arm-go: move an arm to a named pose   (make arm-go ARM=/left_arm POSE=ready)
arm-go:
	@test -n "$(ARM)" -a -n "$(POSE)" || { echo "usage: make arm-go ARM=/left_arm POSE=ready [EXECUTE=1]"; exit 2; }
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor \
	  ./arm_ctl.py go $(POSE) --arm $(ARM) $(if $(EXECUTE),--execute,)

## home: send arms to their saved 'home'  (make home) -- add EXECUTE=1 to move
# No ARM= sends EVERY arm at once. Joint-space, so this is also the recovery
# that works at a singularity, where every Cartesian command is refused.
home:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor \
	  ./arm_ctl.py go home $(if $(ARM),--arm $(ARM),) $(if $(EXECUTE),--execute,)

## start: send arms to their saved 'start' pose   (make start EXECUTE=1)
start:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor \
	  ./arm_ctl.py go start $(if $(ARM),--arm $(ARM),) $(if $(EXECUTE),--execute,)

## save-pose: record where the arms are NOW under a name (NAME=start EXECUTE=1)
# Goes through the running agent -- pose.py cannot, because the agent holds the
# arm's only connection while the rig is up.
save-pose:
	@test -n "$(NAME)" || { echo "usage: make save-pose NAME=start [ARM=/left_arm] [EXECUTE=1]"; exit 2; }
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor \
	  ./arm_ctl.py save $(NAME) $(if $(ARM),--arm $(ARM),) $(if $(EXECUTE),--execute,)

## rig-urdf: rebuild the combined model from sim/rig_params.yaml
# Vendor URDFs + this rig's own geometry -> sim/description/urdf/rig.urdf.
# Edit sim/rig_params.yaml, run this, restart sim. Never hand-edit rig.urdf.
rig-urdf:
	@cd $(MAKEFILE_DIR) && ./tools/build_rig_urdf.py
	@cd $(MAKEFILE_DIR) && $(COMPOSE) restart sim >/dev/null 2>&1 || true

## sim: URL for the 3D view of the rig, drawn live from its own topics
# Read-only: it subscribes to everything and publishes nothing, so it is safe
# to leave open while driving. Autostarts with the rig.
sim:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) ps --services --filter status=running \
	  | grep -qx sim || { echo "  sim is not running:  make up"; exit 1; }
	@echo "  http://localhost:$(or $(RIG_SIM_PORT),8080)"
	@echo "  from another machine:  http://$$(hostname -I | awk '{print $$1}'):$(or $(RIG_SIM_PORT),8080)"

## kill: end the tmux session AND the processes it left in the containers
# `tmux kill-session` alone is not enough: docker compose exec does not kill the
# process inside the container when the client dies, so watch.py and rig_key.py
# keep running and keep publishing to the command topics. tmux-rig.sh installs a
# watcher that handles this automatically; this target is the explicit version,
# and the one to reach for if a session was killed some other way.
kill:
	@cd $(MAKEFILE_DIR) && tmux kill-session -t $(or $(SESSION),rig) 2>/dev/null || true
	@cd $(MAKEFILE_DIR) && ./rig-cleanup.sh --force

## orphans: list container-side control processes without killing anything
orphans:
	@cd $(MAKEFILE_DIR) && ./rig-cleanup.sh --dry-run --force

## tmux: one tmux session, one pane per live device
tmux:
	@cd $(MAKEFILE_DIR) && DOCKER="$(DOCKER)" ./tmux-rig.sh

## key: unified keyboard control -- all three arms, base and lift at once
key:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec monitor ./rig_key.py

## jog: alias for `key`, kept for muscle memory
jog: key

## arm-stop: release every arm
arm-stop:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T monitor ./arm_ctl.py stop

## torque: enable the base's drive motors (it ignores velocity commands without this)
torque:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec -T slate-base bash -lc './base_ctl.py torque on'

## shell: interactive shell in a container   (make shell SVC=monitor)
shell:
	@test -n "$(SVC)" || { echo "usage: make shell SVC=<service>"; exit 2; }
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec $(SVC) bash

## env: per-machine bootstrap -- writes .env with this machine's UID/GID
env:
	@cd $(MAKEFILE_DIR) && ./setup.sh

## check: things that are wrong more often than they should be
check:
	@cd $(MAKEFILE_DIR) && \
	  echo "== .env files (a missing one builds the wrong UID and DDS silently stops)" && \
	  for d in */docker-compose.yml; do p=$$(dirname $$d); \
	    if [ -f "$$p/.env" ]; then echo "   ok   $$p/.env  $$(tr '\n' ' ' < $$p/.env)"; \
	    else echo "   MISSING $$p/.env  -- run: make env"; fi; done && \
	  echo "== container UIDs (must all match, or Fast DDS cannot share memory)" && \
	  for c in $$($(COMPOSE) ps --services 2>/dev/null); do \
	    id=$$($(DOCKER) exec $$c id -u 2>/dev/null || echo "not running"); \
	    echo "   $$c: $$id"; done && \
	  echo "== ipc namespace (must be host, or topics appear but carry no data)" && \
	  for c in $$($(COMPOSE) ps --services 2>/dev/null); do \
	    m=$$($(DOCKER) inspect $$c --format '{{.HostConfig.IpcMode}}' 2>/dev/null || echo "not running"); \
	    echo "   $$c: $$m"; done

## clean: stop and remove EVERY container and image for this rig
clean:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) down --rmi local --remove-orphans
	@echo "  removing any container left behind by a per-directory project ..."
	@for c in left-arm right-arm middle-arm slate-base quest monitor; do \
	  $(DOCKER) rm -f $$c >/dev/null 2>&1 && echo "    removed stray $$c" || true; \
	done
	@echo "  clean. `make` will rebuild from the cached image layers;"
	@echo "  `make fresh` rebuilds ignoring the cache."

## fresh: clean, rebuild from scratch ignoring the cache, and start
fresh: clean
	@cd $(MAKEFILE_DIR) && ./setup.sh
	@cd $(MAKEFILE_DIR) && $(COMPOSE) build --no-cache
	@cd $(MAKEFILE_DIR) && $(COMPOSE) up -d
	@echo
	@echo "  fresh. Check with:  make status"

help:
	@echo "Trossen rig -- run from $(MAKEFILE_DIR)"
	@echo
	@grep -E '^## ' $(MAKEFILE_LIST) | sed 's/^## /  /'
	@echo
	@echo "  ARM=/left_arm POSE=ready EXECUTE=1   for arm-go"
	@echo "  SVC=<service>       target one service: left-arm right-arm middle-arm slate-base quest monitor"
	@echo "  DOCKER='sudo docker'  if you are not in the docker group"
