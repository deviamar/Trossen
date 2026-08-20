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
        drive-test drive torque arms arm-go arm-stop jog shell env check \
        clean fresh help

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

## status: containers, then every topic with its rate and latest value
status: ps topics

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

## jog: keyboard Cartesian jog   (make jog ARM=/left_arm)
jog:
	@cd $(MAKEFILE_DIR) && $(COMPOSE) exec monitor ./arm_key.py --arm $(or $(ARM),/left_arm)

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
