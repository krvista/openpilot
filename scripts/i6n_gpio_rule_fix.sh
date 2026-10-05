#!/usr/bin/env bash
# i6n Phase 47 (report §33/§34): batch AGNOS's gpio udev permission rule at boot.
#
# AGNOS 19.x ships /etc/udev/rules.d/99-gpio.rules (unchanged since 2022, still on the agnos19.8 branch):
#   SUBSYSTEM=="gpio*", PROGRAM="/bin/sh -c 'find -L /sys/class/gpio/ -maxdepth 2 -exec chown root:gpio {} \; -exec chmod 770 {} \; || true'"
# Every gpio uevent walks the whole tree and forks chown + chmod PER FILE. On this device it ran 52-126 times per boot
# from 8.8-15.7 s to 57-72 s of uptime with cores 0-5 at 97-98 % (system time 330-368 % vs 115-144 % afterwards).
# upstream agnos-builder fixed it on master ("remove udev", #592) with the same find batched by `{} +`.
#
# What this does, only when every check passes (otherwise it changes nothing):
#   1. the rule file is byte-identical to the known AGNOS rule (sha256), and is not already a mount point
#   2. writes the batched rule (fixed text, sha256 checked) to a tmpfs file; `udevadm verify` when available
#   3. bind-mounts it over the original (read-only rootfs untouched; gone on reboot), re-checks what udev will read,
#      reloads the rules; any failure -> unmount + reload (exactly today's behaviour)
#   4. sets the gpio permissions once right away (the batched find), so they never depend on the udev queue
#   5. post-check in the background after `udevadm settle`: every exported pin's value/direction must be group gpio,
#      mode 770, and the critical panda pins must be exported; a miss -> re-run the batched find; still a miss ->
#      revert the mount and run the ORIGINAL per-file find once
# Kill switch: touch /data/i6n_gpio_rule_fix_disable (or I6N_GPIO_RULE_FIX=0). Log: /data/i6n_gpio_rule_fix.log
#
# All paths / commands are overridable for phase_tests/test_phase47.py (sandbox, no root).

set -u

RULE_FILE="${RULE_FILE:-/etc/udev/rules.d/99-gpio.rules}"
RUN_DIR="${RUN_DIR:-/run/i6n-gpio}"
GPIO_ROOT="${GPIO_ROOT:-/sys/class/gpio}"
STATUS_FILE="${STATUS_FILE:-/data/i6n_gpio_rule_fix.log}"
DISABLE_FILE="${DISABLE_FILE:-/data/i6n_gpio_rule_fix_disable}"
MOUNTINFO="${MOUNTINFO:-/proc/self/mountinfo}"
GPIO_OWNER="${GPIO_OWNER:-root:gpio}"
GPIO_GROUP="${GPIO_GROUP:-gpio}"
SUDO="${SUDO-sudo}"
MOUNT="${MOUNT:-mount}"
UMOUNT="${UMOUNT:-umount}"
UDEVADM="${UDEVADM:-udevadm}"
SETTLE_TIMEOUT="${SETTLE_TIMEOUT:-90}"
POSTCHECK_SYNC="${POSTCHECK_SYNC:-0}"
# panda reset / boot pins openpilot drives (common/hardware/comma/hardware.py): SOM_ST_IO, ST_RST_N, ST_BOOT0
CRITICAL_PINS="${CRITICAL_PINS:-49 124 134}"

ORIG_SHA="d0b5ac66763335ec4f80c9320ff1bb96f183cfa7c77b97d59acde3004f998f50"
NEW_RULE='SUBSYSTEM=="gpio*", PROGRAM="/bin/sh -c '"'"'find -L /sys/class/gpio/ -maxdepth 2 -exec chown root:gpio {} + -exec chmod 770 {} + || true'"'"'"
SUBSYSTEM=="gpio", KERNEL=="gpiochip[0]", GROUP="gpio", MODE="660"'
NEW_SHA="$(printf '%s\n' "$NEW_RULE" | sha256sum | cut -d' ' -f1)"
NEW_FILE="$RUN_DIR/99-gpio.rules"

log() {
  local line
  line="$(date '+%F %T' 2>/dev/null) [$(cut -d' ' -f1 /proc/uptime 2>/dev/null)s] $*"
  echo "i6n_gpio_rule_fix: $*"
  { [ -f "$STATUS_FILE" ] && tail -n 300 "$STATUS_FILE"; echo "$line"; } > "$STATUS_FILE.tmp" 2>/dev/null \
    && mv -f "$STATUS_FILE.tmp" "$STATUS_FILE" 2>/dev/null
  return 0
}

sha_of() { sha256sum "$1" 2>/dev/null | cut -d' ' -f1; }

is_mountpoint() {
  # field 5 of mountinfo is the mount point
  awk -v p="$1" '$5 == p { found = 1 } END { exit !found }' "$MOUNTINFO" 2>/dev/null
}

batched_perms() {
  [ -d "$GPIO_ROOT" ] || return 0
  timeout 20 $SUDO find -L "$GPIO_ROOT/" -maxdepth 2 -exec chown "$GPIO_OWNER" {} + -exec chmod 770 {} + 2>/dev/null
  return 0
}

original_perms() {
  [ -d "$GPIO_ROOT" ] || return 0
  timeout 120 $SUDO find -L "$GPIO_ROOT/" -maxdepth 2 -exec chown "$GPIO_OWNER" {} \; -exec chmod 770 {} \; 2>/dev/null
  return 0
}

revert() {
  timeout 5 $SUDO "$UMOUNT" "$RULE_FILE" 2>/dev/null
  timeout 5 $SUDO "$UDEVADM" control --reload 2>/dev/null
  if is_mountpoint "$RULE_FILE"; then
    log "REVERT FAILED: $RULE_FILE still mounted ($*)"
  else
    log "reverted: $*"
  fi
}

# prints the bad entries (empty = all good)
perms_problems() {
  local p f g m bad=""
  for p in $CRITICAL_PINS; do
    [ -d "$GPIO_ROOT/gpio$p" ] || bad="$bad missing:gpio$p"
  done
  for f in "$GPIO_ROOT"/gpio[0-9]*/value "$GPIO_ROOT"/gpio[0-9]*/direction; do
    [ -e "$f" ] || continue
    g="$(stat -L -c '%G' "$f" 2>/dev/null)"; m="$(stat -L -c '%a' "$f" 2>/dev/null)"
    if [ "$g" != "$GPIO_GROUP" ] || [ "$m" != "770" ]; then
      bad="$bad ${f#"$GPIO_ROOT"/}:$g:$m"
    fi
  done
  echo "$bad"
}

postcheck() {
  local bad
  timeout $((SETTLE_TIMEOUT + 5)) $SUDO "$UDEVADM" settle --timeout="$SETTLE_TIMEOUT" 2>/dev/null \
    || log "postcheck: udevadm settle did not finish in ${SETTLE_TIMEOUT}s"
  bad="$(perms_problems)"
  if [ -z "$bad" ]; then
    log "postcheck ok"
    return 0
  fi
  log "postcheck: permission problems after settle:$bad -> batched find again"
  batched_perms
  bad="$(perms_problems)"
  if [ -z "$bad" ]; then
    log "postcheck ok after re-run"
    return 0
  fi
  log "postcheck: still bad:$bad -> revert to the AGNOS rule and run its per-file find once"
  revert "postcheck failure"
  original_perms
  bad="$(perms_problems)"
  log "postcheck after revert: ${bad:-ok}"
}

run_postcheck() {
  if [ "$POSTCHECK_SYNC" = "1" ]; then
    postcheck
  else
    ( postcheck ) >/dev/null 2>&1 &
  fi
}

main() {
  if [ "${I6N_GPIO_RULE_FIX:-1}" = "0" ] || [ -e "$DISABLE_FILE" ]; then
    log "disabled (kill switch)"
    return 0
  fi
  if [ ! -f "$RULE_FILE" ]; then
    log "skip: $RULE_FILE not present (AGNOS without the udev gpio rule)"
    return 0
  fi

  local cur
  cur="$(sha_of "$RULE_FILE")"
  if is_mountpoint "$RULE_FILE"; then
    if [ "$cur" = "$NEW_SHA" ]; then
      log "already applied this boot (re-launch)"
      batched_perms
      run_postcheck
    else
      log "skip: $RULE_FILE is a mount point with unexpected content ($cur)"
    fi
    return 0
  fi
  if [ "$cur" != "$ORIG_SHA" ]; then
    log "skip: unknown rule content ($cur) — AGNOS changed it, leaving it alone"
    return 0
  fi

  # 2. the batched rule on tmpfs
  if ! $SUDO mkdir -p "$RUN_DIR" 2>/dev/null \
     || ! printf '%s\n' "$NEW_RULE" | $SUDO tee "$NEW_FILE" >/dev/null 2>&1 \
     || ! $SUDO chmod 644 "$NEW_FILE" 2>/dev/null; then
    log "skip: could not write $NEW_FILE"
    return 0
  fi
  if [ "$(sha_of "$NEW_FILE")" != "$NEW_SHA" ]; then
    log "skip: $NEW_FILE content check failed"
    return 0
  fi
  if timeout 5 "$UDEVADM" verify --help >/dev/null 2>&1; then
    if ! timeout 10 $SUDO "$UDEVADM" verify "$NEW_FILE" >/dev/null 2>&1; then
      log "skip: udevadm verify rejected the batched rule"
      return 0
    fi
  fi

  # 3. bind-mount over the original, check what udev will read, reload
  if ! timeout 5 $SUDO "$MOUNT" --bind "$NEW_FILE" "$RULE_FILE" 2>/dev/null; then
    log "skip: bind mount failed"
    return 0
  fi
  if [ "$(sha_of "$RULE_FILE")" != "$NEW_SHA" ]; then
    revert "content through the mount is not the batched rule"
    return 0
  fi
  if ! timeout 5 $SUDO "$UDEVADM" control --reload 2>/dev/null; then
    revert "udevadm control --reload failed"
    return 0
  fi
  log "applied: batched gpio rule bind-mounted over $RULE_FILE"

  # 4. permissions now, independent of the udev queue
  batched_perms
  # 5. verify after the udev queue drains
  run_postcheck
  return 0
}

main "$@"
exit 0
