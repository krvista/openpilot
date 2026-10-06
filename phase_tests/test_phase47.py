"""Phase 47 (report §34): scripts/i6n_gpio_rule_fix.sh — batch AGNOS 19.x's per-file gpio udev rule, fail-safe.

Every test runs the real script in a sandbox: a fake rule file / mountinfo / sysfs gpio tree under tmp_path, and stub
mount / umount / udevadm that only record their calls (the "bind mount" stub copies the file over, umount restores
it). Nothing outside tmp_path is touched and no root is needed.
"""
import hashlib
import os
import stat
import subprocess
import textwrap

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(os.path.dirname(HERE), "scripts", "i6n_gpio_rule_fix.sh")

ORIG = ("SUBSYSTEM==\"gpio*\", PROGRAM=\"/bin/sh -c 'find -L /sys/class/gpio/ -maxdepth 2 -exec chown root:gpio {} \\; "
        "-exec chmod 770 {} \\; || true'\"\n"
        "SUBSYSTEM==\"gpio\", KERNEL==\"gpiochip[0]\", GROUP=\"gpio\", MODE=\"660\"\n")
ORIG_SHA = "d0b5ac66763335ec4f80c9320ff1bb96f183cfa7c77b97d59acde3004f998f50"   # agnos-builder 99-gpio.rules 2022..19.8
BATCHED = ORIG.replace("{} \\;", "{} +")


def _stub(path, body):
  path.write_text("#!/usr/bin/env bash\n" + textwrap.dedent(body))
  path.chmod(path.stat().st_mode | stat.S_IXUSR)
  return str(path)


class Box:
  """Sandbox: fake /etc rule, /proc mountinfo, /sys/class/gpio, stub commands."""
  def __init__(self, tmp, rule=ORIG, pins=(49, 124, 134, 30), mount_ok=True, mount_copies=True, reload_ok=True,
               verify="absent", bad_perm_after_settle=False):
    self.tmp = tmp
    self.calls = tmp / "calls.log"
    self.rule = tmp / "etc" / "99-gpio.rules"; self.rule.parent.mkdir()
    self.rule.write_text(rule)
    self.backup = tmp / "rule.backup"
    self.mountinfo = tmp / "mountinfo"; self.mountinfo.write_text("22 1 0:20 / / ro - ext4 /dev/root ro\n")
    self.gpio = tmp / "gpio"; self.gpio.mkdir()
    for p in pins:
      d = self.gpio / f"gpio{p}"; d.mkdir()
      for f in ("value", "direction", "edge"):
        (d / f).write_text("0"); (d / f).chmod(0o600)
    (self.gpio / "export").write_text(""); (self.gpio / "unexport").write_text("")
    self.status = tmp / "status.log"
    b = tmp / "bin"; b.mkdir()
    mi = self.mountinfo
    # bind mount emulation: keep the original aside, write the source over the target
    self.mount = _stub(b / "mount", f"""
      echo "mount $*" >> {self.calls}
      {'exit 1' if not mount_ok else ''}
      cp "$3" {self.backup}
      {'cp "$2" "$3"' if mount_copies else ''}
      echo "99 22 0:30 / $3 rw - tmpfs tmpfs rw" >> {mi}
      """)
    self.umount = _stub(b / "umount", f"""
      echo "umount $*" >> {self.calls}
      cp {self.backup} "$1"
      grep -v " $1 " {mi} > {mi}.tmp; mv {mi}.tmp {mi}
      """)
    breaker = f'chmod 600 {self.gpio}/gpio124/value' if bad_perm_after_settle else 'true'
    verify_case = {"absent": 'exit 1', "ok": 'exit 0', "reject": '[ "$2" = "--help" ] && exit 0; exit 1'}[verify]
    self.udevadm = _stub(b / "udevadm", f"""
      echo "udevadm $*" >> {self.calls}
      case "$1" in
        verify) {verify_case} ;;
        control) {'exit 0' if reload_ok else 'exit 1'} ;;
        settle) {breaker}; exit 0 ;;
      esac
      """)

  def run(self, **extra):
    me = f"{os.getuid()}:{os.getgid()}"
    import grp
    env = dict(os.environ, RULE_FILE=str(self.rule), RUN_DIR=str(self.tmp / "run"), GPIO_ROOT=str(self.gpio),
               STATUS_FILE=str(self.status), DISABLE_FILE=str(self.tmp / "disable"), MOUNTINFO=str(self.mountinfo),
               GPIO_OWNER=me, GPIO_GROUP=grp.getgrgid(os.getgid()).gr_name, SUDO="", MOUNT=self.mount,
               UMOUNT=self.umount, UDEVADM=self.udevadm, SETTLE_TIMEOUT="1", POSTCHECK_SYNC="1")
    cmd = extra.pop("_cmd", ["bash", SCRIPT])
    env.update(extra)
    r = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=60)
    assert r.returncode == 0, r.stderr        # never fails the launch script
    return r

  def log(self):
    return self.calls.read_text() if self.calls.exists() else ""

  def status_text(self):
    return self.status.read_text() if self.status.exists() else ""

  def modes(self):
    return {f"{p.parent.name}/{p.name}": oct(p.stat().st_mode & 0o777) for p in self.gpio.glob("gpio*/*")}


def test_known_rule_text_and_hash():
  assert hashlib.sha256(ORIG.encode()).hexdigest() == ORIG_SHA
  src = open(SCRIPT).read()
  assert ORIG_SHA in src


def test_applies_batched_rule_and_sets_perms(tmp_path):
  b = Box(tmp_path)
  b.run()
  assert b.rule.read_text() == BATCHED                          # what udev now reads
  assert "mount --bind" in b.log() and "udevadm control --reload" in b.log()
  assert all(m == "0o770" for m in b.modes().values())          # step 4: perms set at once
  assert "applied" in b.status_text() and "postcheck ok" in b.status_text()
  assert "umount" not in b.log()


def test_batched_rule_differs_only_in_batching(tmp_path):
  b = Box(tmp_path)
  b.run()
  diff = [(o, n) for o, n in zip(ORIG.splitlines(), b.rule.read_text().splitlines()) if o != n]
  assert len(diff) == 1 and diff[0][0].replace("{} \\;", "{} +") == diff[0][1]


@pytest.mark.parametrize("rule", ["", ORIG.replace("770", "775"), ORIG + "# local edit\n", BATCHED])
def test_unknown_content_is_left_alone(tmp_path, rule):
  b = Box(tmp_path, rule=rule)
  before = b.modes()
  b.run()
  assert b.rule.read_text() == rule and "mount" not in b.log() and "control" not in b.log()
  assert b.modes() == before                                    # no permission changes either
  assert "unknown rule content" in b.status_text()


def test_missing_rule_file(tmp_path):
  b = Box(tmp_path); b.rule.unlink()
  b.run()
  assert b.log() == "" and "not present" in b.status_text()


@pytest.mark.parametrize("how", ["file", "env"])
def test_kill_switch(tmp_path, how):
  b = Box(tmp_path)
  if how == "file":
    (tmp_path / "disable").write_text("")
    b.run()
  else:
    b.run(I6N_GPIO_RULE_FIX="0")
  assert b.rule.read_text() == ORIG and b.log() == "" and "kill switch" in b.status_text()


def test_mount_failure_changes_nothing(tmp_path):
  b = Box(tmp_path, mount_ok=False)
  before = b.modes()
  b.run()
  assert b.rule.read_text() == ORIG and "control" not in b.log() and b.modes() == before
  assert "bind mount failed" in b.status_text()


def test_mount_without_effect_is_reverted(tmp_path):
  """The mount 'succeeds' but udev would still read the original: unmount, reload, nothing else."""
  b = Box(tmp_path, mount_copies=False)
  b.run()
  assert "umount" in b.log() and b.rule.read_text() == ORIG
  assert "reverted" in b.status_text()


def test_reload_failure_is_reverted(tmp_path):
  b = Box(tmp_path, reload_ok=False)
  b.run()
  assert "umount" in b.log() and b.rule.read_text() == ORIG and "reverted" in b.status_text()


def test_verify_rejection_stops_before_mount(tmp_path):
  b = Box(tmp_path, verify="reject")
  b.run()
  assert "mount" not in b.log() and "verify rejected" in b.status_text()


def test_verify_ok_then_applies(tmp_path):
  b = Box(tmp_path, verify="ok")
  b.run()
  assert "udevadm verify" in b.log() and b.rule.read_text() == BATCHED


def test_postcheck_repairs_permissions(tmp_path):
  """A pin loses its permission while the udev queue drains: the batched find runs again and fixes it."""
  b = Box(tmp_path, bad_perm_after_settle=True)
  b.run()
  assert "permission problems" in b.status_text() and "postcheck ok after re-run" in b.status_text()
  assert all(m == "0o770" for m in b.modes().values())
  assert "umount" not in b.log()


def test_missing_critical_pin_reverts_to_agnos_rule(tmp_path):
  """ST_RST_N (124) never exported: the batched rule is not trusted — revert and run the original find once."""
  b = Box(tmp_path, pins=(49, 134, 30))
  b.run()
  assert "missing:gpio124" in b.status_text() and "umount" in b.log()
  assert b.rule.read_text() == ORIG and "reverted" in b.status_text()
  assert all(m == "0o770" for m in b.modes().values())         # the original per-file find still ran


def test_relaunch_same_boot_is_idempotent(tmp_path):
  b = Box(tmp_path)
  b.run()
  n_mounts = b.log().count("mount --bind")
  b.run()
  assert b.log().count("mount --bind") == n_mounts and "already applied" in b.status_text()


def test_status_log_is_bounded(tmp_path):
  b = Box(tmp_path)
  b.status.write_text("x\n" * 1000)
  b.run(I6N_GPIO_RULE_FIX="0")
  assert len(b.status.read_text().splitlines()) <= 302


def test_script_is_executable_and_bash_clean():
  assert os.access(SCRIPT, os.X_OK)
  assert subprocess.run(["bash", "-n", SCRIPT]).returncode == 0


def test_background_postcheck_does_not_hold_the_launch(tmp_path):
  """On the device the post-check waits for `udevadm settle` (up to 90 s) in the background; the launch must not."""
  import time
  b = Box(tmp_path)
  slow = _stub(tmp_path / "bin" / "udevadm_slow", f"""
    echo "udevadm $*" >> {b.calls}
    case "$1" in verify) exit 1 ;; control) exit 0 ;; settle) sleep 3; exit 0 ;; esac
    """)
  t = time.monotonic()
  b.run(UDEVADM=slow, POSTCHECK_SYNC="0", SETTLE_TIMEOUT="5")
  assert time.monotonic() - t < 2.5 and b.rule.read_text() == BATCHED


LAUNCH = os.path.join(os.path.dirname(HERE), "launch_chffrplus.sh")


def _launch_line():
  src = open(LAUNCH).read()
  body = src[src.index("function agnos_init {"):src.index("function launch {")]
  lines = [ln.strip() for ln in body.splitlines() if "i6n_gpio_rule_fix.sh" in ln and not ln.strip().startswith("#")]
  assert len(lines) == 1, lines
  # after the AGNOS update check (runs only when /VERSION matches), bounded, and can never stop the launch
  assert body.index(lines[0]) > body.index('$AGNOS_VERSION')
  assert lines[0] == 'timeout 60 "$DIR/scripts/i6n_gpio_rule_fix.sh" || true'
  return lines[0]


def test_launch_line_is_wired_in_agnos_init():
  _launch_line()


def test_launch_line_returns_fast_and_postcheck_survives_timeout(tmp_path):
  """The exact launch line: returns at once with status 0, and the background post-check is not killed by `timeout`."""
  import time
  b = Box(tmp_path)
  slow = _stub(tmp_path / "bin" / "udevadm_slow", f"""
    echo "udevadm $*" >> {b.calls}
    case "$1" in verify) exit 1 ;; control) exit 0 ;; settle) sleep 2; exit 0 ;; esac
    """)
  root = tmp_path / "root"; (root / "scripts").mkdir(parents=True)
  os.symlink(SCRIPT, root / "scripts" / "i6n_gpio_rule_fix.sh")
  t = time.monotonic()
  b.run(UDEVADM=slow, POSTCHECK_SYNC="0", SETTLE_TIMEOUT="5", _cmd=["bash", "-c", f'DIR={root}; {_launch_line()}; echo "rc=$?"'])
  assert time.monotonic() - t < 2.5 and b.rule.read_text() == BATCHED
  for _ in range(60):
    if "postcheck ok" in b.status_text():
      break
    time.sleep(0.1)
  assert "postcheck ok" in b.status_text()


def test_launch_line_tolerates_missing_script(tmp_path):
  r = subprocess.run(["bash", "-c", f'DIR={tmp_path}; {_launch_line()}; echo "rc=$?"'], capture_output=True, text=True, timeout=10)
  assert r.stdout.strip().endswith("rc=0")
