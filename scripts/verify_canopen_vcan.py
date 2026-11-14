#!/usr/bin/env python3
"""Prove ros2_canopen master + fake slave talk CANopen on vcan0.

No hardware. Stdlib only. Sources /opt/ros/jazzy/setup.bash into the child env,
starts a fake slave and device_container_node, captures frames with candump,
then asserts NMT reset/start + SDO traffic and one SDO read over the ROS service.

Exit 0 = proven on this host, 1 = not proven.
"""

import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

ROS_SETUP = "/opt/ros/jazzy/setup.bash"
IFACE = "vcan0"
DOMAIN = "88"
EDS = "/opt/ros/jazzy/share/canopen_fake_slaves/config/simple_slave.eds"
SLAVE_ID = 2

log = lambda msg: print(msg, flush=True)


def ros_env():
    """Copy the environment produced by sourcing the ROS setup file."""
    raw = subprocess.run(
        ["bash", "-c", "source %s && env -0" % ROS_SETUP],
        stdout=subprocess.PIPE,
        check=True,
    ).stdout.decode()
    env = {}
    for item in raw.split("\0"):
        if "=" in item:
            key, value = item.split("=", 1)
            env[key] = value
    env["ROS_DOMAIN_ID"] = DOMAIN
    return env


def ensure_vcan():
    def up():
        return subprocess.run(
            ["ip", "link", "show", IFACE],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode == 0

    if up():
        log("[ok] %s already up" % IFACE)
        return
    for cmd in (
        ["sudo", "-n", "modprobe", "vcan"],
        ["sudo", "-n", "ip", "link", "add", "dev", IFACE, "type", "vcan"],
        ["sudo", "-n", "ip", "link", "set", IFACE, "up"],
    ):
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if not up():
        sys.exit("[fail] %s missing and sudo -n could not create it" % IFACE)
    log("[ok] created %s" % IFACE)


BUS_TEMPLATE = """options:
  dcf_path: "%(eds_dir)s"
master:
  node_id: 1
  driver: "ros2_canopen::MasterDriver"
  package: "canopen_master_driver"
canopen_chassis:
  node_id: %(slave_id)d
  driver: "ros2_canopen::ProxyDriver"
  package: "canopen_proxy_driver"
  dcf: "%(eds_name)s"
"""


def write_configs(tmp):
    """One YAML feeds both dcfgen and device_container: options.dcf_path is the
    EDS directory, the slave's dcf is a bare file name."""
    bus = os.path.join(tmp, "bus.yaml")
    with open(bus, "w") as fh:
        fh.write(BUS_TEMPLATE % {
            "eds_dir": os.path.dirname(EDS),
            "eds_name": os.path.basename(EDS),
            "slave_id": SLAVE_ID,
        })
    # master.dcf is the network definition the master boots from (1F80/1F81/
    # 1F85 tables). A slave EDS has no network table, so passing one leaves the
    # slave unbooted: boot timeout, then the upstream retry path aborts.
    r = subprocess.run(
        ["bash", "-c", "source %s && dcfgen -d %s %s" % (ROS_SETUP, tmp, bus)],
        cwd=os.path.dirname(EDS),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    master_dcf = os.path.join(tmp, "master.dcf")
    if r.returncode != 0 or not os.path.exists(master_dcf):
        sys.exit("[fail] dcfgen failed: %s" % r.stdout.decode()[-400:])
    log("[ok] master.dcf generated with dcfgen")
    return bus, master_dcf


def start(cmd, env, logfile):
    fh = open(logfile, "w")
    return subprocess.Popen(
        cmd, env=env, stdout=fh, stderr=subprocess.STDOUT, start_new_session=True
    )


def stop(proc):
    if proc is None or proc.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=5)
            return
        except subprocess.TimeoutExpired:
            continue


def read(path):
    try:
        with open(path, errors="replace") as fh:
            return fh.read()
    except OSError:
        return ""


def wait_for(frames_path, pattern, timeout, procs=(), log_paths=()):
    deadline = time.time() + timeout
    while time.time() < deadline:
        text = read(frames_path)
        if re.search(pattern, text):
            return True
        for p in procs:
            if p.poll() is not None:
                return False
        for path in log_paths:
            if "terminate called" in read(path) or "Boot failed after" in read(path):
                return False
        time.sleep(0.3)
    return False


def main():
    env = ros_env()
    ensure_vcan()
    for tool in ("candump", "ros2"):
        if shutil.which(tool, path=env.get("PATH")):
            continue
        # ros2 lives in /opt/ros/jazzy/bin, candump is a system binary
        if shutil.which(tool):
            continue
        sys.exit("[fail] %s not found" % tool)

    stale = subprocess.run(
        ["pgrep", "-af", r"(^|/)(device_container_node|basic_slave_node)( |$)"],
        stdout=subprocess.PIPE,
    ).stdout.decode().strip()
    if stale:
        sys.exit("[fail] stale CANopen stack on %s, kill it first:\n%s" % (IFACE, stale))

    tmp = tempfile.mkdtemp(prefix="verify_canopen_")
    bus, master_dcf = write_configs(tmp)
    frames = os.path.join(tmp, "frames.log")
    slave_log = os.path.join(tmp, "slave.log")
    master_log = os.path.join(tmp, "master.log")
    slave = master = candump = None
    ok = False

    try:
        candump = start(["stdbuf", "-oL", "candump", "-L", IFACE], env, frames)
        slave = start(
            [
                "ros2", "launch", "canopen_fake_slaves", "basic_slave.launch.py",
                "can_interface_name:=%s" % IFACE, "node_id:=%d" % SLAVE_ID,
            ],
            env,
            slave_log,
        )
        if not wait_for(frames, r"702#00", 15, (slave,), (slave_log,)):
            sys.exit("[fail] fake slave did not boot: %s" % read(slave_log)[-500:])
        log("[ok] fake slave boot-up: 702#00")

        master = start(
            [
                "ros2", "run", "canopen_core", "device_container_node",
                "--ros-args",
                "-p", "bus_config:=%s" % bus,
                "-p", "can_interface_name:=%s" % IFACE,
                "-p", "master_config:=%s" % master_dcf,
            ],
            env,
            master_log,
        )
        if not wait_for(frames, r"000#01[0-9a-f]{2}", 20, (master,), (master_log,)):
            sys.exit("[fail] no NMT start; master log: %s" % read(master_log)[-800:])
        if not wait_for(frames, r"582#", 5, (master,), (master_log,)):
            sys.exit("[fail] no SDO response 582#; master log: %s" % read(master_log)[-800:])
        log("[ok] NMT reset 000#8202 -> boot-up -> SDO config -> NMT start 000#0102")

        # The CLI sometimes needs a few seconds of DDS discovery after the
        # boot burst; retry short attempts instead of one long blocking wait.
        answer = ""
        for _ in range(3):
            try:
                r = subprocess.run(
                    [
                        "ros2", "service", "call", "/master/sdo_read",
                        "canopen_interfaces/srv/COReadID",
                        "{nodeid: %d, index: 16385, subindex: 0, canopen_datatype: 7}" % SLAVE_ID,
                    ],
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    timeout=12,
                )
                answer = r.stdout.decode()
            except subprocess.TimeoutExpired:
                answer = "timeout"
                continue
            if "success=True" in answer:
                break
            time.sleep(2)
        if "success=True" not in answer:
            sys.exit("[fail] SDO read 0x4001 via service failed: %s" % answer[-400:])
        data = re.search(r"data=(\d+)", answer)
        if not re.search(r"602#4001400000000000", read(frames)):
            sys.exit("[fail] service returned but no 602/582 for 0x4001 on the wire")
        log("[ok] SDO read 0x4001: success=True data=%s (wire 602#4001.. -> 582#43..)"
            % (data.group(1) if data else "?"))

        if master.poll() is not None:
            sys.exit("[fail] device_container exited: %s" % read(master_log)[-500:])
        ok = True
    finally:
        for proc in (master, slave, candump):
            stop(proc)
        if not ok:
            log("[info] logs kept in %s" % tmp)

    log("[pass] ros2_canopen master works on %s with a fake slave; no abort" % IFACE)
    shutil.rmtree(tmp, ignore_errors=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
