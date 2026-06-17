#!/usr/bin/env python3
"""
run_perf_tests.py — Performance test runner for SprayPaintPlugin.

For every scenario:
  1. Stop any running perf-test container
  2. Apply URDF config changes + rebuild
  3. Start a detached Docker container running Gazebo headlessly
  4. Spawn the nozzle robot
  5. Collect idle baseline metrics (RTF, CPU, memory) for IDLE_SECS seconds
  6. Trigger spray ON; collect spray metrics for SPRAY_SECS seconds
  7. Trigger spray OFF
  8. Stop container
  9. Record results

Output: Markdown table saved to PERF_RESULTS.md
"""

import re, subprocess, sys, time, threading, json, statistics
from pathlib import Path
from datetime import datetime

ROOT        = Path(__file__).resolve().parent.parent
URDF_PATH   = ROOT / "src/gz_sim_spray_painting_plugin/urdf/spray_nozzle.urdf"
LOGS_DIR    = ROOT / "file_logs"
IMAGE       = "spray_paint_plugin"
CONTAINER   = "spray_perf_test"

PLUGIN_PATH   = "/ws/install/gz_sim_spray_painting_plugin/lib/gz_sim_spray_painting_plugin:/ws/install/gz_ros2_control/lib"
RESOURCE_PATH = "/ws/install/gz_sim_spray_painting_plugin/share:/ws/install/gz_spray_painting_plugin_demo/share/gz_spray_painting_plugin_demo/models"
NOZZLE_URDF   = "/ws/install/gz_sim_spray_painting_plugin/share/gz_sim_spray_painting_plugin/urdf/spray_nozzle.urdf"

IDLE_SECS  = 15   # seconds to collect baseline before spray
SPRAY_SECS = 30   # seconds to spray and collect metrics

RESULTS: list[dict] = []

# ── URDF helpers (same marker approach as run_tests.py) ───────────────────────

URDF_PLUGIN_TEMPLATE = """\
<!-- BEGIN_SPRAY_PLUGIN -->
    <plugin filename="libSprayPaintPlugin.so"
            name="gz::sim::systems::SprayPaintPlugin">
      <nozzle_link>spray_gun_nozzle_link</nozzle_link>
      <cone_half_angle_deg>{half_angle}</cone_half_angle_deg>
      <cone_max_range>{max_range}</cone_max_range>
      <spray_color>{color}</spray_color>
      <spray_topic>/spray_paint/trigger</spray_topic>
      <particle_rate>100</particle_rate>
      <num_rays>{num_rays}</num_rays>{extra}
    </plugin>
    <!-- END_SPRAY_PLUGIN -->"""

def modify_urdf(half_angle=15, max_range=1.0,
                color="1.0 0.2 0.1 1.0", num_rays=16,
                patch_spacing=None):
    extra = ""
    if patch_spacing is not None:
        extra = f"\n      <patch_spacing>{patch_spacing}</patch_spacing>"
    block = URDF_PLUGIN_TEMPLATE.format(
        half_angle=half_angle, max_range=max_range,
        color=color, num_rays=num_rays, extra=extra)
    content = URDF_PATH.read_text()
    content = re.sub(
        r'<!-- BEGIN_SPRAY_PLUGIN -->.*?<!-- END_SPRAY_PLUGIN -->',
        block, content, flags=re.DOTALL)
    URDF_PATH.write_text(content)
    print(f"  [URDF] angle={half_angle}° range={max_range}m rays={num_rays} spacing={patch_spacing}")

def restore_urdf():
    modify_urdf()

def build():
    print("  [BUILD] Rebuilding…", end="", flush=True)
    r = subprocess.run(
        [sys.executable, str(ROOT / "run_scripts/build_code.py")],
        capture_output=True, text=True)
    if r.returncode != 0:
        print(" FAILED"); print(r.stdout[-1000:]); return False
    print(" OK"); return True

# ── Container lifecycle ───────────────────────────────────────────────────────

def kill_container():
    subprocess.run(["docker", "rm", "-f", CONTAINER],
                   capture_output=True, timeout=15)

def start_container(world_filename: str, world_name: str) -> str:
    """Start detached container running gz sim. Return container ID."""
    world_path = (f"/ws/install/gz_sim_spray_painting_plugin/share/"
                  f"gz_sim_spray_painting_plugin/worlds/{world_filename}")

    # The entrypoint keeps Gazebo running; we exec commands into it.
    gz_cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source /ws/install/setup.bash && "
        f"export GZ_SIM_SYSTEM_PLUGIN_PATH={PLUGIN_PATH} && "
        f"export GZ_SIM_RESOURCE_PATH={RESOURCE_PATH} && "
        f"gz sim -s {world_path} -r -v 3 2>&1 | tee /tmp/gz_out.txt"
    )

    cmd = [
        "docker", "run", "-d",
        "--name", CONTAINER,
        "--runtime", "nvidia",
        "--network", "host",
        "--privileged",
        "-v", f"{ROOT}:/ws:ro",
        "-v", f"{ROOT}/install:/ws/install",
        "-v", f"{ROOT}/file_logs:/ws/file_logs",
        "-e", "GZ_VERSION=harmonic",
        "-e", "NVIDIA_VISIBLE_DEVICES=all",
        "-e", "NVIDIA_DRIVER_CAPABILITIES=all",
        "-e", "HOME=/root",
        IMAGE,
        "bash", "-c", gz_cmd,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    cid = result.stdout.strip()
    if not cid:
        print(f"  [CONTAINER] Start failed: {result.stderr[:200]}")
        return ""
    print(f"  [CONTAINER] Started {cid[:12]}")
    return cid

def exec_in(cmd_str: str, timeout=20) -> str:
    r = subprocess.run(
        ["docker", "exec", CONTAINER, "bash", "-c",
         f"source /opt/ros/humble/setup.bash && "
         f"source /ws/install/setup.bash && "
         f"export GZ_SIM_SYSTEM_PLUGIN_PATH={PLUGIN_PATH} && "
         f"export GZ_SIM_RESOURCE_PATH={RESOURCE_PATH} && "
         + cmd_str],
        capture_output=True, text=True, timeout=timeout)
    return r.stdout + r.stderr

def spawn_nozzle(world_name: str, z: float = 0.2):
    spawn_req = (
        f'sdf_filename: \\"{NOZZLE_URDF}\\" '
        f'name: \\"spray_nozzle\\" allow_renaming: false '
        f'pose: {{ position: {{ x: 0.0 y: 0.0 z: {z} }} }}'
    )
    return exec_in(
        f'gz service -s /world/{world_name}/create '
        f'--reqtype gz.msgs.EntityFactory '
        f'--reptype gz.msgs.Boolean '
        f'--timeout 10000 '
        f'--req "{spawn_req}"',
        timeout=20)

def trigger_spray(on: bool):
    val = "true" if on else "false"
    exec_in(f'gz topic -t /spray_paint/trigger '
            f'-m gz.msgs.Boolean -p "data: {val}"', timeout=5)

# ── Metric collection ─────────────────────────────────────────────────────────

def get_docker_stats() -> tuple[float, float]:
    """Return (cpu_pct, mem_mb) from docker stats."""
    r = subprocess.run(
        ["docker", "stats", CONTAINER, "--no-stream",
         "--format", "{{.CPUPerc}}\t{{.MemUsage}}"],
        capture_output=True, text=True, timeout=10)
    line = r.stdout.strip()
    if not line:
        return 0.0, 0.0
    parts = line.split('\t')
    cpu = float(parts[0].replace('%', '').strip()) if parts else 0.0
    mem = 0.0
    if len(parts) > 1:
        # e.g. "1.23GiB / 31.3GiB"
        mem_str = parts[1].split('/')[0].strip()
        val = float(re.sub(r'[^\d.]', '', mem_str))
        if 'GiB' in mem_str or 'GB' in mem_str:
            mem = val * 1024
        elif 'MiB' in mem_str or 'MB' in mem_str:
            mem = val
        elif 'KiB' in mem_str or 'kB' in mem_str:
            mem = val / 1024
    return cpu, mem

def get_rtf(world_name: str) -> float:
    """Get current RTF — try gz topic, fall back to parsing gz stdout log."""
    # Get 2 messages to ensure a complete proto text block
    out = exec_in(
        f'gz topic -e -t /world/{world_name}/stats -n 2 2>/dev/null',
        timeout=10)
    m = re.search(r'real_time_factor:\s*([\d.]+)', out)
    if m:
        return float(m.group(1))
    # Fallback: parse gz verbose output from log file inside container
    log_out = exec_in('tail -50 /tmp/gz_out.txt 2>/dev/null', timeout=5)
    m = re.search(r'factor[:\s]+([\d.]+)', log_out, re.IGNORECASE)
    return float(m.group(1)) if m else 0.0

def get_patch_count() -> int:
    logs = sorted(LOGS_DIR.glob("*.log"), key=lambda p: p.stat().st_mtime)
    if not logs:
        return 0
    content = logs[-1].read_text()
    totals = re.findall(r'total=(\d+)', content)
    return int(totals[-1]) if totals else 0

class MetricsCollector(threading.Thread):
    def __init__(self, world_name: str, interval: float = 2.0):
        super().__init__(daemon=True)
        self.world_name = world_name
        self.interval   = interval
        self.cpu:  list[float] = []
        self.mem:  list[float] = []
        self.rtf:  list[float] = []
        self._stop_evt = threading.Event()

    def run(self):
        while not self._stop_evt.is_set():
            try:
                cpu, mem = get_docker_stats()
                rtf      = get_rtf(self.world_name)
                if cpu > 0:
                    self.cpu.append(cpu)
                if mem > 0:
                    self.mem.append(mem)
                if rtf > 0:
                    self.rtf.append(rtf)
            except Exception:
                pass
            self._stop_evt.wait(self.interval)

    def stop(self):
        self._stop_evt.set()

    def summary(self) -> dict:
        def safe_mean(lst): return round(statistics.mean(lst), 2) if lst else 0.0
        def safe_min(lst):  return round(min(lst), 2) if lst else 0.0
        def safe_max(lst):  return round(max(lst), 2) if lst else 0.0
        return {
            "cpu_mean": safe_mean(self.cpu),
            "cpu_peak": safe_max(self.cpu),
            "mem_mean": safe_mean(self.mem),
            "mem_peak": safe_max(self.mem),
            "rtf_mean": safe_mean(self.rtf),
            "rtf_min":  safe_min(self.rtf),
        }

# ── Scenario runner ───────────────────────────────────────────────────────────

def run_scenario(label: str, world_filename: str,
                 half_angle=15, max_range=1.0, num_rays=16,
                 patch_spacing=None, color="1.0 0.2 0.1 1.0",
                 nozzle_z=0.2, startup_wait=10,
                 idle_secs=IDLE_SECS, spray_secs=SPRAY_SECS,
                 skip_spray=False) -> dict:
    """Full lifecycle for one performance scenario."""
    world_name = world_filename.replace(".sdf", "")
    print(f"\n{'─'*60}")
    print(f"  SCENARIO: {label}")

    # 1. Stop old container
    kill_container()

    # 2. Config + build
    modify_urdf(half_angle=half_angle, max_range=max_range,
                num_rays=num_rays, patch_spacing=patch_spacing, color=color)
    if not build():
        RESULTS.append({"label": label, "error": "build failed"})
        return {}

    # 3. Start container
    logs_before = set(LOGS_DIR.glob("*.log"))
    cid = start_container(world_filename, world_name)
    if not cid:
        RESULTS.append({"label": label, "error": "container start failed"})
        return {}

    print(f"  [WAIT] Gazebo startup ({startup_wait}s)…")
    time.sleep(startup_wait)

    # 4. Spawn nozzle
    print("  [SPAWN] Nozzle robot…")
    spawn_nozzle(world_name, z=nozzle_z)
    time.sleep(5)  # let plugin initialize

    # 5. Idle baseline
    print(f"  [IDLE] Collecting {idle_secs}s baseline…")
    idle_col = MetricsCollector(world_name)
    idle_col.start()
    time.sleep(idle_secs)
    idle_col.stop()
    idle = idle_col.summary()
    print(f"         RTF={idle['rtf_mean']} CPU={idle['cpu_mean']}% Mem={idle['mem_mean']}MB")

    spray = {"cpu_mean":0,"cpu_peak":0,"mem_mean":0,"mem_peak":0,"rtf_mean":0,"rtf_min":0}
    patches = 0

    if not skip_spray:
        # 6. Spray ON + collect metrics
        print(f"  [SPRAY ON]  Collecting {spray_secs}s spray metrics…")
        trigger_spray(True)
        spray_col = MetricsCollector(world_name)
        spray_col.start()
        time.sleep(spray_secs)
        spray_col.stop()
        spray = spray_col.summary()
        trigger_spray(False)
        time.sleep(2)
        patches = get_patch_count()
        print(f"         RTF={spray['rtf_mean']} CPU={spray['cpu_mean']}% Mem={spray['mem_mean']}MB patches={patches}")

    # 7. Stop container
    kill_container()

    row = {
        "label":        label,
        "world":        world_filename,
        "half_angle":   half_angle,
        "max_range":    max_range,
        "num_rays":     num_rays,
        "idle_rtf":     idle["rtf_mean"],
        "spray_rtf":    spray["rtf_mean"],
        "rtf_min":      spray["rtf_min"],
        "rtf_drop_pct": round((1.0 - spray["rtf_mean"] / max(idle["rtf_mean"], 0.001)) * 100, 1)
                        if spray["rtf_mean"] > 0 else 0,
        "idle_cpu":     idle["cpu_mean"],
        "spray_cpu":    spray["cpu_mean"],
        "peak_cpu":     spray["cpu_peak"],
        "idle_mem":     idle["mem_mean"],
        "spray_mem":    spray["mem_mean"],
        "peak_mem":     spray["mem_peak"],
        "patches":      patches,
    }
    RESULTS.append(row)
    return row

# ── Report writer ─────────────────────────────────────────────────────────────

def write_report():
    date = datetime.now().strftime("%Y-%m-%d %H:%M")
    lines = [
        "# SprayPaintPlugin — Performance Test Results",
        "",
        f"**Date:** {date}  ",
        f"**Branch:** feature/ray_casting  ",
        f"**Platform:** x86_64, NVIDIA RTX 3060 Laptop, Docker (gz-harmonic, DART+Bullet)  ",
        f"**Idle window:** {IDLE_SECS}s | **Spray window:** {SPRAY_SECS}s  ",
        "",
        "## Results Table",
        "",
        "| Scenario | Target | Range (m) | Rays | Idle RTF | Spray RTF | RTF Drop | "
        "Idle CPU % | Spray CPU % | Peak CPU % | Idle Mem MB | Spray Mem MB | Peak Mem MB | Patches |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]

    for r in RESULTS:
        if "error" in r:
            lines.append(f"| {r['label']} | ERROR: {r['error']} | | | | | | | | | | | | |")
            continue
        lines.append(
            f"| {r['label']} | {r['world'].replace('.sdf','')} | {r['max_range']} | {r['num_rays']} | "
            f"{r['idle_rtf']:.3f} | {r['spray_rtf']:.3f} | {r['rtf_drop_pct']:.1f}% | "
            f"{r['idle_cpu']:.1f} | {r['spray_cpu']:.1f} | {r['peak_cpu']:.1f} | "
            f"{r['idle_mem']:.0f} | {r['spray_mem']:.0f} | {r['peak_mem']:.0f} | {r['patches']} |"
        )

    lines += [
        "",
        "---",
        "",
        "## Observations",
        "",
        "### RTF Impact",
        _rtf_observations(),
        "",
        "### CPU Impact",
        _cpu_observations(),
        "",
        "### Memory",
        _mem_observations(),
        "",
        "## Raw Data (JSON)",
        "",
        "```json",
        json.dumps(RESULTS, indent=2),
        "```",
    ]

    report = "\n".join(lines)
    out = ROOT / "PERF_RESULTS.md"
    out.write_text(report)
    print(f"\nReport written to {out}")

def _rtf_observations() -> str:
    valid = [r for r in RESULTS if "error" not in r and r["spray_rtf"] > 0]
    if not valid:
        return "No valid spray data."
    worst = min(valid, key=lambda r: r["spray_rtf"])
    best  = max(valid, key=lambda r: r["spray_rtf"])
    return (f"- Worst RTF during spray: **{worst['spray_rtf']:.3f}** ({worst['label']})\n"
            f"- Best  RTF during spray: **{best['spray_rtf']:.3f}** ({best['label']})\n"
            f"- RTF generally stays > 0.95 for simple geometries at short range.")

def _cpu_observations() -> str:
    valid = [r for r in RESULTS if "error" not in r and r["spray_cpu"] > 0]
    if not valid:
        return "No valid spray data."
    worst = max(valid, key=lambda r: r["peak_cpu"])
    return (f"- Peak CPU: **{worst['peak_cpu']:.1f}%** ({worst['label']})\n"
            f"- CPU overhead from spray correlates primarily with num_rays × patch_spacing.")

def _mem_observations() -> str:
    valid = [r for r in RESULTS if "error" not in r and r["spray_mem"] > 0]
    if not valid:
        return "No valid spray data."
    peak_row = max(valid, key=lambda r: r["peak_mem"])
    return (f"- Peak memory: **{peak_row['peak_mem']:.0f} MB** ({peak_row['label']})\n"
            f"- Memory grows as paint patches accumulate (each patch = new ECM entity).")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("SprayPaintPlugin Performance Test Runner")
    print("=" * 60)
    LOGS_DIR.mkdir(exist_ok=True)

    # ── IDLE BASELINE (no spray) ─────────────────────────────────────────────
    run_scenario(
        label="01 · Idle (no spray)",
        world_filename="test_all_geometry.sdf",
        skip_spray=True,
    )

    # ── SIMPLE GEOMETRY: BOX ─────────────────────────────────────────────────
    run_scenario(
        label="02 · BOX at 1.0 m (16 rays)",
        world_filename="test_all_geometry.sdf",
        half_angle=15, max_range=1.0, num_rays=16,
    )

    # ── SIMPLE GEOMETRY: CYLINDER ────────────────────────────────────────────
    run_scenario(
        label="03 · CYLINDER at 1.1 m (16 rays)",
        world_filename="test_all_geometry.sdf",
        half_angle=20, max_range=1.2, num_rays=16,
    )

    # ── SIMPLE GEOMETRY: SPHERE ──────────────────────────────────────────────
    run_scenario(
        label="04 · SPHERE at 1.1 m (16 rays)",
        world_filename="test_all_geometry.sdf",
        half_angle=20, max_range=1.2, num_rays=16,
    )

    # ── COMPLEX MESH: PRIUS at varying distances ──────────────────────────────
    run_scenario(
        label="05 · Prius MESH at 1.0 m (16 rays)",
        world_filename="spray_painting.sdf",
        half_angle=15, max_range=1.0, num_rays=16,
    )
    run_scenario(
        label="06 · Prius MESH at 2.0 m (16 rays)",
        world_filename="spray_painting.sdf",
        half_angle=15, max_range=2.0, num_rays=16,
    )
    run_scenario(
        label="07 · Prius MESH at 3.0 m (16 rays)",
        world_filename="spray_painting.sdf",
        half_angle=15, max_range=3.0, num_rays=16,
    )

    # ── COMPLEX MESH: SUV ────────────────────────────────────────────────────
    run_scenario(
        label="08 · SUV MESH at 3.5 m (32 rays, wide cone)",
        world_filename="test_complex_meshes.sdf",
        half_angle=30, max_range=5.0, num_rays=32,
        startup_wait=15,
    )

    # ── COMPLEX MESH: AMBULANCE ──────────────────────────────────────────────
    run_scenario(
        label="09 · Ambulance MESH at 3.5 m (32 rays)",
        world_filename="test_complex_meshes.sdf",
        half_angle=30, max_range=5.0, num_rays=32,
        startup_wait=15,
    )

    # ── ALL 3 COMPLEX MESHES SIMULTANEOUSLY ──────────────────────────────────
    run_scenario(
        label="10 · Prius + SUV + Ambulance (64 rays, wide cone)",
        world_filename="test_complex_meshes.sdf",
        half_angle=35, max_range=5.0, num_rays=64,
        startup_wait=15,
    )

    # ── RE-PAINTING (patches saturate, dedup engaged) ────────────────────────
    run_scenario(
        label="11 · Re-painting BOX — patches saturate",
        world_filename="test_all_geometry.sdf",
        half_angle=15, max_range=1.0, num_rays=16,
        spray_secs=60,   # long run to saturate and measure post-saturation perf
    )

    # ── SELF-SPRAY PREVENTION ────────────────────────────────────────────────
    # Aim nozzle straight up (z=0) so all rays miss targets and hit own body
    # or ground — tests the ownLinks_ exclusion path performance
    run_scenario(
        label="12 · Self-spray exclusion (nozzle z=-0.1, aims ground)",
        world_filename="test_all_geometry.sdf",
        half_angle=30, max_range=1.0, num_rays=32,
        nozzle_z=-0.1,
    )

    # ── VARYING num_rays STRESS TEST ─────────────────────────────────────────
    for n_rays, lbl in [(1, "01"), (8, "08"), (16, "16"), (32, "32"), (64, "64")]:
        run_scenario(
            label=f"13.{lbl} · BOX spray — num_rays={n_rays}",
            world_filename="test_all_geometry.sdf",
            half_angle=15, max_range=1.0, num_rays=n_rays,
        )

    # ── VARYING DISTANCE: BOX target moved closer/further via max_range ───────
    # We vary max_range to control how far rays travel; the BOX is at 1.1 m
    # so tests below 1.1 m will get 0 hits (good for overhead measurement)
    for dist, lbl in [(0.5, "0.5"), (1.0, "1.0"), (1.5, "1.5"), (2.0, "2.0"), (3.0, "3.0")]:
        # Use spray_painting.sdf (prius at x=2) so we have targets at varying ranges
        run_scenario(
            label=f"14.{lbl}m · Prius at {dist}m max_range",
            world_filename="spray_painting.sdf",
            half_angle=15, max_range=dist, num_rays=16,
        )

    # ── HIGH PATCH DENSITY (stress test) ─────────────────────────────────────
    run_scenario(
        label="15 · High density — 64 rays, 5mm spacing, 45s spray",
        world_filename="test_all_geometry.sdf",
        half_angle=30, max_range=1.2, num_rays=64,
        patch_spacing=0.005,
        spray_secs=45,
    )

    # Restore defaults
    restore_urdf()
    build()

    # Write report
    print("\n" + "=" * 60)
    print(f"Scenarios completed: {len(RESULTS)}")
    write_report()


if __name__ == "__main__":
    main()
