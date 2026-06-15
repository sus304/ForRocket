#!/usr/bin/env python3
"""
ForRocket integration test suite.

Usage:
    python3 tests/simulation_test.py              # run all tests
    python3 tests/simulation_test.py attitude     # run tests matching "attitude"

Each test creates a temporary working directory, writes JSON/CSV configs,
runs the ForRocket binary, and validates the output flight-log CSV.
New tests: add a function `test_<name>(suite, runner)` and call it in main().
"""

import argparse
import csv
import copy
import json
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Dict, List, Optional


# ── Paths ──────────────────────────────────────────────────────────────────
REPO_ROOT = Path(__file__).resolve().parent.parent
BINARY    = REPO_ROOT / "build" / "ForRocket"
EXAMPLES  = REPO_ROOT / "examples"


# ── Flight-log column name constants ───────────────────────────────────────
class Col:
    """Named constants for flight-log CSV columns."""
    TIME           = "Time [s]"
    BURN_TIME      = "Burn Time [s]"
    ELEVATION      = "Elvation [deg]"       # note: typo in ForRocket source
    AZIMUTH        = "Azimuth [deg]"
    ROLL           = "Roll [deg]"
    VEL_ROLL       = "AngleVelx [deg/s]"    # body p (roll rate)
    VEL_PITCH      = "AngleVely [deg/s]"    # body q (pitch rate)
    VEL_YAW        = "AngleVelz [deg/s]"    # body r (yaw rate)
    ACC_ROLL       = "AngleAccx [rad/s2]"
    ACC_PITCH      = "AngleAccy [rad/s2]"
    ACC_YAW        = "AngleAccz [rad/s2]"
    Q1             = "q1 [-]"
    Q2             = "q2 [-]"
    Q3             = "q3 [-]"
    Q4             = "q4 [-]"
    ALTITUDE       = "Altitude [m]"
    DOWNRANGE      = "Downrange [m]"
    VX_BODY        = "Vx-body [m/s]"
    VY_BODY        = "Vy-body [m/s]"
    VZ_BODY        = "Vz-body [m/s]"
    VX_NED         = "Vx-NED [m/s]"
    VY_NED         = "Vy-NED [m/s]"
    VZ_NED         = "Vz-NED [m/s]"
    MACH           = "MachNumber [-]"
    DYNAMIC_PRESS  = "DynamicPressure [kPa]"
    THRUST         = "Thrust [N]"
    MASS           = "Mass [kg]"
    PROP_MASS      = "Propellant Mass [kg]"
    AOA            = "AoA [deg]"
    AOS            = "AoS [deg]"
    BURNING        = "Burning [0/1]"
    STATIC_MARGIN  = "StaticMargin [%]"
    LATITUDE       = "Latitude [deg]"
    LONGITUDE      = "Longitude [deg]"


# ── FlightLog ──────────────────────────────────────────────────────────────
class FlightLog:
    """Read-only wrapper around a ForRocket flight-log CSV."""

    def __init__(self, path: str):
        with open(path) as f:
            reader = csv.DictReader(f)
            self._rows = list(reader)
        if not self._rows:
            raise ValueError(f"Empty flight log: {path}")

    def col(self, name: str) -> List[float]:
        """Return all values of a column as floats."""
        return [float(r[name]) for r in self._rows]

    def at(self, name: str, t: float) -> float:
        """Return the value of a column at the time step closest to t."""
        times = self.col(Col.TIME)
        idx = min(range(len(times)), key=lambda i: abs(times[i] - t))
        return float(self._rows[idx][name])

    def time(self) -> List[float]:
        return self.col(Col.TIME)

    def max_time(self) -> float:
        return max(self.col(Col.TIME))

    def __len__(self):
        return len(self._rows)


# ── TestSuite ──────────────────────────────────────────────────────────────
class TestSuite:
    """Collects pass/fail results and prints a formatted summary."""

    GREEN = "\033[32m"
    RED   = "\033[31m"
    RESET = "\033[0m"

    def __init__(self):
        self._pass = 0
        self._fail = 0

    def section(self, name: str):
        print(f"\n[{name}]")

    # ---- assertion helpers ----

    def check(self, label: str, val: float, expected: float, tol: float, unit: str = ""):
        """Assert |val - expected| <= tol."""
        diff = abs(val - expected)
        self._record(diff <= tol,
                     f"{label}: {val:.4f}{unit}  (expected {expected:.4f}±{tol:.4f})")

    def check_near(self, label: str, val: float, ref: float, tol: float, unit: str = ""):
        """Assert val is within tol of ref (shows diff)."""
        diff = abs(val - ref)
        self._record(diff <= tol,
                     f"{label}: {val:.4f}{unit}  (ref {ref:.4f}, diff {diff:.4f}±{tol:.4f})")

    def check_angle(self, label: str, val: float, ref: float, tol: float):
        """Assert angular proximity with 360° wrap handling."""
        diff = abs((val - ref + 180.0) % 360.0 - 180.0)
        self._record(diff <= tol,
                     f"{label}: {val:.4f}°  (ref {ref:.4f}°, diff {diff:.4f}°±{tol:.4f}°)")

    def check_cond(self, label: str, cond: bool, detail: str = ""):
        """Assert a boolean condition."""
        msg = label + (f": {detail}" if detail else "")
        self._record(cond, msg)

    def info(self, label: str, val, unit: str = ""):
        """Print an informational value (no pass/fail)."""
        print(f"  [INFO] {label}: {val}{unit}")

    def summary(self) -> int:
        """Print totals and return exit code (0=all pass, 1=any fail)."""
        print("\n" + "=" * 70)
        print(f"TOTAL: {self._pass} passed, {self._fail} failed")
        print("=" * 70)
        return 0 if self._fail == 0 else 1

    def _record(self, ok: bool, msg: str):
        tag = f"{self.GREEN}OK{self.RESET}" if ok else f"{self.RED}NG{self.RESET}"
        print(f"  [{tag}] {msg}")
        if ok:
            self._pass += 1
        else:
            self._fail += 1


# ── ScenarioRunner ─────────────────────────────────────────────────────────
class ScenarioRunner:
    """
    Creates a temporary directory, writes ForRocket JSON configs and
    optional CSV files, runs the binary, and returns a FlightLog.
    """

    MODEL_ID = "test"

    def __init__(self, binary: Path = BINARY, examples: Path = EXAMPLES):
        self.binary   = binary
        self.examples = examples

    def run(
        self,
        label: str,
        rocket: dict,
        seq: Optional[Dict] = None,
        solver: Optional[Dict] = None,
        attitude_csv: Optional[str] = None,
        attitude_csv_name: str = "attitude.csv",
        extra_files: Optional[Dict] = None,
    ) -> FlightLog:
        """
        Run one scenario and return its FlightLog.

        rocket:            full rocket config dict (use make_rocket() as base)
        seq:               sequence-of-event config (use make_seq() as base)
        solver:            solver config (use make_solver() as base)
        attitude_csv:      raw CSV text written as the attitude input file
        attitude_csv_name: filename for the attitude CSV
        extra_files:       {filename: text_content} written to the work dir
        """
        tmpdir = tempfile.mkdtemp(prefix="forrocket_")
        try:
            # Copy shared resource files from examples/
            for name in ["sample_CA.csv", "sample_param_engine.json",
                         "sample_thrust.csv", "sample_wind.csv"]:
                src = self.examples / name
                if src.exists():
                    shutil.copy(src, tmpdir)

            # Write attitude CSV
            if attitude_csv is not None:
                with open(os.path.join(tmpdir, attitude_csv_name), "w") as f:
                    f.write(attitude_csv)
                if "Program Attitude" in rocket:
                    rocket = copy.deepcopy(rocket)
                    rocket["Program Attitude"]["File Path"] = attitude_csv_name

            # Write extra files
            if extra_files:
                for name, content in extra_files.items():
                    with open(os.path.join(tmpdir, name), "w") as f:
                        f.write(content)

            # Write configs
            _writejson(os.path.join(tmpdir, "rocket.json"),   rocket)
            _writejson(os.path.join(tmpdir, "sequence.json"), seq or make_seq())
            _writejson(os.path.join(tmpdir, "solver.json"),   solver or make_solver())
            _writejson(os.path.join(tmpdir, "stage1_list.json"), {
                "Rocket Configuration File Path": "rocket.json",
                "Engine Configuration File Path": "sample_param_engine.json",
                "Sequence of Event File Path":    "sequence.json",
            })

            # Run binary
            result = subprocess.run(
                [str(self.binary), "solver.json", "-q"],
                cwd=tmpdir, capture_output=True, text=True,
            )
            if result.returncode != 0:
                raise RuntimeError(f"ForRocket exited {result.returncode}:\n{result.stderr}")

            log_path = os.path.join(tmpdir, f"{self.MODEL_ID}_stage1_flight_log.csv")
            if not os.path.exists(log_path):
                raise RuntimeError(f"Output file not found: {log_path}")

            log = FlightLog(log_path)
            print(f"  {label}: {len(log)} rows, max_t={log.max_time():.1f}s")
            return log

        finally:
            shutil.rmtree(tmpdir)


# ── Config factory functions ───────────────────────────────────────────────
def make_solver(
    model_id: str = "test",
    lat: float = 40.242865,
    lon: float = 140.01045,
    azimuth: float = 270.0,
    elevation: float = 85.0,
    wind: bool = False,
) -> dict:
    return {
        "Model ID": model_id,
        "Launch DateTime": "2020/08/23 9:00:00.0",
        "Launch Condition": {
            "Latitude [deg]":             lat,
            "Longitude [deg]":            lon,
            "Height for WGS84 [m]":       20.0,
            "Azimuth [deg]":              azimuth,
            "Elevation [deg]":            elevation,
            "North Velocity [m/s]":       0.0,
            "East Velocity [m/s]":        0.0,
            "Down Velocity [m/s]":        0.0,
            "Yaw Angular Velocity [deg/s]":   0.0,
            "Pitch Angular Velocity [deg/s]": 0.0,
            "Roll Angular Velocity [deg/s]":  0.0,
        },
        "Wind Condition": {
            "Enable Wind": wind,
            "Wind File Path": "sample_wind.csv",
        },
        "Number of Stage": 1,
        "Stage1 Config File List": "stage1_list.json",
        "Stage2 Config File List": "stage1_list.json",
        "Stage3 Config File List": "stage1_list.json",
    }


def make_seq(
    end_time: float = 60.0,
    timestep: float = 0.1,
    rail_len: float = 5.0,
) -> dict:
    return {
        "Flight Start Time [s]":   0.0,
        "Engine Ignittion Time [s]": 0.0,
        "Enable Rail-Launcher Launch": True,
        "Rail Launcher": {"Length [m]": rail_len},
        "Enable Engine Cutoff":    False,
        "Cutoff":                  {"Cutoff Time [s]": 0.0},
        "Enable Stage Separation": False,
        "Upper Stage":             {"Stage Separation Time [s]": 0.0, "Upper Stage Mass [kg]": 100.0},
        "Enable Despin Control":   False,
        "Despin":                  {"Time [s]": 20.0},
        "Enable Fairing Jettson":  False,
        "Fairing":                 {"Jettson Time [s]": 0.0, "Mass [kg]": 1.0},
        "Enable Parachute Open":   False,
        "Parachute":               {"Open Time [s]": 30.0, "Drag Factor Cd*S [m2]": 0.3,
                                    "Enable Forced Apogee Open": False},
        "Enable Secondary Parachute Open": False,
        "Secondary Parachute":     {"Open Time [s]": 60.0, "Drag Factor Cd*S [m2]": 1.3},
        "Flight End Time [s]":     end_time,
        "Time Step [s]":           timestep,
        # Pin a tight solver tolerance so these attitude/trajectory assertions
        # validate the dynamics at high accuracy, independent of the (looser,
        # speed-tuned) production default. At the 1e-6 default, integration
        # drift alone exceeds the assertion bands (~0.8 deg roll, ~2.7 deg
        # elevation over 60 s); 1e-7/1e-9 was the pre-optimization default.
        "Solver Tolerance Abs":    1.0e-9,
        "Solver Tolerance Rel":    1.0e-7,
        "Enable Auto Terminate SubOrbital Flight": True,
    }


def make_rocket(program_attitude: Optional[Dict] = None) -> Dict:
    """
    Base rocket config (sample parameters).
    Pass program_attitude=dict(...) to enable attitude control.

    program_attitude keys:
        mode          "Angle" | "Rate"
        enable_yaw    bool
        enable_pitch  bool
        enable_roll   bool
        file_path     str  (CSV filename in work dir)
    """
    enable_prog = program_attitude is not None
    att_cfg = {
        "Mode":         program_attitude.get("mode",         "Angle") if enable_prog else "Angle",
        "Enable Yaw":   program_attitude.get("enable_yaw",   True)    if enable_prog else True,
        "Enable Pitch": program_attitude.get("enable_pitch", True)    if enable_prog else True,
        "Enable Roll":  program_attitude.get("enable_roll",  True)    if enable_prog else True,
        "File Path":    program_attitude.get("file_path",    "attitude.csv") if enable_prog else "attitude.csv",
    }
    return {
        "Diameter [mm]":  180.0,
        "Length [mm]":    3900.0,
        "Mass":           {"Inert [kg]": 52.5, "Propellant [kg]": 41.0},

        "Enable Gas Jet": False,
        "Gas Jet": {"Rolling Moment [N.m]": 0.0, "Duration [s]": 2.5},

        "Enable Program Attitude": enable_prog,
        "Program Attitude": att_cfg,

        "Enable X-C.G. File":   False,
        "X-C.G. File":          {"X-C.G. File Path": "Xcg.csv"},
        "Constant X-C.G.":      {"Constant X-C.G. from BodyTail [mm]": 1100.0},

        "Comment M.I.": "Moment of Inertia",
        "Enable M.I. File":  False,
        "M.I. File":         {"M.I. File Path": "MOI.csv"},
        "Constant M.I.":     {"Yaw Axis [kg-m2]": 45.0, "Pitch Axis [kg-m2]": 45.0,
                              "Roll Axis [kg-m2]": 0.5},

        "Enable X-C.P. File":   False,
        "X-C.P. File":          {"X-C.P. File Path": "Xcp.csv"},
        "Constant X-C.P.":      {"Constant X-C.P. from BodyTail [mm]": 835.0},

        "X-ThrustLoadingPoint from BodyTail [mm]": 300.0,

        "Enable CA File": True,
        "CA File": {"CA File Path": "sample_CA.csv", "BurnOut CA File Path": "sample_CA.csv"},
        "Constant CA":   {"Constant CA [-]": 0.4, "Constant BurnOut CA [-]": 0.5},

        "Enable CNa File": False,
        "CNa File":        {"CNa File Path": "CNa.csv"},
        "Constant CNa":    {"Constant CNa [1/rad]": 10.0},

        "Fin Cant Angle [deg]": 0.0,
        "Enable Cld File":  False,
        "Cld File":         {"Cld File Path": "Cld.csv"},
        "Constant Cld":     {"Constant Cld [1/rad]": 0.0},

        "Enable Clp File":  False,
        "Clp File":         {"Clp File Path": "Clp.csv"},
        "Constant Clp":     {"Constant Clp [-]": 0.03},

        "Enable Cmq File":  False,
        "Cmq File":         {"Cmq File Path": "Cmq.csv"},
        "Constant Cmq":     {"Constant Cmq [-]": 7.0},

        "Enable Cnr File":  False,
        "Cnr File":         {"Cnr File Path": "Cnr.csv"},
        "Constant Cnr":     {"Constant Cnr [-]": 7.0},
    }


# ── Helpers ────────────────────────────────────────────────────────────────
def _writejson(path: str, obj: dict):
    with open(path, "w") as f:
        json.dump(obj, f, indent=4)


# ─────────────────────────────────────────────────────────────────────────────
# Test cases
# Each function signature: test_xxx(suite: TestSuite, runner: ScenarioRunner)
# ─────────────────────────────────────────────────────────────────────────────

def test_attitude_free_axes(suite: TestSuite, runner: ScenarioRunner):
    """
    Attitude control with per-axis enable/disable.
    Non-controlled axes must respond to aerodynamics (not be frozen).
    """

    # ── baseline: no program attitude ────────────────────────────────────
    suite.section("Baseline (no program attitude)")
    baseline = runner.run(
        "Baseline",
        make_rocket(),
    )
    base_elv_30 = baseline.at(Col.ELEVATION, 30)
    base_az_30  = baseline.at(Col.AZIMUTH,   30)
    base_elv_60 = baseline.at(Col.ELEVATION, 60)

    # ── T1: roll angle control only ──────────────────────────────────────
    suite.section("T1: Roll angle=0° control, pitch/yaw free (angle mode)")
    t1_csv = "time,yaw,pitch,roll\n0.0,270,85,0\n60.0,270,85,0\n"
    t1 = runner.run(
        "T1",
        make_rocket({"mode": "Angle", "enable_yaw": False, "enable_pitch": False,
                     "enable_roll": True, "file_path": "att_t1.csv"}),
        attitude_csv=t1_csv, attitude_csv_name="att_t1.csv",
    )
    suite.check("Roll at t=10s ≈ 0°", t1.at(Col.ROLL, 10), 0.0, 0.5, "°")
    suite.check("Roll at t=30s ≈ 0°", t1.at(Col.ROLL, 30), 0.0, 0.5, "°")
    suite.check("Roll at t=60s ≈ 0°", t1.at(Col.ROLL, 60), 0.0, 0.5, "°")
    suite.check_near("Elevation at t=30s matches baseline", t1.at(Col.ELEVATION, 30), base_elv_30, 0.5, "°")
    suite.check_near("Elevation at t=60s matches baseline", t1.at(Col.ELEVATION, 60), base_elv_60, 1.5, "°")
    suite.check_angle("Azimuth at t=30s matches baseline", t1.at(Col.AZIMUTH, 30), base_az_30, 1.0)
    elv_change = abs(t1.at(Col.ELEVATION, 50) - t1.at(Col.ELEVATION, 1))
    suite.check_cond("Elevation changes (not frozen)", elv_change > 1.0, f"Δ={elv_change:.2f}°")

    # ── T2: pitch angle control only (regression) ─────────────────────────
    suite.section("T2: Pitch angle 85→55° control, roll/yaw free (angle mode)")
    t2_csv = "time,yaw,pitch,roll\n0.0,270,85,0\n60.0,270,55,0\n"
    t2 = runner.run(
        "T2",
        make_rocket({"mode": "Angle", "enable_yaw": False, "enable_pitch": True,
                     "enable_roll": False, "file_path": "att_t2.csv"}),
        attitude_csv=t2_csv, attitude_csv_name="att_t2.csv",
    )
    suite.check("Elevation at t=60s = 55°", t2.at(Col.ELEVATION, 60), 55.0, 0.5, "°")
    suite.info("Pitch ang.vel at t=30s (controlled→0)", t2.at(Col.VEL_PITCH, 30), " deg/s")
    suite.info("Roll at t=30s (free)", t2.at(Col.ROLL, 30), "°")
    suite.check_angle("Azimuth at t=30s matches baseline", t2.at(Col.AZIMUTH, 30), base_az_30, 2.0)

    # ── T3: roll rate control only ────────────────────────────────────────
    suite.section("T3: Roll rate=10 deg/s control, pitch/yaw free (rate mode)")
    t3_csv = "time,yaw_rate,pitch_rate,roll_rate\n0.0,0,0,10\n60.0,0,0,10\n"
    t3 = runner.run(
        "T3",
        make_rocket({"mode": "Rate", "enable_yaw": False, "enable_pitch": False,
                     "enable_roll": True, "file_path": "att_t3.csv"}),
        attitude_csv=t3_csv, attitude_csv_name="att_t3.csv",
    )
    suite.check("Roll ang.vel at t=5s  = 10 deg/s", t3.at(Col.VEL_ROLL, 5),  10.0, 0.5, " deg/s")
    suite.check("Roll ang.vel at t=30s = 10 deg/s", t3.at(Col.VEL_ROLL, 30), 10.0, 0.5, " deg/s")
    suite.check("Roll ang.vel at t=59s = 10 deg/s", t3.at(Col.VEL_ROLL, 59), 10.0, 0.5, " deg/s")
    suite.info("Roll ang.vel at t=65s (after control end)", t3.at(Col.VEL_ROLL, 65), " deg/s")
    suite.check_near("Elevation at t=30s matches baseline", t3.at(Col.ELEVATION, 30), base_elv_30, 0.5, "°")
    suite.check_angle("Azimuth at t=30s matches baseline",  t3.at(Col.AZIMUTH,   30), base_az_30,  1.0)


def test_attitude_angle_all_axes(suite: TestSuite, runner: ScenarioRunner):
    """Angle mode with all axes controlled: attitude tracks CSV exactly."""
    suite.section("All-axis angle control (270°/85°/0° constant)")
    csv_text = "time,yaw,pitch,roll\n0.0,270,85,0\n60.0,270,85,0\n"
    log = runner.run(
        "All-axis angle",
        make_rocket({"mode": "Angle", "enable_yaw": True, "enable_pitch": True,
                     "enable_roll": True, "file_path": "att_all.csv"}),
        attitude_csv=csv_text, attitude_csv_name="att_all.csv",
    )
    for t in [5, 15, 30, 45, 55]:
        suite.check_angle(f"Elevation at t={t}s = 85°", log.at(Col.ELEVATION, t), 85.0, 0.05)
        suite.check_angle(f"Azimuth at t={t}s = 270°",  log.at(Col.AZIMUTH,   t), 270.0, 0.05)
        suite.check(      f"Roll at t={t}s = 0°",       log.at(Col.ROLL,      t), 0.0,   0.05, "°")


def test_parachute_dynamics(suite: TestSuite, runner: ScenarioRunner):
    """
    Verify parachute descent physics (vector drag model):
    1. No wind: horizontal velocity decays to ~0, vertical converges to terminal velocity
    2. Constant north wind: horizontal velocity converges to wind speed
    """
    import math

    INERT_MASS = 52.5   # kg
    CDS        = 2.0    # m²  (parachute Cd*S)
    RHO_SL     = 1.225  # kg/m³ (sea-level density)
    G          = 9.81   # m/s²
    V_TERMINAL = math.sqrt(2 * INERT_MASS * G / (RHO_SL * CDS))  # ~20.5 m/s

    seq = make_seq(end_time=300.0, timestep=0.1)
    seq["Enable Parachute Open"] = True
    seq["Parachute"]["Drag Factor Cd*S [m2]"] = CDS
    seq["Parachute"]["Enable Forced Apogee Open"] = True

    # ── No-wind: terminal velocity + horizontal decay ─────────────────────
    suite.section("Parachute no-wind: Vz → terminal velocity, Vx/Vy → 0")
    log = runner.run("Parachute NoWind", make_rocket(), seq=seq)

    t_end = log.max_time()
    vz = log.at(Col.VZ_NED, t_end - 2.0)
    vx = log.at(Col.VX_NED, t_end - 2.0)
    vy = log.at(Col.VY_NED, t_end - 2.0)

    suite.check_near("Vz-NED near terminal velocity", vz, V_TERMINAL, 5.0, " m/s")
    suite.check_near("Vx-NED ≈ 0 (no wind)", vx, 0.0, 1.0, " m/s")
    suite.check_near("Vy-NED ≈ 0 (no wind)", vy, 0.0, 1.0, " m/s")

    # ── Constant north wind: horizontal drifts to wind speed ──────────────
    WIND_NORTH = 5.0  # m/s
    wind_csv = f"alt,u,v\n0,0,{WIND_NORTH}\n20000,0,{WIND_NORTH}\n"

    suite.section(f"Parachute north wind {WIND_NORTH} m/s: Vx → wind, Vy → 0")
    log_w = runner.run(
        "Parachute Wind",
        make_rocket(), seq=seq,
        solver=make_solver(wind=True),
        extra_files={"sample_wind.csv": wind_csv},
    )
    t_end_w = log_w.max_time()
    vx_w = log_w.at(Col.VX_NED, t_end_w - 2.0)
    vy_w = log_w.at(Col.VY_NED, t_end_w - 2.0)

    suite.check_near("Vx-NED → wind north speed", vx_w, WIND_NORTH, 1.0, " m/s")
    suite.check_near("Vy-NED ≈ 0 (no east wind)", vy_w, 0.0, 1.0, " m/s")


def test_parachute_wind_drift(suite: TestSuite, runner: ScenarioRunner):
    """
    Verify parachute landing-point drift direction for all 4 cardinal winds.

    Attitude is locked during ascent (full 3-axis angle control) to suppress
    aerodynamic weather-vaning, isolating the parachute-phase wind effect.
    Landing lat/lon must shift in the wind direction vs no-wind baseline.

    Wind CSV columns: alt, u(east component), v(north component)
    getNED() returns [wind_from_north, wind_from_east, 0]
    """
    WIND_SPEED = 10.0  # m/s
    CDS = 2.0          # m²

    seq = make_seq(end_time=300.0, timestep=0.1)
    seq["Enable Parachute Open"] = True
    seq["Parachute"]["Drag Factor Cd*S [m2]"] = CDS
    seq["Parachute"]["Enable Forced Apogee Open"] = True

    # Lock attitude throughout to eliminate ascent aerodynamic drift
    att_csv = "time,yaw,pitch,roll\n0.0,270,85,0\n300.0,270,85,0\n"
    rocket = make_rocket({"mode": "Angle", "enable_yaw": True, "enable_pitch": True,
                          "enable_roll": True, "file_path": "att_fixed.csv"})

    # ── Baseline: no wind ─────────────────────────────────────────────────
    suite.section("Parachute wind drift: baseline (no wind)")
    log0 = runner.run(
        "Baseline NoWind", rocket, seq=seq,
        attitude_csv=att_csv, attitude_csv_name="att_fixed.csv",
    )
    lat0 = log0.at(Col.LATITUDE,  log0.max_time())
    lon0 = log0.at(Col.LONGITUDE, log0.max_time())
    suite.info("Baseline landing lat", f"{lat0:.6f}", "°")
    suite.info("Baseline landing lon", f"{lon0:.6f}", "°")

    # ── 4-direction wind checks ───────────────────────────────────────────
    # (wind_csv, label, coord, direction_sign)
    # coord: "lat" or "lon"
    # direction_sign: +1 if landing coord should increase, -1 if decrease
    cases = [
        (f"alt,u,v\n0,0,{WIND_SPEED}\n20000,0,{WIND_SPEED}\n",
         "North", "lat", +1),
        (f"alt,u,v\n0,0,{-WIND_SPEED}\n20000,0,{-WIND_SPEED}\n",
         "South", "lat", -1),
        (f"alt,u,v\n0,{WIND_SPEED},0\n20000,{WIND_SPEED},0\n",
         "East",  "lon", +1),
        (f"alt,u,v\n0,{-WIND_SPEED},0\n20000,{-WIND_SPEED},0\n",
         "West",  "lon", -1),
    ]

    for wind_csv, label, coord, sign in cases:
        suite.section(f"Parachute wind drift: {label} wind {WIND_SPEED} m/s")
        log = runner.run(
            f"Wind {label}", rocket, seq=seq,
            solver=make_solver(wind=True),
            attitude_csv=att_csv, attitude_csv_name="att_fixed.csv",
            extra_files={"sample_wind.csv": wind_csv},
        )
        t_end = log.max_time()
        lat = log.at(Col.LATITUDE,  t_end)
        lon = log.at(Col.LONGITUDE, t_end)
        dlat = lat - lat0
        dlon = lon - lon0
        suite.info(f"Landing lat", f"{lat:.6f}", "°")
        suite.info(f"Landing lon", f"{lon:.6f}", "°")
        suite.info(f"Δlat", f"{dlat:+.6f}", "°")
        suite.info(f"Δlon", f"{dlon:+.6f}", "°")
        if coord == "lat":
            suite.check_cond(
                f"{label} wind → landing shifts {'north' if sign > 0 else 'south'} (Δlat {'+' if sign > 0 else '-'})",
                sign * dlat > 0,
                f"Δlat={dlat:+.6f}°",
            )
        else:
            suite.check_cond(
                f"{label} wind → landing shifts {'east' if sign > 0 else 'west'} (Δlon {'+' if sign > 0 else '-'})",
                sign * dlon > 0,
                f"Δlon={dlon:+.6f}°",
            )


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

ALL_TESTS = {
    "attitude_free_axes":       test_attitude_free_axes,
    "attitude_angle_all_axes":  test_attitude_angle_all_axes,
    "parachute_dynamics":       test_parachute_dynamics,
    "parachute_wind_drift":     test_parachute_wind_drift,
}


def main():
    parser = argparse.ArgumentParser(description="ForRocket integration tests")
    parser.add_argument("filter", nargs="?", default="",
                        help="Run only tests whose name contains this string")
    args = parser.parse_args()

    if not BINARY.exists():
        print(f"ERROR: binary not found: {BINARY}", file=sys.stderr)
        print("Run: cmake --build build", file=sys.stderr)
        sys.exit(1)

    suite  = TestSuite()
    runner = ScenarioRunner()

    selected = {name: fn for name, fn in ALL_TESTS.items()
                if args.filter.lower() in name.lower()}

    if not selected:
        print(f"No tests match filter: {args.filter!r}")
        sys.exit(0)

    print(f"Running {len(selected)} test group(s)...")
    for name, fn in selected.items():
        print(f"\n{'='*70}")
        print(f"  TEST GROUP: {name}")
        print(f"{'='*70}")
        fn(suite, runner)

    sys.exit(suite.summary())


if __name__ == "__main__":
    main()
