#!/usr/bin/env python3
import argparse
import bisect
import json
import os
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node

from go2_jump_msgs.msg import JumpControllerState
from unitree_go.msg import SportModeState

try:
    from unitree_go.msg import LowCmd
except ImportError:  # pragma: no cover
    LowCmd = None


def collapse_phase_segments(samples: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    segments: List[Dict[str, Any]] = []
    for sample in samples:
        phase = sample["phase"]
        if not segments or segments[-1]["phase"] != phase:
            segments.append(
                {
                    "phase": phase,
                    "start_s": sample["t"],
                    "end_s": sample["t"],
                    "samples": 1,
                    "min_contact_count": sample["contact_count"],
                    "max_contact_count": sample["contact_count"],
                }
            )
            continue
        segment = segments[-1]
        segment["end_s"] = sample["t"]
        segment["samples"] += 1
        segment["min_contact_count"] = min(
            segment["min_contact_count"], sample["contact_count"]
        )
        segment["max_contact_count"] = max(
            segment["max_contact_count"], sample["contact_count"]
        )
    return segments


def first_index(
    samples: List[Dict[str, Any]], predicate, start_index: int = 0
) -> Optional[int]:
    for index in range(start_index, len(samples)):
        if predicate(samples[index]):
            return index
    return None


def nearest_sample(
    samples: List[Dict[str, Any]], target_time_s: Optional[float]
) -> Optional[Dict[str, Any]]:
    if not samples or target_time_s is None:
        return None
    sample_times = [sample["t"] for sample in samples]
    insert_index = bisect.bisect_left(sample_times, target_time_s)
    candidates = []
    if insert_index < len(samples):
        candidates.append(samples[insert_index])
    if insert_index > 0:
        candidates.append(samples[insert_index - 1])
    if not candidates:
        return None
    return min(candidates, key=lambda sample: abs(sample["t"] - target_time_s))


def compute_rate_hz(sample_times: List[float]) -> Optional[float]:
    if len(sample_times) < 2:
        return None
    elapsed = sample_times[-1] - sample_times[0]
    if elapsed <= 1e-6:
        return None
    return (len(sample_times) - 1) / elapsed


def compute_summary(
    controller_samples: List[Dict[str, Any]],
    sport_samples: List[Dict[str, Any]],
    lowcmd_times: List[float],
    duration_s: float,
) -> Dict[str, Any]:
    segments = collapse_phase_segments(controller_samples)
    phase_sequence = [segment["phase"] for segment in segments]

    takeoff_index = first_index(
        controller_samples,
        lambda sample: sample["phase"] == "flight" and sample["contact_count"] <= 1,
    )
    if takeoff_index is None:
        takeoff_index = first_index(
            controller_samples, lambda sample: sample["phase"] == "flight"
        )

    touchdown_index = None
    if takeoff_index is not None:
        takeoff_time_s = controller_samples[takeoff_index]["t"]
        touchdown_index = first_index(
            controller_samples,
            lambda sample: sample["t"] >= takeoff_time_s + 0.02
            and (
                sample["phase"] in ("landing", "settle")
                or sample["contact_count"] >= 2
            ),
            takeoff_index + 1,
        )
    settle_index = first_index(
        controller_samples, lambda sample: sample["phase"] == "settle"
    )

    takeoff_time_s = (
        controller_samples[takeoff_index]["t"] if takeoff_index is not None else None
    )
    touchdown_time_s = (
        controller_samples[touchdown_index]["t"]
        if touchdown_index is not None
        else None
    )
    settle_time_s = (
        controller_samples[settle_index]["t"] if settle_index is not None else None
    )

    first_landing_segment_index = None
    for index, segment in enumerate(segments):
        if segment["phase"] in ("landing", "settle"):
            first_landing_segment_index = index
            break
    has_flight_relapse_after_landing = False
    if first_landing_segment_index is not None:
        for segment in segments[first_landing_segment_index + 1 :]:
            if segment["phase"] == "flight":
                has_flight_relapse_after_landing = True
                break

    flight_contact_counts = [
        sample["contact_count"]
        for sample in controller_samples
        if sample["phase"] == "flight"
    ]
    max_contact_count_in_flight = (
        max(flight_contact_counts) if flight_contact_counts else None
    )

    initial_sport = sport_samples[0] if sport_samples else None
    takeoff_sport = nearest_sample(sport_samples, takeoff_time_s)
    touchdown_sport = nearest_sample(sport_samples, touchdown_time_s)
    final_sport = (
        nearest_sample(sport_samples, settle_time_s)
        if settle_time_s is not None
        else (sport_samples[-1] if sport_samples else None)
    )

    initial_x = initial_sport["x"] if initial_sport else None
    takeoff_x = takeoff_sport["x"] if takeoff_sport else None
    touchdown_x = touchdown_sport["x"] if touchdown_sport else None
    final_x = final_sport["x"] if final_sport else None

    initial_z = initial_sport["z"] if initial_sport else None
    max_z = max((sample["z"] for sample in sport_samples), default=None)
    max_height_gain_m = (
        max_z - initial_z if max_z is not None and initial_z is not None else None
    )
    total_distance_m = (
        final_x - initial_x if final_x is not None and initial_x is not None else None
    )
    airborne_distance_m = (
        touchdown_x - takeoff_x
        if touchdown_x is not None and takeoff_x is not None
        else None
    )
    post_touchdown_distance_m = (
        final_x - touchdown_x
        if final_x is not None and touchdown_x is not None
        else None
    )
    airborne_share = None
    if (
        total_distance_m is not None
        and airborne_distance_m is not None
        and abs(total_distance_m) > 1e-6
    ):
        airborne_share = airborne_distance_m / total_distance_m

    controller_rate_hz = compute_rate_hz([sample["t"] for sample in controller_samples])
    lowcmd_rate_hz = compute_rate_hz(lowcmd_times)
    max_forward_velocity_mps = max(
        (sample["vx"] for sample in sport_samples), default=None
    )
    max_upward_velocity_mps = max(
        (sample["vz"] for sample in sport_samples), default=None
    )
    min_vertical_velocity_mps = min(
        (sample["vz"] for sample in sport_samples), default=None
    )

    target_distance_m = (
        controller_samples[0]["target_distance_m"] if controller_samples else None
    )
    mainly_airborne_translation = (
        airborne_share is not None and airborne_share >= 0.6
    )

    return {
        "duration_s": duration_s,
        "target_distance_m": target_distance_m,
        "message_counts": {
            "controller_state": len(controller_samples),
            "sport_state": len(sport_samples),
            "lowcmd": len(lowcmd_times),
        },
        "rates_hz": {
            "controller_state": controller_rate_hz,
            "lowcmd": lowcmd_rate_hz,
        },
        "phase": {
            "segments": segments,
            "sequence": phase_sequence,
            "takeoff_time_s": takeoff_time_s,
            "touchdown_time_s": touchdown_time_s,
            "settle_time_s": settle_time_s,
            "has_flight_relapse_after_landing": has_flight_relapse_after_landing,
            "max_contact_count_in_flight": max_contact_count_in_flight,
        },
        "motion": {
            "initial_x_m": initial_x,
            "takeoff_x_m": takeoff_x,
            "touchdown_x_m": touchdown_x,
            "final_x_m": final_x,
            "total_distance_m": total_distance_m,
            "airborne_distance_m": airborne_distance_m,
            "post_touchdown_distance_m": post_touchdown_distance_m,
            "airborne_share": airborne_share,
            "mainly_airborne_translation": mainly_airborne_translation,
            "max_height_gain_m": max_height_gain_m,
            "max_forward_velocity_mps": max_forward_velocity_mps,
            "max_upward_velocity_mps": max_upward_velocity_mps,
            "min_vertical_velocity_mps": min_vertical_velocity_mps,
        },
    }


def print_summary(summary: Dict[str, Any]) -> None:
    phase = summary["phase"]
    motion = summary["motion"]
    rates = summary["rates_hz"]

    def fmt(value: Optional[float], digits: int = 3) -> str:
        if value is None:
            return "n/a"
        return f"{value:.{digits}f}"

    print("Jump trial summary")
    print(f"  target distance: {fmt(summary['target_distance_m'])} m")
    print(f"  duration: {fmt(summary['duration_s'])} s")
    print("  phase sequence: " + " -> ".join(phase["sequence"]))
    print(
        "  takeoff/touchdown/settle: "
        f"{fmt(phase['takeoff_time_s'])} / {fmt(phase['touchdown_time_s'])} / {fmt(phase['settle_time_s'])} s"
    )
    print(
        "  displacement total/airborne/post-touchdown: "
        f"{fmt(motion['total_distance_m'])} / {fmt(motion['airborne_distance_m'])} / "
        f"{fmt(motion['post_touchdown_distance_m'])} m"
    )
    print(
        "  airborne share: "
        f"{fmt(motion['airborne_share'] * 100.0 if motion['airborne_share'] is not None else None, 1)} %"
    )
    print(
        "  peak height gain / vx / vz+: "
        f"{fmt(motion['max_height_gain_m'])} m / {fmt(motion['max_forward_velocity_mps'])} mps / "
        f"{fmt(motion['max_upward_velocity_mps'])} mps"
    )
    print(
        "  controller rate / lowcmd rate: "
        f"{fmt(rates['controller_state'], 1)} / {fmt(rates['lowcmd'], 1)} Hz"
    )
    print(
        "  flight relapsed after landing: "
        f"{phase['has_flight_relapse_after_landing']}"
    )
    print(
        "  max contact count during flight: "
        f"{phase['max_contact_count_in_flight'] if phase['max_contact_count_in_flight'] is not None else 'n/a'}"
    )
    print(
        "  mainly airborne translation: "
        f"{motion['mainly_airborne_translation']}"
    )


class TrialRecorder(Node):
    def __init__(self, start_time_s: float) -> None:
        super().__init__("go2_jump_trial_recorder")
        self.start_time_s = start_time_s
        self.controller_samples: List[Dict[str, Any]] = []
        self.sport_samples: List[Dict[str, Any]] = []
        self.lowcmd_times: List[float] = []
        self.first_controller_time_s: Optional[float] = None

        self.create_subscription(
            JumpControllerState,
            "/go2_jump/controller_state",
            self.on_controller_state,
            100,
        )
        self.create_subscription(
            SportModeState, "/sportmodestate", self.on_sport_state, 100
        )
        if LowCmd is not None:
            self.create_subscription(LowCmd, "/lowcmd", self.on_lowcmd, 200)

    def now_s(self) -> float:
        return time.monotonic() - self.start_time_s

    def on_controller_state(self, msg: JumpControllerState) -> None:
        if self.first_controller_time_s is None:
            self.first_controller_time_s = self.now_s()
        self.controller_samples.append(
            {
                "t": self.now_s(),
                "phase": msg.phase,
                "target_distance_m": msg.target_distance_m,
                "task_elapsed_s": msg.task_elapsed_s,
                "contact_count": int(msg.contact_count),
                "contact_override": bool(msg.contact_override),
                "desired_forward_velocity_mps": msg.desired_forward_velocity_mps,
                "desired_vertical_velocity_mps": msg.desired_vertical_velocity_mps,
                "body_pitch_deg": msg.body_pitch_deg,
                "forward_velocity_mps": msg.forward_velocity_mps,
                "vertical_velocity_mps": msg.vertical_velocity_mps,
            }
        )

    def on_sport_state(self, msg: SportModeState) -> None:
        self.sport_samples.append(
            {
                "t": self.now_s(),
                "x": float(msg.position[0]),
                "y": float(msg.position[1]),
                "z": float(msg.position[2]),
                "vx": float(msg.velocity[0]),
                "vy": float(msg.velocity[1]),
                "vz": float(msg.velocity[2]),
                "body_height": float(msg.body_height),
                "foot_force": [int(value) for value in msg.foot_force],
            }
        )

    def on_lowcmd(self, _msg: LowCmd) -> None:
        self.lowcmd_times.append(self.now_s())


def main() -> int:
    parser = argparse.ArgumentParser(description="Record and summarize one jump trial.")
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--output-json", type=str, default="")
    parser.add_argument("--wait-for-controller-timeout", type=float, default=20.0)
    args = parser.parse_args()

    rclpy.init()
    start_time_s = time.monotonic()
    node = TrialRecorder(start_time_s)

    try:
        initial_deadline_s = start_time_s + args.wait_for_controller_timeout
        final_deadline_s: Optional[float] = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now_s = time.monotonic()
            if node.first_controller_time_s is not None and final_deadline_s is None:
                final_deadline_s = now_s + args.duration
            if final_deadline_s is not None and now_s >= final_deadline_s:
                break
            if final_deadline_s is None and now_s >= initial_deadline_s:
                break
    finally:
        summary = compute_summary(
            node.controller_samples,
            node.sport_samples,
            node.lowcmd_times,
            args.duration,
        )
        report = {
            "summary": summary,
            "controller_samples": node.controller_samples,
            "sport_samples": node.sport_samples,
            "lowcmd_times_s": node.lowcmd_times,
        }
        if args.output_json:
            os.makedirs(os.path.dirname(args.output_json), exist_ok=True)
            with open(args.output_json, "w", encoding="utf-8") as handle:
                json.dump(report, handle, indent=2, ensure_ascii=False)
        print_summary(summary)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
