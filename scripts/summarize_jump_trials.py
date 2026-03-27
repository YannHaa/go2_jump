#!/usr/bin/env python3
import argparse
import json
import os
import statistics
from collections import defaultdict
from typing import Dict, List, Optional


def mean(values: List[float]) -> Optional[float]:
    return statistics.mean(values) if values else None


def stddev(values: List[float]) -> Optional[float]:
    return statistics.stdev(values) if len(values) >= 2 else 0.0 if values else None


def fmt(value: Optional[float], digits: int = 3) -> str:
    if value is None:
        return "n/a"
    return f"{value:.{digits}f}"


def load_reports(paths: List[str]) -> List[Dict]:
    reports = []
    for path in paths:
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        reports.append(
            {
                "path": path,
                "name": os.path.basename(os.path.dirname(path)),
                "summary": payload["summary"],
            }
        )
    return reports


def summarize_group(reports: List[Dict]) -> Dict:
    total_distances = [
        report["summary"]["motion"]["total_distance_m"]
        for report in reports
        if report["summary"]["motion"]["total_distance_m"] is not None
    ]
    airborne_distances = [
        report["summary"]["motion"]["airborne_distance_m"]
        for report in reports
        if report["summary"]["motion"]["airborne_distance_m"] is not None
    ]
    airborne_shares = [
        report["summary"]["motion"]["airborne_share"]
        for report in reports
        if report["summary"]["motion"]["airborne_share"] is not None
    ]
    lowcmd_rates = [
        report["summary"]["rates_hz"]["lowcmd"]
        for report in reports
        if report["summary"]["rates_hz"]["lowcmd"] is not None
    ]
    max_height_gains = [
        report["summary"]["motion"]["max_height_gain_m"]
        for report in reports
        if report["summary"]["motion"]["max_height_gain_m"] is not None
    ]
    targets = [
        report["summary"]["target_distance_m"]
        for report in reports
        if report["summary"]["target_distance_m"] is not None
    ]
    target_distance = targets[0] if targets else None
    errors = [
        value - target_distance
        for value in total_distances
        if target_distance is not None
    ]
    airborne_successes = sum(
        1
        for report in reports
        if report["summary"]["motion"]["mainly_airborne_translation"]
    )
    clean_phase_successes = sum(
        1
        for report in reports
        if not report["summary"]["phase"]["has_flight_relapse_after_landing"]
    )
    return {
        "count": len(reports),
        "target_distance_m": target_distance,
        "mean_total_distance_m": mean(total_distances),
        "std_total_distance_m": stddev(total_distances),
        "mean_error_m": mean(errors),
        "std_error_m": stddev(errors),
        "mean_airborne_distance_m": mean(airborne_distances),
        "std_airborne_distance_m": stddev(airborne_distances),
        "mean_airborne_share": mean(airborne_shares),
        "std_airborne_share": stddev(airborne_shares),
        "mean_lowcmd_rate_hz": mean(lowcmd_rates),
        "std_lowcmd_rate_hz": stddev(lowcmd_rates),
        "mean_peak_height_gain_m": mean(max_height_gains),
        "std_peak_height_gain_m": stddev(max_height_gains),
        "airborne_success_ratio": airborne_successes / len(reports) if reports else 0.0,
        "clean_phase_ratio": clean_phase_successes / len(reports) if reports else 0.0,
        "report_names": [report["name"] for report in reports],
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Aggregate multiple jump trial summary.json files."
    )
    parser.add_argument("paths", nargs="+", help="summary.json files to aggregate")
    parser.add_argument("--output-json", default="")
    args = parser.parse_args()

    reports = load_reports(args.paths)
    grouped: Dict[float, List[Dict]] = defaultdict(list)
    for report in reports:
        grouped[report["summary"]["target_distance_m"]].append(report)

    aggregate = {
        "report_count": len(reports),
        "groups": {},
    }

    for target_distance, group_reports in sorted(grouped.items()):
        aggregate["groups"][f"{target_distance:.3f}"] = summarize_group(group_reports)

    print("Jump batch summary")
    for key, group in aggregate["groups"].items():
        print(
            f"  d={key} m | n={group['count']} | total={fmt(group['mean_total_distance_m'])} +/- {fmt(group['std_total_distance_m'])}"
            f" | error={fmt(group['mean_error_m'])} +/- {fmt(group['std_error_m'])}"
            f" | airborne={fmt(group['mean_airborne_distance_m'])} +/- {fmt(group['std_airborne_distance_m'])}"
            f" | airborne_share={fmt(None if group['mean_airborne_share'] is None else 100.0 * group['mean_airborne_share'], 1)}%"
            f" | lowcmd={fmt(group['mean_lowcmd_rate_hz'], 1)} Hz"
            f" | airborne_ok={fmt(100.0 * group['airborne_success_ratio'], 1)}%"
            f" | clean_phase={fmt(100.0 * group['clean_phase_ratio'], 1)}%"
        )

    if args.output_json:
        with open(args.output_json, "w", encoding="utf-8") as handle:
            json.dump(aggregate, handle, indent=2, ensure_ascii=False)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
