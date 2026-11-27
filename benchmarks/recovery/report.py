"""Summarize paired simulator results and check their input identity."""

import argparse
from collections import Counter
from itertools import product
import json
from pathlib import Path

BASE_SCENARIOS = ("nominal", "receive", "deliver", "symmetric", "repeat")
DEFAULT_SEEDS = (17, 23, 41, 53, 67, 89)
MODES = ("hold", "recover")
LEGS = ["REACHED receive", "REACHED deliver", "REACHED return"]


def load_pairs(output, scenarios, seeds):
    pairs = {}
    codes = set()
    for scenario, seed in product(scenarios, seeds):
        pair = {}
        for mode in MODES:
            path = output / scenario / str(seed) / mode / "result.json"
            row = json.loads(path.read_text())
            if (row["scenario"], row["seed"], row["mode"]) != (scenario, seed, mode):
                raise ValueError(f"Trial identity mismatch: {path}")
            if row["outcome"] == "infrastructure_error":
                raise ValueError(f"Trial has no outcome: {path}: {row['reason']}")
            codes.add(row["code_id"])
            if row["injected"] != len(row["faults"]) or row["planned"] != len(row["fault_plan"]):
                raise ValueError(f"Inconsistent fault counts: {path}")
            for fault, plan in zip(row["faults"], row["fault_plan"], strict=False):
                if any(fault[key] != value for key, value in plan.items()):
                    raise ValueError(f"Injected fault differs from its plan: {path}")
            if row["injected"] > row["planned"]:
                raise ValueError(f"Unplanned injection: {path}")
            pair[mode] = row
        if pair["hold"]["fault_plan"] != pair["recover"]["fault_plan"]:
            raise ValueError(f"Different fault plans in {scenario}/{seed}")
        pairs[scenario, seed] = pair
    if len(codes) != 1:
        raise ValueError("Trials have different source fingerprints")
    return pairs, codes.pop()


def verified_completion(row):
    reached = [event for _, event in row["events"] if event.startswith("REACHED ")]
    return row["completed"] and reached == LEGS


def verified_reopening(row, fault):
    leg = fault["leg"]
    opened = fault["reopened_ns"]
    return (opened is not None and not fault["false_resume"] and not fault["unverified_resume"]
            and any(t >= opened and event == f"RESUMED {leg}" for t, event in row["events"]))


def resumed_once(row):
    return row["injected"] == 1 and verified_reopening(row, row["faults"][0])


def held_for_operator(row):
    if row["outcome"] != "needs_help" or row["injected"] != row["planned"]:
        return False
    fault = row["faults"][-1]
    reason = "ambiguous_pose" if row["scenario"] == "symmetric" else "retry_limit"
    return (row["reason"] == reason and fault["reopened_ns"] is None
            and fault["held_ns"] is not None and fault["zero_s"] is not None
            and (row["scenario"] != "repeat" or verified_reopening(row, row["faults"][0])))


def print_report(pairs, code, output):
    rows = [row for pair in pairs.values() for row in pair.values()]
    recover = [row for row in rows if row["mode"] == "recover"]
    print("# Paired warehouse recovery")
    print(f"\n{len(pairs)} pairs, {len(rows)} runs. Input fingerprint `{code}`.")
    print("\n| Scenario | Hold only delivered | Recovery delivered | Hold only completed | Recovery completed |")
    print("| --- | ---: | ---: | ---: | ---: |")
    for scenario in dict.fromkeys(key[0] for key in pairs):
        group = [pair for key, pair in pairs.items() if key[0] == scenario]
        totals = [sum(measure(pair[mode]) for pair in group) for measure in
                  (lambda row: row["delivered"], verified_completion) for mode in MODES]
        print(f"| {scenario} | " + " | ".join(f"{count}/{len(group)}" for count in totals) + " |")
    print("\n| Recovery behavior | Observed |")
    print("| --- | ---: |")
    continuations = [row for row in recover if row["planned"] == 1 and row["fault_plan"][0]["resumable"]]
    if continuations:
        resumed = [row for row in continuations if resumed_once(row)]
        print(f"| Verified pose and resumed the interrupted leg | {len(resumed)}/{len(continuations)} |")
        print(f"| Verified recovery, same-leg resume and all stops reached | "
              f"{sum(verified_completion(row) for row in resumed)}/{len(continuations)} |")
    for scenario, label in (("symmetric", "Held after ambiguous localization"),
                            ("repeat", "Held after a second relocation on the retried leg")):
        group = [row for row in recover if row["scenario"] == scenario]
        if group:
            print(f"| {label} | {sum(held_for_operator(row) for row in group)}/{len(group)} |")
    faults = [fault for row in rows for fault in row["faults"]]
    print(f"\nConfirmed loss on {sum(fault['lost_ns'] is not None for fault in faults)}/{len(faults)} relocations.")
    openings = sum(fault["reopened_ns"] is not None for fault in faults)
    print(f"False or unverified reopenings {sum(fault['false_resume'] or fault['unverified_resume'] for fault in faults)}/{openings}.")
    print(f"Unmatched HOLD events {sum(row['false_stops'] for row in rows)}.")
    attempts = (json.loads(path.read_text()) for scenario, seed in pairs for mode in MODES
                for path in (output / scenario / str(seed) / mode).glob("attempt-*/result.json"))
    counts = Counter(row["outcome"] for row in attempts)
    print(f"Infrastructure errors across recorded attempts {counts['infrastructure_error']}.")
    anomalies = [row for row in rows if row["outcome"] in {"mission_failed", "deadline"} or row["false_stops"]]
    if anomalies:
        print("\n## Runs with mission failures, deadlines or unmatched holds\n")
        for row in anomalies:
            label = f"{row['scenario']}/{row['seed']}/{row['mode']}"
            print(f"- {label} · {row['outcome']} {row['reason']} · {row['false_stops']} unmatched HOLD")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--scenarios", nargs="+", default=BASE_SCENARIOS)
    parser.add_argument("--seeds", nargs="+", type=int, default=DEFAULT_SEEDS)
    args = parser.parse_args()
    pairs, fingerprint = load_pairs(args.output, tuple(dict.fromkeys(args.scenarios)), tuple(dict.fromkeys(args.seeds)))
    print_report(pairs, fingerprint, args.output)


if __name__ == "__main__":
    main()
