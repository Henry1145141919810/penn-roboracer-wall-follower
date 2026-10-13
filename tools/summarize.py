#!/usr/bin/env python3
"""Summarise runs/results.jsonl as a Markdown table, one row per label.

    python3 tools/summarize.py                 # every label
    python3 tools/summarize.py B1 B2 F0        # labels starting with these prefixes
    python3 tools/summarize.py experiments/results_final.jsonl   # a saved results file

Lap 1 of every run is dropped: the simulator's lap timer is not restarted by a
reset, so lap 1 also contains the idle time before the car starts. Laps 2+
are "flying" laps and are the only ones compared. Laps within one run are correlated
(same reset and warm-up), so the standard deviation of the run means is shown as well.
"""

import json
import math
import os
import sys

RESULTS = os.path.join(os.path.dirname(__file__), '..', 'runs', 'results.jsonl')


def main():
    global RESULTS
    if len(sys.argv) > 1 and sys.argv[1].endswith('.jsonl'):  # a saved results file
        RESULTS = sys.argv.pop(1)
    prefixes = sys.argv[1:]
    groups = {}
    with open(RESULTS) as f:
        for line in f:
            r = json.loads(line)
            if r.get('raw_throttle') is not None:
                continue
            if prefixes and not any(r['label'].startswith(p) for p in prefixes):
                continue
            groups.setdefault(r['label'], []).append(r)

    print('| variant | clean runs | flying laps | best (s) | mean ± lap std (s) | '
          'run-mean std (s) | collisions |')
    print('|---|---|---|---|---|---|---|')
    for label, runs in groups.items():
        clean = sum(r['status'] == 'ok' for r in runs)
        flying = [t for r in runs for t in r['laps'][1:]]
        coll = sum(r['collisions'] for r in runs)
        run_means = [sum(r['laps'][1:]) / len(r['laps'][1:]) for r in runs if len(r['laps']) > 1]
        if flying:
            mean = sum(flying) / len(flying)
            std = math.sqrt(sum((t - mean) ** 2 for t in flying) / max(len(flying) - 1, 1))
            rm = sum(run_means) / len(run_means)
            rstd = math.sqrt(sum((m - rm) ** 2 for m in run_means) / max(len(run_means) - 1, 1))
            best, stats = f'{min(flying):.3f}', f'{mean:.3f} ± {std:.3f}'
            rstats = f'{rstd:.3f} (n={len(run_means)})' if len(run_means) > 1 else '-'
        else:
            best, stats, rstats = '-', '-', '-'
        print(f'| {label} | {clean}/{len(runs)} | {len(flying)} | {best} | {stats} | {rstats} | '
              f'{coll} |')
        pairs = [(r['label'], r['run']) for r in runs]
        if len(set(pairs)) != len(pairs):
            print(f'  WARNING: {label} has repeated (label, run) rows')
        hashes = {r.get('node_sha256') for r in runs}
        if len(hashes) > 1:
            print(f'  WARNING: {label} spans {len(hashes)} controller versions {sorted(hashes)}')


if __name__ == '__main__':
    main()
