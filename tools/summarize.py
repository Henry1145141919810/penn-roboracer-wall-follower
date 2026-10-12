#!/usr/bin/env python3
"""Summarise runs/results.jsonl as a Markdown table, one row per label.

    python3 tools/summarize.py                 # every label
    python3 tools/summarize.py B1 B2 F0        # labels starting with these prefixes

Lap 1 of every run is dropped: the simulator's lap timer is not restarted by a
reset, so lap 1 also contains the idle time before the car starts. Laps 2+
are "flying" laps and are the only ones compared.
"""

import json
import math
import os
import sys

RESULTS = os.path.join(os.path.dirname(__file__), '..', 'runs', 'results.jsonl')


def main():
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

    print('| variant | clean runs | flying laps | best (s) | mean ± std (s) | collisions |')
    print('|---|---|---|---|---|---|')
    for label, runs in groups.items():
        clean = sum(r['status'] == 'ok' for r in runs)
        flying = [t for r in runs for t in r['laps'][1:]]
        coll = sum(r['collisions'] for r in runs)
        if flying:
            mean = sum(flying) / len(flying)
            std = math.sqrt(sum((t - mean) ** 2 for t in flying) / max(len(flying) - 1, 1))
            best, stats = f'{min(flying):.3f}', f'{mean:.3f} ± {std:.3f}'
        else:
            best, stats = '-', '-'
        print(f'| {label} | {clean}/{len(runs)} | {len(flying)} | {best} | {stats} | {coll} |')


if __name__ == '__main__':
    main()
