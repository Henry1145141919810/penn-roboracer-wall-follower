#!/usr/bin/env python3
"""Black-box tuning of the controller with CMA-ES.

Use ONE worker. Several headless simulators on the same PC change the
simulator's behaviour (measured: ~190 new LiDAR scans/s instead of 40, and the
final config crashed on all three), so parallel results do not transfer.

Each worker owns one simulator (its own container, bridge and port) and runs
jobs that the optimiser writes into runs/tune/<worker>/. A job is one call of
tools/experiment.py (reset + 3 laps). The score is the mean flying lap; a wall
contact scores CRASH_SCORE, so the optimiser only rewards clean configs.

    pip3 install cma==3.3.0                              # in each container that optimises
    python3 tools/tune.py worker w1                      # one per container/simulator
    python3 tools/tune.py optimise --workers w1 --generations 10 --popsize 6

Results: runs/tune/history.jsonl (every evaluation) and the best config printed
at the end. The search space and start point are defined below.
"""

import argparse
import glob
import json
import os
import subprocess
import sys
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(TOOLS, '..', 'runs', 'tune')
CRASH_SCORE = 12.0  # s; worse than any clean lap seen so far

# name: (low, high, start). Start = the hand-tuned config plus traction control.
SPACE = {
    'v_max': (6.0, 9.0, 7.0),
    'lat_accel': (5.5, 8.0, 6.5),
    'brake_decel': (3.0, 8.0, 4.0),
    'v_corner': (0.0, 3.0, 0.0),
    'stop_margin': (0.1, 0.6, 0.5),
    'preview_gain': (0.1, 0.6, 0.3),
    'kp': (0.6, 1.4, 0.9),
    'lookahead': (0.3, 1.0, 0.5),
    'traction_slip': (0.05, 0.3, 0.1),
    'side_clearance': (0.25, 0.4, 0.35),
}


def to_params(x):
    """Map a point of the unit cube to parameter values."""
    out = {}
    for xi, (name, (lo, hi, _)) in zip(x, SPACE.items()):
        out[name] = round(lo + min(max(xi, 0.0), 1.0) * (hi - lo), 4)
    return out


def worker(wid):
    """Run job files for this worker until killed."""
    jobs = os.path.join(ROOT, wid)
    os.makedirs(jobs, exist_ok=True)
    print(f'worker {wid} waiting for jobs in {jobs}', flush=True)
    while True:
        pending = sorted(glob.glob(os.path.join(jobs, 'job_*.json')))
        if not pending:
            time.sleep(0.5)
            continue
        path = pending[0]
        job = json.load(open(path))
        cmd = [sys.executable, os.path.join(TOOLS, 'experiment.py'), '--laps', '3',
               '--timeout', '90', '--label', job['label'], '--out-dir', jobs]
        for k, v in job['params'].items():
            cmd += ['-p', f'{k}={v}']
        with open(os.path.join(jobs, 'worker.log'), 'a') as log:
            subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT)
        result = None  # stays None (scored as a crash) if the run failed
        results = os.path.join(jobs, 'results.jsonl')
        if os.path.exists(results):
            with open(results) as f:
                for line in f:
                    r = json.loads(line)
                    if r['label'] == job['label']:
                        result = r
        out = path.replace('job_', 'result_')
        json.dump(result, open(out + '.tmp', 'w'))
        os.replace(out + '.tmp', out)
        os.remove(path)


def score(result):
    if result is None or result['status'] != 'ok' or len(result['laps']) < 3:
        return CRASH_SCORE
    flying = result['laps'][1:]
    return sum(flying) / len(flying)


def optimise(workers, generations, popsize, sigma, tag='tune', start=None, bounds=()):
    import cma
    for item in bounds:  # e.g. kp=0.3:1.0 widens or moves a range
        name, rng = item.split('=')
        lo, hi = (float(v) for v in rng.split(':'))
        SPACE[name] = (lo, hi, SPACE[name][2])
    for name, value in (start or {}).items():
        lo, hi, _ = SPACE[name]
        SPACE[name] = (lo, hi, value)
    names = list(SPACE)
    x0 = [(SPACE[n][2] - SPACE[n][0]) / (SPACE[n][1] - SPACE[n][0]) for n in names]
    es = cma.CMAEvolutionStrategy(x0, sigma, {'bounds': [0, 1], 'popsize': popsize,
                                              'seed': 1, 'verbose': -9})
    history = os.path.join(ROOT, 'history.jsonl')
    best = (float('inf'), None)
    for gen in range(generations):
        xs = es.ask()
        tickets = []
        for i, x in enumerate(xs):
            wid = workers[i % len(workers)]
            label = f'{tag}_g{gen:02d}_{i:02d}'
            job = {'label': label, 'params': to_params(x)}
            os.makedirs(os.path.join(ROOT, wid), exist_ok=True)
            path = os.path.join(ROOT, wid, f'job_{label}.json')
            json.dump(job, open(path + '.tmp', 'w'))
            os.replace(path + '.tmp', path)
            tickets.append(os.path.join(ROOT, wid, f'result_{label}.json'))
        scores = []
        for x, ticket in zip(xs, tickets):
            while not os.path.exists(ticket):
                time.sleep(1.0)
            result = json.load(open(ticket))
            s = score(result)
            scores.append(s)
            entry = {'tag': tag, 'gen': gen, 'score': s, 'params': to_params(x),
                     'laps': result['laps'] if result else None,
                     'status': result['status'] if result else 'missing'}
            with open(history, 'a') as f:
                f.write(json.dumps(entry) + '\n')
            if s < best[0]:
                best = (s, to_params(x))
        es.tell(xs, scores)
        clean = sum(s < CRASH_SCORE for s in scores)
        print(f'gen {gen}: best {min(scores):.3f}  clean {clean}/{len(scores)}  '
              f'overall best {best[0]:.3f}', flush=True)
    print('BEST', json.dumps({'score': best[0], 'params': best[1]}), flush=True)


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    w = sub.add_parser('worker')
    w.add_argument('wid')
    o = sub.add_parser('optimise')
    o.add_argument('--workers', nargs='+', required=True)
    o.add_argument('--generations', type=int, default=10)
    o.add_argument('--popsize', type=int, default=6)
    o.add_argument('--sigma', type=float, default=0.2)
    o.add_argument('--tag', default='tune', help='label prefix (must differ between runs)')
    o.add_argument('--start', default='', help='JSON dict of start values')
    o.add_argument('--bounds', nargs='*', default=[], help='name=low:high overrides')
    args = ap.parse_args()
    if args.cmd == 'worker':
        worker(args.wid)
    else:
        optimise(args.workers, args.generations, args.popsize, args.sigma, args.tag,
                 json.loads(args.start) if args.start else None, args.bounds)


if __name__ == '__main__':
    main()
