#!/usr/bin/env python3
"""Run the approved PR465 pair with the existing, unchanged replay observer."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'ws_livox/src/FAST-LIVO2-master'


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry / 'cmdline').read_bytes().split(b'\0')
        except (OSError, ProcessLookupError):
            continue
        if argv and Path(os.fsdecode(argv[0])).name == 'fastlivo_mapping':
            raise RuntimeError('Existing fastlivo_mapping found; no replay started')
    contract = json.loads((ROOT / 'experiments/pr465/contract.json').read_text())
    args.out.mkdir(parents=True, exist_ok=False)
    binaries = {'baseline': args.baseline.resolve(), 'candidate': args.candidate.resolve()}
    manifest = dict(contract=contract, runs={}, binaries={k: {'path': str(v), 'sha256': digest(v)} for k, v in binaries.items()},
                    runner_sha256=digest(Path(__file__)),
                    observer_sha256=digest(PKG / 'scripts/benchmark_tf_lifetime.py'),
                    configs_sha256={name: digest(PKG / 'config' / name) for name in ['nav_test.yaml', 'camera_pinhole.yaml']})
    manifest_path = args.out / 'manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
    for name in contract['run_order']:
        variant, dataset = name.split('_', 1)
        bag = contract['bags'][dataset]
        for filename, sha in bag['sha256'].items():
            if digest(Path(bag['path']) / filename) != sha:
                raise RuntimeError('Frozen bag changed: ' + filename)
        telemetry_file = (args.out / (name + '_tegrastats.log')).open('w')
        telemetry = subprocess.Popen(['tegrastats', '--interval', '1000'], stdout=telemetry_file, stderr=subprocess.STDOUT)
        command = [sys.executable, str(PKG / 'scripts/benchmark_tf_lifetime.py'), 'run',
                   '--binary', str(binaries[variant]), '--out', str(args.out / name), '--bag', bag['path'],
                   '--domain', '176', '--surf-filter', '0.15', '--playback-timeout', str(bag['duration_s'] + 90),
                   '--post-play-seconds', '30']
        wrapped = ['bash', str(ROOT / 'scripts/run_trial.sh'), 'pr465-' + name, '--', *command]
        started = time.monotonic()
        print('START', name, flush=True)
        try:
            result = subprocess.run(wrapped, cwd=ROOT, check=False)
        finally:
            telemetry.terminate()
            telemetry.wait(timeout=10)
            telemetry_file.close()
        manifest['runs'][name] = {'command': wrapped, 'exit_code': result.returncode,
                                  'wall_s': time.monotonic() - started}
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
        if result.returncode:
            raise RuntimeError(name + ' failed; inspect saved raw logs')
        worktree = Path('/tmp/fastlivo_pr465_' + variant)
        shutil.copytree(worktree / 'ws_livox/src/FAST-LIVO2-master/Log', args.out / name / 'source_logs')
        assert digest(binaries[variant]) == manifest['binaries'][variant]['sha256']
        for config, sha in manifest['configs_sha256'].items():
            assert digest(PKG / 'config' / config) == sha
        print('DONE', name, flush=True)
    subprocess.run([sys.executable, str(ROOT / 'scripts/summarize_pr465_ab.py'), str(args.out)], check=True)


if __name__ == '__main__':
    main()
