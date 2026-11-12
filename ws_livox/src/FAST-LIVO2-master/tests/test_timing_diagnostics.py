"""Exercise actual CSV capture and offline statistics without ROS or sensors."""
import csv
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TimingDiagnosticsTest(unittest.TestCase):
    def test_capture_and_summary(self):
        self.assertTrue((ROOT / 'include/timing_diagnostics.h').is_file(),
                        'diagnostic recorder has not been implemented')
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            source = work / 'exercise.cpp'
            source.write_text(r'''
#include "timing_diagnostics.h"
#include <cassert>
#include <csignal>
#include <sys/resource.h>
int main(int argc, char **argv) {
  fastlivo_diag::Recorder recorder(argv[1]);
  assert(recorder.enabled());
  for (int i = 1; i <= 1200; ++i) {
    fastlivo_diag::Row row;
    row.event = "full_cycle";
    row.frame_id = i;
    row.start_steady_s = i * 0.1;
    row.duration_ms = i;
    row.status = 1;
    row.ros_s = 1000 + i;
    row.scan_end_s = row.ros_s - 0.25;
    row.imu_q = 20;
    recorder.record(row);
  }
  // Several input callbacks in a spin must not inherit a processed-frame id.
  recorder.frame_id = 1200;
  recorder.scan_end_s = 70;
  recorder.target_end_s = 71;
  recorder.beginInputLoop();
  { auto a = recorder.scope("scenario_callback"); a.row.raw_stamp_s = 80; }
  { auto b = recorder.scope("scenario_callback"); b.row.raw_stamp_s = 81; }
  recorder.frame_id = 1201;
  recorder.target_end_s = 80.1;
  {
    auto process = recorder.scope("scenario_process");
    process.row.state_before_s = 70;
    process.row.scan_end_s = 80.1;
  }
  {
    auto scope = recorder.scope("early_return");
    scope.row.status = -3;
  }
  recorder.close();
  assert(!recorder.enabled());
  // A failed diagnostics path must not throw or prevent the algorithm running.
  fastlivo_diag::Recorder failed("/dev/null/events.csv");
  assert(!failed.enabled());
  failed.record(fastlivo_diag::Row{});
  // Simulate disk exhaustion while the final partial buffer is being written.
  fastlivo_diag::Recorder exhausted(std::string(argv[1]) + ".exhausted");
  assert(exhausted.enabled());
  std::signal(SIGXFSZ, SIG_IGN);
  struct rlimit limit{0, RLIM_INFINITY};
  assert(setrlimit(RLIMIT_FSIZE, &limit) == 0);
  fastlivo_diag::Row large;
  large.event = "write_failure_test_012345678901234567890123456789012345678901234567890123456789012345678901234567890123456789";
  for (int i = 0; i < 500; ++i) exhausted.record(large);
  exhausted.close();
  assert(!exhausted.enabled());
}
''')
            binary = work / 'exercise'
            subprocess.run(['g++', '-std=c++17', '-O3', '-ffast-math', '-Wall', '-Wextra',
                            '-I', str(ROOT / 'include'), str(source), '-o', str(binary)],
                           check=True, capture_output=True, text=True)
            capture = work / 'events.csv'
            subprocess.run([str(binary), str(capture)], check=True,
                           capture_output=True, text=True)
            with capture.open() as stream:
                rows = list(csv.DictReader(stream))
            frames = [r for r in rows if r['event'] == 'full_cycle']
            self.assertEqual(len(frames), 1200, 'buffer rollover must lose no records')
            self.assertEqual([int(r['frame_id']) for r in frames], list(range(1, 1201)))
            self.assertTrue(all(float(r['age_s']) == 0.25 for r in frames))
            self.assertTrue(any(r['event'] == 'diagnostic_flush' for r in rows))
            callbacks = [r for r in rows if r['event'] == 'scenario_callback']
            self.assertEqual(len(callbacks), 2)
            self.assertTrue(all(r['frame_id'] == '0' and r['loop_id'] == '1' for r in callbacks))
            self.assertTrue(all(math.isnan(float(r['scan_end_s'])) and
                                math.isnan(float(r['target_end_s'])) for r in callbacks))
            process = next(r for r in rows if r['event'] == 'scenario_process')
            self.assertEqual(process['frame_id'], '1201')
            self.assertEqual(float(process['state_before_s']), 70)
            self.assertEqual(float(process['scan_end_s']), 80.1)
            early = next(r for r in rows if r['event'] == 'early_return')
            self.assertEqual(int(early['status']), -3)
            self.assertGreaterEqual(float(early['duration_ms']), 0)
            result = subprocess.run(['python3', str(ROOT / 'scripts/summarize_timing.py'),
                                     str(capture)], check=True, capture_output=True, text=True)
            summary = json.loads(result.stdout)
            cycle = summary['events']['full_cycle']['duration_ms']
            self.assertEqual(cycle['count'], 1200)
            self.assertAlmostEqual(cycle['mean'], 600.5)
            self.assertEqual(cycle['p95'], 1140, 'P95 uses nearest rank')
            self.assertEqual(cycle['max'], 1200)
            self.assertEqual(summary['events']['full_cycle']['age_s']['p95'], 0.25)

    def test_empty_and_nonfinite_statistics(self):
        path = ROOT / 'scripts/summarize_timing.py'
        self.assertTrue(path.is_file(), 'offline statistics have not been implemented')
        spec = importlib.util.spec_from_file_location('summarize_timing', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertEqual(module.stats([]), {'count': 0})
        self.assertEqual(module.stats([float('nan'), float('inf')]), {'count': 0})
        self.assertEqual(module.stats([1, 2, 3, 100])['p95'], 100)

    def test_status_groups_and_nested_event_order(self):
        path = ROOT / 'scripts/summarize_timing.py'
        spec = importlib.util.spec_from_file_location('timing_summary_groups', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        fields = ('event', 'start_steady_s', 'count', 'status', 'duration_ms',
                  'age_s', 'header_age_s', 'corrected_header_age_s', 'callback_gap_ms',
                  'imu_q', 'lidar_q', 'prop_imu_q', 'input_points', 'down_points',
                  'effective_points', 'map_voxels', 'path_poses', 'iterations')
        with tempfile.TemporaryDirectory() as directory:
            capture = Path(directory) / 'nested.csv'
            with capture.open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for status, stamp, duration in [(0, 50, 1), (1, 49, 150), (0, 51, 2)]:
                    row = dict.fromkeys(fields, 'nan')
                    row.update(event='full_cycle', start_steady_s=stamp, count=1,
                               status=status, duration_ms=duration)
                    writer.writerow(row)
            result = module.summarize(str(capture))['events']['full_cycle']
            self.assertEqual(result['duration_ms']['count'], 3)
            self.assertEqual(result['by_status']['1']['duration_ms']['mean'], 150)
            self.assertEqual(result['by_status']['0']['duration_ms']['mean'], 1.5)
            skipped = module.summarize(str(capture), 1)['events']['full_cycle']
            self.assertEqual(skipped['duration_ms']['count'], 2)
            self.assertEqual(skipped['statuses'], {'0': 2})


if __name__ == '__main__':
    unittest.main()
