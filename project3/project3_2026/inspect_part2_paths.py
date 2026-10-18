"""Choose examples near query medians, inspect plan_arm --visualize, save Viser views.

Requires Playwright and a browser. Run after timed benchmarking has finished.
Examples are fresh illustrative runs, never added to the benchmark statistics.
"""
import argparse
import csv
import json
from pathlib import Path
import subprocess
import sys
import threading

import numpy as np
from PIL import Image
from playwright.sync_api import sync_playwright
from yourdfpy import URDF
from viser_motion_planning import Visualizer

from arm_scene import PANDA_URDF, scene_environment
from visualize_path import playback_samples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--browser-executable')
    parser.add_argument('--reuse-candidates', action='store_true')
    args = parser.parse_args()
    out = Path('results/part2/paths')
    out.mkdir(parents=True, exist_ok=True)
    specs = [('RTP', ['--goal-bias', '.2']),
             ('RRT', ['--goal-bias', '.05', '--range', '1']),
             ('RRTConnect', ['--range', '1']), ('PRM', ['--max-neighbors', '10'])]
    records = []
    # All planning candidates run sequentially, in independent Python processes.
    for planner, options in specs:
        candidates = []
        for seed in range(18101, 18106):
            dest = out / f'{planner}_seed{seed}.json'
            command = [sys.executable, 'plan_arm.py', '--planner', planner, *options,
                       '--query', 'query1', '--time-limit', '5', '--seed', str(seed),
                       '--output', str(dest)]
            if not args.reuse_candidates or not dest.exists():
                with (out / f'{planner}_seed{seed}.txt').open('w') as log:
                    subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            result = json.loads(dest.read_text())
            candidates.append(dict(seed=seed, file=str(dest), status=result['status'],
                                   length=result['length'], time=result['planning_time']))
        exact = sorted([r for r in candidates if r['status'] == 'exact'], key=lambda r: r['length'])
        if not exact:
            raise RuntimeError(f'No exact illustrative candidate for {planner}')
        config = {'RTP':'RTP_gb0.2', 'RRT':'RRT_gb0.05_r1.0',
                  'RRTConnect':'RRTConnect_r1.0', 'PRM':'PRM_k10'}[planner]
        summaries = list(csv.DictReader(Path('results/part2/per_query.csv').open()))
        target = float(next(s['solution_length_median'] for s in summaries
                            if s['config'] == config and s['query'] == 'query1'))
        chosen = min(exact, key=lambda r: abs(r['length'] - target))
        records.append(dict(planner=planner, options=options, candidates=candidates,
                            chosen_seed=chosen['seed'], candidate_exact=len(exact),
                            benchmark_query1_median_length=target,
                            selection='Exact candidate closest to benchmark query1 median length among five prespecified seeds'))

    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=True, executable_path=args.browser_executable,
                                    args=['--use-gl=angle', '--use-angle=swiftshader',
                                          '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
        for index, record in enumerate(records):
            planner, seed = record['planner'], record['chosen_seed']
            dest = out / f'{planner}.json'
            port = 8210 + index
            command = [sys.executable, 'plan_arm.py', '--planner', planner, *record['options'],
                       '--query', 'query1', '--time-limit', '5', '--seed', str(seed),
                       '--output', str(dest), '--visualize', '--port', str(port)]
            with (out / f'{planner}_visualize.txt').open('w') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                try:
                    page = browser.new_page(viewport={'width': 1100, 'height': 900})
                    # A bounded retry waits for the actual plan_arm Viser server.
                    for attempt in range(90):
                        if process.poll() is not None:
                            raise RuntimeError(f'Viser exited; inspect {planner}_visualize.txt')
                        try:
                            page.goto(f'http://localhost:{port}', timeout=2000)
                            break
                        except Exception:
                            page.wait_for_timeout(500)
                    else:
                        raise RuntimeError('Viser server did not start')
                    page.wait_for_timeout(5000)
                    page.screenshot(path=str(out / f'{planner}_playback.png'))
                    page.get_by_text('Playing', exact=True).click()
                    page.wait_for_timeout(3000)
                    page.screenshot(path=str(out / f'{planner}_playback_mid.png'))
                    page.wait_for_timeout(3000)
                    record['playback_inspected'] = True
                    page.close()
                finally:
                    process.terminate()
                    process.wait(timeout=20)
            result = json.loads(dest.read_text())
            reference = next(r for r in record['candidates'] if r['seed'] == seed)
            assert result['status'] == 'exact'
            assert np.isclose(result['length'], reference['length'])
            record['saved'] = str(dest)
            record['metrics'] = {k: result[k] for k in ['length', 'smoothness', 'min_clearance', 'planning_time']}

            # A static Viser overview of that same raw path, with ghost waypoints.
            viz = Visualizer(port=8230 + index)
            viz.server.gui.configure_theme(control_layout='collapsible', show_logo=False)
            viz.add_grid()
            viz.add_environment(scene_environment(result['scene']))
            path = np.array(result['path'])
            viz.add_robot('start', PANDA_URDF, color=(.2,.5,1), opacity=.65).update_cfg(path[0])
            viz.add_robot('goal', PANDA_URDF, color=(.15,.8,.3), opacity=.65).update_cfg(path[-1])
            for i, q in enumerate(path[1:-1]):
                viz.add_robot(f'waypoint{i}', PANDA_URDF, color=(.95,.55,.1), opacity=.17).update_cfg(q)
            kin = URDF.load(PANDA_URDF, load_meshes=False)
            ee = []
            for q in playback_samples(path):
                kin.update_cfg(dict(zip(result['robot']['joint_names'], q, strict=True)))
                ee.append(kin.get_transform('panda_grasptarget')[:3, 3])
            viz.add_path_line('gripper', np.array(ee), color=(.8,.04,.04), line_width=3)
            ready = threading.Event()
            clients = []
            @viz.server.on_client_connect
            def connect(client):
                client.camera.up_direction = (0,0,1)
                client.camera.position = (2.2,-2.6,1.9)
                client.camera.look_at = (.25,0,.55)
                client.camera.fov = .75
                clients.append(client)
                ready.set()
            page = browser.new_page(viewport={'width': 1000, 'height': 900})
            page.goto(f'http://localhost:{8230 + index}')
            if not ready.wait(30):
                raise RuntimeError('No Viser overview client')
            page.wait_for_timeout(5000)
            pixels = clients[0].camera.get_render(height=900, width=1000, transport_format='png', timeout=30)
            im = Image.fromarray(pixels).convert('RGBA')
            Image.alpha_composite(Image.new('RGBA', im.size, 'white'), im).convert('RGB').save(
                f'fig/part2_{planner}.png')
            page.close()
            viz.server.stop()
        browser.close()
    (out / 'selection.json').write_text(json.dumps(records, indent=2) + '\n')
    print(json.dumps(records, indent=2))


if __name__ == '__main__':
    main()
