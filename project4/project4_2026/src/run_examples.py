"""Reproduce checkpoint examples in a NEW directory, with one fresh process per seed."""
import argparse
from pathlib import Path
import subprocess
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args()
a.output.mkdir(parents=True,exist_ok=False)
for system in ['pendulum','car']:
    for planner in ['RRT','KPIECE1']:
        for torque in ([3,5,10] if system=='pendulum' else [3]):
            name=f'{system}_{planner}'+(f'_{torque}' if system=='pendulum' else '')
            seed=451 if system=='car' and planner=='KPIECE1' else 450
            seconds=120 if seed==451 else (60 if system=='car' else 30)
            cmd=[str(ROOT/('Project4Car' if system=='car' else 'Project4Pendulum')),
                 '--planner',planner,'--torque',str(torque),'--seed',str(seed),'--time',str(seconds),
                 '--output',str((a.output/f'{name}.json').resolve())]
            r=subprocess.run(cmd,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            (a.output/f'{name}.log').write_text(r.stdout)
            print(name,'exit',r.returncode,flush=True)
            if r.returncode: raise SystemExit(r.returncode)
