"""Independent SciPy replay, saved-path audit, and path figures (checkpoint and final).
Run from project4_2026 with: python src/analyze_paths.py
"""
from pathlib import Path
import csv
import json
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/project4-matplotlib')
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
ROOT = Path(__file__).resolve().parents[1]
OBSTACLES = [(3,0,1,6.5),(6,3.5,1,6.5),(9,0,1,6.5)]

def wrap(x):
    return (x+np.pi) % (2*np.pi)-np.pi

def audit(path):
    d = json.loads(path.read_text())
    car = d['system'] == 'car'
    states, controls, durations, dense = [np.array(d[k], dtype=float) for k in ('states','controls','durations','dense')]
    ai = 2 if car else 0
    assert states.shape == (len(durations)+1, 4 if car else 2)
    assert controls.shape == (len(durations), 2 if car else 1)
    assert all(np.isfinite(a).all() for a in (states, controls, durations, dense))
    assert d['exact'] and np.all(durations >= .02-1e-12) and np.all(durations <= .5+1e-12)
    assert np.max(np.abs(durations/.02-np.round(durations/.02))) < 1e-10
    assert np.max(np.abs(controls)) <= (1 if car else d['torque'])+1e-12
    start = [1,1,np.pi/2,0] if car else [-np.pi/2,0]
    assert np.max(np.abs(states[0]-start)) < 1e-12
    samples = [states[0:1]]
    geometric_replay = [states[0:1]]
    max_error = 0.
    cumulative = states[0].copy()
    cumulative_error = 0.
    def rhs(u):
        if car:
            return lambda t,q: [q[3]*np.cos(q[2]), q[3]*np.sin(q[2]), u[0], u[1]]
        return lambda t,q: [q[1], -9.81*np.cos(q[0])+u[0]]
    for i, (u,dt) in enumerate(zip(controls,durations)):
        t = np.linspace(0,dt,int(np.ceil(dt/.002))+1)
        sol = solve_ivp(rhs(u), (0,dt), states[i], t_eval=t, rtol=1e-11, atol=1e-12, dense_output=True)
        assert sol.success
        grid = np.linspace(0,dt,int(round(dt/.02))+1)
        geometric_replay.append(sol.sol(grid).T[1:])
        replay = sol.y.T
        err = replay[-1]-states[i+1]; err[ai] = wrap(err[ai])
        max_error = max(max_error, np.max(np.abs(err)))
        assert max_error < 1e-7, (path.name, i, max_error)
        accum = solve_ivp(rhs(u), (0,dt), cumulative, rtol=1e-11, atol=1e-12)
        cumulative = accum.y[:,-1]
        cumulative[ai] = wrap(cumulative[ai])
        err = cumulative-states[i+1]; err[ai] = wrap(err[ai])
        cumulative_error = max(cumulative_error, np.max(np.abs(err)))
        replay[:,ai] = wrap(replay[:,ai]); samples.append(replay[1:])
    replay = np.concatenate(samples)
    geometric_replay = np.concatenate(geometric_replay)
    assert geometric_replay.shape == dense.shape
    delta = geometric_replay-dense; delta[:,ai] = wrap(delta[:,ai])
    assert np.max(np.abs(delta)) < 1e-7
    assert cumulative_error < 1e-6
    # Car speed is linear within a segment, so its extremes are the (sampled) endpoints; pendulum speed can
    # move by at most .001*(g+torque) between samples .002 s apart.
    assert np.max(np.abs(replay[:,-1])) + (0 if car else .001*(9.81+d['torque'])) <= (2 if car else 10)
    for points in (states, dense, replay):
        assert np.max(np.abs(points[:,ai])) <= np.pi+1e-10
        assert np.max(np.abs(points[:,-1])) <= (2 if car else 10)+1e-9
        if car:
            assert np.all(points[:,:2] >= -1e-10) and np.all(points[:,:2] <= [12+1e-10,10+1e-10])
            for x,y,w,h in OBSTACLES:
                assert not np.any((points[:,0]>=x)&(points[:,0]<=x+w)&(points[:,1]>=y)&(points[:,1]<=y+h))
    if car:
        goal_distance = np.linalg.norm(states[-1,:2]-[11,9])+.5*abs(wrap(states[-1,2]-np.pi/2))+abs(states[-1,3])
        assert goal_distance <= .5+1e-10
        gaps=[]
        for x,y,w,h in OBSTACLES:
            delta=np.maximum(np.maximum([x,y]-replay[:,:2],replay[:,:2]-[x+w,y+h]),0)
            gaps.append(np.linalg.norm(delta,axis=1))
        clearance = float(np.min(gaps))
        # Speed <=2: over .002 s any point is <=.002 from its nearest sample.
        assert clearance > .002, 'Not enough clearance for the conservative dense-sampling check'
    else:
        goal_distance = abs(wrap(states[-1,0]-np.pi/2))+abs(states[-1,1])
        assert goal_distance <= .15+1e-10
        clearance = None
    assert abs(goal_distance-d['goal_distance']) < 1e-9
    result = dict(file=path.name, system=d['system'], planner=d['planner'], torque=d['torque'] if not car else '',
                  seed=d['seed'], solve_seconds=d['time'], trajectory_seconds=float(sum(durations)),
                  controls=len(controls), goal_distance=float(goal_distance), max_abs_speed=float(max(abs(replay[:,-1]))),
                  local_replay_max_error=float(max_error), cumulative_replay_max_error=float(cumulative_error),
                  samples=len(replay), min_obstacle_clearance=clearance)
    return d, result

def main():
    paths=sorted((ROOT/'results').glob('pendulum_*.json'))+sorted((ROOT/'results').glob('car_*.json'))
    assert len(paths)==12, 'Expected nine pendulum paths and three car paths'
    records=[]; data={}
    for path in paths:
        d,r=audit(path); records.append(r)
        data[(d['system'],d['planner'],d['torque'] if d['system']=='pendulum' else 0)]=d
        print(path.name, 'PASS', 'local error',r['local_replay_max_error'])
    with (ROOT/'results'/'summary.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=records[0].keys()); writer.writeheader(); writer.writerows(records)
    (ROOT/'results'/'validation.json').write_text(json.dumps({'method':'Independent SciPy RK45 adaptive replay, rtol=1e-11, atol=1e-12; samples <=0.002 s. Local and cumulative integration.', 'paths':records},indent=2)+'\n')
    figure(data,['RRT','KPIECE1'],'checkpoint_paths')
    figure(data,['RRT','KPIECE1','RG-RRT'],'all_paths')
    print('All twelve paths passed independent validation.')

def figure(data,planners,name):
    fig,axes=plt.subplots(len(planners),4,figsize=(10,2.12*len(planners)+.0),layout='constrained')
    for row,planner in enumerate(planners):
        for col,torque in enumerate([3,5,10]):
            d=data['pendulum',planner,torque]; q=np.array(d['dense']); ax=axes[row,col]
            q=q.copy(); q[np.r_[False,abs(np.diff(q[:,0]))>np.pi],:]=np.nan
            ax.plot(q[:,0],q[:,1],lw=.8,color='#17659c')
            ax.scatter([-np.pi/2],[0],c='#27934b',s=26,marker='o',zorder=4)
            ax.scatter([np.pi/2],[0],c='#b42d32',s=38,marker='*',zorder=4)
            ax.set(xlim=(-np.pi,np.pi),ylim=(-10,10),xlabel=r'$\theta$ (rad)',ylabel=r'$\omega$ (rad/s)',title=f'{planner}, |τ| ≤ {torque}')
            ax.set_xticks([-np.pi,0,np.pi],['−π','0','π']);ax.grid(alpha=.2)
        d=data['car',planner,0]; q=np.array(d['dense']);ax=axes[row,3]
        for x,y,w,h in OBSTACLES: ax.add_patch(Rectangle((x,y),w,h,color='#555d65'))
        ax.plot(q[:,0],q[:,1],lw=1,color='#17659c')
        ax.scatter(1,1,c='#27934b',s=26,zorder=4); ax.scatter(11,9,c='#b42d32',s=38,marker='*',zorder=4)
        ax.set(xlim=(0,12),ylim=(0,10),xlabel='x (m)',ylabel='y (m)',title=f'{planner}, point car',aspect='equal'); ax.grid(alpha=.2)
    (ROOT/'fig').mkdir(exist_ok=True)
    fig.savefig(ROOT/'fig'/f'{name}.pdf');fig.savefig(ROOT/'fig'/f'{name}.png',dpi=180)
if __name__=='__main__': main()
