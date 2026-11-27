
'''Move the seven simulated aircraft onto the road and later land as a group.'''
import csv
from itertools import combinations
import json
from math import atan2, cos, sin
from time import monotonic

import numpy as np
import rclpy

from run_case1_gazebo import yaw_of
from x3_altitude_demo import TARGET_Z

PAIRS = tuple(combinations(range(7),2))


def fly_stage(node, output, geometry, *, landing=False):
    node.prepare()
    phase = 'landing_hold' if landing else 'takeoff'
    stage_name = 'landing' if landing else 'approach'
    values = np.array(geometry, dtype=float)
    phase_position = values[:, :2] if landing else None
    yaw_targets = None if landing else values[:, 2]
    next_tick = 0.
    steady_since = descend_since = first_stamp = None
    min_spacing = float('inf')
    log = output / f'{stage_name}_states.csv'
    output.mkdir(parents=True,exist_ok=True)
    started = monotonic()
    with log.open('w',newline='') as stream:
        records = csv.writer(stream)
        records.writerow(('sim_time_s','phase','id','x_m','y_m','z_m','speed_xy_m_s','speed_z_m_s'))
        print(f'MISSION {phase}',flush=True)
        while monotonic()-started < 80.:
            rclpy.spin_once(node,timeout_sec=.10)
            if any(monotonic()-a.received > .8 for a in node.aircraft):
                raise RuntimeError(f'Odometry stale during {phase}')
            stamps = [a.stamp for a in node.aircraft]
            stamp = min(stamps)
            if stamp < next_tick or max(stamps)-stamp > .07:
                continue
            next_tick = stamp+.095
            poses = [a.odom.pose.pose for a in node.aircraft]
            xyz = np.array([[p.position.x,p.position.y,p.position.z] for p in poses])
            speed = [a.odom.twist.twist.linear for a in node.aircraft]
            xy_speed = np.array([np.hypot(v.x,v.y) for v in speed])
            z_speed = np.array([abs(v.z) for v in speed])
            if first_stamp is None:
                first_stamp = stamp
            if phase_position is None:
                phase_position = xyz[:,:2].copy()
            errors = phase_position-xyz[:,:2]
            height = TARGET_Z if descend_since is None else max(0.,TARGET_Z-.22*(stamp-descend_since))
            spacing = min(float(np.linalg.norm(xyz[i]-xyz[j])) for i,j in PAIRS)
            min_spacing = min(min_spacing,spacing)
            if spacing < .8:
                raise RuntimeError(f'Inter-aircraft clearance guard triggered during {phase}')
            for i,aircraft in enumerate(node.aircraft):
                rate = 0.
                if yaw_targets is not None:
                    delta = yaw_targets[i]-yaw_of(poses[i].orientation)
                    rate = float(np.clip(1.5*atan2(sin(delta),cos(delta)),-.35,.35))
                aircraft.send_velocity(*(.92*errors[i]),.52,yaw_rate=rate,
                                       target_z=height,vertical_limit=.30 if landing else .50)
                records.writerow((stamp,phase,f'UAV-{i+1}',*xyz[i],xy_speed[i],z_speed[i]))

            max_error = float(max(np.linalg.norm(errors,axis=1)))
            level = max(abs(xyz[:,2]-TARGET_Z)) <= .035
            still = max(xy_speed) <= .04 and max(z_speed) <= .045
            if phase == 'road_entry':
                ready = level and still and max_error <= .025
            elif phase == 'descent':
                ready = height == 0. and max(xyz[:,2]) <= .085 and still and max_error <= .10
            else:
                ready = level and still and max_error <= .05
            if not ready:
                steady_since = None
            elif steady_since is None:
                steady_since = stamp
            elif stamp-steady_since >= (2.0 if phase == 'descent' else 1.0):
                if phase == 'takeoff':
                    phase = 'road_entry'
                    phase_position = values[:,:2]
                elif phase == 'landing_hold':
                    phase = 'descent'
                    descend_since = stamp
                else:
                    summary = dict(outcome='landed' if landing else 'road_entry_reached',
                                   start_sim_time_s=first_stamp,end_sim_time_s=stamp,
                                   max_xy_error_m=max_error,min_inter_uav_m=min_spacing,
                                   z_m=xyz[:,2].tolist(),peak_final_xy_speed_m_s=float(max(xy_speed)),
                                   peak_final_z_speed_m_s=float(max(z_speed)))
                    (output/f'{stage_name}.json').write_text(json.dumps(summary,indent=2))
                    print('MISSION',json.dumps(summary),flush=True)
                    return
                print(f'MISSION {phase}',flush=True)
                steady_since = None
                stream.flush()
    raise RuntimeError(f'Timed out during {phase}')
