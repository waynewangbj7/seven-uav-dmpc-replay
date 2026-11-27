"""Independent local OCP comparison; original paper uses PSO.

Architecture reference: https://github.com/derekc22/dmpc (MIT).
Written for this project's model/cost; no source copied from the reference.
Dependencies are isolated CPython 3.12 wheels in solver_dependencies/; see ../README.md.
"""
from pathlib import Path
import sys
import time

dependency_path = Path(__file__).resolve().parent / 'solver_dependencies'
if dependency_path.exists():
    if sys.version_info[:2] != (3,12):
        raise RuntimeError('Bundled CasADi requires Python 3.12. Use run_engineering_case1.ps1 or a Python 3.12 virtual environment.')
    sys.path.insert(0, str(dependency_path))
import casadi as ca
import numpy as np

from dmpc_core import A_MIN, A_MAX, W_MIN, W_MAX, V_MIN, V_MAX, DT, N, rollout, slot_offset


class CasadiBackend:
    name = 'CasADi-IPOPT single shooting; 16 controls; Euclidean norm smoothing 0.01m'
    distance_floor = 0.01  # max per-error perturbation; compare with exact norm

    def __init__(self, allow_feasible_fallback=False):
        self.models = {}
        self.logs = []
        self.allow_feasible_fallback = allow_feasible_fallback

    def model(self, count):
        if count in self.models:
            return self.models[count]
        u = ca.SX.sym('u', 2 * N)
        p = ca.SX.sym('p', 4 + 2 * N * count)
        x = p[:4]
        cost = 0
        optimizer_cost = 0
        for k in range(N):
            yaw = x[3] + DT * u[2*k+1]
            x = ca.vertcat(x[0] + DT*x[2]*ca.cos(x[3]),
                           x[1] + DT*x[2]*ca.sin(x[3]),
                           ca.fmin(V_MAX, ca.fmax(V_MIN, x[2] + DT*u[2*k])),
                           ca.atan2(ca.sin(yaw), ca.cos(yaw)))
            for j in range(count):
                index = 4 + 2 * (j*N+k)
                error = x[:2] - p[index:index+2]
                cost += ca.norm_2(error)  # evaluation only, not differentiated
                optimizer_cost += ca.sqrt(ca.dot(error,error) + self.distance_floor**2)
        solver = ca.nlpsol(f'ocp_{count}', 'ipopt',
                          {'x': u, 'p': p, 'f': optimizer_cost},
                          {'print_time': False, 'ipopt.print_level': 0,
                           'ipopt.sb': 'yes', 'ipopt.max_iter': 120,
                           'ipopt.max_wall_time': 0.09,
                           'ipopt.bound_relax_factor': 0,
                           'ipopt.honor_original_bounds': 'yes',
                           'ipopt.tol': 1e-6, 'ipopt.acceptable_tol': 1e-5})
        evaluate = ca.Function(f'cost_{count}', [u, p], [cost])
        self.models[count] = solver, evaluate
        return solver, evaluate

    def solve(self, state, targets, warm, role):
        targets = np.asarray(targets, dtype=float)
        assert targets.ndim == 3 and targets.shape[1:] == (N, 2)
        solver, _ = self.model(len(targets))
        parameters = np.r_[state, targets.reshape(-1)]
        started = time.perf_counter()
        lower,upper = np.tile([A_MIN,W_MIN],N),np.tile([A_MAX,W_MAX],N)
        attempts = []
        candidates = [np.clip(warm.reshape(-1),lower,upper).reshape(N,2)]
        for initial in ([warm,np.zeros_like(warm)] if self.allow_feasible_fallback else [warm]):
            result = solver(x0=initial.reshape(-1), p=parameters,lbx=lower,ubx=upper)
            stats = solver.stats()
            attempts.append({'status':stats['return_status'],'success':bool(stats['success'])})
            candidate = np.asarray(result['x']).reshape(N,2)
            feasible = np.all(np.isfinite(candidate)) and np.all(candidate.reshape(-1)>=lower-1e-9) and np.all(candidate.reshape(-1)<=upper+1e-9)
            if feasible:
                candidates.append(candidate)
            if stats['success']:
                if not feasible:
                    raise RuntimeError('solver reported success with invalid input bounds')
                break
        self.logs.append({'role': role, 'status': stats['return_status'],
                          'success': bool(stats['success']),
                          'iterations': int(stats['iter_count']),
                          'attempts':attempts,
                          'seconds_excluding_model_build': time.perf_counter()-started})
        if not stats['success'] and not self.allow_feasible_fallback:
            raise RuntimeError(f"IPOPT failed for {role}: {stats['return_status']}")
        if stats['success']:
            plan = candidates[-1]
        else:
            def exact_cost(plan):
                predicted = rollout(state,plan)
                return float(np.linalg.norm(predicted[None,1:,:2]-targets,axis=2).sum())
            plan = min(candidates,key=exact_cost)
            self.logs[-1]['accepted_feasible_best_effort'] = True
        states = rollout(state, plan)
        exact = float(np.linalg.norm(states[None, 1:, :2] - targets, axis=2).sum())
        self.logs[-1]['unmodified_position_cost'] = exact
        return plan

    def solve_leader(self, state, t0, warm, seed):
        del seed  # deterministic local solver; event RNG is unchanged
        targets = np.array([[[90.0*(t0+k*DT), 0.0] for k in range(1, N+1)]])
        return self.solve(state, targets, warm, 'leader')

    def solve_role(self, state, role, warm, seed, q=None, neighbor_estimates=None,
                   coordinator_estimate=None, coordinator_slots=None):
        del seed
        if role == 'follower':
            targets = np.array([coordinator_estimate[1:, :2] + slot_offset(q)])
        elif role == 'coordinator':
            # w=1 for all included neighbors, exactly as current baseline.
            targets = np.array([estimate[1:, :2] - slot_offset(slot)
                                for estimate, slot in zip(neighbor_estimates, coordinator_slots)])
        else:
            raise ValueError(role)
        return self.solve(state, targets, warm, role)
