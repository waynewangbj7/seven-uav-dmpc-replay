"""Planar model, fixed Case 1 data, slot geometry and prediction helpers.

SI units: x,y in m; v in m/s; yaw in rad; a in m/s^2; omega in rad/s.
Engineering changes live in engineering_coordination.py and the run script.
"""
import numpy as np
from role_common import Agent,RoleState,COORDINATOR,FOLLOWER,SINGLE
DT=0.3
N=8
DS=200.0
A_MIN,A_MAX=-10.0,10.0
W_MIN,W_MAX=-0.2,0.2
V_MIN,V_MAX=10.0,150.0

D_CM = 250.0
CASE1_EVENT_SEED = 1
IDS = ("UAV-1", "UAV-2", "UAV-3", "UAV-4", "UAV-5", "UAV-6", "UAV-7")
INITIAL = np.array([
    [0.0,    0.0,    75.0,  0.0],
    [-105.0, -185.0, 89.0, -0.15],
    [-223.0, 15.0,   86.0,  0.04],
    [-495.0, 240.0,  86.0, -0.2],
    [-750.0, 150.0,  95.0, -0.01],
    [-370.0, -305.0, 100.0, -0.15],
    [-530.0, -450.0, 101.0,  0.2],
])
INITIAL_ROLE = {
    "UAV-1": "leader", "UAV-2": FOLLOWER, "UAV-3": COORDINATOR,
    "UAV-4": SINGLE, "UAV-5": SINGLE, "UAV-6": FOLLOWER,
    "UAV-7": COORDINATOR,
}

def wrap_angle(x):
    return (x + np.pi) % (2.0 * np.pi) - np.pi

def step(state, control):
    """Forward Euler integration of xdot=v cos(psi), ydot=v sin(psi), vdot=a, psidot=omega."""
    x, y, v, psi = state
    a, omega = control
    return np.array([x + DT*v*np.cos(psi),
                     y + DT*v*np.sin(psi),
                     np.clip(v + DT*a, V_MIN, V_MAX),
                     wrap_angle(psi + DT*omega)])

def rollout(state0, controls):
    xs = [state0.copy()]
    for u in controls:
        xs.append(step(xs[-1], u))
    return np.asarray(xs)

def slot_offset(q):
    theta = (q - 1) * np.deg2rad(60.0)  # Fig. 8: q=n=1 points along +x
    return DS * np.array([np.cos(theta), np.sin(theta)])

def constant_velocity_prediction(state):
    """Bootstrap the N+1 position estimate samples before any plan exists."""
    pred = np.tile(state, (N + 1, 1))
    for k in range(1, N + 1):
        pred[k, 0] = pred[k - 1, 0] + DT * pred[k - 1, 2] * np.cos(pred[k - 1, 3])
        pred[k, 1] = pred[k - 1, 1] + DT * pred[k - 1, 2] * np.sin(pred[k - 1, 3])
    return pred

def initialize_events(states, event_seed=CASE1_EVENT_SEED):
    agents = [Agent(uid, *states[i, :2], INITIAL_ROLE[uid])
              for i, uid in enumerate(IDS)]
    events = RoleState(agents, d_cm=D_CM, seed=event_seed)
    graph = events.adjacency()
    initial_links = []

    # Table 1 gives roles, not parent links or initial slot IDs. Infer both
    # from the t=0 neighbor graph and the paper's nearest-free-slot rule.
    for uid in IDS:
        agent = events.agents[uid]
        if agent.role not in (FOLLOWER, "leader"):
            continue
        coordinators = [j for j in graph[uid]
                        if events.agents[j].role == COORDINATOR]
        if len(coordinators) != 1:
            raise RuntimeError(f"expected one initial coordinator neighbor for {uid}; got {coordinators}")
        coordinator_id = coordinators[0]
        slot = events._nearest_free_slot(coordinator_id, agent.position)
        if slot is None:
            raise RuntimeError(f"no initial slot available for {uid}")
        agent.coordinator, agent.slot = coordinator_id, slot
        initial_links.append({"agent": uid, "role": agent.role,
                              "coordinator": coordinator_id, "paper_slot_n": slot})
    return events, initial_links

def pairwise_minimum(states_k):
    best = (float("inf"), None)
    for i in range(len(IDS)):
        for j in range(i + 1, len(IDS)):
            d = float(np.linalg.norm(states_k[i, :2] - states_k[j, :2]))
            if d < best[0]:
                best = d, (IDS[i], IDS[j])
    return best

def estimate_from_previous_control(previous_plan, current_state):
    """Paper Eq. (8): shift controls, repeat final input, integrate from now."""
    controls = np.vstack((previous_plan[1:], previous_plan[-1]))
    return rollout(current_state, controls)
