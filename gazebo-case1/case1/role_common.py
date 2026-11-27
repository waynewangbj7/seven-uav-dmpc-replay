"""Shared role data and local event predicates used by the engineering controller."""
from dataclasses import dataclass
from itertools import combinations
from math import cos,hypot,pi,sin
import random
SINGLE,COORDINATOR,FOLLOWER='single','coordinator','follower'

@dataclass
class Agent:
    id: str
    x: float
    y: float
    role: str
    coordinator: str | None = None
    slot: int | None = None
    relay_to: str | None = None
    relay_slot: int | None = None
    relay_offset: tuple[float, float] | None = None
    relay_updates: int = 0

    @property
    def position(self):
        return self.x, self.y

class RoleState:

    def __init__(self, agents, d_cm=250.0, d_s=200.0, seed=7):
        self.agents = {a.id: a for a in agents}
        self.d_cm, self.d_s = (d_cm, d_s)
        self.seed = seed
        self.rng = random.Random(seed)
        self.events = []
        self.assignment_history = []
        self.status = 'running'

    def adjacency(self):
        graph = {i: set() for i in self.agents}
        for a, b in combinations(self.agents.values(), 2):
            if hypot(a.x - b.x, a.y - b.y) <= self.d_cm:
                graph[a.id].add(b.id)
                graph[b.id].add(a.id)
        return graph

    def _free_slots(self, coordinator_id, exclude_follower=None):
        used = {a.slot for a in self.agents.values() if a.id != exclude_follower and a.coordinator == coordinator_id and (a.slot is not None)}
        used.update((a.relay_slot for a in self.agents.values() if a.id != exclude_follower and a.relay_to == coordinator_id and (a.relay_slot is not None)))
        return [n for n in range(1, 7) if n not in used]

    def _slot_position(self, coordinator_id, n):
        c = self.agents[coordinator_id]
        theta = (n - 1) * pi / 3
        return (c.x + self.d_s * cos(theta), c.y + self.d_s * sin(theta))

    def _nearest_free_slot(self, coordinator_id, position, exclude_follower=None):
        free = self._free_slots(coordinator_id, exclude_follower)
        if not free:
            return None
        return min(free, key=lambda n: hypot(position[0] - self._slot_position(coordinator_id, n)[0], position[1] - self._slot_position(coordinator_id, n)[1]))

    def _assign(self, follower_id, coordinator_id, slot=None, reason='assignment'):
        follower = self.agents[follower_id]
        old_coordinator = follower.coordinator
        slot = slot if slot is not None else self._nearest_free_slot(coordinator_id, follower.position, exclude_follower=follower_id)
        if slot is None:
            return False
        follower.role, follower.coordinator, follower.slot = (FOLLOWER, coordinator_id, slot)
        follower.relay_to = follower.relay_slot = follower.relay_offset = None
        follower.relay_updates = 0
        self.assignment_history.append({'follower_id': follower_id, 'old_coordinator': old_coordinator, 'new_coordinator': coordinator_id, 'slot': slot, 'reason': reason})
        return True

    def _event_kind(self, a, b, graph):
        if a.role == COORDINATOR and b.role == COORDINATOR:
            return 'S6'
        if a.role == SINGLE and b.role == SINGLE:
            return 'S2'
        if {a.role, b.role} == {SINGLE, COORDINATOR}:
            single = a if a.role == SINGLE else b
            coordinator = b if a.role == SINGLE else a
            if single.relay_to == coordinator.id:
                return None
            return 'S1'
        if {a.role, b.role} == {SINGLE, FOLLOWER}:
            follower = a if a.role == FOLLOWER else b
            home = self.agents.get(follower.coordinator)
            if home and home.role == COORDINATOR and (home.id in graph[follower.id]):
                return 'S3'
            return None
        if {a.role, b.role} == {COORDINATOR, FOLLOWER}:
            follower = a if a.role == FOLLOWER else b
            coordinator = b if a.role == FOLLOWER else a
            home = self.agents.get(follower.coordinator)
            if home and home.role == COORDINATOR and (home.id != coordinator.id) and (home.id in graph[follower.id]) and (coordinator.id not in graph[home.id]):
                return 'S4'
        if a.role == FOLLOWER and b.role == FOLLOWER:
            ca, cb = (self.agents.get(a.coordinator), self.agents.get(b.coordinator))
            if ca and cb and (ca.id != cb.id) and (ca.id in graph[a.id]) and (cb.id in graph[b.id]) and (cb.id not in graph[ca.id]):
                return 'S5'
        return None

    def snapshot(self):
        graph = self.adjacency()
        return {'status': self.status, 'adjacency': {i: sorted(js) for i, js in graph.items()}, 'roles': {i: {'role': a.role, 'coordinator': a.coordinator, 'slot': a.slot, 'relay_to': a.relay_to, 'relay_slot': a.relay_slot, 'relay_offset': a.relay_offset, 'relay_updates': a.relay_updates} for i, a in self.agents.items()}, 'events': list(self.events), 'assignment_history': list(self.assignment_history), 'rng_seed': self.seed, 'formation_result': None if self.status != 'running' else 'not evaluated'}
