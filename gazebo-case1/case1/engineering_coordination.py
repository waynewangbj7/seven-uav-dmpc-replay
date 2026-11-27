"""Engineering completion, NOT the unpublished S6 from Zhao et al.

Local physical encounter detection + serialized topology transactions.
Component lattice metadata is shared in this simulator for merge validation.
No physical collision-avoidance guarantee is claimed.
"""
from copy import deepcopy
from itertools import combinations
from math import hypot

from role_common import RoleState, COORDINATOR, FOLLOWER, SINGLE

# Axial coordinates: +x, +60, +120, +180, +240, +300 degrees.
AXIAL = ((1, 0), (0, 1), (-1, 1), (-1, 0), (0, -1), (1, -1))


def opposite(n):
    return (n + 2) % 6 + 1


class EngineeringEvents(RoleState):
    def __init__(self, agents, seed=1):
        super().__init__(agents, seed=seed)
        self.links = {(a.coordinator, a.id): a.slot for a in agents if a.coordinator}
        self.rejections = set()
        self.validate()

    def component(self, root):
        cells = {root: (0, 0)}
        queue = [root]
        while queue:
            a = queue.pop(0)
            for (c, j), n in self.links.items():
                if a not in (c, j):
                    continue
                b, sign = (j, 1) if a == c else (c, -1)
                step = AXIAL[n-1]
                cell = tuple(cells[a][k] + sign*step[k] for k in (0, 1))
                if b in cells:
                    assert cells[b] == cell, 'inconsistent lattice cycle'
                else:
                    cells[b] = cell
                    queue.append(b)
        return cells

    def same_group(self, a, b):
        return b in self.component(a)

    def free(self, c):
        occupied = {n for (i, _), n in self.links.items() if i == c}
        return [n for n in range(1, 7) if n not in occupied]

    def ordered_slots(self, c, j):
        a, b = self.agents[c], self.agents[j]
        return sorted(self.free(c), key=lambda n: (hypot(
            b.x-self._slot_position(c,n)[0], b.y-self._slot_position(c,n)[1]), n))

    def can_join(self, a, b, n):
        left, right = self.component(a), self.component(b)
        step = AXIAL[n-1]
        if b in left:
            return left[b] == step
        # Both component maps are rooted at the two endpoints. Only translation
        # is allowed: existing internal assignments cannot rotate or overlap.
        shifted = {(cell[0]+step[0], cell[1]+step[1]) for cell in right.values()}
        return not set(left.values()).intersection(shifted)

    def connect(self, a, b):
        if (a, b) in self.links:
            return True
        # Fewer free positions wins; IDs resolve ties deterministically.
        a, b = sorted((a, b), key=lambda uid: (len(self.free(uid)), uid))
        for n in self.ordered_slots(a, b):
            if opposite(n) in self.free(b) and self.can_join(a,b,n):
                self.links[a,b], self.links[b,a] = n, opposite(n)
                return True
        return False

    def promote(self, uid):
        a = self.agents[uid]
        parent, n = a.coordinator, a.slot
        assert a.role == FOLLOWER and parent
        a.role, a.coordinator, a.slot = COORDINATOR, None, None
        # Keep parent's slot and add the reciprocal slot: no missing J_C term.
        self.links[uid,parent] = opposite(n)

    def attach(self, uid, c, reason):
        for n in self.ordered_slots(c,uid):
            if self.can_join(c,uid,n):
                self._assign(uid,c,n,reason)
                self.links[c,uid] = n
                return True
        return False

    def emit(self, kind, pair, detail):
        self.events.append({'event': kind, 'agents': tuple(pair), 'status': 'handled',
                            'detail': detail})

    def update(self):
        graph = self.adjacency()
        # Phase 1: pending relays, then snapshot-detected role events. Slot
        # occupancy is rechecked at commit, avoiding duplicate S1 allocation.
        candidates = []
        for a in self.agents.values():
            if a.relay_to and a.relay_to in graph[a.id]:
                candidates.append((-1, 'S3_RELAY_COMPLETE', (a.id,a.relay_to)))
        order = {'S2': 0, 'S1': 1, 'S3': 2, 'S4': 3, 'S5': 4}
        for a,b in combinations(sorted(self.agents),2):
            if b not in graph[a]:
                continue
            kind = self._event_kind(self.agents[a],self.agents[b],graph)
            if kind in order:
                candidates.append((order[kind],kind,(a,b)))
        claimed = set()
        for _,kind,pair in sorted(candidates):
            if claimed.intersection(pair):
                continue  # deterministic serialization; reconsider next sample
            a,b = (self.agents[i] for i in pair)
            if kind != 'S3_RELAY_COMPLETE' and self._event_kind(a,b,graph) != kind:
                continue
            if kind in ('S4','S5') and self.same_group(*pair):
                continue
            old_agents,old_links = deepcopy(self.agents),dict(self.links)
            history_start = len(self.assignment_history)
            accepted = False
            if kind == 'S2':
                c,f = (a,b) if self.rng.randrange(2) == 0 else (b,a)
                c.role = COORDINATOR
                accepted = self.attach(f.id,c.id,'S2')
            elif kind in ('S1','S3_RELAY_COMPLETE'):
                single = a if a.role == SINGLE else b
                c = b if single is a else a
                accepted = self.attach(single.id,c.id,kind)
            elif kind == 'S3':
                single = a if a.role == SINGLE else b
                f = b if single is a else a
                if single.relay_to:
                    continue
                c = f.coordinator
                slots = [n for n in self.ordered_slots(c,single.id) if self.can_join(c,single.id,n)]
                if slots:
                    single.relay_to,single.relay_slot = c,slots[0]
                    single.relay_updates = 0
                    accepted = True
                else:
                    self.promote(f.id)
                    accepted = self.attach(single.id,f.id,'S3_FULL_PROMOTION')
            elif kind == 'S4':
                f = a if a.role == FOLLOWER else b
                c = b if f is a else a
                self.promote(f.id)
                accepted = self.connect(f.id,c.id)
            elif kind == 'S5':
                self.promote(a.id)
                self.promote(b.id)
                accepted = self.connect(a.id,b.id)
            if accepted:
                self.validate()
                claimed.update(pair)
                self.emit(kind,pair,'engineering atomic role/slot transaction')
            else:
                self.agents,self.links = old_agents,old_links
                del self.assignment_history[history_start:]
                self.reject(kind,pair)

        # Phase 2 uses committed roles but unchanged physical positions. It
        # connects new coordinators in this cycle, never one-cycle zero-input.
        for a,b in combinations(sorted(self.agents),2):
            if b not in graph[a] or any(self.agents[i].role != COORDINATOR for i in (a,b)):
                continue
            if (a,b) in self.links:
                continue
            if self.connect(a,b):
                self.emit('S6',(a,b),'reciprocal slots; component lattice non-overlap checked')
            else:
                self.reject('S6',(a,b))
        for a in self.agents.values():
            if a.relay_to:
                a.relay_updates += 1
        self.validate()
        return self.snapshot()

    def reject(self, kind, pair):
        key = (kind,*pair)
        if key not in self.rejections:
            self.rejections.add(key)
            self.events.append({'event': kind+'_DEFERRED','agents':pair,'status':'deferred',
                                'detail':'no cycle-consistent non-overlapping available slot; retry next sample'})

    def validate(self):
        for c in self.agents:
            slots = [n for (i,_),n in self.links.items() if i == c]
            assert len(slots) == len(set(slots)) and all(1 <= n <= 6 for n in slots)
            cells = self.component(c)
            assert len(cells) == len(set(cells.values())), 'overlapping formation cells'
        for (c,j),n in self.links.items():
            assert self.agents[c].role == COORDINATOR
            if self.agents[j].role == COORDINATOR:
                assert self.links[j,c] == opposite(n)
        for a in self.agents.values():
            if a.role == FOLLOWER:
                assert self.links[a.coordinator,a.id] == a.slot

    def snapshot(self):
        result = super().snapshot()
        result['valid_links'] = [{'coordinator':c,'neighbor':j,'slot':n}
                                 for (c,j),n in sorted(self.links.items())]
        return result
