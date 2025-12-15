import sys
from pathlib import Path


def parse_instance(path):
    lines = [l.split('%')[0].strip() for l in Path(path).read_text().splitlines()]
    toks = [l for l in lines if l]
    it = iter(toks)
    # width height
    width, height = map(int, next(it).split())

    # crane section
    while True:
        line = next(it)
        if line.lower().startswith('crane'):
            break
    n_cranes = int(next(it).split()[0])
    cranes = []
    dispatches = []
    for _ in range(n_cranes):
        parts = list(map(int, next(it).split()))
        cid = parts[0]
        rect = (parts[1], parts[2], parts[3], parts[4])
        nd = parts[5]
        dispatch_ids = []
        for k in range(nd):
            did = parts[6 + 3*k]
            dx = parts[6 + 3*k + 1]
            dy = parts[6 + 3*k + 2]
            dispatches.append({'id': did, 'rect': (dx, dy, dx+3, dy+1)})
            dispatch_ids.append(did)
        cranes.append({'id': cid, 'rect': rect, 'dispatch_ids': dispatch_ids})

    # storages
    while True:
        line = next(it)
        if line.lower().startswith('storage'):
            break
    n_stor = int(next(it).split()[0])
    storages = []
    for _ in range(n_stor):
        v = list(map(int, next(it).split()))
        sid = v[0]
        blx, bly = v[1], v[2]
        rect = (blx, bly, blx+1, bly+3)
        # staging bl for storage: x1-1, y1-2, dir Up
        staging_bl = (blx-1, bly-2)
        storages.append({'id': sid, 'rect': rect, 'staging_bl': staging_bl, 'staging_dir':'up'})

    # carriers
    while True:
        line = next(it)
        if line.lower().startswith('carrier section') or line.lower().startswith('carrier section'.split()[0]):
            break
    # The parser expects a number, but some files use 'carrier section' label; read next number
    # Find next numeric line
    # Backtrack: current line might be 'carrier section'
    # Seek next numeric
    # Using simple approach: iterate until a numeric-only line
    # fetch next token that starts with digit
    # (we already consumed 'carrier section')
    # So find next numeric line
    # Seek in remaining tokens
    rest = list(it)
    idx = 0
    while idx < len(rest) and not rest[idx].split()[0].lstrip('-').isdigit():
        idx += 1
    if idx >= len(rest):
        raise SystemExit('Bad format: no carrier count')
    n_car = int(rest[idx].split()[0]); idx += 1
    carriers = []
    for _ in range(n_car):
        parts = list(map(int, rest[idx].split())); idx += 1
        cid = parts[0]; assigned = parts[1]; blx = parts[2]; bly = parts[3]
        carriers.append({'id': cid, 'assigned': assigned, 'bl': (blx, bly), 'dir': 'down', 'carrying': None, 'time':0})

    # Continue from idx in rest iterator
    rest2 = rest[idx:]
    # containers count
    # find next numeric line
    j = 0
    while j < len(rest2) and not rest2[j].split()[0].lstrip('-').isdigit():
        j += 1
    if j >= len(rest2):
        raise SystemExit('Bad format: no containers count')
    n_cont = int(rest2[j].split()[0]); j += 1
    storage_index_by_id = {s['id']:i for i,s in enumerate(storages)}
    storage_stacks = [[] for _ in storages]
    for _ in range(n_cont):
        parts = list(map(int, rest2[j].split())); j += 1
        cid, sid = parts[0], parts[1]
        idx_s = storage_index_by_id[sid]
        storage_stacks[idx_s].append(cid)

    inst = {
        'storages': storages,
        'dispatches': dispatches,
        'carriers': carriers,
        'storage_stacks': storage_stacks,
    }
    return inst


def parse_solution(path):
    lines = [l.strip() for l in Path(path).read_text().splitlines() if l.strip()]
    cmds = []
    carrier = None
    for l in lines:
        if l.startswith('carrier'):
            carrier = int(l.split()[1]); continue
        parts = l.split()
        t = int(parts[0]); typ = parts[1]
        if typ == 'move':
            k = int(parts[2]); cmds.append(('move', t, k))
        elif typ == 'face':
            d = parts[2]; cmds.append(('face', t, d))
        elif typ == 'load':
            cmds.append(('load', t))
        elif typ == 'unload':
            cmds.append(('unload', t))
    return cmds


def run_validation(inst_path, sol_path):
    inst = parse_instance(inst_path)
    cmds = parse_solution(sol_path)

    c = inst['carriers'][0]
    storages = inst['storages']
    storage_stacks = inst['storage_stacks']
    dispatches = inst['dispatches']

    cur_time = 0
    carrying = None

    def dims(dir):
        if dir in ('up','down'):
            return (4,8)
        return (8,4)

    def center_from_bl(bl, dir):
        w,h = dims(dir)
        return (bl[0] + w//2, bl[1] + h//2)

    def bl_from_center(center, dir):
        w,h = dims(dir)
        return (center[0] - w//2, center[1] - h//2)

    def move_forward(dir, bl, steps):
        dx,dy = {'up':(0,1),'down':(0,-1),'left':(-1,0),'right':(1,0)}[dir]
        return (bl[0] + dx*steps, bl[1] + dy*steps)

    for cmd in cmds:
        if cmd[0] == 'move':
            _, t, k = cmd
            if t != cur_time:
                print(f'Time mismatch for move: expected {cur_time}, got {t}'); return
            # duration = abs(k)
            # update bl
            bl_before = c['bl']
            new_bl = move_forward(c['dir'], bl_before, k)
            dur = abs(k)
            cur_time += dur
            c['bl'] = new_bl
        elif cmd[0] == 'face':
            _, t, d = cmd
            if t != cur_time:
                print(f'Time mismatch for face: expected {cur_time}, got {t}'); return
            # change dir and recompute bl
            center = center_from_bl(c['bl'], c['dir'])
            c['dir'] = d
            c['bl'] = bl_from_center(center, c['dir'])
            cur_time += 1
        elif cmd[0] == 'load':
            _, t = cmd
            if t != cur_time:
                print(f'Time mismatch for load: expected {cur_time}, got {t}'); return
            # check if at any storage staging
            found = False
            for si, s in enumerate(storages):
                if tuple(s['staging_bl']) == tuple(c['bl']) and s['staging_dir'] == c['dir']:
                    # must have a container on top
                    if not storage_stacks[si]:
                        print(f'Error: Load at storage {s["id"]} but storage empty at t={t}'); return
                    cid = storage_stacks[si].pop()
                    carrying = cid
                    found = True
                    break
            if not found:
                print(f'Error: Load at non-storage location bl={c["bl"]} dir={c["dir"]} t={t}'); return
            cur_time += 1
        elif cmd[0] == 'unload':
            _, t = cmd
            if t != cur_time:
                print(f'Time mismatch for unload: expected {cur_time}, got {t}'); return
            # must be at a dispatch staging
            found = False
            for d in dispatches:
                # compute staging_bl for dispatch: x1-2, y1-1
                x1,y1,_,_ = d['rect']
                st_bl = (x1-2, y1-1)
                if st_bl == tuple(c['bl']):
                    found = True
                    if carrying is None:
                        print(f'Error: Unload at dispatch but carrier empty at t={t}'); return
                    # deliver
                    carrying = None
                    break
            if not found:
                print(f'Error: Unload at non-dispatch bl={c["bl"]} t={t}'); return
            cur_time += 1

    print('Validation finished: no time/position/load-unload mismatches detected')


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print('Usage: validate_solution.py <instance> <solution>')
        sys.exit(1)
    run_validation(sys.argv[1], sys.argv[2])
