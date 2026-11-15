"""Dump the reference kind grid + walkable grid from check_map.py's own logic,
so the MATLAB build can be diffed cell by cell.

    python tools/dump_reference_grid.py <kind_out.txt> <walk_out.txt>
"""
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_map as cm

LETTERS = 'RrB SWP'   # ROAD RING BLOCKED START WORK PAD -> placeholder index 3


def main():
    kind_out = sys.argv[1] if len(sys.argv) > 1 else 'tmp_kind_ref.txt'
    walk_out = sys.argv[2] if len(sys.argv) > 2 else 'tmp_walk_ref.txt'

    kind = cm.build_kind()
    walk = cm.build_walkable(kind)
    names = ['ROAD', 'RING', 'BLOCKED', 'START', 'WORK', 'PAD']

    with open(kind_out, 'w', encoding='utf-8') as fh:
        for cy in range(cm.SIZE):
            fh.write(''.join(str(kind[cy][cx]) for cx in range(cm.SIZE)) + '\n')

    with open(walk_out, 'w', encoding='utf-8') as fh:
        for cy in range(cm.SIZE):
            fh.write(''.join('1' if walk[cy][cx] else '0' for cx in range(cm.SIZE)) + '\n')

    counts = {}
    for cy in range(cm.SIZE):
        for cx in range(cm.SIZE):
            counts[kind[cy][cx]] = counts.get(kind[cy][cx], 0) + 1
    print('reference kind histogram:')
    for i, nm in enumerate(names):
        print('   %-8s %5d' % (nm, counts.get(i, 0)))
    print('   walkable %5d' % sum(sum(1 for v in row if v) for row in walk))


if __name__ == '__main__':
    main()
