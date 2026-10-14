#!/usr/bin/env python3
"""edits.py -- the editing steps of tools/apply_layout.sh (one-off).

Usage (repo root; every step is idempotent and prints what it does):
  edits.py move MOVES.tsv [--dry-run] [--protect FILE...]
      git mv each OLD -> NEW row whose OLD still exists, then rewrite, in
      every tracked text file: path tokens OLD -> NEW, C #include "..."
      spellings of moved files (relative includes recomputed from each
      file's new place), and relative Markdown links.
  edits.py delete-func NAME FILE [--dry-run]
      delete NAME's definition (with its prolog box) from FILE and its
      prototype (with a comment that documents it alone) from every header.
  edits.py make-static NAME FILE HEADER [--dry-run]
      drop NAME's prototype from HEADER and define it `static` in FILE (a
      static forward declaration is added when FILE uses it before the
      definition).
  edits.py delete-proto NAME HEADER [--dry-run]
  edits.py doc-edits [--dry-run]
      the hand-written layout lines of docs/ARCHITECTURE.md.
  edits.py drop-include FILE HEADER [--dry-run]
      delete FILE's #include line that resolves to HEADER.
No step changes what any function does: moves, visibility, deletion of
functions with no caller, and includes whose removal leaves the object code
identical (verified when the lists were made, and again by the gates).
"""
import os
import re
import subprocess
import sys

DRY = '--dry-run' in sys.argv
ARGS = [a for a in sys.argv[1:] if a != '--dry-run']
COMMENT = re.compile(r'/\*.*?\*/|//[^\n]*', re.S)
TEXT = ('.c', '.h', '.py', '.md', '.txt', '.tsv', '.sh', '.lua', '.cocci', '.tcl', '.json',
        '.cfg', '.spec', 'Makefile', '.patch', '.dot', 'CMakeLists.txt')


def read(p):
    with open(p, encoding='utf-8') as fh:
        return fh.read()


def write(p, t):
    if DRY:
        print('DRY edit', p)
        return
    with open(p, 'w', encoding='utf-8') as fh:
        fh.write(t)


def tracked():
    out = subprocess.run(['git', 'ls-files'], capture_output=True, text=True, check=True).stdout
    return [f for f in out.split('\n') if f]


# ---------------------------------------------------------------- move
def cmd_move(mapfile, protect):
    moves = []
    for line in open(mapfile, encoding='utf-8'):
        line = line.rstrip('\n')
        if line and not line.startswith('#'):
            old, new = line.split('\t')[:2]
            moves.append((old, new))
    m = dict(moves)
    inv = {}
    for old, new in moves:
        if os.path.exists(old) and not os.path.exists(new):
            inv[new] = old
            if DRY:
                print('DRY git mv', old, new)
            else:
                os.makedirs(os.path.dirname(new) or '.', exist_ok=True)
                subprocess.run(['git', 'mv', old, new], check=True)
                print('moved', old, '->', new)
    pat = re.compile(r'(?<![\w.-])(?<![\w.-]/)(' + '|'.join(re.escape(o) for o, _ in sorted(moves, key=lambda x: -len(x[0]))) + r')(?![\w/-])')
    inc = re.compile(r'^(\s*#\s*include\s*")([^"]+)(")', re.M)
    link = re.compile(r'\]\(([^)#\s]+)(#[^)\s]*)?\)')
    edited = 0
    for f in tracked():
        if not f.endswith(TEXT) or not os.path.isfile(f):
            continue
        try:
            t = read(f)
        except UnicodeDecodeError:
            continue
        n = pat.sub(lambda x: m[x.group(1)], t)
        oldself = inv.get(f, f)
        if f.endswith(('.c', '.h')) and f in inv and os.path.basename(f) != os.path.basename(inv[f]):
            n = n.replace('FILE NAME: ' + os.path.basename(inv[f]), 'FILE NAME: ' + os.path.basename(f))
        if f.endswith(('.c', '.h')):
            def fix_inc(x):
                spelled = x.group(2)
                cand = os.path.normpath(os.path.join(os.path.dirname(oldself), spelled))
                if (cand in m and not os.path.exists(cand)) or (os.path.exists(cand) and oldself != f):
                    dest = m.get(cand, cand)
                    if dest.startswith('include/'):
                        return x.group(1) + dest[len('include/'):] + x.group(3)
                    return x.group(1) + os.path.relpath(dest, os.path.dirname(f)) + x.group(3)
                inc_path = os.path.join('include', spelled)
                if inc_path in m:
                    dest = m[inc_path]
                    return x.group(1) + dest[len('include/'):] + x.group(3)
                return x.group(0)
            n = inc.sub(fix_inc, n)
        if f.endswith('.md'):
            def fix_link(x):
                tgt, frag = x.group(1), x.group(2) or ''
                if '://' in tgt or tgt.startswith('/'):
                    return x.group(0)
                resolved = os.path.normpath(os.path.join(os.path.dirname(oldself), tgt))
                if not ((resolved in m and not os.path.exists(resolved)) or (oldself != f and os.path.exists(resolved))):
                    return x.group(0)
                return '](' + os.path.relpath(m.get(resolved, resolved), os.path.dirname(f) or '.') + frag + ')'
            n = link.sub(fix_link, n)
        if n != t:
            if f in protect:
                print('STALE (protected, not edited):', f)
                continue
            edited += 1
            write(f, n)
    print('move: %d rows, %d files edited' % (len(moves), edited))


# ---------------------------------------------------------------- C edits
def find_def(lines, name):
    """Index of the line that starts NAME's definition (not a prototype)."""
    rx = re.compile(r'^[A-Za-z_][\w\s\*]*?\b' + re.escape(name) + r'\s*\(')
    for i, line in enumerate(lines):
        if rx.match(line):
            j = i
            seg = ''
            while j < len(lines):
                seg += lines[j]
                if ';' in COMMENT.sub('', seg) and '{' not in seg:
                    break
                if '{' in seg or (j + 1 < len(lines) and lines[j + 1].startswith('{')):
                    return i
                j += 1
    return -1


def body_end(lines, i):
    depth = 0
    started = False
    for j in range(i, len(lines)):
        for ch in COMMENT.sub('', lines[j]):
            if ch == '{':
                depth += 1
                started = True
            elif ch == '}':
                depth -= 1
        if started and depth == 0:
            return j
    raise SystemExit('no body end')


def comment_above(lines, i):
    """First line of the block comment that ends right above line i, else i."""
    j = i - 1
    if j < 0 or not lines[j].rstrip().endswith('*/'):
        return i
    while j >= 0 and '/*' not in lines[j]:
        j -= 1
    return j if j >= 0 else i


def tidy(lines):
    out = []
    for line in lines:
        if line.strip() == '' and out and out[-1].strip() == '':
            continue
        out.append(line)
    return out


def proto_range(lines, name):
    rx = re.compile(r'\b' + re.escape(name) + r'\s*\(')
    for i, line in enumerate(lines):
        code = COMMENT.sub('', line)
        if rx.search(code) and not code.lstrip().startswith(('#', 'return', '(')):
            j = i
            seg = code
            while ';' not in seg and j + 1 < len(lines):
                j += 1
                seg += COMMENT.sub('', lines[j])
            if '{' in seg:
                continue
            start = i
            while start > 0 and COMMENT.sub('', lines[start - 1]).strip() and not COMMENT.sub('', lines[start - 1]).rstrip().endswith((';', '*/', '}', '{')) and not lines[start - 1].lstrip().startswith('#'):
                start -= 1
            c = comment_above(lines, start)
            after = lines[j + 1] if j + 1 < len(lines) else ''
            before = lines[c - 1] if c > 0 else ''
            if c < start and after.strip() == '' and before.strip() == '':
                start = c
            return start, j
    return None


def remove_proto(header, name):
    lines = read(header).split('\n')
    r = proto_range(lines, name)
    if r is None:
        print('no prototype of', name, 'in', header)
        return False
    del lines[r[0]:r[1] + 1]
    write(header, '\n'.join(tidy(lines)))
    print('prototype removed:', name, header)
    return True


def cmd_delete_func(name, path):
    lines = read(path).split('\n')
    i = find_def(lines, name)
    if i < 0:
        print('already gone:', name, path)
    else:
        e = body_end(lines, i)
        s = comment_above(lines, i)
        del lines[s:e + 1]
        write(path, '\n'.join(tidy(lines)))
        print('deleted', name, 'from', path)
    for h in tracked():
        if h.endswith('.h') and re.search(r'\b' + re.escape(name) + r'\s*\(', read(h)):
            remove_proto(h, name)


def cmd_make_static(name, path, header):
    lines = read(path).split('\n')
    i = find_def(lines, name)
    if i < 0:
        print('SKIP make-static: no definition of %s in %s (the tree moved on; drop the row)' % (name, path))
        return
    if lines[i].startswith('static '):
        print('already static:', name)
    else:
        code_before = COMMENT.sub('', '\n'.join(lines[:comment_above(lines, i)]))
        lines[i] = 'static ' + lines[i]
        if re.search(r'\b' + re.escape(name) + r'\b', code_before):
            j = i
            sig = lines[i]
            while ')' not in COMMENT.sub('', sig) or sig.count('(') > sig.count(')'):
                j += 1
                sig += '\n' + lines[j]
            sig = sig[:sig.rindex(')') + 1].replace('\n', ' ')
            last_inc = max(k for k, line in enumerate(lines[:i]) if line.startswith('#include'))
            decl = re.sub(r'\s+', ' ', sig) + ';'
            k = last_inc + 2
            if k < len(lines) and lines[k - 1] == '' and lines[k].startswith('static ') and lines[k].endswith(');'):
                while k < len(lines) and lines[k].startswith('static ') and lines[k].endswith(');'):
                    k += 1
                lines.insert(k, decl)
            else:
                lines.insert(last_inc + 1, '')
                lines.insert(last_inc + 2, decl)
        write(path, '\n'.join(tidy(lines)))
        print('static:', name, path)
    if re.search(r'\b' + re.escape(name) + r'\s*\(', read(header)):
        remove_proto(header, name)


def cmd_drop_include(path, header):
    lines = read(path).split('\n')
    for k, line in enumerate(lines):
        x = re.match(r'\s*#\s*include\s*"([^"]+)"', line)
        if not x:
            continue
        sp = x.group(1)
        if os.path.normpath(os.path.join(os.path.dirname(path), sp)) == header or os.path.join('include', sp) == header:
            del lines[k]
            write(path, '\n'.join(lines))
            print('include dropped:', path, header)
            return
    print('no include of', header, 'in', path)


DOC_EDITS = [
    ('docs/ARCHITECTURE.md', 'internal.h), display_if.c, noun_*.c', 'internal.h), display_interface.c, noun_*.c'),
    ('docs/ARCHITECTURE.md', 'shared: dapif.c, state.c,', 'shared: interface.c (DAP_INTERFACE_SUBROUTINES), state.c,'),
    ('docs/ARCHITECTURE.md', '  prelaunch_align.c (< 3 files in prelaunch/)', '  prelaunch/        align, gyrocompass, verify, internal.h (P01-P03, IMUTEST)'),
    ('docs/ARCHITECTURE.md', 'NASSP engine shim (nassp_engine.c)', 'NASSP engine shim (engine.c)'),
    ('docs/ARCHITECTURE.md', '`make nassp-mission`, host scripts, Lua, scenario images)', '`make nassp-mission`, NASSP patches, host scripts, Lua, scenario images)'),
]


def cmd_doc_edits():
    for path, old, new in DOC_EDITS:
        t = read(path)
        if old in t:
            write(path, t.replace(old, new))
            print('doc edit:', path, '|', new)


def main():
    if not ARGS:
        raise SystemExit(__doc__)
    cmd = ARGS[0]
    if cmd == 'move':
        protect = set(ARGS[ARGS.index('--protect') + 1:]) if '--protect' in ARGS else set()
        cmd_move(ARGS[1], protect)
    elif cmd == 'delete-func':
        cmd_delete_func(ARGS[1], ARGS[2])
    elif cmd == 'make-static':
        cmd_make_static(ARGS[1], ARGS[2], ARGS[3])
    elif cmd == 'delete-proto':
        for h in ARGS[2:]:
            remove_proto(h, ARGS[1])
    elif cmd == 'doc-edits':
        cmd_doc_edits()
    elif cmd == 'drop-include':
        cmd_drop_include(ARGS[1], ARGS[2])
    else:
        raise SystemExit(__doc__)


if __name__ == '__main__':
    main()
