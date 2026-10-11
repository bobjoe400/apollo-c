#!/usr/bin/env python3
"""fix_comments.py -- comment placement, paragraphs and continuation layout.

Usage (from the repo root; `make format` runs it around Uncrustify):
    python3 -I tools/fix_comments.py [--stats] [--long TSV] FILE...

Rules (docs/CODING_STANDARD.md "Comment placement and wrapping"; user
direction 2026-10-11, NASA SEL-94-003 2.2.1 and 2.3):

1. A comment never sits inside a statement.  A comment inside a statement
   of a function body (after `=`, inside a call's parentheses, between
   arguments or operands, on a continuation line, or trailing a statement
   that spans several lines) moves to its own line immediately above the
   statement, at the statement's indentation.  Several comments of one
   statement merge into one, in order, separated by "; ".  The words are
   kept verbatim (only the white space around them is normalised).
   Function prologs, `#define` lines, struct/union/enum bodies and `case`
   labels are not touched.  A trailing comment after the `;` of a one-line
   statement stays.
2. Continuation lines indent a fixed 8 columns from the statement's first
   line and never align to an open parenthesis.  A statement that spans
   lines (or passes column 80) is re-broken from scratch: the fewest lines
   that fit in 80 columns, breaking after `,`, a binary operator or `=`,
   preferring the outermost nesting level.  `for` headers, statements
   with a multi-line brace initialiser, statements that still contain a
   comment and preprocessor lines are left to Uncrustify.
3. A standalone comment that follows a statement or declaration (a line
   ending in `;` or `}`, or a preprocessor line) starts a paragraph: a
   blank line goes above it.  Consecutive comment lines stay together.
4. Within a paragraph (consecutive non-blank lines) the single-line
   trailing comments at one brace level start in one column, one space
   after the longest code among them; a comment after an open brace is not
   aligned.  Where that column would push a comment past column 80, the
   comment moves above its statement (rule 1 layout) and the paragraph is
   re-aligned.

Uses the libclang token stream (python3 clang.cindex), so string and
character literals, comments and preprocessor lines are recognised exactly.
Only white space and comment positions change: `gcc -E` output (comments
stripped) is token-identical before and after.  Idempotent.
"""
import argparse
import os
import re
import sys

import clang.cindex as ci

WIDTH = 80
CONT = 8
# Lexing only: headers are not needed (and not read).
CLANG_ARGS = ['-x', 'c', '-std=c11', '-nostdinc']

BREAK_OPS = {
    '&&': 0, '||': 0, '?': 4, ':': 4,
    '==': 12, '!=': 12, '<': 12, '>': 12, '<=': 12, '>=': 12,
    '+': 8, '-': 8, '/': 8, '%': 8, '|': 10, '^': 10, '<<': 10, '>>': 10,
}
ASSIGN_OPS = {'=', '+=', '-=', '*=', '/=', '%=', '<<=', '>>=', '&=', '|=',
              '^='}
KEYWORD_PAREN = {'if', 'while', 'for', 'switch', 'return', 'sizeof'}

_INDEX = None


class Tok:
    __slots__ = ('kind', 'text', 'start', 'end', 'line', 'pp')

    def __init__(self, kind, text, start, end, line):
        self.kind = kind
        self.text = text
        self.start = start
        self.end = end
        self.line = line
        self.pp = False


class Seg:
    """A statement-like run of tokens: ends at `;`, before a block `{`,
    at a block `}` or after a `case X:` label."""
    __slots__ = ('toks', 'kind', 'ctx', 'level')

    def __init__(self, toks, kind, ctx, level):
        self.toks = toks
        self.kind = kind
        self.ctx = ctx
        self.level = level


class Source:
    def __init__(self, path, text):
        self.path = path
        self.text = text
        self.line_starts = [0]
        for m in re.finditer('\n', text):
            self.line_starts.append(m.end())
        self.toks = lex(path, text, self)
        self.sig = [t for t in self.toks if t.kind != 'c' and not t.pp]
        self.comments = [t for t in self.toks if t.kind == 'c' and not t.pp]
        self.segs, self.brace_kind, self.level_at, self.inner_at = \
            segment(self)
        self.sig_index = {id(t): k for k, t in enumerate(self.sig)}

    def line_of(self, off):
        lo, hi = 0, len(self.line_starts) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self.line_starts[mid] <= off:
                lo = mid
            else:
                hi = mid - 1
        return lo

    def line_text(self, ln):
        s = self.line_starts[ln]
        e = self.text.find('\n', s)
        return self.text[s:] if e < 0 else self.text[s:e]

    def col(self, off):
        return off - self.line_starts[self.line_of(off)]


def lex(path, text, src):
    global _INDEX
    if _INDEX is None:
        _INDEX = ci.Index.create()
    tu = _INDEX.parse(path, args=CLANG_ARGS, unsaved_files=[(path, text)],
                      options=ci.TranslationUnit.PARSE_SKIP_FUNCTION_BODIES)
    kinds = {ci.TokenKind.COMMENT: 'c', ci.TokenKind.PUNCTUATION: 'p',
             ci.TokenKind.KEYWORD: 'k', ci.TokenKind.IDENTIFIER: 'i',
             ci.TokenKind.LITERAL: 'l'}
    out = []
    for t in tu.get_tokens(extent=tu.cursor.extent):
        ext = t.extent
        s = ext.start.offset
        e = ext.end.offset
        out.append(Tok(kinds[t.kind], text[s:e], s, e, 0))
    out.sort(key=lambda x: x.start)
    for t in out:
        t.line = src.line_of(t.start)
    # Preprocessor directives: `#` first on its line, through `\` lines.
    pp_lines = set()
    first_on_line = {}
    for t in out:
        if t.kind != 'c' and t.line not in first_on_line:
            first_on_line[t.line] = t
    for ln, t in first_on_line.items():
        if t.text == '#':
            k = ln
            pp_lines.add(k)
            while src.line_text(k).rstrip().endswith('\\'):
                k += 1
                pp_lines.add(k)
    for t in out:
        if t.line in pp_lines:
            t.pp = True
    src.pp_lines = pp_lines
    return out


def segment(src):
    """Split the significant tokens into segments; classify each `{`."""
    segs = []
    stack = []          # 'func' | 'block' | 'record' | 'enum' | 'init'
    brace_kind = {}     # index of `{` in src.sig -> kind
    level_at = []       # brace depth before each sig token
    inner_at = []       # (innermost brace kind, paren depth) at each token
    cur = []
    paren = 0
    sig = src.sig

    def close(kind):
        nonlocal cur
        if cur:
            segs.append(Seg(cur, kind, tuple(stack),
                            sum(1 for s in stack if s != 'init')))
        cur = []

    for i, t in enumerate(sig):
        level_at.append(len(stack))
        inner_at.append((stack[-1] if stack else None, paren))
        s = t.text
        in_init = bool(stack) and stack[-1] == 'init'
        if s in ('(', '['):
            paren += 1
            cur.append(i)
            continue
        if s in (')', ']'):
            paren -= 1
            cur.append(i)
            continue
        if s == '{':
            p = sig[i - 1] if i else None
            kind = None
            if p is not None:
                if p.text in ('=', ',', '(', 'return', '[') or \
                        (paren > 0 and p.text != ')'):
                    kind = 'init'
                elif p.text == '{':
                    kind = 'init' if brace_kind.get(i - 1) == 'init' \
                        else None
                elif p.text == ')' and p.line == t.line:
                    kind = 'init'    # compound literal (Allman tree)
            if kind == 'init':
                brace_kind[i] = 'init'
                stack.append('init')
                cur.append(i)
                continue
            head = [sig[k].text for k in cur]
            if 'func' not in stack and 'block' not in stack and \
                    '(' in head and not any(
                        h in ('struct', 'union', 'enum') for h in
                        head[:head.index('(')]):
                kind = 'func'
            elif any(h in ('struct', 'union') for h in head):
                kind = 'record'
            elif 'enum' in head:
                kind = 'enum'
            else:
                kind = 'block'
            close('header')
            brace_kind[i] = kind
            stack.append(kind)
            continue
        if s == '}':
            if in_init:
                stack.pop()
                cur.append(i)
                continue
            close('tail')
            if stack:
                stack.pop()
            continue
        cur.append(i)
        if paren == 0 and not in_init:
            if s == ';':
                close('stmt')
            elif s == ':' and sig[cur[0]].text in ('case', 'default'):
                close('label')
            elif s == ',' and stack and stack[-1] == 'enum':
                close('item')       # one enumerator
    close('tail')
    return segs, brace_kind, level_at, inner_at


# ---------------------------------------------------------------------------
# Text helpers
# ---------------------------------------------------------------------------

def comment_words(text):
    """Inner text of a comment, white space normalised per line."""
    if text.startswith('//'):
        return text[2:].strip()
    body = text[2:-2]
    lines = body.split('\n')
    parts = []
    for k, ln in enumerate(lines):
        ln = ln.strip()
        if k > 0 and ln.startswith('*'):
            ln = ln[1:].strip()
        if ln:
            parts.append(ln)
    return ' '.join(parts)


def comment_block(indent, words):
    """`/* words */` at `indent`, wrapped at column 80 with ` * ` lines."""
    pad = ' ' * indent
    one = pad + '/* ' + words + ' */'
    if len(one) <= WIDTH or ' ' not in words:
        return [one]
    out = []
    line = ''
    for w in words.split(' '):
        lead = pad + ('/* ' if not out else ' * ')
        cand = (line + ' ' + w) if line else w
        if line and len(lead) + len(cand) + 3 > WIDTH:
            out.append(lead + line)
            line = w
        else:
            line = cand
    lead = pad + ('/* ' if not out else ' * ')
    out.append(lead + line + ' */')
    return out


def indent_of(src, ln):
    s = src.line_text(ln)
    return len(s) - len(s.lstrip(' '))


def apply_edits(text, edits):
    """edits: (start, end, replacement); non-overlapping."""
    edits = sorted(edits, key=lambda e: (e[0], e[1]))
    out = []
    pos = 0
    for s, e, r in edits:
        if s < pos:
            raise RuntimeError('overlapping edits')
        out.append(text[pos:s])
        out.append(r)
        pos = e
    out.append(text[pos:])
    return ''.join(out)


def removal(src, c):
    """Edit range that deletes comment token c and its white space."""
    text = src.text
    ls = src.line_starts[src.line_of(c.start)]
    le = text.find('\n', c.end)
    le = len(text) if le < 0 else le
    before = text[ls:c.start]
    after = text[c.end:le]
    if not before.strip() and not after.strip():
        return (ls, min(le + 1, len(text)), '')
    if not after.strip():
        s = c.start
        while s > ls and text[s - 1] in ' \t':
            s -= 1
        return (s, le, '')
    e = c.end
    while e < le and text[e] in ' \t':
        e += 1
    return (c.start, e, '')


# ---------------------------------------------------------------------------
# Phase 1: comments inside statements
# ---------------------------------------------------------------------------

def movable(src, seg, how):
    """Comments inside statements of function bodies move; so does the
    trailing comment of a multi-line file-scope declaration (a prototype),
    but not of a table with a brace initialiser.  Struct/union/enum bodies,
    labels and initialiser tables are never touched."""
    if seg.kind == 'label' or 'record' in seg.ctx or 'enum' in seg.ctx:
        return False
    if 'func' in seg.ctx:
        return True
    return how == 'trail' and not seg.ctx and seg.kind == 'stmt' and \
        not any(src.sig[k].text == '{' for k in seg.toks)


def seg_of_comments(src):
    """Map each comment to (segment, 'in' | 'trail') when it is inside a
    segment or trails its last token on the same line."""
    sig = src.sig
    res = {}
    spans = [(sig[g.toks[0]].start, sig[g.toks[-1]].end, g)
             for g in src.segs]
    spans.sort(key=lambda x: x[0])
    starts = [s[0] for s in spans]
    import bisect
    sig_starts = [t.start for t in sig]
    for c in src.comments:
        k = bisect.bisect_right(starts, c.start) - 1
        if k < 0:
            continue
        s, e, g = spans[k]
        if c.start < e:
            res[id(c)] = (g, 'in')
            continue
        last = sig[g.toks[-1]]
        if last.line == c.line:
            j = bisect.bisect_left(sig_starts, last.end)
            if j >= len(sig) or sig[j].start > c.start:
                res[id(c)] = (g, 'trail')
    return res


def move_inside(src, stats, trailing):
    """trailing=False: comments inside statements; True: comments that
    trail a statement spanning several lines (run after the re-break)."""
    where = seg_of_comments(src)
    groups = {}
    order = []
    for c in src.comments:
        w = where.get(id(c))
        if not w:
            continue
        g, how = w
        if (how == 'trail') != trailing or not movable(src, g, how):
            continue
        first = src.sig[g.toks[0]]
        last = src.sig[g.toks[-1]]
        if how == 'trail' and first.line == last.line:
            continue
        if how == 'trail' and c.text.count('\n'):
            continue
        if id(g) not in groups:
            groups[id(g)] = (g, [])
            order.append(id(g))
        groups[id(g)][1].append(c)
    if not groups:
        return src.text, 0
    edits = []
    n = 0
    for gid in order:
        g, cs = groups[gid]
        first = src.sig[g.toks[0]]
        ind = indent_of(src, first.line)
        words = '; '.join(w for w in (comment_words(c.text) for c in cs)
                          if w)
        ls = src.line_starts[first.line]
        if words:
            edits.append((ls, ls, '\n'.join(comment_block(ind, words))
                          + '\n'))
        for c in cs:
            edits.append(removal(src, c))
            n += 1
    stats['moved'] += n
    return apply_edits(src.text, edits), n


# ---------------------------------------------------------------------------
# Phase 2: continuation layout
# ---------------------------------------------------------------------------

def joined_sep(a, b):
    """Separator for two tokens that were on different lines."""
    if a.text in ('(', '[') or b.text in (')', ']', ',', ';', '[') or \
            a.text in ('->', '.') or b.text in ('->', '.'):
        return ''
    if b.text == '(' and a.kind == 'i':
        return ''
    if a.text in ('!', '~', '++', '--') or b.text in ('++', '--'):
        return ''
    return ' '


def break_penalty(sig, toks, k, depth):
    """Penalty for a line break after toks[k], or None (not allowed)."""
    t = sig[toks[k]]
    s = t.text
    if t.kind == 'l' and s.endswith('"') and k + 1 < len(toks) and \
            sig[toks[k + 1]].text.startswith('"'):
        return 30 * depth * depth       # between concatenated strings
    if t.kind not in ('p',):
        return None
    if s == ',':
        kind = 0
    elif s in ASSIGN_OPS:
        kind = 2
    elif s in BREAK_OPS:
        if s in ('+', '-'):
            p = sig[toks[k - 1]] if k else None
            if p is None or not (p.kind in ('i', 'l') or
                                 p.text in (')', ']')):
                return None
        if s == ':' and not any(sig[toks[j]].text == '?' for j in range(k)):
            return None
        kind = BREAK_OPS[s]
    elif s == '(':
        kind = 400
    else:
        return None
    return 30 * depth * depth + kind


def layout(widths, seps, pens, col0, cind):
    """Fewest-penalty line breaking.  Returns list of break indices (break
    after token k)."""
    n = len(widths)
    INF = float('inf')
    best = [INF] * (n + 1)
    back = [0] * (n + 1)
    best[0] = 0.0
    for i in range(n):
        if best[i] == INF:
            continue
        if i > 0 and pens[i - 1] is None:
            continue
        start = col0 if i == 0 else cind
        w = start
        base = best[i] + (0 if i == 0 else 100 + pens[i - 1])
        for j in range(i, n):
            w += widths[j] + (seps[j] if j > i else 0)
            over = max(0, w - WIDTH)
            cost = base + over * 1000
            if j == n - 1:
                if cost < best[n]:
                    best[n] = cost
                    back[n] = i
            elif pens[j] is not None and cost < best[j + 1]:
                best[j + 1] = cost
                back[j + 1] = i
            if over > 0 and j > i and w > WIDTH + 60:
                break
    brks = []
    k = n
    while k > 0:
        i = back[k]
        if i > 0:
            brks.append(i - 1)
        k = i
    return sorted(brks)


def reflow_seg(src, g):
    """Return (start, end, text) replacing the segment's lines, or None."""
    sig = src.sig
    toks = g.toks
    first = sig[toks[0]]
    last = sig[toks[-1]]
    if g.kind == 'label' or first.text == 'for':
        return None
    if g.kind == 'tail' and ('record' in g.ctx or 'enum' in g.ctx):
        return None
    if any(ln in src.pp_lines for ln in range(first.line, last.line + 1)):
        return None
    multi = first.line != last.line
    if not multi and src.col(last.end) <= WIDTH:
        return None
    # No comment may remain inside; no multi-line brace initialiser.
    for c in src.comments:
        if first.start < c.start < last.end:
            return None
    if multi:
        for k in toks:
            if sig[k].text == '{' and k + 0 in src.brace_kind:
                close = _match(sig, k)
                if close is not None and sig[close].line != sig[k].line:
                    return None
    widths = []
    seps = []
    for n_, k in enumerate(toks):
        t = sig[k]
        widths.append(len(t.text))
        if n_ == 0:
            seps.append(0)
            continue
        p = sig[toks[n_ - 1]]
        if p.line == t.line:
            gap = src.text[p.end:t.start]
            seps.append(len(gap))
        else:
            seps.append(len(joined_sep(p, t)))
    depth = 0
    pens = []
    for n_, k in enumerate(toks):
        s = sig[k].text
        if s in ('(', '[', '{'):
            depth += 1
        elif s in (')', ']', '}'):
            depth -= 1
        pens.append(break_penalty(sig, toks, n_, depth)
                    if n_ < len(toks) - 1 else None)
    ls = src.line_starts[first.line]
    col0 = first.start - ls
    cind = indent_of(src, first.line) + CONT
    brks = set(layout(widths, seps, pens, col0, cind))
    out = [src.text[ls:first.start]]
    for n_, k in enumerate(toks):
        t = sig[k]
        if n_ > 0:
            p = sig[toks[n_ - 1]]
            if (n_ - 1) in brks:
                out.append('\n' + ' ' * cind)
            elif p.line == t.line:
                out.append(src.text[p.end:t.start])
            else:
                out.append(joined_sep(p, t))
        out.append(t.text)
    le = src.text.find('\n', last.end)
    le = len(src.text) if le < 0 else le
    new = ''.join(out).rstrip(' ')
    tail = src.text[last.end:le]
    new += tail.rstrip(' ') if tail.strip() else ''
    old = src.text[ls:le]
    if new == old:
        return None
    return (ls, le, new)


def _match(sig, k):
    d = 0
    for j in range(k, len(sig)):
        if sig[j].text == '{':
            d += 1
        elif sig[j].text == '}':
            d -= 1
            if d == 0:
                return j
    return None


def reflow(src, stats):
    edits = []
    used = set()
    for g in src.segs:
        e = reflow_seg(src, g)
        if e is None:
            continue
        ln0 = src.line_of(e[0])
        ln1 = src.line_of(e[1])
        if any(ln in used for ln in range(ln0, ln1 + 1)):
            continue
        used.update(range(ln0, ln1 + 1))
        edits.append(e)
    stats['reflowed'] += len(edits)
    return apply_edits(src.text, edits), len(edits)


# ---------------------------------------------------------------------------
# Phase 3: paragraphs and trailing-comment alignment
# ---------------------------------------------------------------------------

def line_info(src):
    """Per line: last significant token, first token, trailing comment."""
    last_sig = {}
    first_tok = {}
    for t in src.toks:
        if t.line not in first_tok:
            first_tok[t.line] = t
        if t.kind != 'c':
            last_sig[t.line] = t
    return last_sig, first_tok


def ends_group(src, prev, last_sig):
    """True when line `prev` ends a statement, declaration, table row or
    preprocessor line, so a comment below it starts a new paragraph."""
    if prev in src.pp_lines:
        return not src.line_text(prev).rstrip().endswith('\\')
    lt = last_sig.get(prev)
    if lt is None:
        # The last line of a trailing comment that spans lines: judge the
        # code line it trails.
        for c in src.comments:
            if c.start <= src.line_starts[prev] < c.end:
                if c.line != prev and c.line in last_sig:
                    return ends_group(src, c.line, last_sig)
                return False
        return False
    k = src.sig_index.get(id(lt))
    if k is None:
        return False
    if lt.text == ';':
        return src.inner_at[k][1] == 0       # not inside `for (;;)`
    if lt.text == '}':
        if src.brace_kind.get(_open_of(src, k)) == 'init':
            nxt = src.sig[k + 1].text if k + 1 < len(src.sig) else ''
            return nxt not in (')', ';')
        return True
    if lt.text == ',':
        kind, paren = src.inner_at[k]
        return paren == 0 and kind == 'init'
    return False


def _open_of(src, k):
    d = 0
    for j in range(k, -1, -1):
        t = src.sig[j].text
        if t == '}':
            d += 1
        elif t == '{':
            d -= 1
            if d == 0:
                return j
    return None


def blank_before_comments(src, stats):
    last_sig, first_tok = line_info(src)
    ins = []
    for c in src.comments:
        if first_tok.get(c.line) is not c or c.line == 0:
            continue
        prev = c.line - 1
        if not src.line_text(prev).strip():
            continue
        if not ends_group(src, prev, last_sig):
            continue
        ins.append((src.line_starts[c.line], src.line_starts[c.line], '\n'))
    stats['blanks'] += len(ins)
    return apply_edits(src.text, ins), len(ins)


def trailing_groups(src):
    """Trailing single-line comments grouped by paragraph and brace level.
    Each item is (comment, code_end_col, line, level)."""
    sig_index = src.sig_index
    items = []
    prev = None
    for c in src.toks:
        p = prev
        prev = c
        if c.kind != 'c' or '\n' in c.text or p is None or \
                p.line != c.line or p.kind == 'c':
            continue
        if src.line_text(c.line).rstrip().endswith('\\'):
            continue
        k = sig_index.get(id(p))
        if k is not None and k + 1 < len(src.sig) and \
                src.sig[k + 1].line == c.line:
            continue            # comment inside the line's code
        if p.text == '{':
            lvl = ('{', c.line)      # after `{`: never aligned with others
        elif p.pp or k is None:
            lvl = -1
        else:
            lvl = src.level_at[k] - (1 if p.text == '}' else 0)
        items.append((c, p.end - src.line_starts[c.line], c.line, lvl))
    groups = []
    cur = []
    for it in items:
        if cur:
            pl = cur[-1][2]
            gap = any(not src.line_text(k).strip()
                      for k in range(pl + 1, it[2]))
            if gap or it[3] != cur[-1][3]:
                groups.append(cur)
                cur = []
        cur.append(it)
    if cur:
        groups.append(cur)
    return groups


def overflow_move(src, c):
    """Edits moving trailing comment c off its code line, or None.

    The comment goes on its own line above the code when the line holds
    whole items -- it starts after `;`, `,`, `{`, `}` or a label at
    parenthesis depth 0 and ends in `;`, `,` or `}` (a one-line statement,
    declaration, struct member, enumerator or table row), or is exactly one
    statement header (`if (...)`, `else`) -- or is a one-line preprocessor
    directive.  After a block's `{` it goes on the line below, one level
    in."""
    toks = [t for t in src.sig if t.line == c.line]
    ls = src.line_starts[c.line]
    words = comment_words(c.text)
    if c.pp:
        line = src.line_text(c.line)
        if c.line - 1 in src.pp_lines and \
                src.line_text(c.line - 1).rstrip().endswith('\\'):
            return None
        if line.rstrip().endswith('\\') or not line.startswith('#'):
            return None
        return [(ls, ls, '\n'.join(comment_block(0, words)) + '\n'),
                removal(src, c)]
    if not toks:
        return None
    first, last = toks[0], toks[-1]
    if last.end > c.start or src.text[ls:first.start].strip():
        return None
    kl = src.sig_index[id(last)]
    if last.text == '{' and src.brace_kind.get(kl) != 'init':
        le = src.text.find('\n', c.end)
        le = len(src.text) if le < 0 else le + 1
        block = comment_block(indent_of(src, c.line) + 4, words)
        return [removal(src, c), (le, le, '\n'.join(block) + '\n')]
    k = src.sig_index[id(first)]
    whole = (k == 0 or src.sig[k - 1].text in (';', ',', '{', '}', ':')) \
        and src.inner_at[k][1] == 0 and last.text in (';', ',', '}') \
        and src.inner_at[kl][1] == 0
    if not whole:
        whole = any(src.sig[g.toks[0]] is first and src.sig[g.toks[-1]] is
                    last for g in src.segs if g.kind != 'label')
    if not whole:
        return None
    return [(ls, ls, '\n'.join(comment_block(indent_of(src, c.line),
                                             words)) + '\n'),
            removal(src, c)]


def multiline_trailing(src):
    """Trailing comments that span lines (they always move)."""
    out = []
    prev = None
    for t in src.toks:
        if t.kind == 'c' and '\n' in t.text and prev is not None and \
                prev.kind != 'c' and prev.line == t.line:
            out.append(t)
        prev = t
    return out


def align_and_overflow(src, stats):
    """Align trailing comments per group, or move the comments that would
    pass column 80 (one per group per round).  Returns (text, moved)."""
    edits = []
    for c in multiline_trailing(src):
        mv = overflow_move(src, c)
        if mv is not None:
            edits.extend(mv)
    if edits:
        stats['overflow'] += len(edits) // 2
        return apply_edits(src.text, edits), len(edits) // 2
    groups = trailing_groups(src)
    final = []
    for grp in groups:
        grp = list(grp)
        while grp:
            colm = max(ce for _, ce, _, _ in grp) + 1
            worst = None
            for it in grp:
                c, ce, ln, _ = it
                rest = len(src.line_text(ln)) - src.col(c.start)
                if colm + rest > WIDTH:
                    key = (ce + rest, -ln)
                    if worst is None or key > worst[0]:
                        worst = (key, it)
            if worst is None:
                break
            mv = overflow_move(src, worst[1][0])
            if mv is not None:
                edits.extend(mv)
                break
            grp.remove(worst[1])     # cannot move: left out of the column
            final.append([worst[1]])
        final.append(grp)
    if edits:
        stats['overflow'] += len(edits) // 2
        return apply_edits(src.text, edits), len(edits) // 2
    for grp in final:
        if not grp:
            continue
        colm = max(ce for _, ce, _, _ in grp) + 1
        for c, ce, ln, _ in grp:
            ls = src.line_starts[ln]
            if c.start != ls + colm:
                edits.append((ls + ce, c.start, ' ' * (colm - ce)))
    return apply_edits(src.text, edits), 0


# ---------------------------------------------------------------------------

def process(path, text, stats):
    src = Source(path, text)
    text, _ = move_inside(src, stats, False)
    src = Source(path, text)
    text, _ = reflow(src, stats)
    src = Source(path, text)
    text, _ = move_inside(src, stats, True)
    for _ in range(200):
        src = Source(path, text)
        text, _ = blank_before_comments(src, stats)
        src = Source(path, text)
        text, moved = align_and_overflow(src, stats)
        if not moved:
            break
    return text


def long_statements(path, text):
    """(line, lines) for segments with more than two continuation lines."""
    src = Source(path, text)
    out = []
    for g in src.segs:
        a = src.sig[g.toks[0]].line
        b = src.sig[g.toks[-1]].line
        if b - a + 1 > 3 and not any(ln in src.pp_lines
                                     for ln in range(a, b + 1)):
            out.append((a + 1, b - a + 1))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('files', nargs='+')
    ap.add_argument('--stats', action='store_true')
    ap.add_argument('--long', metavar='TSV',
                    help='write statements with > 2 continuation lines')
    args = ap.parse_args()
    stats = {'moved': 0, 'reflowed': 0, 'blanks': 0, 'overflow': 0,
             'files': 0}
    long_rows = []
    for path in args.files:
        with open(path, encoding='utf-8') as f:
            text = f.read()
        if args.long:
            for ln, n in long_statements(path, text):
                long_rows.append('%s:%d\t%d' % (path, ln, n))
            continue
        new = process(path, text, stats)
        if new != text:
            stats['files'] += 1
            with open(path, 'w', encoding='utf-8') as f:
                f.write(new)
    if args.long:
        with open(args.long, 'w', encoding='utf-8') as f:
            f.write('location\tlines\n')
            f.write('\n'.join(long_rows) + ('\n' if long_rows else ''))
        print('%d statements with > 2 continuation lines' % len(long_rows))
    if args.stats:
        print(' '.join('%s=%d' % kv for kv in sorted(stats.items())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
