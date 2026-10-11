#!/usr/bin/env python3
"""fix_comments.py -- comment placement, wraps, paragraphs and alignment.

Usage (from the repo root; `make format` runs it around Uncrustify):
    python3 -I tools/fix_comments.py [--stats] FILE...
    python3 -I tools/fix_comments.py --long TSV FILE...   (report only)

Rules (docs/CODING_STANDARD.md "Comment placement and wrapping", user
direction 2026-10-11; NASA SEL-94-003 2.2.1, 2.3, 5.4):

1. No column limit; one statement per line.  A statement that spans lines
   is joined back onto one line, except in the one allowed wrap shape: the
   OUTERMOST call of the statement has its arguments one per line (the
   first line of a call may carry two, e.g. `ip` and the first real
   argument; at least two continuation lines, so a list wrapped once is
   joined), every line break sits after a comma of that call, and
   Uncrustify aligns each argument under the first.  A break inside a
   nested call, after `=`, or after an operator is always joined.  Brace
   initialiser tables, statements around preprocessor lines and macros are
   left alone.  A brace initialiser `{...}` is joined onto one line like
   any other statement unless it is a table: one element per line (a
   nested brace is one element; at least two continuation lines), or with
   a comment inside (rows with trailing comments).
2. A comment never sits inside a statement.  In a function body every
   comment inside a statement (after `=`, inside a call, between operands,
   on a continuation line) merges with the statement's trailing comment
   into one trailing comment on the statement's (first) line, in source
   order, separated by "; ".  The
   words are kept verbatim.  Function prologs, `#define` lines, struct /
   union / enum bodies, `case` labels and initialiser tables are not
   touched.  A trailing comment that itself spans lines is put on one line
   (no column limit), so every trailing comment is a single line.
3. A standalone comment that follows a statement, declaration, table row
   or preprocessor line starts a paragraph: a blank line goes above it.
   Consecutive comment lines stay together.
4. Within a paragraph (consecutive non-blank lines), at one brace level,
   one-line statements line up as columns: declaration type | name (the
   `*` dangles left of the name) | assignment operator | expression |
   trailing comment, the comment column one space past the longest code
   line of the paragraph.  A comment after `{` is not aligned.  The
   continuation lines of a kept wrap move with their first line.

Uses the libclang token stream (python3 clang.cindex) for lexing, so
string and character literals, comments and preprocessor lines are
recognised exactly.  Only white space, line breaks and comment positions
change: `gcc -E` output (comments stripped) is token-identical before and
after, except for the white space inside a `#x` stringification of a
joined macro argument (assertion text).  Idempotent; `make format` runs it
before and after Uncrustify, and `make format-check` runs the whole
pipeline on a copy.
"""
import argparse
import re
import sys

import clang.cindex as ci

# Lexing only: headers are not needed (and not read).
CLANG_ARGS = ['-x', 'c', '-std=c11', '-nostdinc']
ASSIGN_OPS = {'=', '+=', '-=', '*=', '/=', '%=', '<<=', '>>=', '&=', '|=',
              '^='}
STMT_KEYWORDS = {'return', 'break', 'continue', 'goto', 'case', 'default',
                 'if', 'while', 'for', 'do', 'switch', 'else', 'sizeof',
                 'typedef'}
LONG_WIDTH = 120

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
    level_at = []       # innermost open brace (index) around each token;
    bidx = []           # for a `}` the brace around the block it closes
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
        level_at.append(bidx[-1] if bidx else -1)
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
                bidx.append(i)
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
            bidx.append(i)
            continue
        if s == '}':
            if bidx:
                bidx.pop()
            level_at[-1] = bidx[-1] if bidx else -1
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


# ---------------------------------------------------------------------------
# Phase 1: wraps and comments inside statements
# ---------------------------------------------------------------------------

def paren_info(sig, toks):
    """inner[n]: position (in toks) of the innermost open bracket around
    toks[n]; call: position of the outermost call's `(` (a `(` after an
    identifier with no call around it), or None."""
    inner = []
    stack = []
    calls = []
    call = None
    for n, k in enumerate(toks):
        t = sig[k]
        inner.append(stack[-1] if stack else None)
        if t.text in ('(', '[', '{'):
            c = t.text == '(' and n > 0 and sig[toks[n - 1]].kind == 'i'
            if c and call is None and not any(calls):
                call = n
            stack.append(n)
            calls.append(c)
        elif t.text in (')', ']', '}') and stack:
            stack.pop()
            calls.pop()
    return inner, call


def is_declarator(src, g):
    """A function definition header or prototype at file scope."""
    return not g.ctx and g.kind in ('header', 'stmt')


def kept_wrap(src, g):
    """True when the segment is in the one allowed wrap shape."""
    sig = src.sig
    toks = g.toks
    inner, p = paren_info(sig, toks)
    if p is None or sig[toks[p]].line != sig[toks[0]].line:
        return False
    breaks = [n for n in range(len(toks) - 1)
              if sig[toks[n]].line != sig[toks[n + 1]].line]
    if not breaks:
        return False
    for n in breaks:
        if sig[toks[n]].text != ',' or inner[n] != p:
            return False
    first_args = 1 + sum(1 for m in range(p + 1, breaks[0])
                         if sig[toks[m]].text == ',' and inner[m] == p)
    for i, n in enumerate(breaks):
        end = breaks[i + 1] if i + 1 < len(breaks) else len(toks) - 1
        for m in range(n + 1, end):
            if sig[toks[m]].text == ',' and inner[m] == p:
                return False
    if len(breaks) < 2:
        return False
    if is_declarator(src, g):
        return first_args == 1
    return first_args <= 2


def seg_comments(src, g):
    """(interior comments, trailing comments) of a segment.  Trailing ones
    follow the last token on its line, before any further code."""
    sig = src.sig
    first = sig[g.toks[0]]
    last = sig[g.toks[-1]]
    kl = src.sig_index[id(last)]
    nxt = sig[kl + 1].start if kl + 1 < len(sig) else len(src.text) + 1
    inner = []
    trail = []
    for c in src.comments:
        if first.start < c.start < last.end:
            inner.append(c)
        elif last.end <= c.start < nxt and c.line == last.line:
            trail.append(c)
    return inner, trail


def join_text(src, toks):
    """The tokens on one line: same-line gaps kept, others one space or
    none (Uncrustify settles the spacing)."""
    sig = src.sig
    out = []
    for n, k in enumerate(toks):
        t = sig[k]
        if n:
            p = sig[toks[n - 1]]
            gap = src.text[p.end:t.start]
            if p.line == t.line and '/*' not in gap and '//' not in gap:
                out.append(gap)
            elif p.line == t.line:
                out.append(' ')
            else:
                out.append(joined_sep(p, t))
        out.append(t.text)
    return ''.join(out)


def merged_words(trail, inner):
    """The comments' words in source order, separated by '; '."""
    words = [comment_words(c.text)
             for c in sorted(trail + inner, key=lambda t: t.start)]
    return '; '.join(w for w in words if w)


def init_is_table(src, o, c):
    """True when the multi-line brace initialiser sig[o]..sig[c] is a
    table that is kept as written: one element per line (each element
    starts its own line, the first may share the `{` line; at least two
    continuation lines; a nested brace counts as one element), or a
    documented table (any comment inside: rows with trailing comments).
    Anything else is a partial wrap and is joined onto one line."""
    sig = src.sig
    if any(sig[o].start < x.start < sig[c].start for x in src.comments):
        return True
    if sig[c].line - sig[o].line < 2:
        return False
    depth = 0
    firsts = []
    start = True
    for j in range(o + 1, c):
        t = sig[j]
        if start:
            firsts.append(j)
            start = False
        if t.text in ('(', '[', '{'):
            depth += 1
        elif t.text in (')', ']', '}'):
            depth -= 1
        elif t.text == ',' and depth == 0:
            start = True
    seen = set()
    for n, j in enumerate(firsts):
        ln = sig[j].line
        if ln in seen:
            return False            # two elements on one line
        seen.add(ln)
        if n == 0 and ln == sig[o].line:
            continue
        if sig[j - 1].line == ln:
            return False            # not first on its line
    return True


def fix_segment(src, g):
    """Edits for one segment, or []."""
    sig = src.sig
    toks = g.toks
    first = sig[toks[0]]
    last = sig[toks[-1]]
    if g.kind == 'label' or g.kind == 'tail' and \
            ('record' in g.ctx or 'enum' in g.ctx):
        return []
    if any(ln in src.pp_lines for ln in range(first.line, last.line + 1)):
        return []
    multi = first.line != last.line
    for k in toks:
        if sig[k].text == '{' and src.brace_kind.get(k) == 'init' and \
                src.inner_at[k][0] != 'init':
            close = _match(sig, k)
            if close is None:
                return []
            if sig[close].line != sig[k].line and \
                    init_is_table(src, k, close):
                return []           # a brace initialiser table: kept
    inner, trail = seg_comments(src, g)
    in_func = 'func' in g.ctx and 'record' not in g.ctx and \
        'enum' not in g.ctx
    if inner and not in_func:
        return []                   # struct/enum/file-scope: untouched
    if not multi and not inner:
        return []
    ls = src.line_starts[first.line]
    prefix = src.text[ls:first.start]
    if multi and kept_wrap(src, g):
        # Allowed wrap: the statement's comments become one trailing
        # comment on its first line.
        on_first = [c for c in inner if c.line == first.line and not any(
            sig[k].line == first.line and sig[k].start > c.start
            for k in toks)]
        others = [c for c in inner if c not in on_first]
        if not others and not trail:
            return []
        words = merged_words(on_first + trail, others)
        fl = [sig[k] for k in toks if sig[k].line == first.line][-1]
        edits = [removal(src, c) for c in inner + trail]
        edits.append((fl.end, fl.end, ' /* ' + words + ' */'))
        return edits
    if trail:
        endc = trail[-1]
        le = src.text.find('\n', endc.end)
    else:
        le = src.text.find('\n', last.end)
    le = len(src.text) if le < 0 else le
    tail_end = trail[-1].end if trail else last.end
    if src.text[tail_end:le].strip():
        return []                   # more code on the line: leave it
    new = prefix + join_text(src, toks)
    if inner:
        new += ' /* ' + merged_words(trail, inner) + ' */'
    elif trail:
        new += ' ' + ' '.join(c.text for c in trail)
    if new == src.text[ls:le]:
        return []
    return [(ls, le, new)]


def fix_statements(src, stats):
    edits = []
    for g in src.segs:
        e = fix_segment(src, g)
        if e:
            edits.extend(e)
            stats['statements'] += 1
    return apply_edits(src.text, edits), len(edits)


# ---------------------------------------------------------------------------
# Phase 2: paragraphs (blank line before a standalone comment) and
# trailing comments that span lines
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Phase 3: paragraph alignment
# ---------------------------------------------------------------------------

def parse_decl(sig, lhs):
    """(type words, stars, name position) when the tokens before `=` (or
    `;`) are a plain declaration `type... [*...]name[[...]]`, else None."""
    if not lhs or sig[lhs[0]].kind not in ('i', 'k') or \
            sig[lhs[0]].text in STMT_KEYWORDS:
        return None
    n = 0
    words = []
    while n < len(lhs) and sig[lhs[n]].kind in ('i', 'k'):
        words.append(n)
        n += 1
    stars = 0
    while n < len(lhs) and sig[lhs[n]].text == '*':
        stars += 1
        n += 1
    if stars:
        if n >= len(lhs) or sig[lhs[n]].kind != 'i':
            return None
        name = n
        n += 1
    else:
        if len(words) < 2:
            return None
        name = words.pop()
    if not words:
        return None
    depth = 0
    for m in range(n, len(lhs)):
        s = sig[lhs[m]].text
        if s == '[':
            depth += 1
        elif s == ']':
            depth -= 1
        elif depth == 0:
            return None
    return words, stars, name


def statement_lines(src):
    """line -> (segment, lhs positions, op position or None, decl)."""
    sig = src.sig
    out = {}
    for g in src.segs:
        if g.kind != 'stmt' or 'enum' in g.ctx or 'init' in g.ctx:
            continue
        toks = g.toks
        first = sig[toks[0]]
        if first.pp or src.text[src.line_starts[first.line]:first.start]\
                .strip():
            continue
        depth = 0
        op = None
        for n, k in enumerate(toks):
            t = sig[k]
            if t.line != first.line:
                break
            if t.text in ('(', '[', '{'):
                depth += 1
            elif t.text in (')', ']', '}'):
                depth -= 1
            elif depth == 0 and t.text in ASSIGN_OPS:
                op = n
                break
        if op is None and sig[toks[-1]].line != first.line:
            continue
        lhs = toks[:op] if op is not None else toks[:-1]
        decl = parse_decl(sig, lhs)
        if op is None and decl is None:
            continue
        out[first.line] = (g, lhs, op, decl)
    return out


def blank_after_top_decls(src, stats):
    """5.4: a blank line after the declarations that open a function body
    (later declarations may share a paragraph with the statements that use
    them)."""
    sig = src.sig
    stl = statement_lines(src)
    by_body = {}
    for g in src.segs:
        k = src.sig_index[id(sig[g.toks[0]])]
        b = src.level_at[k]
        if b >= 0 and src.brace_kind.get(b) == 'func':
            by_body.setdefault(b, []).append(g)
    ins = []
    for segs in by_body.values():
        prev = None
        for g in segs:
            first = sig[g.toks[0]]
            info = stl.get(first.line)
            if info is not None and info[0] is g and info[3] is not None:
                prev = g
                continue
            if prev is not None and \
                    first.line == sig[prev.toks[-1]].line + 1:
                ls = src.line_starts[first.line]
                ins.append((ls, ls, '\n'))
            break
    stats['blanks'] += len(ins)
    return apply_edits(src.text, ins), len(ins)


def paragraphs(src):
    """Lists of consecutive non-blank line numbers."""
    paras = []
    cur = []
    for ln in range(len(src.line_starts)):
        if src.line_text(ln).strip():
            cur.append(ln)
        elif cur:
            paras.append(cur)
            cur = []
    if cur:
        paras.append(cur)
    return paras


def align(src, stats):
    sig = src.sig
    stl = statement_lines(src)
    new_code = {}       # line -> new code text (no trailing comment)
    shift = {}          # continuation line -> column delta
    line_level = {}
    for ln, (g, lhs, op, decl) in stl.items():
        line_level[ln] = src.level_at[src.sig_index[id(sig[g.toks[0]])]]
    for para in paragraphs(src):
        by_level = {}
        for ln in para:
            if ln in stl:
                by_level.setdefault(line_level[ln], []).append(ln)
        for lines in by_level.values():
            parts = {}
            ncol = 0
            for ln in lines:
                g, lhs, op, decl = stl[ln]
                if decl:
                    words, stars, name = decl
                    ttext = ' '.join(sig[lhs[w]].text for w in words)
                    ncol = max(ncol, len(ttext) + 1 + stars)
            for ln in lines:
                g, lhs, op, decl = stl[ln]
                if decl:
                    words, stars, name = decl
                    ttext = ' '.join(sig[lhs[w]].text for w in words)
                    ntext = src.text[sig[lhs[name]].start:sig[lhs[-1]].end]
                    left = ttext + ' ' * (ncol - len(ttext) - stars) + \
                        '*' * stars + ntext
                else:
                    left = src.text[sig[lhs[0]].start:sig[lhs[-1]].end]
                parts[ln] = left
            ecol = max([len(parts[ln]) for ln in lines
                        if stl[ln][2] is not None] or [0]) + 1
            for ln in lines:
                g, lhs, op, decl = stl[ln]
                toks = g.toks
                first = sig[toks[0]]
                ls = src.line_starts[ln]
                indent = src.text[ls:first.start]
                on_line = [sig[k] for k in toks if sig[k].line == ln]
                code_end = on_line[-1].end
                if op is not None:
                    opt = sig[toks[op]]
                    code = indent + parts[ln] + \
                        ' ' * (ecol - len(parts[ln])) + \
                        src.text[opt.start:code_end]
                    old_op = opt.start - ls
                    new_op = len(indent) + ecol
                else:
                    code = indent + parts[ln] + \
                        src.text[sig[lhs[-1]].end:code_end]
                    old_op = new_op = 0
                new_code[ln] = code
                last = sig[toks[-1]]
                if last.line != ln and new_op != old_op:
                    for k in range(ln + 1, last.line + 1):
                        shift[k] = new_op - old_op
    # Trailing comments: one column per paragraph and level.
    groups = trailing_groups(src)
    cmt_col = {}
    for grp in groups:
        width = 0
        for c, ce, ln, _ in grp:
            width = max(width, len(new_code.get(ln, src.line_text(ln)[:ce])))
        for c, ce, ln, _ in grp:
            cmt_col[ln] = (c, width + 1)
    edits = []
    for ln in range(len(src.line_starts)):
        old = src.line_text(ln)
        if ln in cmt_col:
            c, col = cmt_col[ln]
            ce = c.start - src.line_starts[ln]
            code = new_code.get(ln, old[:ce].rstrip())
            new = code + ' ' * (col - len(code)) + old[ce:].rstrip()
        elif ln in new_code:
            code_old_end = len(old.rstrip())
            # keep anything after the code (e.g. an unaligned comment)
            g = stl[ln][0]
            on_line = [src.sig[k] for k in g.toks if src.sig[k].line == ln]
            ce = on_line[-1].end - src.line_starts[ln]
            new = new_code[ln] + old[ce:code_old_end]
        elif ln in shift and old.strip():
            d = shift[ln]
            lead = len(old) - len(old.lstrip(' '))
            new = ' ' * max(0, lead + d) + old.lstrip(' ')
        else:
            continue
        if new != old:
            ls = src.line_starts[ln]
            edits.append((ls, ls + len(old), new))
    stats['aligned'] += len(edits)
    return apply_edits(src.text, edits), len(edits)


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
            lvl = src.level_at[k]
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


# ---------------------------------------------------------------------------

def process(path, text, stats):
    src = Source(path, text)
    text, _ = fix_statements(src, stats)
    for _ in range(10):
        src = Source(path, text)
        text, n1 = blank_before_comments(src, stats)
        src = Source(path, text)
        text, n2 = blank_after_top_decls(src, stats)
        n1 += n2
        src = Source(path, text)
        edits = [(c.start, c.end, '/* ' + comment_words(c.text) + ' */')
                 for c in multiline_trailing(src)]
        if edits:
            stats['joined_comments'] += len(edits)
            text = apply_edits(src.text, edits)
        if not n1 and not edits:
            break
    src = Source(path, text)
    text, _ = align(src, stats)
    return text


def long_statements(path, text):
    """(line, lines, width) for statements kept on several lines with more
    than two continuation lines, or wider than LONG_WIDTH (brace
    initialiser tables excluded): candidates for named temporaries."""
    src = Source(path, text)
    out = []
    for g in src.segs:
        first = src.sig[g.toks[0]]
        last = src.sig[g.toks[-1]]
        if first.pp or g.kind == 'label' or any(
                src.brace_kind.get(k) == 'init' and
                src.inner_at[k][0] != 'init' and
                src.sig[_match(src.sig, k) or k].line != src.sig[k].line and
                init_is_table(src, k, _match(src.sig, k))
                for k in g.toks if src.sig[k].text == '{'):
            continue                # tables are not statements to split
        n = last.line - first.line + 1
        width = max(len(src.line_text(k).rstrip())
                    for k in range(first.line, last.line + 1))
        if n > 3 or width > LONG_WIDTH:
            out.append((first.line + 1, n, width))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('files', nargs='+')
    ap.add_argument('--stats', action='store_true')
    ap.add_argument('--long', metavar='TSV',
                    help='report statements with > 2 continuation lines or '
                    'wider than %d columns (no edits)' % LONG_WIDTH)
    args = ap.parse_args()
    stats = {'statements': 0, 'blanks': 0, 'joined_comments': 0, 'aligned': 0,
             'files': 0}
    rows = []
    for path in args.files:
        with open(path, encoding='utf-8') as f:
            text = f.read()
        if args.long:
            for ln, n, w in long_statements(path, text):
                rows.append('%s:%d\t%d\t%d' % (path, ln, n, w))
            continue
        new = process(path, text, stats)
        if new != text:
            stats['files'] += 1
            with open(path, 'w', encoding='utf-8') as f:
                f.write(new)
    if args.long:
        with open(args.long, 'w', encoding='utf-8') as f:
            f.write('location\tlines\twidth\n')
            f.write(''.join(r + '\n' for r in rows))
        print('%d long statements' % len(rows))
    if args.stats:
        print(' '.join('%s=%d' % kv for kv in sorted(stats.items())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
