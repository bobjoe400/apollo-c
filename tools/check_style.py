#!/usr/bin/env python3
"""check_style.py -- mechanical checks for docs/STYLE.md (NASA SEL-94-003).

Usage:
    python3 -I tools/check_style.py [options] <dir|file>...

Options:
    --summary        print only the count of findings per rule
    --rule ID[,ID]   report only these rule ids (see RULES below)
    --list-rules     print the rule ids with their guide sections and exit
    --exit-zero      always exit 0 (report-only use in CI)

Covers the rows of docs/STYLE.md marked `check_rules` that are cheap to test
on text: prologs, `return`/`sizeof` parentheses, explicit numeric tests,
one statement / one declaration per line, comments on declarations and
constants, numeric literal forms, side effects, switch defaults, long-block
closing comments, dummy loop bodies, naming case. Formatting rules
(indentation, braces, spacing, alignment, line length) belong to
clang-format (`make format-check`) and are NOT repeated here.

The script only reads files; it never edits them. Exit status 1 when any
finding is reported (unless --exit-zero).
"""
import os
import re
import sys

# rule id -> (guide section, one-line description)
RULES = {
    'file-prolog':           ('4.1',     'file starts with a prolog comment'),
    'file-prolog-fields':    ('4.1/3.4', 'file prolog has the template fields'),
    'function-prolog':       ('5.1',     'function preceded by a prolog comment'),
    'function-separator':    ('4.6',     'prolog opens with a row of asterisks'),
    'function-prolog-agc':   ('5.1/CS',  'prolog cites AGC label/page or "AGC: none"'),
    'function-prolog-args':  ('5.1',     'prolog names every argument'),
    'function-prolog-returns': ('5.1',   'non-void prolog describes the return'),
    'return-parens':         ('7.1.2',   'return expression in parentheses'),
    'sizeof-parens':         ('7.1.2',   'sizeof object in parentheses'),
    'numeric-test':          ('7.1.2',   'explicit comparison on numeric tests'),
    'one-statement':         ('7.1.1',   'one statement per line'),
    'one-declaration':       ('6.1',     'one variable per declaration line'),
    'decl-comment':          ('5.4/6',   'each variable declaration has a comment'),
    'scale-comment':         ('CS',      'SP/DP/TP/vector comment gives the scale'),
    'define-comment':        ('6.2',     'each constant #define has a comment'),
    'define-parens':         ('6.2.4',   'macro value with operators parenthesised'),
    'constant-case':         ('2.4.3',   'macros and enum constants UPPER_CASE'),
    'variable-case':         ('2.4.3',   'variables lower_case'),
    'case-unique':           ('2.4',     'names unique irrespective of case'),
    'number-format':         ('6.3.1',   '0.5 / 5.0, 0x hex with A-F, L suffix'),
    'qualifier-base':        ('6.3.2',   'short/long/unsigned with base type'),
    'side-effect-condition': ('6.7/6.8', 'no side effects in if/while/switch tests'),
    'multiple-side-effects': ('6.7/7.1.1', 'at most one side effect per statement'),
    'ternary-comment':       ('6.9',     '?: commented, never nested'),
    'dummy-body':            ('7.1.2',   'empty loop body: `;` on next line + comment'),
    'long-block-comment':    ('7.1.2',   'blocks > 40 lines: comment on closing brace'),
    'switch-default':        ('7.2.5',   'switch has a default, and it is last'),
    'extern-in-c':           ('5.3/CS',  'no extern declarations in .c files'),
    'header-name':           ('3.4',     'no header named like a standard header'),
    'main-first':            ('4.6',     'main() is the first function'),
}

FILE_FIELDS_C = ('FILE NAME', 'PURPOSE', 'AGC SOURCE', 'EXTERNAL VARIABLES',
                 'EXTERNAL REFERENCES', 'ABNORMAL TERMINATION')
FILE_FIELDS_H = ('FILE NAME', 'PURPOSE', 'AGC SOURCE', 'REQUIRES',
                 'GLOBAL VARIABLES')

STD_HEADERS = {
    'assert.h', 'complex.h', 'ctype.h', 'errno.h', 'fenv.h', 'float.h',
    'inttypes.h', 'iso646.h', 'limits.h', 'locale.h', 'math.h', 'setjmp.h',
    'signal.h', 'stdalign.h', 'stdarg.h', 'stdatomic.h', 'stdbool.h',
    'stddef.h', 'stdint.h', 'stdio.h', 'stdlib.h', 'stdnoreturn.h',
    'string.h', 'tgmath.h', 'threads.h', 'time.h', 'uchar.h', 'wchar.h',
    'wctype.h',
}

KEYWORDS = {
    'auto', 'break', 'case', 'char', 'const', 'continue', 'default', 'do',
    'double', 'else', 'enum', 'extern', 'float', 'for', 'goto', 'if',
    'inline', 'int', 'long', 'register', 'restrict', 'return', 'short',
    'signed', 'sizeof', 'static', 'struct', 'switch', 'typedef', 'union',
    'unsigned', 'void', 'volatile', 'while', 'bool', 'true', 'false',
    '_Bool', '_Static_assert', 'NULL', 'defined', 'include', 'define',
    'ifdef', 'ifndef', 'endif', 'elif', 'pragma', 'undef', 'error',
}

INT_BASE_TYPES = {'int', 'short', 'long', 'unsigned', 'signed', 'char',
                  'size_t', 'ptrdiff_t', 'intptr_t', 'uintptr_t'}
SCALED_TYPES = {'agc_sp_t', 'agc_dp_t', 'agc_tp_t', 'agc_vec_t'}

TYPE_WORD = (r'(?:struct\s+\w+|enum\s+\w+|union\s+\w+|bool|_Bool|char|short|'
             r'int|long|float|double|signed|unsigned|void|\w+_t)')
DECL_RE = re.compile(
    r'^(?P<type>(?:(?:static|const|volatile|register|extern)\s+)*'
    + TYPE_WORD + r'(?:\s+(?:const|volatile|' + TYPE_WORD[3:-1] + r'))*)'
    r'(?P<decls>\s*\**\s*[A-Za-z_]\w*\s*(?:\[[^\]]*\]\s*)*'
    r'(?:[=,;].*)?)$', re.S)
DECLARATOR_RE = re.compile(r'\s*\**\s*(?:const\s+)?([A-Za-z_]\w*)\s*((?:\[[^\]]*\]\s*)*)')

TOKEN_RE = re.compile(
    r'\s*(?:(?P<id>[A-Za-z_]\w*)|(?P<num>\.?\d[\w.]*(?:[eEpP][-+]\d+)?)|'
    r'(?P<op><<=|>>=|->|\+\+|--|<<|>>|<=|>=|==|!=|&&|\|\||[-+*/%&|^]=|'
    r'[-+*/%&|^~!=<>?:,.;()\[\]{}#]))')
ASSIGN_OPS = {'=', '+=', '-=', '*=', '/=', '%=', '&=', '|=', '^=', '<<=', '>>='}
LOGICAL_OPS = {'==', '!=', '<', '>', '<=', '>=', '&&', '||', '!'}
ARITH_OPS = {'%', '&', '|', '^', '<<', '>>', '+', '-', '*', '/'}

FINDINGS = []
INT_TYPEDEFS = set()


def report(path, line, rule, msg):
    FINDINGS.append((path, line, rule, msg))


# --------------------------------------------------------------------------
# Lexing: blank comments and literal contents, keep layout and line numbers.
# --------------------------------------------------------------------------
class Source:
    def __init__(self, path, text):
        self.path = path
        self.text = text
        self.code, self.comments = self._strip(text)
        self.lines = self.code.split('\n')
        self.raw_lines = text.split('\n')
        self.line_start = [0]
        for ln in self.lines:
            self.line_start.append(self.line_start[-1] + len(ln) + 1)
        self.comment_lines = set()
        for c in self.comments:
            self.comment_lines.update(range(c['start'], c['end'] + 1))
        self.pp_lines = self._pp_lines()
        # code with preprocessor lines blanked: for structural analysis
        out = []
        for i, ln in enumerate(self.lines, 1):
            out.append(' ' * len(ln) if i in self.pp_lines else ln)
        self.scode = '\n'.join(out)

    @staticmethod
    def _strip(text):
        out = []
        comments = []
        i = 0
        n = len(text)
        line = 1
        while i < n:
            ch = text[i]
            if text.startswith('/*', i):
                j = text.find('*/', i + 2)
                j = n if j < 0 else j + 2
                body = text[i:j]
                bol = text.rfind('\n', 0, i) + 1
                standalone = text[bol:i].strip() == ''
                comments.append({'start': line, 'end': line + body.count('\n'),
                                 'text': body, 'standalone': standalone})
                out.append(re.sub(r'[^\n]', ' ', body))
                line += body.count('\n')
                i = j
            elif text.startswith('//', i):
                j = text.find('\n', i)
                j = n if j < 0 else j
                bol = text.rfind('\n', 0, i) + 1
                comments.append({'start': line, 'end': line, 'text': text[i:j],
                                 'standalone': text[bol:i].strip() == ''})
                out.append(' ' * (j - i))
                i = j
            elif ch in '"\'':
                j = i + 1
                while j < n and text[j] != ch and text[j] != '\n':
                    j += 2 if text[j] == '\\' else 1
                j = min(j + 1, n)
                out.append(ch + ' ' * (j - i - 2) + (text[j - 1] if j - i >= 2 else ''))
                i = j
            else:
                if ch == '\n':
                    line += 1
                out.append(ch)
                i += 1
        return ''.join(out), comments

    def _pp_lines(self):
        pp = set()
        cont = False
        for i, ln in enumerate(self.lines, 1):
            if cont or ln.lstrip().startswith('#'):
                pp.add(i)
                cont = self.raw_lines[i - 1].rstrip().endswith('\\')
            else:
                cont = False
        return pp

    def line_of(self, pos):
        lo, hi = 0, len(self.line_start) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if self.line_start[mid] <= pos:
                lo = mid
            else:
                hi = mid - 1
        return lo + 1

    def has_comment(self, first, last=None):
        last = first if last is None else last
        return any(l in self.comment_lines for l in range(first, last + 1))


def match_paren(code, pos):
    """pos is at '(' or '{' or '['; return index of the matching closer."""
    pairs = {'(': ')', '{': '}', '[': ']'}
    opener = code[pos]
    closer = pairs[opener]
    depth = 0
    for k in range(pos, len(code)):
        c = code[k]
        if c == opener:
            depth += 1
        elif c == closer:
            depth -= 1
            if depth == 0:
                return k
    return len(code) - 1


def tokens(text):
    out = []
    pos = 0
    while pos < len(text):
        m = TOKEN_RE.match(text, pos)
        if not m or m.end() == pos:
            pos += 1
            continue
        pos = m.end()
        if m.group('id'):
            out.append(('id', m.group('id')))
        elif m.group('num'):
            out.append(('num', m.group('num')))
        elif m.group('op'):
            out.append(('op', m.group('op')))
    return out


def top_level(toks):
    """Tokens at paren depth 0 of a token list."""
    depth = 0
    out = []
    for t in toks:
        if t[1] in '([':
            depth += 1
            continue
        if t[1] in ')]':
            depth -= 1
            continue
        if depth == 0:
            out.append(t)
    return out


def binary_ops(toks):
    """Operators used in binary position (previous token is an operand)."""
    ops = []
    prev = None
    for t in toks:
        if t[0] == 'op' and prev is not None and (
                prev[0] in ('id', 'num') or prev[1] in (')', ']')):
            ops.append(t[1])
        prev = t
    return ops


# --------------------------------------------------------------------------
# Structure: blocks, functions, statements, declarations.
# --------------------------------------------------------------------------
def find_blocks(src):
    """Every {...} block with its header text and kind."""
    code = src.scode
    blocks = []
    stack = []
    last_boundary = 0
    for k, c in enumerate(code):
        if c == '{':
            header = code[last_boundary:k]
            stack.append((k, header, len(stack)))
            last_boundary = k + 1
        elif c == '}':
            if stack:
                start, header, depth = stack.pop()
                blocks.append({'open': start, 'close': k, 'header': header,
                               'depth': depth})
            last_boundary = k + 1
        elif c == ';':
            last_boundary = k + 1
    for b in blocks:
        h = b['header'].strip()
        hs = re.sub(r'\s+', ' ', h)
        if b['depth'] == 0 and re.search(r'\)\s*$', h) and '=' not in h and \
                not re.match(r'(typedef|struct|enum|union)\b', hs):
            b['kind'] = 'function'
        elif re.search(r'\bswitch\s*\(', hs):
            b['kind'] = 'switch'
        elif re.match(r'(typedef\s+)?enum\b', hs):
            b['kind'] = 'enum'
        elif re.match(r'(typedef\s+)?(struct|union)\b', hs) or \
                re.search(r'\b(struct|union)\s*\w*\s*$', hs):
            b['kind'] = 'record'
        elif re.search(r'(=|,)\s*$', hs) or '=' in hs.split('(')[0]:
            b['kind'] = 'initializer'
        elif hs == '':
            b['kind'] = 'plain'
        elif re.search(r'\bdo\s*$', hs):
            b['kind'] = 'do'
        else:
            b['kind'] = 'control'
        b['open_line'] = src.line_of(b['open'])
        b['close_line'] = src.line_of(b['close'])
    blocks.sort(key=lambda b: b['open'])
    return blocks


def parse_function(src, block):
    h = block['header']
    m = re.search(r'([A-Za-z_]\w*)\s*\(([^()]*(?:\([^()]*\)[^()]*)*)\)\s*$', h, re.S)
    if not m:
        return None
    name = m.group(1)
    ret = h[:m.start(1)].strip()
    ret = re.sub(r'\s+', ' ', ret)
    params = []
    for p in m.group(2).split(','):
        p = p.strip()
        if not p or p == 'void':
            continue
        pm = re.search(r'([A-Za-z_]\w*)\s*(\[[^\]]*\]\s*)*$', p)
        if pm and pm.group(1) not in KEYWORDS:
            params.append((pm.group(1), p[:pm.start(1)].strip()))
    first = len(h) - len(h.lstrip())
    start_pos = block['open'] - len(h) + first
    return {'name': name, 'ret': ret, 'params': params,
            'start_line': src.line_of(start_pos), 'block': block}


def statements(src, code):
    """Chunks of code between ; { } at paren depth 0: (text, start_pos, end_pos)."""
    out = []
    depth = 0
    begin = 0
    for k, c in enumerate(code):
        if c in '([':
            depth += 1
        elif c in ')]':
            depth = max(0, depth - 1)
        elif depth == 0 and c in ';{}':
            out.append((code[begin:k], begin, k, c))
            begin = k + 1
    return out


def blank_for_headers(code):
    out = list(code)
    for m in re.finditer(r'\bfor\s*\(', code):
        p = m.end() - 1
        q = match_paren(code, p)
        for k in range(p + 1, q):
            if out[k] != '\n':
                out[k] = ' '
    return ''.join(out)


def is_int_type(type_text):
    words = set(re.findall(r'\w+', type_text))
    if '*' in type_text:
        return False
    return bool(words & INT_BASE_TYPES) or bool(words & INT_TYPEDEFS) or \
        any(re.fullmatch(r'u?int(8|16|32|64)_t', w) for w in words)


def declarations(src, blocks):
    """Yield variable declarations: dict(type, names, start, end, in_record)."""
    record_spans = [(b['open'], b['close']) for b in blocks if b['kind'] == 'record']
    enum_spans = [(b['open'], b['close']) for b in blocks if b['kind'] == 'enum']
    decls = []
    for text, begin, end, term in statements(src, src.scode):
        if term != ';':
            continue
        body = text.strip()
        if not body or body.startswith(('typedef', 'return', 'goto', 'case',
                                        'else', 'do')):
            continue
        if any(a < begin < b for a, b in enum_spans):
            continue
        m = DECL_RE.match(body)
        if not m:
            continue
        rest = m.group('decls')
        # split declarators at top-level commas
        parts = []
        depth = 0
        cur = ''
        for c in rest:
            if c in '([{':
                depth += 1
            elif c in ')]}':
                depth -= 1
            if c == ',' and depth == 0:
                parts.append(cur)
                cur = ''
            else:
                cur += c
        parts.append(cur)
        names = []
        for part in parts:
            lhs = part.split('=')[0]
            dm = DECLARATOR_RE.match(lhs)
            if not dm:
                break
            after = lhs[dm.end():].strip()
            if after.startswith('('):
                names = []  # prototype
                break
            names.append(dm.group(1))
        if not names or any(n in KEYWORDS for n in names):
            continue
        type_text = m.group('type')
        if re.fullmatch(r'(static|const|extern|volatile|\s)*', type_text):
            continue
        first = begin + (len(text) - len(text.lstrip()))
        decls.append({'type': type_text, 'names': names,
                      'start': src.line_of(first), 'end': src.line_of(end),
                      'record': any(a < begin < b for a, b in record_spans),
                      'pointer': '*' in rest.split('=')[0]})
    return decls


# --------------------------------------------------------------------------
# Rules
# --------------------------------------------------------------------------
def check_file_prolog(src):
    path = src.path
    stripped = src.text.lstrip()
    if not stripped.startswith('/*'):
        report(path, 1, 'file-prolog', 'file does not open with a prolog comment')
        return
    prolog = src.comments[0]['text']
    fields = FILE_FIELDS_H if path.endswith('.h') else FILE_FIELDS_C
    missing = [f for f in fields if f not in prolog.upper()]
    if missing:
        report(path, 1, 'file-prolog-fields',
               'prolog lacks: ' + ', '.join(missing) +
               ' (docs/templates/%s)' % ('header_prolog.h' if path.endswith('.h')
                                         else 'file_prolog.c'))
    m = re.search(r'FILE NAME:\s*(\S+)', prolog)
    if m and m.group(1) != os.path.basename(path):
        report(path, 1, 'file-prolog-fields',
               'FILE NAME says %s' % m.group(1))


def check_functions(src, funcs):
    by_end = {}
    for c in src.comments:
        if c['standalone']:
            by_end[c['end']] = c
    for idx, f in enumerate(funcs):
        line = f['start_line']
        if f['name'] == 'main' and idx > 0:
            report(src.path, line, 'main-first', 'main() is not the first function')
        c = by_end.get(line - 1)
        if c is None:
            report(src.path, line, 'function-prolog',
                   '%s has no prolog comment directly above it' % f['name'])
            continue
        text = c['text']
        if not re.match(r'/\*{20,}', text):
            report(src.path, c['start'], 'function-separator',
                   'prolog of %s does not open with a row of asterisks' % f['name'])
        if not (re.search(r'\.agc\b', text) or re.search(r'\bpp?\.\s*\d', text)
                or re.search(r'AGC:\s*none', text, re.I)):
            report(src.path, c['start'], 'function-prolog-agc',
                   'prolog of %s cites no AGC page/file (or "AGC: none")' % f['name'])
        missing = [p for p, _ in f['params']
                   if not re.search(r'\b' + re.escape(p) + r'\b', text)]
        if missing:
            report(src.path, c['start'], 'function-prolog-args',
                   'prolog of %s does not describe: %s' % (f['name'], ', '.join(missing)))
        ret = re.sub(r'\b(static|inline|extern)\b', '', f['ret']).strip()
        if ret != 'void' and not re.search(r'\breturn', text, re.I):
            report(src.path, c['start'], 'function-prolog-returns',
                   'prolog of %s does not describe the return value' % f['name'])


def check_return_sizeof(src):
    code = src.scode
    for m in re.finditer(r'\breturn\b', code):
        k = m.end()
        while k < len(code) and code[k] in ' \t\n':
            k += 1
        if k >= len(code) or code[k] == ';':
            continue
        ok = False
        if code[k] == '(':
            q = match_paren(code, k)
            rest = code[q + 1:q + 40].lstrip()
            ok = rest.startswith(';')
        if not ok:
            report(src.path, src.line_of(m.start()), 'return-parens',
                   'write return (expression);')
    for m in re.finditer(r'\bsizeof\b\s*(?!\()', code):
        report(src.path, src.line_of(m.start()), 'sizeof-parens',
               'write sizeof (object)')


def int_names(decls, funcs):
    names = set()
    for d in decls:
        if is_int_type(d['type']) and not d['pointer']:
            names.update(d['names'])
    for f in funcs:
        for p, t in f['params']:
            if is_int_type(t):
                names.add(p)
    return names


def numeric_condition(toks, ints):
    top = top_level(toks)
    if any(t[1] in LOGICAL_OPS for t in top if t[0] == 'op'):
        return False
    if any(t[1] in ASSIGN_OPS for t in top):
        return False
    if any(op in ARITH_OPS for op in binary_ops(top)):
        return True
    return len(toks) == 1 and toks[0][0] == 'id' and toks[0][1] in ints


def check_numeric_tests(src, ints):
    code = src.scode
    for m in re.finditer(r'\b(if|while)\s*\(', code):
        p = m.end() - 1
        q = match_paren(code, p)
        toks = tokens(code[p + 1:q])
        if numeric_condition(toks, ints):
            report(src.path, src.line_of(m.start()), 'numeric-test',
                   'numeric test without explicit comparison (write `!= 0`)')
    for m in re.finditer(r'!(?!=)', code):
        k = m.end()
        while k < len(code) and code[k] in ' \t':
            k += 1
        if k < len(code) and code[k] == '(':
            q = match_paren(code, k)
            if numeric_condition(tokens(code[k + 1:q]), set()):
                report(src.path, src.line_of(m.start()), 'numeric-test',
                       '`!(numeric)`: write `(...) == 0`')
        else:
            im = re.match(r'([A-Za-z_]\w*)\s*(\(|->|\.|\[)?', code[k:])
            if im and not im.group(2) and im.group(1) in ints:
                report(src.path, src.line_of(m.start()), 'numeric-test',
                       '`!%s` on an integer: write `%s == 0`' % (im.group(1), im.group(1)))


def check_statement_lines(src, blocks):
    do_closes = {b['close_line'] for b in blocks if b['kind'] == 'do'}
    paren = 0
    for i, ln in enumerate(src.lines, 1):
        if i in src.pp_lines:
            continue
        semis = 0
        depth = paren
        for c in ln:
            if c == '(':
                depth += 1
            elif c == ')':
                depth -= 1
            elif c == ';' and depth == 0:
                semis += 1
        paren = max(0, depth)
        s = ln.strip()
        if semis > 1:
            report(src.path, i, 'one-statement', '%d statements on one line' % semis)
        m = re.match(r'(if|while|for|switch)\s*\(', s)
        if m:
            q = match_paren(s, m.end() - 1)
            after = s[q + 1:].strip()
            if after == ';' and m.group(1) in ('for', 'while'):
                prev = i - 1
                while prev > 0 and src.lines[prev - 1].strip() == '':
                    prev -= 1
                if not (m.group(1) == 'while' and prev in do_closes):
                    report(src.path, i, 'dummy-body',
                           'empty loop body: put `;` on the next line with a comment')
            elif after and after != '{' and q < len(s) - 1 and semis <= 1:
                report(src.path, i, 'one-statement',
                       'statement on the same line as `%s (...)`' % m.group(1))
        elif re.match(r'else\b', s) and not re.match(r'else\s*(if\b|\{|$)', s):
            report(src.path, i, 'one-statement', 'statement on the same line as `else`')
        elif re.match(r'(case\b[^:]*|default\s*):\s*\S', s) and \
                not re.match(r'(case\b[^:]*|default\s*):\s*\{?\s*$', s):
            report(src.path, i, 'one-statement', 'statement on the same line as a case label')
        elif s == ';':
            if not src.has_comment(i):
                report(src.path, i, 'dummy-body', 'empty body `;` needs a comment')


def check_declarations(src, decls):
    for d in decls:
        names = d['names']
        if len(names) > 1 and not set(names) <= {'i', 'j', 'k'}:
            report(src.path, d['start'], 'one-declaration',
                   'declare %s on separate lines' % ', '.join(names))
        has = src.has_comment(d['start'], d['end'])
        if not has:
            report(src.path, d['start'], 'decl-comment',
                   '%s has no short comment' % ', '.join(names))
        elif set(re.findall(r'\w+', d['type'])) & SCALED_TYPES and not d['pointer']:
            text = ' '.join(c['text'] for c in src.comments
                            if c['start'] <= d['end'] and c['end'] >= d['start'])
            if not re.search(r'2\^|scale|unscaled|\bB-?\d+\b', text, re.I):
                report(src.path, d['start'], 'scale-comment',
                       '%s: comment gives no scale (2^N unit)' % ', '.join(names))
        for n in names:
            if re.search(r'[A-Z]', n) and not d['record'] and 'const' not in d['type']:
                report(src.path, d['start'], 'variable-case',
                       'variable %s is not lower_case' % n)


def check_defines(src):
    for i in sorted(src.pp_lines):
        m = re.match(r'\s*#\s*define\s+([A-Za-z_]\w*)(\()?(.*)$', src.lines[i - 1])
        if not m:
            continue
        name, func, value = m.group(1), m.group(2), m.group(3).strip()
        if re.search(r'[a-z]', name):
            report(src.path, i, 'constant-case', 'macro %s is not UPPER_CASE' % name)
        if func or not value:
            continue
        last = i
        while src.raw_lines[last - 1].rstrip().endswith('\\'):
            last += 1
        if not src.has_comment(i, last):
            report(src.path, i, 'define-comment', 'constant %s has no comment' % name)
        v = value.rstrip('\\').strip()
        core = v.lstrip('-+~ ')
        toks = tokens(core)
        wrapped = v.startswith('(') and match_paren(v, 0) == len(v) - 1
        if not wrapped and any(op in ARITH_OPS | {'?', ':'} for op in binary_ops(top_level(toks))):
            report(src.path, i, 'define-parens',
                   'value of %s needs enclosing parentheses' % name)


def check_enums(src, blocks):
    for b in blocks:
        if b['kind'] != 'enum':
            continue
        body = src.scode[b['open'] + 1:b['close']]
        for m in re.finditer(r'(?:^|,)\s*([A-Za-z_]\w*)', body):
            if re.search(r'[a-z]', m.group(1)):
                report(src.path, src.line_of(b['open'] + 1 + m.start(1)),
                       'constant-case', 'enum constant %s is not UPPER_CASE' % m.group(1))


def check_numbers(src):
    for i, ln in enumerate(src.lines, 1):
        for m in re.finditer(r'(?<![\w.])(\.\d+|\d+\.(?![\d\w.]))', ln):
            report(src.path, i, 'number-format',
                   '%s: digits on both sides of the point' % m.group(1))
        for m in re.finditer(r'\b0[xX][0-9A-Fa-f]+[uUlL]*\b', ln):
            lit = m.group(0)
            if lit[1] == 'X' or re.search(r'[a-f]', lit[2:].rstrip('uUlL')):
                report(src.path, i, 'number-format', '%s: write 0x with A-F' % lit)
        for m in re.finditer(r'\b(?:0[xX][0-9A-Fa-f]+|\d+)[uU]?(l|ll)[uU]?\b', ln):
            report(src.path, i, 'number-format', '%s: long suffix in upper case' % m.group(0))


def check_qualifiers(src):
    for i, ln in enumerate(src.lines, 1):
        if i in src.pp_lines:
            continue
        for m in re.finditer(r'\b(unsigned|signed|short|long)\b(?!\s+(int|char|long|double|short)\b)', ln):
            report(src.path, i, 'qualifier-base',
                   '`%s` without its base type (write `%s int`)' % (m.group(1), m.group(1)))


def check_side_effects(src):
    code = src.scode
    for m in re.finditer(r'\b(if|while|switch)\s*\(', code):
        p = m.end() - 1
        q = match_paren(code, p)
        toks = tokens(code[p + 1:q])
        if any(t[1] in ASSIGN_OPS or t[1] in ('++', '--') for t in toks if t[0] == 'op'):
            report(src.path, src.line_of(m.start()), 'side-effect-condition',
                   'assignment or ++/-- inside the `%s` test' % m.group(1))
    nofor = blank_for_headers(code)
    for text, begin, end, term in statements(src, nofor):
        toks = tokens(text)
        n = 0
        for k, t in enumerate(toks):
            if t[0] != 'op' or (t[1] not in ASSIGN_OPS and t[1] not in ('++', '--')):
                continue
            if t[1] == '=' and k >= 2 and toks[k - 2][1] == '.' and toks[k - 1][0] == 'id':
                continue  # designated initializer `.field =`
            n += 1
        if n > 1:
            report(src.path, src.line_of(begin + len(text) - len(text.lstrip())),
                   'multiple-side-effects', '%d side effects in one statement' % n)
        q = sum(1 for t in toks if t[1] == '?')
        if q > 1:
            report(src.path, src.line_of(begin + len(text) - len(text.lstrip())),
                   'ternary-comment', 'nested conditional expression')
    for i, ln in enumerate(src.lines, 1):
        if '?' in ln and i not in src.pp_lines and not src.has_comment(i):
            report(src.path, i, 'ternary-comment', 'conditional expression without a comment')


def check_blocks(src, blocks):
    for b in blocks:
        if b['kind'] in ('function', 'control', 'do', 'switch', 'plain') and \
                b['close_line'] - b['open_line'] > 40 and not src.has_comment(b['close_line']):
            report(src.path, b['close_line'], 'long-block-comment',
                   'block of %d lines: comment the closing brace'
                   % (b['close_line'] - b['open_line']))
        if b['kind'] != 'switch':
            continue
        body = src.scode[b['open'] + 1:b['close']]
        depth = 0
        labels = []
        for m in re.finditer(r'[{}]|\bcase\b|\bdefault\s*:', body):
            t = m.group(0)
            if t == '{':
                depth += 1
            elif t == '}':
                depth -= 1
            elif depth == 0:
                labels.append('default' if t.startswith('default') else 'case')
        if 'default' not in labels:
            report(src.path, b['open_line'], 'switch-default', 'switch has no default')
        elif labels[-1] != 'default':
            report(src.path, b['open_line'], 'switch-default', 'default is not the last label')


def check_misc(src):
    if src.path.endswith('.c'):
        for i, ln in enumerate(src.lines, 1):
            if re.search(r'\bextern\b', ln) and i not in src.pp_lines:
                report(src.path, i, 'extern-in-c',
                       'extern in a .c file: declare it in the owning header')
    if os.path.basename(src.path) in STD_HEADERS:
        report(src.path, 1, 'header-name',
               'header shares its name with the standard <%s>' % os.path.basename(src.path))
    seen = {}
    for i, ln in enumerate(src.lines, 1):
        for m in re.finditer(r'\b[A-Za-z_]\w*\b', ln):
            w = m.group(0)
            if w in KEYWORDS:
                continue
            spellings = seen.setdefault(w.lower(), {})
            if w not in spellings:
                spellings[w] = i
    for low, spellings in seen.items():
        # NAMING.md rule 7: AGC_JOB_X (enum/macro) and agc_job_x (function)
        # deliberately share a stem; only mixed-case variants are reported.
        mixed = [w for w in spellings if not (w.isupper() or w.islower())]
        if len(spellings) > 1 and mixed:
            items = sorted(spellings.items(), key=lambda kv: kv[1])
            report(src.path, items[1][1], 'case-unique',
                   'names differ only by case: ' + ', '.join(k for k, _ in items))


def collect_typedefs(paths):
    for path in paths:
        if not os.path.isfile(path):
            continue
        with open(path, encoding='utf-8', errors='replace') as fh:
            for m in re.finditer(r'typedef\s+([\w\s]+?)\s+(\w+)\s*;', fh.read()):
                if is_int_type(m.group(1)):
                    INT_TYPEDEFS.add(m.group(2))


def check_file(path):
    try:
        with open(path, encoding='utf-8', errors='replace') as fh:
            src = Source(path, fh.read())
    except FileNotFoundError:
        return  # moved or deleted while the tree was being walked
    blocks = find_blocks(src)
    funcs = [f for f in (parse_function(src, b) for b in blocks
                         if b['kind'] == 'function') if f]
    decls = declarations(src, blocks)
    check_file_prolog(src)
    check_functions(src, funcs)
    check_return_sizeof(src)
    check_numeric_tests(src, int_names(decls, funcs))
    check_statement_lines(src, blocks)
    check_declarations(src, decls)
    check_defines(src)
    check_enums(src, blocks)
    check_numbers(src)
    check_qualifiers(src)
    check_side_effects(src)
    check_blocks(src, blocks)
    check_misc(src)


def gather(args):
    files = []
    for a in args:
        if os.path.isfile(a):
            files.append(a)
            continue
        for dp, dirs, fs in os.walk(a):
            dirs.sort()
            for f in sorted(fs):
                if f.endswith(('.c', '.h')):
                    files.append(os.path.join(dp, f))
    return files


def main(argv):
    summary = '--summary' in argv
    exit_zero = '--exit-zero' in argv
    wanted = None
    args = []
    it = iter(argv[1:])
    for a in it:
        if a == '--rule':
            wanted = set(next(it, '').split(','))
        elif a.startswith('--rule='):
            wanted = set(a.split('=', 1)[1].split(','))
        elif a == '--list-rules':
            for rid, (sec, desc) in RULES.items():
                print('%-24s %-10s %s' % (rid, sec, desc))
            return 0
        elif a.startswith('--'):
            continue
        else:
            args.append(a)
    if not args:
        print(__doc__)
        return 2
    if wanted and not wanted <= set(RULES):
        print('unknown rule id(s): ' + ', '.join(sorted(wanted - set(RULES))))
        return 2
    files = gather(args)
    collect_typedefs(files)
    for path in files:
        check_file(path)
    shown = [f for f in FINDINGS if wanted is None or f[2] in wanted]
    if summary:
        counts = {}
        for f in shown:
            counts[f[2]] = counts.get(f[2], 0) + 1
        for rid in RULES:
            if rid in counts:
                print('%-24s %-10s %6d' % (rid, RULES[rid][0], counts[rid]))
    else:
        for path, line, rid, msg in shown:
            print('%s:%d: [%s %s] %s' % (path, line, rid, RULES[rid][0], msg))
    print('%d style findings in %d files' % (len(shown), len(files)))
    return 0 if exit_zero or not shown else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
