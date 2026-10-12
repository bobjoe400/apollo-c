"""Delete the `return;` that ends a void function's body (rule 7 / guide
5.6: a return with nothing after it is dead).  Its trailing comment (an
AGC exit such as ENDOFJOB or RESUME) joins the previous statement's
trailing comment, or stays as a standalone comment when the previous line
closes a block.  Used after tools/cocci/always_ok_void.cocci.

Usage: python3 -I tools/drop_final_return.py FILE...
"""
import re
import sys

import clang.cindex as ci

RET_RE = re.compile(r'^(\s*)return\s*;\s*(?:/\*\s*(.*?)\s*\*/)?\s*$')
STMT_RE = re.compile(r'^\s+\S.*;(\s*/\*.*\*/)?\s*$')


def final_returns(path):
    tu = ci.Index.create().parse(path, args=['-std=c11', '-Iinclude'])
    out = []
    for c in tu.cursor.get_children():
        if c.kind != ci.CursorKind.FUNCTION_DECL or not c.is_definition():
            continue
        if c.location.file is None or c.location.file.name != path or c.result_type.spelling != 'void':
            continue
        body = [k for k in c.get_children() if k.kind == ci.CursorKind.COMPOUND_STMT]
        if not body:
            continue
        stmts = list(body[0].get_children())
        if stmts and stmts[-1].kind == ci.CursorKind.RETURN_STMT and not list(stmts[-1].get_children()):
            out.append(stmts[-1].extent.start.line)
    return out


def drop(path):
    lines = open(path).read().split('\n')
    for ln in sorted(final_returns(path), reverse=True):
        i = ln - 1
        m = RET_RE.match(lines[i])
        if not m:
            continue
        indent, text = m.group(1), m.group(2)
        del lines[i]
        j = i - 1
        while j >= 0 and not lines[j].strip():
            del lines[j]
            j -= 1
        if not text:
            continue
        cm = re.search(r'\s*/\*\s*(.*?)\s*\*/\s*$', lines[j])
        if STMT_RE.match(lines[j]) and cm:
            lines[j] = lines[j][:cm.start()] + ' /* ' + cm.group(1) + '; ' + text + ' */'
        elif STMT_RE.match(lines[j]):
            lines[j] = lines[j].rstrip() + ' /* ' + text + ' */'
        else:
            lines[j + 1:j + 1] = ['', indent + '/* ' + text + ' */']
    open(path, 'w').write('\n'.join(lines))


if __name__ == '__main__':
    for p in sys.argv[1:]:
        drop(p)
