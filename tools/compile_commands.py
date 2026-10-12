"""Write compile_commands.json for editors (clangd, VS Code C/C++).

Usage: python3 -I tools/compile_commands.py "<flags>" file.c ...
Run through `make compile_commands` so the flags match the Makefile.
"""
import json
import os
import sys


def main():
    flags, files = sys.argv[1], sys.argv[2:]
    cwd = os.getcwd()
    entries = [{"directory": cwd, "file": f, "command": f"{flags} -c {f}"} for f in files]
    with open("compile_commands.json", "w") as fh:
        json.dump(entries, fh, indent=1)
    print(f"compile_commands.json: {len(entries)} entries")


if __name__ == "__main__":
    main()
