#!/usr/bin/env python3
# check_binsource.py -- compare a yaYUL rope (.bin) with a virtualagc .binsource
# transcription, bank by bank.  yaYUL stores fixed-fixed banks 02,03 first in
# the .bin, then 00,01,04..43; each word is big-endian 16 bits, value << 1.
# Usage: python3 -I check_binsource.py X.binsource ropes/X.bin
import re, sys, struct
src, binf = sys.argv[1], sys.argv[2]
banks = {}; cur = None
for line in open(src, errors='replace'):
    line = line.split(';')[0]
    m = re.search(r'BANK\s*=\s*([0-7]+)', line, re.I)
    if m: cur = int(m.group(1), 8); banks.setdefault(cur, []); continue
    if cur is None: continue
    for tok in line.split():
        if re.fullmatch(r'[0-7]{5}', tok): banks[cur].append(int(tok, 8))
        elif tok == '@': banks[cur].append(None)
data = open(binf, 'rb').read()
total = 0; bad = 0
for b, ws in sorted(banks.items()):
    fb = {0: 2, 1: 3, 2: 0, 3: 1}.get(b, b)
    off = fb * 0o2000 * 2
    for i, w in enumerate(ws):
        if w is None: continue
        bw = struct.unpack('>H', data[off+2*i:off+2*i+2])[0] >> 1
        total += 1
        if bw != w:
            bad += 1
            if bad <= 3: print('  bank %02o word %04o: binsource %05o bin %05o' % (b, i, w, bw))
print(src.split('/')[-1], 'banks', len(banks), 'words compared', total, 'mismatches', bad)
