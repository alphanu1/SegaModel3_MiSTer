#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# ppc_pdec.sv's tables from the decoder's dump (run.sh): one word per opcode, or per
# extended opcode for opcodes 19 and 31; a_used and rc may also read rA = 0 and Rc
import collections, sys
d=collections.defaultdict(dict)
for l in open('dump.txt'):
    o,x,c,v=map(int,l.split()); d[(o,x)][c]=v
def enc(cs):
    # cs: corner -> value; corner bit0 = ra0, bit1 = rcb
    base=cs[0]
    for c in range(4):
        diff=cs[c]^base
        assert diff & ~((1<<0)|(1<<6))==0, (cs,)
    ua={ (c&1): (cs[c]>>0)&1 for c in range(4)}
    assert all(((cs[c]>>0)&1)==ua[c&1] for c in range(4))
    rc={ (c>>1): (cs[c]>>6)&1 for c in range(4)}
    assert all(((cs[c]>>6)&1)==rc[c>>1] for c in range(4))
    v=base & ~((1<<0)|(1<<6))
    # ua: ra0=1 -> ua[1], ra0=0 -> ua[0]
    if ua[1] and ua[0]: v|=1<<0
    elif ua[0] and not ua[1]: v|=1<<27
    elif ua[1] and not ua[0]: raise Exception('ua only with rA=0')
    if rc[0] and rc[1]: v|=1<<6
    elif rc[1] and not rc[0]: v|=1<<28
    elif rc[0] and not rc[1]: raise Exception('rc without Rc')
    return v
W=29
def case(name, sel, keys, default=None):
    vals={k:enc(d[k]) for k in keys}
    cnt=collections.Counter(vals.values()); dflt=cnt.most_common(1)[0][0] if default is None else default
    lines=[f"      case ({sel})"]
    groups=collections.defaultdict(list)
    for k,v in vals.items():
        if v!=dflt: groups[v].append(k[1] if sel=='x' else k[0])
    for v,ks in sorted(groups.items(), key=lambda t: min(t[1])):
        ks=sorted(ks); w=10 if sel=='x' else 6
        lines.append("        " + ", ".join(f"{w}'d{k}" for k in ks) + f": {name} = {W}'h{v:08X};")
    lines.append(f"        default: {name} = {W}'h{dflt:08X};")
    lines.append("      endcase")
    return "\n".join(lines)
out=[]
out.append(case('vo','op',[(o,0) for o in range(64) if o not in (19,31)]))
out.append(case('v19','x',[(19,x) for x in range(1024)]))
out.append(case('v31','x',[(31,x) for x in range(1024)]))
print("\n".join(out))
