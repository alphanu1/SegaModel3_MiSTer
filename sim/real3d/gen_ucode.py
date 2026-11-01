#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Sega Model 3 core for MiSTer FPGA
# Copyright (C) 2026 alphanu1
#
# Micro-code for r3d_vpu: the Real3D geometry kernels, each a list of
# operations in MAME's evaluation order, list-scheduled against the engine's
# fixed latencies (no interlocks in the RTL). Writes
#   rtl/real3d/r3d_ucode.sv      the ROM (M10K)
#   rtl/real3d/r3d_ucode_pkg.sv  kernel entry points, register and broadcast map
#   sim/real3d/r3d_ucode.h       the same constants for the C++ benches
#
# Operation: r = (a * b) + c, then an optional post-op. a is a lane register
# or a side-memory word (the T&L inputs, the projection queue); b a lane
# register (port B) or a broadcast entry (port 0); c none (a pure product), a
# lane register (port B) or a broadcast entry (port 1), optionally negated.
# The constants are broadcast entries too: bank 1, entries BC_K.., loaded by
# the controller at the start of a frame. Port B carries one register per instruction.
# DIV: r = b / a, correctly rounded (the reciprocal is DIV with b = 1.0), on
# the shared divide unit, one lane a cycle.
#
# Results go to a lane register, a broadcast entry (lane j into bank j), or a
# side memory (the vertex store from T&L, the projection output buffer). The
# matrix stack is the upper half of the lane register file (the block RAM
# holds 256 words; the registers use 128): an operand flagged 'stk' reads
# word {1, sp_a, ra[1:0]}, a result flagged 'stk' writes {1, sp_d, dst[1:0]};
# sp_a and sp_d are given at kernel start. The stack top is a stack slot: a
# pushed level without a matrix of its own shares the slot below, so PUSH and
# POP need no kernel.
#
# Post-ops (r3d_vpu, by SM_LIGHT): 1 the sunFactor clamp (MAME max(r, 0);
# Supermodel clamp(r, -1, 1) floored at 0), 2 the intensity limit (MAME none;
# Supermodel min(r, 1)), 3 the shade clamp (MAME clamp(r, 0, 255); Supermodel
# none). A word flagged 'tr' reads broadcast entry {bank, idx} as
# {idx[1:0], idx[4:2], bank} when SM_LIGHT is set: the T&L normal transform
# reads MAME's transpose of the model matrix, Supermodel's the matrix itself.
#
# A run of empty cycles is one word, valid = 0, with the run length less one
# in the ra field: the sequencer holds the program counter for that many
# cycles. The last word of a kernel is valid = 0 with rb = 1.
#
# Timing (r3d_vpu.sv; the register files are synchronous): a MAC issued at t
# writes at the end of t+12 and is readable from t+13. DIV issued at t writes
# lane k at the end of t+15+k, readable from t+16+k. A write must land after
# every earlier read of the old value has issued.

import os
import sys

MAC_W = 12         # write cycle after issue
R3 = '--3r' in sys.argv   # (set True for the set-up output ROM)
# a third lane-register read (port C, r3d_vpu SETUP_3R): c from any register
DIV_W = 15         # lane 0 write cycle after issue; lane k at +k

K = {'zero': 0, 'one': 1, 'half': 2, 'k255': 3, 'k99999': 4, 'mone': 5, 'two': 6, 'wplane': 7}

# ---- lane register file (128 words) ------------------------------------------
CO, PR, TR = 8, 12, 16
MT = 0                                 # node matrix product accumulators (4)
GA, GB, GC, GO = 20, 21, 22, 23        # scalar set-up kernels, one value a lane
CT = GO                                # culling result
TLB, TL_W = 24, 10                     # T&L temporaries: 4 rows x (P0-3, N0-2, F/Dp, D1, D2)
PJB, PJ_W = 64, 6                      # projection temporaries: 4 rows x (OOW, x, y, z, u, v)
LB = 88                                # clip lerp: prev/cur/result, set 0 at LB, set 1 at LB + 3
CS = 94                                # clip terms: lanes 0-2 (pw, cw, E) at CS, (pa, ca, F) at CS + 1
CX, CZ = 96, 97                        # clip: X / Y / N (lanes 0-2), Z = wdiv (lane 0)
PA, PB, PC = 98, 99, 100               # culling plane terms
CF = 124                               # culling: full coordinate matrix
# Supermodel's frustum (SMFR): constants loaded at frame start (ONE all 1.0,
# SGN {1 1 -1 -1}, F0 {0 1 1 0}), the viewport words per viewport (SC {jo jo
# io io}, SB {cv cv cw cw}), temporaries after them
SONE, SSGN, SF0, SC, SB, SD, SN, SLR, SQ, SV, SP = range(101, 112)
# side memories: T&L inputs (row, comp: x y z at 0-2, nx ny nz at 4-6, F at
# 7); the vertex store (row, comp: x y z w shade shade' at 0-5); the
# projection queue and output (row, comp: x y z w u v shade shade' at 0-7);
# shade' is the inverted copy's (double-sided polygons)
TL_NV, PJ_NV = 4, 4
# broadcast map: entry index (all four banks), bank 0 scalars
BC_MB, BC_TOP, BC_TR, BC_VP = 0, 4, 8, 12
BC_L0, BC_L1, BC_L2, BC_IP, BC_AMB, BC_VW, BC_VX, BC_VH, BC_VY, BC_T = range(16, 26)
BC_CU = 26
BC_CL = 27                             # clip: X, Y, N (banks 0-2)
BC_ZS, BC_ZO = 28, 29                  # depth map: z = (z/w * 0.5 + 0.5) * ZS + ZO (MAME 99999, 1; Supermodel 1, -0)
BC_L2N = 30                            # -L2, for the inverted copy's sunFactor
BC_K = 16                              # constants, bank 1 entries 16-23 (K order)

class Op:
    def __init__(self, a, b, c=None, negc=False, post=0, dst=None, dstb=None, side=None, div=False,
                 absr=False, lanes=0xF, dstk=None, tr=False, sdup=False):
        self.a, self.b, self.c, self.negc, self.post = a, b, c, negc, post
        self.sdup = sdup   # also write the side memory at dst's address (r3d_vpu writes both)
        self.dst, self.dstb, self.side, self.div, self.absr, self.lanes = dst, dstb, side, div, absr, lanes
        self.dstk = dstk
        self.tr = tr

def rf(r): return ('rf', r)
def sa(r): return ('sa', r)
def stk(r): return ('stk', r)
def bc(bank, idx): return ('bc', bank, idx)
def k(name): return ('bc', 1, BC_K + K[name])

def mac(a, b, c=None, **kw): return Op(a, b, c, **kw)

def reads(op):
    r = []
    if op.a[0] in ('rf', 'stk'): r.append(op.a)
    for x in (op.b, op.c):
        if x is None: continue
        if x[0] == 'rf': r.append(('rf', x[1]))
        if x[0] == 'bc': r.append(('bc', x[2]))
    return r

def writes(op):
    w = []
    if op.dst is not None: w.append(('rf', op.dst))
    if op.dstb is not None: w.append(('bc', op.dstb))
    if op.dstk is not None: w.append(('stk', op.dstk))
    return w

def top_lane(op):
    return max(l for l in range(4) if (op.lanes >> l) & 1)

def ready(op, t):
    return t + (DIV_W + 1 + top_lane(op) if op.div else MAC_W + 1)

def schedule(ops):
    n = len(ops)
    last_w, readers = {}, {}
    raw = [[] for _ in range(n)]; war = [[] for _ in range(n)]; waw = [[] for _ in range(n)]
    for i, op in enumerate(ops):
        for loc in reads(op):
            if loc in last_w: raw[i].append(last_w[loc])
        for loc in writes(op):
            if loc in last_w: waw[i].append(last_w[loc])
            war[i].extend(readers.get(loc, []))
        for loc in reads(op): readers.setdefault(loc, []).append(i)
        for loc in writes(op):
            last_w[loc] = i
            readers[loc] = []
    succ = [[] for _ in range(n)]
    for i in range(n):
        for j in raw[i]: succ[j].append(i)
    height = [0] * n
    for i in reversed(range(n)):
        lat = ready(ops[i], 0)
        height[i] = lat + max([height[s] for s in succ[i]], default=0)
    t_issue = [None] * n
    wbusy = set()
    done = 0
    t = 0
    def wcycles(i, t):
        # a divide holds the write port for all four lane slots
        return [t + DIV_W + kk for kk in range(4)] if ops[i].div else [t + MAC_W]
    while done < n:
        best = None
        for i in range(n):
            if t_issue[i] is not None: continue
            ok = True
            for j in raw[i]:
                if t_issue[j] is None or t < ready(ops[j], t_issue[j]): ok = False; break
            if not ok: continue
            wc = wcycles(i, t)
            for j in war[i]:
                if t_issue[j] is None or min(wc) <= t_issue[j]: ok = False; break
            if not ok: continue
            for j in waw[i]:
                if t_issue[j] is None or min(wc) <= max(wcycles(j, t_issue[j])): ok = False; break
            if not ok: continue
            if any(c in wbusy for c in wc): continue
            if best is None or height[i] > height[best]: best = i
        if best is not None:
            t_issue[best] = t
            for c in wcycles(best, t): wbusy.add(c)
            done += 1
        t += 1
    end = max(max(wcycles(i, t_issue[i])) for i in range(n)) + 1
    return t_issue, end

# ---- word layout ----------------------------------------------------------------
FIELDS = [('valid', 1), ('div', 1), ('absr', 1), ('lanes', 4), ('aside', 1), ('ra', 7), ('rb', 7),
          ('bsel', 1), ('csel', 2), ('negc', 1), ('bc0', 7), ('bc1', 7), ('post', 2),
          ('dst', 7), ('wrf', 1), ('wbc', 1), ('dstb', 5), ('wside', 1), ('astk', 1), ('dstk', 1), ('tr', 1)]
POS = {}
_p = 0
for _n, _w in FIELDS:
    POS[_n] = (_p, _w); _p += _w
WORD_W = _p

def encode(op, end=False, valid=True):
    w = 0
    def put(name, v):
        nonlocal w
        lo, width = POS[name]
        assert 0 <= v < (1 << width), (name, v)
        w |= v << lo
    put('valid', 1 if valid else 0)
    # the kernel's end: an empty word (valid = 0) with rb = 1 (a wait word
    # leaves rb at 0), so the ROM fits three M10K at 60 bits
    if end: put('rb', 1)
    if op is None:
        return w
    put('div', 1 if op.div else 0)
    put('absr', 1 if op.absr else 0)
    put('lanes', op.lanes)
    put('aside', 1 if op.a[0] == 'sa' else 0)
    put('astk', 1 if op.a[0] == 'stk' else 0)
    put('ra', op.a[1])
    rb = 0
    bsel, csel, bc0, bc1 = 0, 0, 0, 0
    if op.b is not None:
        if op.b[0] == 'rf': bsel, rb = 0, op.b[1]
        else: bsel, bc0 = 1, (op.b[1] << 5) | op.b[2]
    if op.c is None: csel = 0
    elif op.c[0] == 'rf':
        if op.b is not None and op.b[0] == 'rf' and op.b[1] != op.c[1]:
            # port C (SETUP_3R) reads c at the bc1 field's address
            assert R3, 'port B carries one register'
            csel, bc1 = 3, op.c[1]
        else:
            csel, rb = 1, op.c[1]
    else: csel, bc1 = 2, (op.c[1] << 5) | op.c[2]
    put('rb', rb); put('bsel', bsel); put('csel', csel); put('negc', 1 if op.negc else 0)
    put('bc0', bc0); put('bc1', bc1); put('post', op.post)
    assert (op.dst is not None) + (op.side is not None) + (op.dstk is not None) <= 1
    # r3d_vpu applies a post-op only to the lane registers and (post 3, MAME's
    # shade clamp) the side memories, not to the broadcast file
    assert not op.post or op.dstb is None
    assert op.post not in (1, 2) or (op.side is None and not op.sdup)
    d = op.dst if op.dst is not None else op.side if op.side is not None else op.dstk if op.dstk is not None else 0
    put('dst', d)
    put('wrf', 1 if (op.dst is not None or op.dstk is not None) else 0)
    put('dstk', 1 if op.dstk is not None else 0)
    put('wbc', 1 if op.dstb is not None else 0)
    put('dstb', op.dstb if op.dstb is not None else 0)
    assert not op.sdup or op.dst is not None
    put('wside', 1 if (op.side is not None or op.sdup) else 0)
    put('tr', 1 if op.tr else 0)
    return w

# ---- kernels -----------------------------------------------------------------

def dot4_col(out_reg, out_bc, a_regs, bcb):
    """out[i] = ((a0*B(i,0) + a1*B(i,1)) + a2*B(i,2)) + a3*B(i,3) for i = 0..3,
    with a_k lane registers (column j of the right-hand matrix) and B(i,k) a
    broadcast entry (row i of the left-hand matrix, bank k). Interleaved by i."""
    ops = []
    for kk in range(4):
        for i in range(4):
            o = out_reg + i if out_reg is not None else LB + i   # transient
            b = bc(kk, bcb + i)
            if kk == 0: ops.append(mac(rf(a_regs[0]), b, None, dst=o))
            elif kk < 3: ops.append(mac(rf(a_regs[kk]), b, rf(o), dst=o))
            else: ops.append(mac(rf(a_regs[3]), b, rf(o), dst=o, dstb=(out_bc + i) if out_bc is not None else None))
    return ops

def k_matmul():
    # S' = M x S: MAME's matrix_multiply(M, S): tmp[i][j] = sum_k M[i][k] S[k][j];
    # S column j in lane j from stack slot sp_a, S' into slot sp_d and the
    # broadcast copy of the top
    ops = []
    for kk in range(4):
        for i in range(4):
            o = MT + i
            b = bc(kk, BC_MB + i)
            if kk == 0: ops.append(mac(stk(0), b, None, dst=o))
            elif kk < 3: ops.append(mac(stk(kk), b, rf(o), dst=o))
            else: ops.append(mac(stk(3), b, rf(o), dstk=i, dstb=BC_TOP + i))
    return ops

def k_model():
    # transform = top x coord; vp = transform x projection (lane j: column j)
    ops = dot4_col(TR, BC_TR, [CO + q for q in range(4)], BC_TOP)
    ops += dot4_col(None, BC_VP, [PR + q for q in range(4)], BC_TR)
    return ops

def k_tl(rows=TL_NV):
    # inputs from the T&L side memory (x y z at 0-2, nx ny nz at 4-6, F at 7
    # of a row of 8), results x y z w shade shade' into the vertex store (row
    # of 8, 0-5). Lighting: N = n x T (tr: Supermodel's orientation), then
    # Dp = (N0 L0 + F) + N1 L1, sf = N2 L2 + Dp, sf' = N2 (-L2) + (-Dp) (the
    # inverted normal, exact), each clamped (post 1), I = sf Ip + amb (post
    # 2), shade = I * 255 (post 3). F is -0, or a fixed-shade vertex's shade
    # with its normal staged as zero.
    ops = []
    rb = [TLB + v * TL_W for v in range(rows)]
    P, N, F, D1, D2 = 0, 4, 7, 8, 9
    stages = []
    for step in range(4):
        for v, b in enumerate(rb):
            si = v * 8
            for j in range(4):
                o = b + P + j
                if step == 0: stages.append(mac(sa(si + 0), bc(j, BC_VP + 0), None, dst=o))
                elif step == 1: stages.append(mac(sa(si + 1), bc(j, BC_VP + 1), rf(o), dst=o))
                elif step == 2: stages.append(mac(sa(si + 2), bc(j, BC_VP + 2), rf(o), dst=o))
                else: stages.append(mac(rf(o), k('one'), bc(j, BC_VP + 3), side=si + j))
            if step < 3:
                for i in range(3):
                    o = b + N + i
                    stages.append(mac(sa(si + 4 + step), bc(step, BC_TR + i), None if step == 0 else rf(o), dst=o, tr=True))
            if step == 0:
                stages.append(mac(sa(si + 7), k('one'), None, dst=b + F))
    ops += stages
    for b in rb:
        ops.append(mac(rf(b + N + 0), bc(0, BC_L0), rf(b + F), dst=b + F))
    for b in rb:
        ops.append(mac(rf(b + N + 1), bc(0, BC_L1), rf(b + F), dst=b + F))
    for b in rb:
        ops.append(mac(rf(b + N + 2), bc(0, BC_L2), rf(b + F), dst=b + D1, post=1))
        ops.append(mac(rf(b + N + 2), bc(0, BC_L2N), rf(b + F), negc=True, dst=b + D2, post=1))
    for b in rb:
        ops.append(mac(rf(b + D1), bc(0, BC_IP), bc(0, BC_AMB), dst=b + D1, post=2))
        ops.append(mac(rf(b + D2), bc(0, BC_IP), bc(0, BC_AMB), dst=b + D2, post=2))
    for v, b in enumerate(rb):
        ops.append(mac(rf(b + D1), k('k255'), None, side=v * 8 + 4, post=3))
        ops.append(mac(rf(b + D2), k('k255'), None, side=v * 8 + 5, post=3))
    return ops

def k_proj(rows=PJ_NV):
    # inputs from the projection queue (x y z w u v shade at 0-6 of a row of
    # 8), results into the projection output buffer: x y at 0-1, 1/w at 2
    # (the rasteriser's depth and perspective plane, R62), u/w v/w at 4-5,
    # shade shade' at 6-7. Window z and w are not sent (R62).
    ops = []
    rb = [PJB + v * PJ_W for v in range(rows)]
    OOW, X, Y, Z, U, V = range(6)
    for v, b in enumerate(rb):
        ops.append(Op(sa(v * 8 + 3), k('one'), None, dst=b + OOW, div=True))       # 1 / w
    for comp, r in ((0, X), (1, Y)):
        for v, b in enumerate(rb):
            ops.append(mac(sa(v * 8 + comp), rf(b + OOW), None, dst=b + r))
    for comp in (4, 5):
        for v, b in enumerate(rb):
            ops.append(mac(sa(v * 8 + comp), rf(b + OOW), None, side=v * 8 + comp))
    for v, b in enumerate(rb):
        ops.append(mac(rf(b + OOW), k('one'), None, side=v * 8 + 2))            # 1 / w out
    for comp in (6, 7):
        for v, b in enumerate(rb):
            ops.append(mac(sa(v * 8 + comp), k('one'), None, side=v * 8 + comp))  # shade, shade' pass through
    for r in (X, Y):
        for b in rb:
            ops.append(mac(rf(b + r), k('half'), k('half'), dst=b + r))
    for v, b in enumerate(rb):
        ops.append(mac(rf(b + X), bc(0, BC_VW), bc(0, BC_VX), side=v * 8 + 0))
        ops.append(mac(rf(b + Y), bc(0, BC_VH), bc(0, BC_VY), side=v * 8 + 1))
    return ops

def k_clip():
    # one crossing of MAME's Sutherland-Hodgman clip: lanes 0-2 X = pw - pa,
    # Y = cw - ca, N = E - F (E - F = X, or W_PLANE - pw on the W plane, where
    # pa = ca = +0); lane 0 Z = X - Y = wdiv; t = |N / Z|; then every
    # component r = (b - a) * t + a (set 0 x y z w, set 1 u v shade)
    ops = [mac(rf(CS), k('one'), rf(CS + 1), negc=True, dst=CX, dstb=BC_CL, lanes=0x7),
           mac(rf(CX), k('one'), bc(1, BC_CL), negc=True, dst=CZ, lanes=0x1),
           Op(rf(CZ), bc(2, BC_CL), None, dstb=BC_T, div=True, absr=True, lanes=0x1)]
    for s in range(2):
        A, B, R = LB + s * 3, LB + s * 3 + 1, LB + s * 3 + 2
        ops.append(mac(rf(B), k('one'), rf(A), negc=True, dst=R))
    for s in range(2):
        A, R = LB + s * 3, LB + s * 3 + 2
        ops.append(mac(rf(R), bc(0, BC_T), rf(A), dst=R))
    return ops

def k_add(neg):  # out = a + c (or a - c)
    return [mac(rf(GA), k('one'), rf(GC), negc=neg, dst=GO)]
def k_mul():     # out = a * b
    return [mac(rf(GA), rf(GB), None, dst=GO)]
def k_div():     # out = b / a, every lane
    return [Op(rf(GA), rf(GB), None, dst=GO, div=True)]

def k_smfr():
    # Supermodel's Mat4::FrustumRZ from viewport words 8-11, the arithmetic
    # of the controller's old SUB, DIV, SUB, DIV sequence with its lane moves
    # done here: lanes read each other's results through the broadcast file
    # (scratch entries BC_MB + 0..2, free while a viewport is set up).
    # l r b t = {0 - jo, 1 - jo, -(1 - io), -(0 - io)} / {cv cv cw cw};
    # P00 P20 P11 P21 = {2, r + l, 2, t + b} / {r - l, r - l, t - b, t - b}
    # into the projection rows (lane j column j): row 0 lane 0, row 1 lane 1,
    # row 2 lanes 0, 1; the rows' constant lanes are loaded at frame start.
    X, Y, Z = BC_MB, BC_MB + 1, BC_MB + 2
    one = rf(SONE)
    return [mac(rf(SF0), k('one'), rf(SC), negc=True, dst=SD),
            mac(rf(SD), rf(SSGN), None, dst=SN),                       # x -1 exactly: the negation
            Op(rf(SB), rf(SN), None, dst=SLR, dstb=X, div=True),
            mac(one, bc(1, X), bc(0, X), negc=True, dst=SQ, dstb=Y, lanes=0x1),   # r - l
            mac(one, bc(1, X), bc(0, X), dst=SQ, lanes=0x2),                       # r + l
            mac(one, bc(3, X), bc(2, X), negc=True, dst=SQ, dstb=Y, lanes=0x4),   # t - b
            mac(one, bc(3, X), bc(2, X), dst=SQ, lanes=0x8),                       # t + b
            mac(one, bc(0, Y), None, dst=SV, lanes=0x3),
            mac(one, bc(2, Y), None, dst=SV, lanes=0xC),
            Op(rf(SV), k('two'), None, dst=SP, dstb=Z, div=True, lanes=0x5),
            Op(rf(SV), rf(SQ), None, dst=SP, dstb=Z, div=True, lanes=0xA),
            mac(rf(SP), k('one'), None, dst=PR + 0, lanes=0x1),
            mac(one, bc(2, Z), None, dst=PR + 1, lanes=0x2),
            mac(one, bc(1, Z), None, dst=PR + 2, lanes=0x1),
            mac(one, bc(3, Z), None, dst=PR + 2, lanes=0x2)]

def k_copyb():
    # stack slot sp_a into the broadcast file (after a POP): x * 1 + (-0) = x
    return [mac(stk(i), k('one'), None, dstb=BC_TOP + i) for i in range(4)]

# node culling (Supermodel), off by default: lane i (0..2) u_i = ((s0 o0i + s1 o1i)
# + s2 o2i) + o3i with s the stack top's row 3 (broadcast) and o the full
# coordinate matrix column i (lane registers CF..CF+3); then lane p (plane)
# t_p = ((u0 A_p) + (u1 B_p)) + (u2 C_p) with the plane terms in PA, PB, PC.
def k_cullu():
    o = BC_TOP + 3
    return [mac(rf(CF + 0), bc(0, o), None, dst=CX),
            mac(rf(CF + 1), bc(1, o), rf(CX), dst=CX),
            mac(rf(CF + 2), bc(2, o), rf(CX), dst=CX),
            mac(rf(CX), k('one'), rf(CF + 3), dstb=BC_CU)]
def k_cullt():
    return [mac(rf(PA), bc(0, BC_CU), None, dst=CT),
            mac(rf(PB), bc(1, BC_CU), rf(CT), dst=CT),
            mac(rf(PC), bc(2, BC_CU), rf(CT), dst=CT)]

# The rasteriser's triangle set-up (m3-notes section 9) as a pool kernel, one
# triangle a lane, for measuring what sharing the pool would cost: the 9
# edge terms, the determinant, 1/det on the divide, then pdx, pdy, ps of np
# parameters (5 textured, 2 not) as dot(p, a) * idet in the rasteriser's
# order. A MAC reads at most two lane registers (port A, port B; c shares
# port B), so a three-term dot is three products and two adds before the
# * idet. Inputs (X1..3, Y1..3, p1..3 per parameter) in registers 24..;
# outputs over them. Scratch 24..97 (T&L, projection and clip temporaries,
# free between kernels). Built only with gen_ucode.py --setup-kernels.
def k_setup(np_):
    if R3: return k_setup3(np_)
    ops = []
    X = [24, 25, 26]; Y = [27, 28, 29]
    P = [[30 + 3 * i + v for v in range(3)] for i in range(np_)]
    base = 30 + 3 * np_
    A = {(r, c): base + 3 * r + c for r in range(3) for c in range(3)}
    U = base + 9; DET = U + 3; IDET = DET + 1; T0 = IDET + 1
    one = k('one')
    i1, i2, i3 = (1, 2), (2, 0), (0, 1)   # a_r0 = Y[j] - Y[k], a_r1 = X[k] - X[j], a_r2 = X[j] Y[k] - X[k] Y[j]
    for r, (j, kk) in enumerate((i1, i2, i3)):
        ops.append(mac(rf(Y[j]), one, rf(Y[kk]), negc=True, dst=A[(r, 0)]))
        ops.append(mac(rf(X[kk]), one, rf(X[j]), negc=True, dst=A[(r, 1)]))
        ops.append(mac(rf(X[kk]), rf(Y[j]), None, dst=U + r))
        ops.append(mac(rf(X[j]), rf(Y[kk]), None, dst=A[(r, 2)]))
        ops.append(mac(rf(A[(r, 2)]), one, rf(U + r), negc=True, dst=A[(r, 2)]))
    ops.append(mac(rf(A[(0, 2)]), one, rf(A[(1, 2)]), dst=DET))
    ops.append(mac(rf(DET), one, rf(A[(2, 2)]), dst=DET))
    ops.append(Op(rf(DET), one, None, dst=IDET, div=True))
    nd = 3 * np_
    pool = list(range(T0 + nd, 98))      # product temporaries, shared round-robin
    outs = []
    for i in range(np_):
        for c in range(3):
            d = 3 * i + c
            acc = T0 + d
            ta, tb = pool[(2 * d) % len(pool)], pool[(2 * d + 1) % len(pool)]
            ops.append(mac(rf(P[i][0]), rf(A[(0, c)]), None, dst=acc))
            ops.append(mac(rf(P[i][1]), rf(A[(1, c)]), None, dst=ta))
            ops.append(mac(rf(acc), one, rf(ta), dst=acc))
            ops.append(mac(rf(P[i][2]), rf(A[(2, c)]), None, dst=tb))
            ops.append(mac(rf(acc), one, rf(tb), dst=acc))
            ops.append(mac(rf(acc), rf(IDET), None, dst=acc))
            outs.append(acc)
    assert T0 + nd < 98 and len(pool) >= 8, (T0, nd)
    return ops

def k_setup3(np_):
    # the same set-up with port C: a cross term is a product and a MAC, a dot
    # three MACs before the * idet; the rounding order is unchanged
    ops = []
    X = [24, 25, 26]; Y = [27, 28, 29]
    P = [[30 + 3 * i + v for v in range(3)] for i in range(np_)]
    base = 30 + 3 * np_
    A = {(r, c): base + 3 * r + c for r in range(3) for c in range(3)}
    U = base + 9; DET = U + 3; IDET = DET + 1; T0 = IDET + 1
    one = k('one')
    for r, (j, kk) in enumerate(((1, 2), (2, 0), (0, 1))):
        ops.append(mac(rf(Y[j]), one, rf(Y[kk]), negc=True, dst=A[(r, 0)]))
        ops.append(mac(rf(X[kk]), one, rf(X[j]), negc=True, dst=A[(r, 1)]))
        ops.append(mac(rf(X[kk]), rf(Y[j]), None, dst=U + r))
        ops.append(mac(rf(X[j]), rf(Y[kk]), rf(U + r), negc=True, dst=A[(r, 2)]))
    ops.append(mac(rf(A[(0, 2)]), one, rf(A[(1, 2)]), dst=DET))
    ops.append(mac(rf(DET), one, rf(A[(2, 2)]), dst=DET))
    ops.append(Op(rf(DET), one, None, dst=IDET, div=True))
    for i in range(np_):
        for c in range(3):
            acc = T0 + 3 * i + c
            ops.append(mac(rf(P[i][0]), rf(A[(0, c)]), None, dst=acc))
            ops.append(mac(rf(P[i][1]), rf(A[(1, c)]), rf(acc), dst=acc))
            ops.append(mac(rf(P[i][2]), rf(A[(2, c)]), rf(acc), dst=acc))
            ops.append(mac(rf(acc), rf(IDET), None, dst=acc))
    assert T0 + 3 * np_ < 98
    return ops

# The set-up as built (r3d_geo SETUP_OUT, m2-notes section 15): one triangle a
# lane, its inputs in the set-up staging side memory (the projection queue's
# upper half, written by the set-up port): X_v at v, Y_v at 3 + v, parameter i
# of sorted vertex v at 6 + 3 i + v (textured 1/w, u/w, v/w, shade; untextured
# 1/w, shade). Operand a reads them straight from the side memory; X and Y are
# also needed at ports B and C, so six copies put them in registers, and the
# same operations write them out (sdup: register and set-up output at one
# address). Outputs, the set-up output memory, a 32-word slot a lane:
#   0-5 x1 y1 x2 y2 x3 y3, 6 det, 8 + 4 i + {0, 1, 2} pdx, pdy, ps of
#   parameter i, 8 + 4 i + 3 parameter i of sorted vertex 1 (ps when flat).
# Rounding order as k_setup3 (rtl_tri::setup's).
SO_X, SO_DET = 32, 38   # X_v, Y_v at 32 + 2v, 33 + 2v (output words 0-5), det at 38 (word 6)
def k_setup_so(np_):
    ops = []
    one = k('one')
    X = [SO_X + 2 * v for v in range(3)]; Y = [SO_X + 2 * v + 1 for v in range(3)]
    sX = [0, 1, 2]; sY = [3, 4, 5]
    sP = lambda i, v: 6 + 3 * i + v
    A = {(r, c): 40 + 3 * r + c for r in range(3) for c in range(3)}
    U = 49; DET = SO_DET; IDET = 52; T0 = 53
    for v in range(3):
        ops.append(mac(sa(sY[v]), one, None, dst=Y[v], sdup=True))
        ops.append(mac(sa(sX[v]), one, None, dst=X[v], sdup=True))
    for r, (j, kk) in enumerate(((1, 2), (2, 0), (0, 1))):
        ops.append(mac(sa(sY[j]), one, rf(Y[kk]), negc=True, dst=A[(r, 0)]))
        ops.append(mac(sa(sX[kk]), one, rf(X[j]), negc=True, dst=A[(r, 1)]))
        ops.append(mac(sa(sX[kk]), rf(Y[j]), None, dst=U + r))
        ops.append(mac(sa(sX[j]), rf(Y[kk]), rf(U + r), negc=True, dst=A[(r, 2)]))
    ops.append(mac(rf(A[(0, 2)]), one, rf(A[(1, 2)]), dst=DET))
    ops.append(mac(rf(DET), one, rf(A[(2, 2)]), dst=DET, sdup=True))
    ops.append(Op(rf(DET), one, None, dst=IDET, div=True))
    for i in range(np_):
        ops.append(mac(sa(sP(i, 0)), one, None, side=8 + 4 * i + 3))
        for c in range(3):
            acc = T0 + 3 * i + c
            ops.append(mac(sa(sP(i, 0)), rf(A[(0, c)]), None, dst=acc))
            ops.append(mac(sa(sP(i, 1)), rf(A[(1, c)]), rf(acc), dst=acc))
            ops.append(mac(sa(sP(i, 2)), rf(A[(2, c)]), rf(acc), dst=acc))
            ops.append(mac(rf(acc), rf(IDET), None, side=8 + 4 * i + c))
    assert T0 + 3 * np_ <= 98 and IDET < T0
    return ops

def k_face():
    # the per-polygon back-face test, one polygon a lane, four rows: the face
    # normal into view space (n x T, 9 MACs), folded through the projection
    # into a clip-space plane (4), dotted with vertex 0's clip x, y, w (3, port
    # C). Measurement only: it charges the pool what the test costs (inputs
    # in 24.., results unused)
    ops = []
    for row in range(4):
        b = 24 + 18 * row
        n, v, nv, q, d = b, b + 3, b + 6, b + 9, b + 12
        for i in range(3):
            for kk in range(3):
                ops.append(mac(rf(n + kk), bc(kk, BC_TR + i), None if kk == 0 else rf(nv + i), dst=nv + i))
        ops.append(mac(rf(nv + 0), bc(0, BC_VW), None, dst=q + 0))
        ops.append(mac(rf(nv + 1), bc(0, BC_VH), None, dst=q + 1))
        ops.append(mac(rf(nv + 0), bc(0, BC_VX), rf(nv + 2), negc=True, dst=q + 2))
        ops.append(mac(rf(nv + 1), bc(0, BC_VY), rf(q + 2), dst=q + 2))
        ops.append(mac(rf(v + 0), rf(q + 0), None, dst=d))
        ops.append(mac(rf(v + 1), rf(q + 1), rf(d), dst=d))
        ops.append(mac(rf(v + 2), rf(q + 2), rf(d), dst=d))
    return ops

KERNELS = [
    ('MM', k_matmul),
    ('MODEL', k_model),
    ('TL', k_tl),
    ('PROJ', k_proj),
    # 1-12 vertices take TL3 (TL1 and TL2 would save ~12 cycles but not fit a
    # 512-word ROM); 5-8 projected vertices take PROJ3
    ('TL3', lambda: k_tl(3)),
    ('PROJ1', lambda: k_proj(1)), ('PROJ3', lambda: k_proj(3)),
    ('CLIP', k_clip),
    ('ADD', lambda: k_add(False)),
    ('SUB', lambda: k_add(True)),
    ('MUL', k_mul),
    ('DIV', k_div),
    ('COPYB', k_copyb),
    ('CULLU', k_cullu),
    ('CULLT', k_cullt),
    ('SMFR', k_smfr),
]

def build(kernels):
    words, entries, lengths, nops = [], {}, {}, {}
    for name, fn in kernels:
        ops = fn()
        t_issue, end = schedule(ops)
        entries[name] = len(words)
        slot = [None] * (end + 1)
        for op, t in zip(ops, t_issue):
            assert slot[t] is None
            slot[t] = op
        prog, c = [], 0
        while c < end:
            if slot[c] is not None:
                prog.append(encode(slot[c])); c += 1
            else:
                r = 0
                while c + r < end and slot[c + r] is None and r < 128: r += 1
                prog.append(encode(None, valid=False) | ((r - 1) << POS['ra'][0])); c += r
        prog.append(encode(None, end=True, valid=False))
        words += prog
        lengths[name] = end + 1
        nops[name] = len(ops)
        print('%-8s %4d ops, %4d cycles, %4d words' % (name, len(ops), end + 1, len(prog)), file=sys.stderr)
    return words, entries, lengths, nops

HDR = ("// SPDX-License-Identifier: GPL-3.0-or-later\n//\n// Sega Model 3 core for MiSTer FPGA\n"
       "// Copyright (C) 2026 alphanu1\n//\n// Generated by sim/real3d/gen_ucode.py; do not edit.\n\n")

def write_rom(path, module, words, note):
    depth = 1 << (len(words) - 1).bit_length()
    print('%s: %d words of %d bits (depth %d)' % (module, len(words), WORD_W, depth), file=sys.stderr)
    with open(path, 'w') as f:
        f.write(HDR)
        f.write("// r3d_vpu micro-code ROM%s, %d words of %d bits, in block RAM.\n\n" % (note, len(words), WORD_W))
        f.write("module %s (\n  input  logic        clk,\n  input  logic [9:0]  addr,\n"
                "  output logic [%d:0] q\n);\n" % (module, WORD_W - 1))
        f.write("  (* romstyle = \"M10K\" *) logic [%d:0] rom [0:%d];\n" % (WORD_W - 1, depth - 1))
        f.write("  initial begin\n")
        for i in range(depth):
            w = words[i] if i < len(words) else 0
            f.write("    rom[%d] = %d'h%x;\n" % (i, WORD_W, w))
        f.write("  end\n")
        f.write("  always_ff @(posedge clk) q <= rom[addr[%d:0]];\nendmodule\n" % ((depth - 1).bit_length() - 1))

def main():
    global R3
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    setup = '--setup-kernels' in sys.argv
    base = list(KERNELS)
    if setup:
        # measurement (R60, R65): textured: 1/w, u/w, v/w, shade; untextured: 1/w, shade (R62: no w plane)
        KERNELS.extend([('SETUP4', lambda: k_setup(4)), ('SETUP2', lambda: k_setup(2))])
        if R3: KERNELS.append(('FACE', k_face))
    words, entries, lengths, nops = build(KERNELS)
    assert len(words) <= (1024 if setup else 512)
    write_rom(os.path.join(root, 'rtl/real3d/r3d_ucode.sv'), 'r3d_ucode', words, '')
    if not setup:
        # the set-up output ROM (r3d_vpu SU_ROM, r3d_geo SETUP_OUT): every kernel of
        # the default ROM at the same address, then the set-up kernels with port C
        R3 = True
        print('-- set-up output ROM', file=sys.stderr)
        sw, se, sl, sn = build(base + [('SETUP4', lambda: k_setup_so(4)), ('SETUP2', lambda: k_setup_so(2))])
        assert sw[:len(words)] == words and len(sw) <= 1024
        write_rom(os.path.join(root, 'rtl/real3d/r3d_ucode_su.sv'), 'r3d_ucode_su', sw, ' with the set-up kernels (SETUP_OUT)')
        for n in ('SETUP4', 'SETUP2'):
            entries[n], lengths[n], nops[n] = se[n], sl[n], sn[n]
    consts = [('MT', MT), ('CO', CO), ('PR', PR), ('TR', TR),
              ('GA', GA), ('GB', GB), ('GC', GC), ('GO', GO), ('CT', CT),
              ('TLB', TLB), ('TL_W', TL_W), ('PJB', PJB), ('PJ_W', PJ_W), ('TL_NV', TL_NV), ('PJ_NV', PJ_NV),
              ('LB', LB), ('CS', CS), ('CX', CX), ('CZ', CZ), ('PA', PA), ('PB', PB), ('PC', PC), ('CF', CF),
              ('SONE', SONE), ('SSGN', SSGN), ('SF0', SF0), ('SC', SC), ('SB', SB),
              ('BC_MB', BC_MB), ('BC_TOP', BC_TOP), ('BC_TR', BC_TR), ('BC_VP', BC_VP),
              ('BC_L0', BC_L0), ('BC_L1', BC_L1), ('BC_L2', BC_L2), ('BC_IP', BC_IP), ('BC_AMB', BC_AMB),
              ('BC_VW', BC_VW), ('BC_VX', BC_VX), ('BC_VH', BC_VH), ('BC_VY', BC_VY), ('BC_T', BC_T),
              ('BC_CU', BC_CU), ('BC_CL', BC_CL), ('BC_ZS', BC_ZS), ('BC_ZO', BC_ZO), ('BC_L2N', BC_L2N), ('BC_K', BC_K)]
    with open(os.path.join(root, 'rtl/real3d/r3d_ucode_pkg.sv'), 'w') as f:
        f.write(HDR)
        f.write("package r3d_ucode_pkg;\n")
        for name in entries:
            note = '  (set-up output ROM, r3d_ucode_su)' if (not setup and name.startswith('SETUP')) else ''
            f.write("  localparam logic [9:0] K_%s = 10'd%d;  // %d cycles, %d ops%s\n" % (name, entries[name], lengths[name], nops[name], note))
        if 'FACE' not in entries:
            f.write("  localparam logic [9:0] K_FACE = 10'd0;  // not built (--setup-kernels --3r)\n")
        for n, v in consts:
            f.write("  localparam int R_%s = %d;\n" % (n, v))
        f.write("  localparam int UW = %d;\n" % WORD_W)
        for n, (lo, w) in POS.items():
            f.write("  localparam int U_%s = %d, UW_%s = %d;\n" % (n.upper(), lo, n.upper(), w))
        f.write("endpackage\n")
    with open(os.path.join(root, 'sim/real3d/r3d_ucode.h'), 'w') as f:
        f.write(HDR)
        f.write("#pragma once\nnamespace r3d_uc {\n")
        for name in entries:
            f.write("constexpr unsigned K_%s = %d, LEN_%s = %d;\n" % (name, entries[name], name, lengths[name]))
        for n, v in consts:
            f.write("constexpr unsigned R_%s = %d;\n" % (n, v))
        f.write("}\n")

if __name__ == '__main__':
    main()
