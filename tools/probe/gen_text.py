#!/usr/bin/env python3
# gen_text.py: rtl/probe/dp_text_pkg.sv, the DDR3 probe core's static screens,
# its 5 x 7 font and the reporter's line templates. A screen is 21 rows of 48
# character cells (41 shown); a character is its ASCII code - 32.
import sys
W = 48
CASES = []
for bg in ('--', 'L ', 'LS'):
    for n in (1, 2, 4, 8, 16):
        CASES.append((n, bg, 'R4'))
CASES.append((16, '--', 'S4'))
CASES.append((32, '--', 'R8'))
# the automatic sweep's screen
rows = {}
rows[0]  = 'DDR3 PROBE PASS 00000 ERR 00000 CASE 00'
rows[1]  = ' N BG LN    MB/S MIN MED   MAX AC BGM'
for k, (n, bg, ln) in enumerate(CASES):
    rows[2 + k] = f'{n:2d} {bg} {ln}  000.000 000 000 00000 00 000'
rows[19] = 'BG L 22+22 LS +100 MB/S. LN BEATS, R/S'
rows[20] = ''   # the debug row, written by the reporter
# the manual mode's screen
mrows = {}
mrows[0]  = 'DDR3 PROBE PASS 00000 ERR 00000 MANUAL'
mrows[2]  = 'DIR ADDR BURST INFL BG  CLK'
mrows[3]  = '  R    R   001   01 --  100'
mrows[5]  = '    MB/S MIN MED   MAX AC BGM'
mrows[6]  = ' 000.000 000 000 00000 00 000'
mrows[8]  = 'A WINDOW OF 1 S, REPEATED. LATENCY IN'
mrows[9]  = 'CYCLES OF THE CLOCK SHOWN, COMMAND ON'
mrows[10] = 'THE PINS TO BEAT 1 READ, OR BEAT 1'
mrows[11] = 'TAKEN WRITE. AC: READS HELD AT MOST.'
mrows[19] = 'BG L 22+22 LS +100 MB/S'
mrows[20] = ''
for rr in (rows, mrows):
    for r, t in rr.items():
        assert len(t) <= 41, (r, len(t), t)

FONT = {
 ' ': [0]*7,
 '0': ['01110','10001','10011','10101','11001','10001','01110'],
 '1': ['00100','01100','00100','00100','00100','00100','01110'],
 '2': ['01110','10001','00001','00010','00100','01000','11111'],
 '3': ['11111','00010','00100','00010','00001','10001','01110'],
 '4': ['00010','00110','01010','10010','11111','00010','00010'],
 '5': ['11111','10000','11110','00001','00001','10001','01110'],
 '6': ['00110','01000','10000','11110','10001','10001','01110'],
 '7': ['11111','00001','00010','00100','01000','01000','01000'],
 '8': ['01110','10001','10001','01110','10001','10001','01110'],
 '9': ['01110','10001','10001','01111','00001','00010','01100'],
 'A': ['01110','10001','10001','11111','10001','10001','10001'],
 'B': ['11110','10001','10001','11110','10001','10001','11110'],
 'C': ['01110','10001','10000','10000','10000','10001','01110'],
 'D': ['11100','10010','10001','10001','10001','10010','11100'],
 'E': ['11111','10000','10000','11110','10000','10000','11111'],
 'G': ['01110','10001','10000','10111','10001','10001','01111'],
 'H': ['10001','10001','10001','11111','10001','10001','10001'],
 'I': ['01110','00100','00100','00100','00100','00100','01110'],
 'L': ['10000','10000','10000','10000','10000','10000','11111'],
 'M': ['10001','11011','10101','10101','10001','10001','10001'],
 'N': ['10001','10001','11001','10101','10011','10001','10001'],
 'O': ['01110','10001','10001','10001','10001','10001','01110'],
 'P': ['11110','10001','10001','11110','10000','10000','10000'],
 'R': ['11110','10001','10001','11110','10100','10010','10001'],
 'S': ['01111','10000','10000','01110','00001','00001','11110'],
 'T': ['11111','00100','00100','00100','00100','00100','00100'],
 'U': ['10001','10001','10001','10001','10001','10001','01110'],
 'X': ['10001','10001','01010','00100','01010','10001','10001'],
 'F': ['11111','10000','10000','11110','10000','10000','10000'],
 'K': ['10001','10010','10100','11000','10100','10010','10001'],
 'Q': ['01110','10001','10001','10001','10101','10010','01101'],
 'V': ['10001','10001','10001','10001','10001','01010','00100'],
 'W': ['10001','10001','10001','10101','10101','10101','01010'],
 'Z': ['11111','00001','00010','00100','01000','10000','11111'],
 'J': ['00111','00010','00010','00010','00010','10010','01100'],
 ':': ['00000','01100','01100','00000','01100','01100','00000'],
 'Y': ['10001','10001','01010','00100','00100','00100','00100'],
 '+': ['00000','00100','00100','11111','00100','00100','00000'],
 '-': ['00000','00000','00000','11111','00000','00000','00000'],
 '.': ['00000','00000','00000','00000','00000','01100','01100'],
 ',': ['00000','00000','00000','00000','01100','00100','01000'],
 '/': ['00000','00001','00010','00100','01000','10000','00000'],
}

used = set(''.join(rows.values()) + ''.join(mrows.values()))
missing = used - set(FONT)
assert not missing, missing

# ---- the reporter's line templates (rtl/probe/dp_report.sv) ----------------------
# An element is 32 bits: [31:30] op (0 end of line, 1 a character, 2 a decimal
# field, 3 a hex field), [29:25] screen row (31: the case's row), [24] on the
# screen, [23:18] column, [17] to the UART, [16:11] field, [10:7] top digit,
# [6:3] bottom digit, [2] a point after digit 6; a character [7:0] (1-6: the
# build date's characters, 7: the field's value as a character).
F = dict(iw=0, rd=1, bt=2, busy=3, st=4, k=5, n=6, bg=7, ln=8, bytes=9, lmin=10, lmed=11, lmax=12,
         ac=13, bgb=14, pass_=15, err=16, drop=17, seq=18, base=19, acc=20, oc=21, t=22, mhz=23, knext=24,
         dirc=25, adrc=26, bgc1=27, bgc2=28, status=29, modec=30, rdy=31, lrv=32, mst=33, rq=34, rrise=35, rsince=36, rfirst=37, prd=38, vrd=39)
CR = 31
def el(op, row=None, col=0, u=1, low=0):
    scr = 0 if row is None else 1
    return (op << 30) | ((row or 0) << 25) | (scr << 24) | (col << 18) | (u << 17) | low
def ch(c, row=None, col=0, u=1): return el(1, row, col, u, c if isinstance(c, int) else ord(c))
def fch(f, row=None, col=0, u=1): return el(1, row, col, u, (F[f] << 11) | 7)
def num(f, hi, lo=0, row=None, col=0, u=1, dot=0, hexa=0): return el(3 if hexa else 2, row, col, u, (F[f] << 11) | (hi << 7) | (lo << 3) | (dot << 2))
def txt(t, row=None, col=0, u=1): return [ch(c, row, (col + i) if row is not None else 0, u) for i, c in enumerate(t)]
END = 0
BITS = ' BITS 1 MODE 2 DIR 3 ADDR 6:4 BURST 9:7 INFL 11:10 BG 13:12 CLK 14 NOINIT 15 LPATH 16 INIT1'
L_START = txt('DDR3 PROBE BUILD ') + [ch(i) for i in range(1, 7)] + txt(' BASE ') + [num('base', 7, 0, hexa=1)] + \
          txt(' CLK ') + [num('mhz', 2)] + txt(' MODE ') + [fch('modec')] + txt(' STATUS ') + [num('status', 3, hexa=1)] + txt(BITS) + [END]
L_DBG = txt('D T') + [num('t', 4)] + txt(' S') + [num('st', 0, hexa=1, row=20, col=40)] + \
        txt(' ') + [ch('I', 20, 0)] + [num('iw', 6, row=20, col=1)] + txt(' ') + [ch('R', 20, 9)] + [num('rd', 7, row=20, col=10)] + \
        txt(' ') + [ch('B', 20, 19)] + [num('bt', 7, row=20, col=20)] + txt(' ') + [ch('Y', 20, 29)] + [num('busy', 7, row=20, col=30)] + \
        [ch('S', 20, 39, 0)] + txt(' A') + [num('acc', 1)] + txt(' O') + [num('oc', 1)] + txt(' Q') + [num('rdy', 7)] + \
        txt(' L') + [num('lrv', 7)] + txt(' K') + [num('mst', 1, hexa=1)] + txt(' E') + [num('rq', 7, hexa=1)] + \
        txt(' X') + [num('rrise', 2)] + txt(' Z') + [num('rsince', 9)] + txt(' W') + [num('rfirst', 0)] + \
        txt(' P') + [num('prd', 7)] + txt(' V') + [num('vrd', 7)] + txt(' DROP') + [num('drop', 4)] + [END]
L_CASE = txt('C ') + [num('k', 1)] + txt(' N ') + [num('n', 1)] + txt(' BG ') + [num('bg', 0)] + txt(' LN ') + [num('ln', 2)] + \
         txt(' SEQ ') + [num('seq', 0)] + txt(' CLK ') + [num('mhz', 2)] + txt(' MBS ') + [num('bytes', 8, 3, row=CR, col=10, dot=1)] + \
         txt(' MIN ') + [num('lmin', 2, row=CR, col=18)] + txt(' MED ') + [num('lmed', 2, row=CR, col=22)] + \
         txt(' MAX ') + [num('lmax', 4, row=CR, col=26)] + txt(' AC ') + [num('ac', 1, row=CR, col=32)] + \
         txt(' BGM ') + [num('bgb', 8, 6, row=CR, col=35)] + txt(' PASS ') + [num('pass_', 4, row=0, col=16)] + \
         txt(' ERR ') + [num('err', 4, row=0, col=26)] + [num('knext', 1, row=0, col=37, u=0)] + [END]
L_MAN = txt('M DIR ') + [fch('dirc', 3, 2)] + txt(' ADDR ') + [fch('adrc', 3, 7)] + txt(' LN ') + [num('ln', 2, row=3, col=11)] + \
        txt(' N ') + [num('n', 1, row=3, col=17)] + txt(' BG ') + [fch('bgc1', 3, 20), fch('bgc2', 3, 21)] + \
        txt(' CLK ') + [num('mhz', 2, row=3, col=24)] + txt(' MBS ') + [num('bytes', 8, 3, row=6, col=1, dot=1)] + \
        txt(' MIN ') + [num('lmin', 2, row=6, col=9)] + txt(' MED ') + [num('lmed', 2, row=6, col=13)] + \
        txt(' MAX ') + [num('lmax', 4, row=6, col=17)] + txt(' AC ') + [num('ac', 1, row=6, col=23)] + \
        txt(' BGM ') + [num('bgb', 8, 6, row=6, col=26)] + txt(' PASS ') + [num('pass_', 4, row=0, col=16)] + \
        txt(' ERR ') + [num('err', 4, row=0, col=26)] + [END]
TM = L_START + L_DBG + L_CASE + L_MAN
assert len(TM) <= 512, len(TM)
starts = (0, len(L_START), len(L_START) + len(L_DBG), len(L_START) + len(L_DBG) + len(L_CASE))

o = []
o.append('// Generated by tools/probe/gen_text.py: do not edit.')
o.append('//')
o.append("// The DDR3 probe core's static screens (21 rows of 48 cells, 41 shown), its")
o.append('// 5 x 7 font and the reporter\'s line templates. A character is its ASCII')
o.append('// code - 32.')
o.append('package dp_text_pkg;')
o.append(f'  localparam int NCASE = {len(CASES)};')
for name, rr in (('stext', rows), ('stext_m', mrows)):
    o.append(f'  function automatic logic [5:0] {name}(input logic [9:0] a);')
    o.append('    case (a)')
    for r, t in sorted(rr.items()):
        for c, chh in enumerate(t):
            if chh != ' ':
                o.append(f"      10'd{r * W + c}: {name} = 6'd{ord(chh) - 32};")
    o.append(f"      default: {name} = 6'd0;")
    o.append('    endcase')
    o.append('  endfunction')
o.append('  // row r (0 top) of character ch: five bits, bit 4 the left pixel')
o.append('  function automatic logic [4:0] glyph(input logic [5:0] ch, input logic [2:0] r);')
o.append('    logic [34:0] g;')
o.append('    case (ch)')
for chh, bm in sorted(FONT.items()):
    if chh == ' ': continue
    o.append(f"      6'd{ord(chh) - 32}: g = 35'b{''.join(bm)};   // {chh}")
o.append("      default: g = '0;")
o.append('    endcase')
o.append("    glyph = (r < 3'd7) ? g[34 - 5 * r -: 5] : 5'd0;")
o.append('  endfunction')
o.append(f"  localparam logic [8:0] T_START = 9'd{starts[0]}, T_DBG = 9'd{starts[1]}, T_CASE = 9'd{starts[2]}, T_MAN = 9'd{starts[3]};")
o.append('  function automatic logic [31:0] tmpl(input logic [8:0] i);')
o.append('    case (i)')
for i, e in enumerate(TM):
    o.append(f"      9'd{i}: tmpl = 32'h{e:08X};")
o.append("      default: tmpl = 32'h0;")
o.append('    endcase')
o.append('  endfunction')
o.append('endpackage')
open(sys.argv[1], 'w').write('\n'.join(o) + '\n')
for name, rr in (('auto', rows), ('manual', mrows)):
    print('==', name)
    for r, t in sorted(rr.items()):
        print(f'{r:2d} {t}')
print('templates', len(TM), starts)
