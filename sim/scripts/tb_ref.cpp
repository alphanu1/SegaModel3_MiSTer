// SPDX-License-Identifier: GPL-3.0-or-later
//
// Sega Model 3 core for MiSTer FPGA
// Copyright (C) 2026 alphanu1
//
// tb_ref --form hw|sm|mame <file.vec>...
//
// Replays a capture through the reference: every CPU access in order, every
// register read and the interrupt line after every access compared with what
// the emulator returned, every instruction and memory move compared with what
// it ran. The SM form must match Supermodel's captures exactly and the MAME
// form MAME's; the HW form lists each difference by register, lane and bits.

#include "m3s_ref.h"
#include "m3s_vec.h"
#include <cstring>
#include <map>
#include <tuple>

int main(int argc, char **argv) {
  M3sRef::Form form = M3sRef::HW;
  std::vector<std::string> files;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--form") && i + 1 < argc) {
      std::string f = argv[++i];
      form = f == "sm" ? M3sRef::SM : f == "mame" ? M3sRef::MAME : M3sRef::HW;
    } else files.push_back(argv[i]);
  }
  int bad_files = 0;
  for (auto &path : files) {
    auto v = load_vec(path);
    BenchMem mem;
    M3sRef ref;
    ref.form = form;
    ref.rd = [&](uint32_t a) { return mem.rd(a); };
    ref.wr = [&](uint32_t a, uint32_t d) { mem.wr(a, d); };
    ref.reset();
    uint64_t acc = 0, reads = 0, starts = 0, insn = 0, moves = 0, bytes = 0, err = 0, qerr = 0, xerr = 0, perr = 0;
    std::map<std::tuple<unsigned, uint32_t, uint32_t>, uint64_t> diff;   // (off, expected, got)
    size_t xi = 0, mi = 0;
    bool pending = false;       // insns/moves of the last start still to be matched
    auto close = [&]() {
      if (pending && (xi != ref.insns.size() || mi != ref.moves.size())) xerr++;
      pending = false;
    };
    for (auto &r : v) {
      switch (r.k) {
        case 'L': mem.wr(r.a, r.b); break;
        case 'P': close(); ref.cfg_write(r.a, r.b); break;
        case 'p': close(); if (ref.cfg_read(r.a) != r.b) perr++; break;
        case 'W':
          close(); acc++;
          ref.insns.clear(); ref.moves.clear();
          ref.write(r.a, r.b, r.c);
          if (!ref.insns.empty()) { starts++; pending = true; xi = mi = 0; }
          break;
        case 'R': {
          close(); acc++; reads++;
          uint32_t got = ref.read(r.a, r.b);
          if (got != r.c) { err++; diff[{r.a, r.c, got}]++; }
          break;
        }
        case 'Q': if (ref.irq() != (r.a != 0)) qerr++; break;
        case 'X':
          insn++;
          if (xi >= ref.insns.size() || ref.insns[xi].dsp != r.a || ref.insns[xi].w0 != r.b) xerr++;
          xi++;
          break;
        case 'M':
          moves++; bytes += r.c;
          if (mi >= ref.moves.size() || ref.moves[mi].src != r.a || ref.moves[mi].dst != r.b || ref.moves[mi].len != r.c) xerr++;
          mi++;
          break;
        default: break;
      }
    }
    close();
    std::printf("%s [%s]: %llu accesses (%llu reads), %llu starts, %llu instructions, %llu moves, %llu bytes\n",
                path.c_str(), form == M3sRef::HW ? "hw" : form == M3sRef::SM ? "sm" : "mame",
                (unsigned long long)acc, (unsigned long long)reads, (unsigned long long)starts,
                (unsigned long long)insn, (unsigned long long)moves, (unsigned long long)bytes);
    std::printf("  read mismatches %llu, interrupt-line mismatches %llu, instruction/move mismatches %llu, config %llu\n",
                (unsigned long long)err, (unsigned long long)qerr, (unsigned long long)xerr, (unsigned long long)perr);
    for (auto &d : diff)
      std::printf("    off %02X: capture %08X, reference %08X  x%llu\n", std::get<0>(d.first), std::get<1>(d.first),
                  std::get<2>(d.first), (unsigned long long)d.second);
    if (err || qerr || xerr || perr) bad_files++;
  }
  return bad_files ? 1 : 0;
}
