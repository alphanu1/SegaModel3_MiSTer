// tb_loader_sw.cpp (make -f sim/system/system.mk test_loader_sw): m3_loader's DIP bank from the MRA's <switches> (index 254, study R133), in Main's
// order (the configuration, the image, then the switches; again on every change in the OSD).
#include "Vm3_loader.h"
#include "verilated.h"
#include <cstdio>
#include <vector>
static Vm3_loader *t;
static int busy_seen;
static void tick() { t->clk = 0; t->eval(); t->sdr_wr_ack = t->sdr_wr_req; t->x_ack = t->x_req; t->clk = 1; t->eval(); }
static void dl(int ix, const std::vector<uint8_t> &d) {
  busy_seen = 0;
  t->ioctl_index = ix; t->ioctl_download = 1; tick();
  for (size_t a = 0; a < d.size(); a += 2) {
    while (t->ioctl_wait) tick();
    t->ioctl_wr = 1; t->ioctl_addr = a; t->ioctl_dout = d[a] | (a + 1 < d.size() ? d[a + 1] << 8 : 0); tick();
    busy_seen |= t->busy;
    t->ioctl_wr = 0; tick(); tick(); busy_seen |= t->busy;
  }
  t->ioctl_download = 0; for (int k = 0; k < 2000; k++) tick();
}
static std::vector<uint8_t> sw(uint8_t b0, uint8_t b1 = 0) { return {b0, b1, 0, 0, 0, 0, 0, 0}; }
int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  t = new Vm3_loader;
  t->mem_ready = 1; t->x_ok = 1; t->rst = 1; for (int k = 0; k < 4; k++) tick(); t->rst = 0; tick();
  int bad = 0;
  auto chk = [&](const char *what, unsigned got, unsigned want) {
    std::printf("%-52s %02X (want %02X)%s\n", what, got, want, got == want ? "" : "  FAIL"); if (got != want) bad++;
  };
  std::vector<uint8_t> lw = {0x15, 0x00, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0xFF, 0xFF};
  std::vector<uint8_t> img(64, 0x5A);
  chk("after reset", t->cfg_dsw, 0x7F);
  dl(4, lw); dl(0, img); chk("1 Lost World: configuration, image", t->cfg_dsw, 0x7F);
  chk("  rom_loaded", t->rom_loaded, 1);
  dl(254, sw(0xFF, 0x7F)); chk("  then switches FF (byte 1 7F)", t->cfg_dsw, 0xFF);
  chk("  rom_loaded after the switches", t->rom_loaded, 1);
  chk("  busy during the switches", busy_seen, 0);
  chk("  step kept", t->cfg_step, 0x15);
  dl(254, sw(0xFE)); chk("2 an OSD change: SW8 ON", t->cfg_dsw, 0xFE);
  dl(4, lw); chk("3 the next game's configuration starts: 7F", t->cfg_dsw, 0x7F);
  dl(0, img); dl(254, sw(0x7F)); chk("  its image, its switches 7F", t->cfg_dsw, 0x7F);
  dl(254, sw(0x01)); dl(0, img); chk("4 VF3 order (no configuration): image resets", t->cfg_dsw, 0x7F);
  dl(254, sw(0x01)); chk("  then its switches 01", t->cfg_dsw, 0x01);
  dl(2, std::vector<uint8_t>(16, 0)); chk("5 an NVRAM download keeps the switches", t->cfg_dsw, 0x01);
  std::printf("%s: %d failed\n", bad ? "FAIL" : "PASS", bad);
  delete t; return bad ? 1 : 0;
}
