//============================================================================
//  Namco NA-1 / NA-2 -- shared constants
//
//  Evidence tags used throughout this project:
//    HW_CONFIRMED         proven by PCB / chip / BIOS / schematic evidence
//    MAME_VERIFIED        taken from MAME and believed correct
//    FBNEO_CROSSCHECK     independently agrees with FBNeo
//    ROM_DISASM_CONFIRMED proven by disassembling the game or C70 ROM
//    INFERRED             deduced, not directly proven
//    TEMPORARY_HLE        stand-in, must be replaced
//    UNKNOWN              behaviour not understood
//============================================================================
`ifndef NA2_PKG_SVH
`define NA2_PKG_SVH

// ---------------------------------------------------------------------------
// Clocks
// HW_CONFIRMED: NA-2 GAME PCB carries a single 50.113 MHz oscillator; the PCB
// notes in MAME namcona1.cpp state "68000 clock - 12.5282MHz [50.113/4]".
// MAME_VERIFIED: MASTER_CLOCK/4 is used for both the 68000 and the C70.
// Divergence to investigate: FBNeo runs the M377xx at 12528250/2 (see
// MAME_FBNEO_DIFF.md, "CPU clocks").
// ---------------------------------------------------------------------------
`define NA2_MASTER_HZ  50_113_000
`define NA2_CPU_DIV    4              // 68000 = master/4 = 12.5282 MHz
`define NA2_MCU_DIV    4              // C70    = master/4 (MAME); see DIFF doc

// ---------------------------------------------------------------------------
// 68000 address map
// MAME_VERIFIED: namcona1_state::namcona1_main_map (src/mame/namco/namcona1.cpp)
// FBNEO_CROSSCHECK: d_namcona1.cpp namcona1_read/write_word agree on every
//   region below.
// ROM_DISASM_CONFIRMED: the entry code at 0xC00000 writes 0xEFFF1A, and the
//   version handshake at 0xC2DA48 writes 0x3F8008; both land where expected.
// ---------------------------------------------------------------------------
`define NA2_WORKRAM_LO   24'h000000   // 512 KiB, shared with the C70
`define NA2_WORKRAM_HI   24'h07FFFF
`define NA2_MAILBOX_LO   24'h3F8000   // 8 x 16-bit, mirrored through 0x3FFFFF
`define NA2_MAILBOX_HI   24'h3FFFFF
`define NA2_MASKROM_LO   24'h400000   // 8 MiB graphics/data mask ROM
`define NA2_MASKROM_HI   24'hBFFFFF
`define NA2_PRGROM_LO    24'hC00000   // 2 MiB program EPROM
`define NA2_PRGROM_HI    24'hDFFFFF
`define NA2_EEPROM_LO    24'hE00000   // 28C16, byte-wide on the odd lane
`define NA2_EEPROM_HI    24'hE00FFF
`define NA2_KEYCUS_LO    24'hE40000   // the ROM board's key chip; which one
                                      // is a per-game input, see na2_keycus.sv
`define NA2_KEYCUS_HI    24'hE4000F
`define NA2_VREG_LO      24'hEFFF00   // video + blitter registers
`define NA2_VREG_HI      24'hEFFFFF
`define NA2_PALETTE_LO   24'hF00000
`define NA2_PALETTE_HI   24'hF01FFF
`define NA2_GFXRAM_LO    24'hF40000   // CG RAM / shape RAM, banked by vreg $0C
`define NA2_GFXRAM_HI    24'hF7FFFF
`define NA2_VIDEORAM_LO  24'hFF0000   // 4 tilemaps + ROZ
`define NA2_VIDEORAM_HI  24'hFFBFFF
`define NA2_SCRATCH_LO   24'hFFC000   // plain RAM; some games need it to boot
`define NA2_SCRATCH_HI   24'hFFDFFF
`define NA2_SCROLL_LO    24'hFFE000   // per-line scroll / line select
`define NA2_SCROLL_HI    24'hFFEFFF
`define NA2_SPRITERAM_LO 24'hFFF000
`define NA2_SPRITERAM_HI 24'hFFFFFF

// Named video registers, byte offsets inside 0xEFFF00.
// MAME_VERIFIED: vreg_w(), blit(), screen_update(), and the register map
// comment at the end of src/mame/namco/namcona1_v.cpp.
`define VREG_BLT_SRC1     8'h02
`define VREG_BLT_DST1     8'h08
`define VREG_GFXBANK      8'h0C
`define VREG_BLT_SRCADDR  8'h0E       // 0x0E hi, 0x10 lo -- dword offsets
`define VREG_BLT_DSTADDR  8'h12       // 0x12 hi, 0x14 lo
`define VREG_BLT_NBYTES   8'h16
`define VREG_BLT_TRIGGER  8'h18       // any write starts the blit
`define VREG_IRQ_ENABLE   8'h1A       // 0 bit enables that IRQ level
`define VREG_SPRITE_CTRL  8'h22       // bit 0 selects the spriteram bank
`define VREG_CRTC_XMIN    8'h80
`define VREG_CRTC_XMAX    8'h82
`define VREG_CRTC_YMIN    8'h84
`define VREG_CRTC_YMAX    8'h86
`define VREG_POSIRQ_LINE  8'h8A
`define VREG_GFX_ENABLE   8'h8E
`define VREG_FLIPSCREEN   8'h98
`define VREG_BPP_FLAGS    8'hBC
`define VREG_CLR_ROZ      8'hBA       // also the screen background colour
`define VREG_CLR_ROZ_IDX  7'h5D       // 0xBA >> 1

// Per-layer priority and colour, and the ROZ priority.
// MAME_VERIFIED, FBNEO_CROSSCHECK: both take the ROZ priority from
// `vreg[0xa0/2 + 5]`, i.e. 0xAA -- *not* from 0xA8, which the register-map
// comment leaves as "????".
`define VREG_PRI0         8'hA0
`define VREG_PRI_ROZ      8'hAA
`define VREG_CLR0         8'hB0

// ---------------------------------------------------------------------------
// C70 (M37702M2) address map
// MAME_VERIFIED: namcona1_state::namcona1_mcu_map + m37702m2_device::map.
// HW_CONFIRMED: c70.bin sets the stack to 0x027E at reset (LDA #$027E / TCS at
//   0x00C03A), which is exactly the top of the M37702M2's internal RAM
//   (0x000080-0x00027F) -- so the internal RAM size is proven by the BIOS.
// ---------------------------------------------------------------------------
`define C70_SFR_LO       24'h000000   // special function registers
`define C70_SFR_HI       24'h00007F
`define C70_IRAM_LO      24'h000080   // 512 bytes internal RAM
`define C70_IRAM_HI      24'h00027F
`define C70_MAILBOX_LO   24'h000800   // same mailbox the 68000 sees at 0x3F8000
`define C70_MAILBOX_HI   24'h000FFF
`define C70_C219_LO      24'h001000   // C219 sound chip, mirrored to 0x1FFF
`define C70_C219_HI      24'h0011FF
`define C70_SHARED0_LO   24'h002000   // mirror of the first page of work RAM
`define C70_SHARED0_HI   24'h002FFF
`define C70_XRAM_LO      24'h003000   // external SRAM (LH5168, 8Kx8)
`define C70_XRAM_HI      24'h00AFFF
`define C70_IROM_LO      24'h00C000   // internal 16 KiB mask ROM = c70.bin
`define C70_IROM_HI      24'h00FFFF
`define C70_SHARED_LO    24'h200000   // 68000 work RAM, byte-swapped
`define C70_SHARED_HI    24'h27FFFF

// C70 port bits.
// HW_CONFIRMED from c70.bin:
//   0x00C067 LDM #$04,$0A  -> P4 = 0x04 at boot (bit 3 clear, 68000 held)
//   0x00C646 LDM #$0C,$0A  -> P4 = 0x0C          (bit 3 set,  68000 released)
//   0x00C061 LDM #$80,$0E  -> P6 = 0x80 at boot (input mux disabled)
//   0x00C06D LDM #$00,$11  -> P7 direction = all inputs
// MAME_VERIFIED: port4_w / port6_w / port7_r in namcona1.cpp.
`define C70_P4_68K_RUN    3            // P4 bit 3: 1 releases the 68000
`define C70_P6_MUX_DIS    7            // P6 bit 7: 1 disables the input mux
// P6 bits 6:5 select which input port appears on P7: 0=P4in 1=DSW 2=P1 3=P2

// ---------------------------------------------------------------------------
// FPGA ROM download map (asset load index 0).
// Must stay in step with games/numanath/romset.json "download_map" and with
// the <rom> stream emitted by tools/build_rom.py --mra.
// ---------------------------------------------------------------------------
`define ROMDL_PRG_BASE    25'h0000000  // 0x000000  2 MiB  68000 program ROM
`define ROMDL_PRG_SIZE    25'h0200000
`define ROMDL_MASK_BASE   25'h0200000  // 0x200000  8 MiB  mask ROM
`define ROMDL_MASK_SIZE   25'h0800000
`define ROMDL_C70_BASE    25'h0A00000  // 0xA00000 16 KiB  C70 BIOS
`define ROMDL_C70_SIZE    25'h0004000
`define ROMDL_TOTAL       25'h0A04000

// ---------------------------------------------------------------------------
// Boot constants deposited by the C70 into shared work RAM.
// ROM_DISASM_CONFIRMED, c70.bin 0x00C62E:
//   LDM #$0000,$0000 / LDM #$0004,$0002 / LDM #$C000,$0004 / LDM #$0000,$0006
// with the data bank register set to $20, i.e. writes land in the shared work
// RAM window.  Byte-swapping the MCU view gives the 68000 vector below, and
// the SSP agrees with the game's own "LEA $0400.W,A7" at 0xC0000C.
// ---------------------------------------------------------------------------
`define NA2_BOOT_SSP      32'h0000_0400
`define NA2_BOOT_PC       32'h00C0_0000

`endif // NA2_PKG_SVH
