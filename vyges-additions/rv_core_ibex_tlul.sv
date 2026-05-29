// rv_core_ibex_tlul.sv — Thin TL-UL wrapper around ibex_top for Edge Sensor SoC
//
// Bridges ibex's native OBI-style instruction + data interfaces to TL-UL:
//   - Instruction fetch: synchronous BRAM (Vivado/sim inferred).
//       BootRomFile = "" (default) → all-NOP fill (simulation/elaboration).
//       BootRomFile = "boot.hex"  → $readmemh pre-loaded firmware (FPGA).
//       Vivado infers BootRomDepth×32b as BRAM36 automatically.
//   - Data access: tlul_adapter_host → xbar_main → UART / FFT / ROM / RAM
//
// Parameters:
//   BootRomFile  : path to hex firmware image; "" means NOP-fill
//   BootRomDepth : ROM depth in 32-bit words (default 8192 = 32 KB)
//   SecureIbex=0  ICache=0  BranchPredictor=0  WritebackStage=0

`include "prim_assert.sv"

module rv_core_ibex_tlul
  import ibex_pkg::*;
  import prim_mubi_pkg::*;
#(
  parameter string BootRomFile  = "",   // "" = NOP-fill; non-empty = $readmemh firmware
  parameter int    BootRomDepth = 8192, // 32-bit words; 8192 × 4 B = 32 KB
  // Root-of-Trust hardening toggle. 0 (default) = standard mode: no lockstep,
  // dummy instructions, or register-file/memory ECC. 1 = SecureIbex. Driven by
  // the soc-spec cpus[].config.security_mode knob ('secure' -> 1). Forwarded to
  // ibex_top, which derives MemECC/Lockstep/DummyInstructions from it.
  parameter bit    SecureIbex   = 1'b0
) (
  input  logic         clk_i,
  input  logic         rst_ni,
  input  logic [31:0]  boot_addr_i,    // typically ROM base = 32'h00008000
  input  logic [31:0]  hart_id_i,

  // TL-UL data bus to crossbar
  output tlul_pkg::tl_h2d_t tl_o,
  input  tlul_pkg::tl_d2h_t tl_i,

  // Interrupts
  input  logic irq_software_i,
  input  logic irq_timer_i,
  input  logic irq_external_i,

  // Status
  output logic core_sleep_o
);

  // ── Instruction fetch: synchronous BRAM ────────────────────────────────────
  // BootRomFile = "" → NOP-fill (all 32'h00000013); Ibex loops without hanging.
  // BootRomFile set  → $readmemh firmware; Vivado infers BootRomDepth×32b BRAM.
  // Timing: grant is always immediate; rvalid fires 1 cycle after req.
  // Address: boot_addr_i = 0x00008000; word index = addr[clog2(Depth)+1 : 2].
  logic       instr_req;
  logic       instr_gnt;
  logic [31:0] instr_addr;
  logic        instr_rvalid;
  logic [31:0] instr_rdata;
  logic [6:0]  instr_rdata_intg;
  logic        instr_err;

  // Boot ROM — depth in 32-bit words
  logic [31:0] boot_rom [BootRomDepth];
  initial begin
    if (BootRomFile != "") begin
      $readmemh(BootRomFile, boot_rom);
    end else begin
      for (int i = 0; i < BootRomDepth; i++) boot_rom[i] = 32'h00000013; // NOP
    end
  end

  localparam int unsigned RomAddrW = $clog2(BootRomDepth);
  logic [RomAddrW-1:0] instr_word_addr;
  assign instr_word_addr = instr_addr[RomAddrW+1 : 2];

  assign instr_gnt        = instr_req;   // always ready
  assign instr_rdata_intg = 7'h0;
  assign instr_err        = 1'b0;

  // Synchronous read — Vivado infers as BRAM; 1-cycle latency matches rvalid
  always_ff @(posedge clk_i) begin
    if (instr_req)
      instr_rdata <= boot_rom[instr_word_addr];
  end

  // rvalid 1 cycle after accepted request (gnt is always 1, so just req)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) instr_rvalid <= 1'b0;
    else         instr_rvalid <= instr_req;
  end

  // ── Data bus: tlul_adapter_host → crossbar ──────────────────────────────────
  logic        data_req;
  logic        data_gnt;
  logic        data_we;
  logic [3:0]  data_be;
  logic [31:0] data_addr;
  logic [31:0] data_wdata;
  logic [6:0]  data_wdata_intg;
  logic        data_rvalid;
  logic [31:0] data_rdata;
  logic [6:0]  data_rdata_intg;
  logic        data_err;

  tlul_adapter_host #(
    .MAX_REQS            (1),
    // Must be 1: Ibex with MemECC=0 drives wdata_intg_i=0, but any slave that
    // runs tlul_cmd_intg_chk (e.g. uart_reg_top) decodes data_intg
    // unconditionally and trips d_error when it doesn't match a_data. With
    // EnableDataIntgGen=1 the tlul_cmd_intg_gen wrapper inside the adapter
    // computes correct data_intg from a_data regardless of wdata_intg_i.
    // EnableRspDataIntgCheck stays 0 so we don't demand integrity on return
    // data; rsp_intg (always decoded by tlul_rsp_intg_chk) is a separate
    // field sourced from the slave side, fixed by the slave-side rsp_intg_gen.
    .EnableDataIntgGen   (1),
    .EnableRspDataIntgCheck (0)
  ) u_data_adapter (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .req_i         (data_req),
    .gnt_o         (data_gnt),
    .addr_i        (data_addr),
    .we_i          (data_we),
    .wdata_i       (data_wdata),
    .wdata_intg_i  (data_wdata_intg),
    .be_i          (data_be),
    .instr_type_i  (MuBi4False),       // data accesses are never instruction type
    .user_rsvd_i   ('0),
    .valid_o       (data_rvalid),
    .rdata_o       (data_rdata),
    .rdata_intg_o  (data_rdata_intg),
    .err_o         (data_err),
    .intg_err_o    (),                 // unused — MemECC=0
    .tl_o          (tl_o),
    .tl_i          (tl_i)
  );

  // ── ibex_top ────────────────────────────────────────────────────────────────
  /* verilator lint_off UNUSED */
  logic [6:0]  data_wdata_intg_shadow;
  logic        data_req_shadow;
  logic        data_we_shadow;
  logic [3:0]  data_be_shadow;
  logic [31:0] data_addr_shadow;
  logic [31:0] data_wdata_shadow;
  logic        instr_req_shadow;
  logic [31:0] instr_addr_shadow;

  logic        alert_minor;
  logic        alert_major_internal;
  logic        alert_major_bus;
  logic        double_fault;
  ibex_pkg::crash_dump_t crash_dump;

  prim_ram_1p_pkg::ram_1p_cfg_rsp_t [ibex_pkg::IC_NUM_WAYS-1:0] ram_cfg_rsp_icache_tag;
  prim_ram_1p_pkg::ram_1p_cfg_rsp_t [ibex_pkg::IC_NUM_WAYS-1:0] ram_cfg_rsp_icache_data;
  ibex_mubi_t lockstep_cmp_en;
  /* verilator lint_on UNUSED */

  ibex_top #(
    .PMPEnable          (1'b0),
    .RV32E              (1'b0),
    .RV32M              (ibex_pkg::RV32MFast),
    .RV32B              (ibex_pkg::RV32BNone),
    .RegFile            (ibex_pkg::RegFileFF),
    .BranchTargetALU    (1'b0),
    .WritebackStage     (1'b0),
    .ICache             (1'b0),
    .ICacheECC          (1'b0),
    .BranchPredictor    (1'b0),
    .DbgTriggerEn       (1'b0),
    .SecureIbex         (SecureIbex),
    .LockstepOffset     (1)
  ) u_ibex (
    .clk_i                       (clk_i),
    .rst_ni                      (rst_ni),
    .test_en_i                   (1'b0),
    .ram_cfg_icache_tag_i        ('0),
    .ram_cfg_rsp_icache_tag_o    (ram_cfg_rsp_icache_tag),
    .ram_cfg_icache_data_i       ('0),
    .ram_cfg_rsp_icache_data_o   (ram_cfg_rsp_icache_data),
    .hart_id_i                   (hart_id_i),
    .boot_addr_i                 (boot_addr_i),
    // Instruction fetch
    .instr_req_o                 (instr_req),
    .instr_gnt_i                 (instr_gnt),
    .instr_rvalid_i              (instr_rvalid),
    .instr_addr_o                (instr_addr),
    .instr_rdata_i               (instr_rdata),
    .instr_rdata_intg_i          (instr_rdata_intg),
    .instr_err_i                 (instr_err),
    // Data memory
    .data_req_o                  (data_req),
    .data_gnt_i                  (data_gnt),
    .data_rvalid_i               (data_rvalid),
    .data_we_o                   (data_we),
    .data_be_o                   (data_be),
    .data_addr_o                 (data_addr),
    .data_wdata_o                (data_wdata),
    .data_wdata_intg_o           (data_wdata_intg),
    .data_rdata_i                (data_rdata),
    .data_rdata_intg_i           (data_rdata_intg),
    .data_err_i                  (data_err),
    // Interrupts
    .irq_software_i              (irq_software_i),
    .irq_timer_i                 (irq_timer_i),
    .irq_external_i              (irq_external_i),
    .irq_fast_i                  ('0),
    .irq_nm_i                    (1'b0),
    // Scrambling (disabled)
    .scramble_key_valid_i        (1'b0),
    .scramble_key_i              ('0),
    .scramble_nonce_i            ('0),
    .scramble_req_o              (),
    // Debug
    .debug_req_i                 (1'b0),
    .crash_dump_o                (crash_dump),
    .double_fault_seen_o         (double_fault),
    // Fetch enable (IbexMuBiOn = 4'b0101)
    .fetch_enable_i              (ibex_pkg::IbexMuBiOn),
    .alert_minor_o               (alert_minor),
    .alert_major_internal_o      (alert_major_internal),
    .alert_major_bus_o           (alert_major_bus),
    .core_sleep_o                (core_sleep_o),
    // Lockstep (SecureIbex=0, these exist but are unused)
    .scan_rst_ni                 (1'b1),
    .lockstep_cmp_en_o           (lockstep_cmp_en),
    // Shadow data outputs (SecureIbex=0)
    .data_req_shadow_o           (data_req_shadow),
    .data_we_shadow_o            (data_we_shadow),
    .data_be_shadow_o            (data_be_shadow),
    .data_addr_shadow_o          (data_addr_shadow),
    .data_wdata_shadow_o         (data_wdata_shadow),
    .data_wdata_intg_shadow_o    (data_wdata_intg_shadow),
    .instr_req_shadow_o          (instr_req_shadow),
    .instr_addr_shadow_o         (instr_addr_shadow)
  );

endmodule
