# Vyges additions — opentitan-rv-core-ibex

Vyges-authored files shipped alongside the upstream `rtl/` mirror.
Excluded from upstream-sync pulls; declared in
`vyges-metadata.json → vyges_additions[]` and vendored automatically
by the Vyges SoC Generator.

## Files

| File | Purpose |
|---|---|
| `rv_core_ibex_tlul.sv` | Thin TL-UL adapter around upstream `ibex_top`. Bridges Ibex's native OBI-style instruction + data interfaces to TL-UL host ports so the core attaches cleanly to an `opentitan-tlul`-based crossbar. Instantiates `tlul_adapter_host` on the data side and infers a synchronous BRAM for instruction fetch (FPGA: `$readmemh`-preloaded; ASIC: hard-macro ROM). Also includes the `EnableDataIntgGen=1` setting on the data adapter so outgoing writes carry correct SECDED `data_intg` (Fix F from the 2026-04-18 FPGA banner bring-up). |

See top-level README for the signed vs baseline TL-UL domain
contract; `rv_core_ibex_tlul` is **signed** on its TL-UL host
output and expects a signed crossbar downstream.
