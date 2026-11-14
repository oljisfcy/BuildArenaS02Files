# Weight 30 baseline — landing precision still being tuned

The verified baseline is `runs/lattice_full_v4_retry`, run ID `5d0778e6-cc55-4c73-b679-54cea0f105a8`. It is not a complete pass of the requested landing error below 1.

- Machine: 139 blocks, official MCP export, SHA-256 `9b0944ec8c2b169041f9109883a1d225aaa6d9bd5b7636f194fc8861674cef51`.
- One passive payload assembly: Starting Block, one Double Wooden Block, five Grabbers, five Boulders, one Skate Wheel and three Braces. Native registry weight: **30.0**. The Starting Block remains in this payload after release.
- Carrier: 123 blocks, native registry weight **32.65**. Its fixed return reference is declared before launch and is not the Starting Block.
- Release: 131.32 s; independent qualification begins at 138.08 s, after anchor clearance exceeds 50.
- Official recorder: **15,280 frames**, 0.04–611.20 s, maximum gap 0.04 s; all 139 original GUIDs present in every frame.
- Independently recomputed orbital progress: **3.171979 revolutions**; third revolution at 582.76 s.
- Whole-payload conservative radial lower bound: **1553.644**, including individual native collider/connector bounds and 10 units of temporal allowance.
- Minimum payload/carrier anchor separation during qualification: **50.493**. Maximum change in payload pairwise anchor distances: 0.00643.
- Carrier touchdown: 309.68 s. **Landing error: 6.307374**, measured from the original launch direction, not a retrospectively chosen target.
- Zero thrust requests during glide. All control outputs zero after touchdown.
- All 123 retained parts passed the independent 12 s pose-window check. Filtered maximum position excursion: 0.001230; rigid pose/mount-axis rotation excursion: 0.20654 degrees. Internal rotor motion is reported separately; this is not the unavailable official recovery detector.
- Source binding audit passed all checks: exact machine input, official prepared file, controller, configuration and original block identities.
- Fuel data are missing: no valid native fuel resource rows. `F_0`, `F_returned`, `F_expended` and official efficiency score remain unset.

The frozen baseline controller, config, machine and histories are in `run_inputs/lattice_full_v4_retry`. Current `control/` files may contain subsequent tuning; do not substitute them for the baseline's frozen inputs.

Original successful weight 10 files remain untouched in `../payload10_20261113`.
