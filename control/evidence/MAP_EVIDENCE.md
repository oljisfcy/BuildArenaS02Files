# Installed map identity and planet center

Read-only examination on 2026-11-13, using UnityPy 1.25.4 and TypeTreeGeneratorAPI 0.0.10. No game asset was changed.

The native `globalgamemanagers` BuildSettings scene array identifies index 13 as `Assets/Scenes/SANDBOXES/LONE ORB.unity`. The corresponding `level13` LevelAttributes component, path ID 57185, has `sandBoxLevel = 1` and `levelNameLocalisationIndex = 5282`. The installed English localization resolves 5282 to `ASTRO PLAYGROUND` (Chinese: 星际游乐场). This establishes the scene-name alias directly.

The launch-world main gravity object `(moon)`, GameObject path ID 9335 and GravityController path ID 55002, has local position `(0, -600, 0)`. Its three ancestors have zero translation, identity rotation and unit scale. Its world center is exactly `(0, -600, 0)`. The body's own rotation and scale do not move that origin. Native gravity is 22 and gravitational reference radius is 620, giving a nominal inverse-square parameter of 8,456,800. The gravitational reference radius is not the scoring surface radius.

Raw native fields, transform ancestry, scene identity and asset SHA-256 are retained in `diagnostics/native_planet_evidence.json`. The extraction scripts only read installed files. The runner reloads this native sandbox for each attempt.

This resolves the previously unknown native scene alias and native main-body center. The separate S02 scorer, official mass/fuel tables and recovery detector have still not been obtained. These findings do not turn missing fuel resources into measured zero or authorize an official score claim.

Reader documentation: https://github.com/K0lb3/UnityPy
