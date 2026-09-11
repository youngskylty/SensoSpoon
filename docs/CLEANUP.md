# Project cleanup — 2026-09-11

The active project now contains one physical Simulink model and one top-level
entry point. Source is grouped by responsibility. Tests, manufacturer PDFs,
generated geometry and runtime output have separate locations. Runtime code
uses project-root paths, so MATLAB's current folder does not determine where
assets or results are read and written. Simulink cache is redirected to output/cache.

Removed from active use: legacy 3D/RC-Flex models, old prescribed-motion and
predictive-controller workflows, duplicate launcher wrappers, revision archives,
old rendered videos/results, temporary downloads and cache folders. The direct
linkage function retains the current equations and drops its unused bellcrank
implementation. Obsolete controller/surrogate parameters were removed. The
robustness analysis now calculates its nominal case instead of loading an
unversioned result left by an earlier geometry.

The machine-readable move/removal list is CLEANUP_MANIFEST.json. Before the
cleanup, 189 source/model/document/assembly files were saved into
`backup/before_cleanup_20260911.zip` (ZIP integrity checked). This is a recovery
snapshot, not a complete copy of old generated media and temporary caches.
Extract it into a separate folder if historical work is needed; do not add it
to the active MATLAB path.

## Validation

- Twelve deterministic pre-cleanup states and base/voltage inputs were compared
  with the reorganized plant. Maximum absolute difference across all state
  derivatives and physical outputs was zero.
- All nine current validation groups passed after moving the files.
- The shell clearance check also passed over ±17 degrees.
- Model rebuilding, full voltage simulation and MP4 export are checked from
  output/cache as the MATLAB current folder, exercising absolute-path handling.

The cleanup changes organization and removes inactive code; it does not tune
an algorithm or alter the accepted mechanical design.
