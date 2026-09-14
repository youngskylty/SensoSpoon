#!/usr/bin/env bash
# Re-download the exact GRABMyo subset used as this project's real-sEMG test case.
#
# Dataset : GrabMyo - Large-Sample and Multiday Surface Electromyogram Recordings
#           of Hand Gestures, v1.1.0 (PhysioNet)
# DOI     : https://doi.org/10.13026/rzbf-ts96
# Licence : Creative Commons Attribution 4.0 International (CC BY 4.0)
#
# The full dataset is 9.4 GB. This pulls ~12 MB: two participants, session 1,
# three gestures (wrist flexion, wrist extension, rest), three trials each.
# That is everything the stages GRABMyo can actually answer need, and nothing
# more -- see docs/SEMG_DATASET.md.
set -euo pipefail
cd "$(dirname "$0")"

BASE="https://physionet.org/files/grabmyo/1.1.0"

echo "Fetching documentation..."
for f in LICENSE.txt readme.txt subject-info.csv Electrodelocation.pdf GestureList.JPG MotionSequence.txt; do
  curl -sfL --max-time 120 "$BASE/$f" -o "$f" && echo "  $f"
done

# gesture 11 = wrist flexion, 12 = wrist extension, 17 = rest
echo "Fetching signal subset..."
for p in 1 2; do
  d="raw/session1_participant${p}"
  mkdir -p "$d"
  for g in 11 12 17; do
    for t in 1 2 3; do
      n="session1_participant${p}_gesture${g}_trial${t}"
      for e in hea dat; do
        curl -sfL --max-time 120 "$BASE/session1_participant${p}/${n}.${e}" -o "$d/${n}.${e}"
      done
      echo "  $n"
    done
  done
done

echo
echo "Done. $(find raw -name '*.dat' | wc -l) records, $(du -sh raw | cut -f1) total."
echo "Convert with:  S = import_grabmyo('data/external/grabmyo', 1, 1, C);"
