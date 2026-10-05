#!/usr/bin/env bash
# Re-download the exact tremordb subset used as this project's real-tremor input.
#
# Dataset : Effect of Deep Brain Stimulation on Parkinsonian Tremor (DBSPDTREMOR)
#           v1.0.0, PhysioNet
# Creators: Anne Beuter, Michele S. Titcombe, Leon Glass
# Cite    : Beuter A, Titcombe MS, Richer F, Gross C, Guehl D (2001). Effect of
#           deep brain stimulation on amplitude and frequency characteristics of
#           rest tremor in Parkinson's disease. Thalamus and Related Systems.
# Licence : PhysioNet Open Access
#
# WHAT IS PULLED AND WHY. Only the stimulation-OFF, medication-OFF records: that
# is Parkinsonian rest tremor with neither intervention acting on it, which is
# the most natural tremor the database contains and therefore the only condition
# whose waveform structure we are entitled to treat as untreated human tremor.
# It is 60 records over 14 subjects -- 14 in the primary condition (one per
# subject) and 46 in the washout series, recorded 15/30/45/60 minutes after
# stimulation was stopped. About 2.5 MB in total.
#
# Records are plain ASCII, one sample per line, 100 Hz, index-fingertip velocity
# from a laser transducer. Per-record full-scale range and units are in
# file_description.txt, which is fetched alongside.
set -euo pipefail
cd "$(dirname "$0")"

BASE="https://physionet.org/files/tremordb/1.0.0"

echo "Fetching documentation..."
for f in README RECORDS file_description.txt subject_description.txt SHA256SUMS.txt; do
  curl -sfL --max-time 120 "$BASE/$f" -o "$f" && echo "  $f"
done

# Directory layout is <condition><h|l>/, h = high-amplitude tremor group
# (subjects 1-8), l = low-amplitude group (subjects 9-16).
echo "Fetching signal subset (stimulation off, medication off)..."
mkdir -p raw
n=0
for cond in rof r15of r30of r45of r60of; do
  for grp in h l; do
    d="${cond}${grp}"
    # The index page is the authoritative list of what exists for this
    # condition; not every subject was tested under every one.
    for f in $(curl -sfL --max-time 120 "$BASE/$d/" \
               | grep -oE "[a-z]+[0-9]+${cond}\.(let|rit)" | sort -u); do
      curl -sfL --max-time 120 "$BASE/$d/$f" -o "raw/$f" && n=$((n+1))
    done
  done
done
echo "Fetched $n records into raw/"
