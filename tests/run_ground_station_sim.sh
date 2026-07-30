#!/usr/bin/env bash
set -euo pipefail

readonly repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
readonly run_id="$(date -u +%Y%m%dT%H%M%SZ)-$$-ground-station-sim"
readonly evidence_relative=".omo/evidence/ground-station-sim/$run_id"
readonly evidence_dir="$repo_root/$evidence_relative"
mkdir -p "$evidence_dir"

record_failure() {
    local exit_code="$1"
    if ((exit_code != 0)); then
        printf 'GROUND_STATION_SIM_RED exit_code=%s\n' "$exit_code" >"$evidence_dir/FAILED"
    fi
}
trap 'record_failure "$?"' EXIT

readonly sim="$repo_root/tests/ground_station_sim"
readonly scenarios=(ping nominal stale_car auth_mismatch sequence_replay boot_id_change sequence_wrap)
readonly duration_ms=8000
failed=0

{
    printf 'command=%s\n' "${BASH_SOURCE[0]}"
    printf 'run_id=%s\n' "$run_id"
    printf 'duration_ms=%s\n' "$duration_ms"
    printf 'scenarios='
    printf '%s ' "${scenarios[@]}"
    printf '\n'
} >"$evidence_dir/command.txt"

for scenario in "${scenarios[@]}"; do
    log_file="$evidence_dir/$scenario.log"
    printf -- '--- %s ---\n' "$scenario" | tee -a "$evidence_dir/runner.log"
    if "$sim" --scenario "$scenario" --duration-ms "$duration_ms" 2>&1 | tee "$log_file"; then
        printf -- '  PASS: %s\n' "$scenario" | tee -a "$evidence_dir/runner.log"
    else
        printf -- '  FAIL: %s\n' "$scenario" | tee -a "$evidence_dir/runner.log"
        failed=$((failed + 1))
    fi
done

if ((failed == 0)); then
    printf 'GROUND_STATION_SIM_GREEN\n' | tee "$evidence_dir/SUCCESS"
    printf 'All %d scenarios passed.\n' "${#scenarios[@]}" | tee -a "$evidence_dir/runner.log"
else
    printf '%d scenario(s) failed.\n' "$failed" | tee -a "$evidence_dir/runner.log"
    exit 1
fi
