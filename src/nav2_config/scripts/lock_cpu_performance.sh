#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "This script must run as root. Try: sudo ros2 run nav2_config lock_cpu_performance.sh" >&2
  exit 1
fi

for cpu in /sys/devices/system/cpu/cpu[0-7]; do
  cpufreq="${cpu}/cpufreq"
  [[ -d "${cpufreq}" ]] || continue

  if [[ -w "${cpufreq}/scaling_governor" ]]; then
    echo performance > "${cpufreq}/scaling_governor"
  fi

  if [[ -r "${cpufreq}/cpuinfo_max_freq" && -w "${cpufreq}/scaling_max_freq" ]]; then
    max_freq="$(<"${cpufreq}/cpuinfo_max_freq")"
    echo "${max_freq}" > "${cpufreq}/scaling_max_freq"
    if [[ -w "${cpufreq}/scaling_min_freq" ]]; then
      echo "${max_freq}" > "${cpufreq}/scaling_min_freq"
    fi
  fi
done

for cpu in /sys/devices/system/cpu/cpu[0-7]/cpufreq; do
  [[ -d "${cpu}" ]] || continue
  printf '%s governor=%s cur=%s max=%s min=%s\n' \
    "$(basename "$(dirname "${cpu}")")" \
    "$(<"${cpu}/scaling_governor")" \
    "$(<"${cpu}/scaling_cur_freq")" \
    "$(<"${cpu}/scaling_max_freq")" \
    "$(<"${cpu}/scaling_min_freq")"
done
