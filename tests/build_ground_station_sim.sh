#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
proto_include="$root/embedded/shared_protocol/src"
proto_source="$proto_include/DTaskProtocol.cpp"
sim_source="$root/tests/ground_station_sim.cpp"
output="$root/tests/ground_station_sim"

g++ -std=c++17 -Wall -Wextra -Werror -Wno-error=cpp \
    -I "$proto_include" \
    -c "$proto_source" -o /tmp/dtask_protocol.o
g++ -std=c++17 -Wall -Wextra -Werror -Wno-error=cpp \
    -I "$proto_include" \
    -c "$sim_source" -o /tmp/ground_station_sim.o
g++ /tmp/dtask_protocol.o /tmp/ground_station_sim.o -o "$output"
echo "Simulator built: $output"
