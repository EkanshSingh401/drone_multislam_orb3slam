#!/usr/bin/env bash
# run_batch.sh <listfile> <results.jsonl> : run flights listed as "name type world [ENV=val ...]" sequentially,
# summarizing each into results.jsonl (overnight protocol). Skips names already present.
LIST=$1; RES=$2
while read -r name type world envs; do
  [[ -z "$name" || "$name" == \#* ]] && continue
  grep -q "\"flight\": \"$name\"" "$RES" 2>/dev/null && continue
  free=$(df --output=avail -BG /out | tail -1 | tr -d ' G'); (( free < 50 )) && { echo "disk preflight: only ${free}G free on /out, stopping"; exit 5; }
  echo "=== $(date +%T) $name $type $world $envs"
  env WORLD=$world $envs /out/cl/cl_flight.sh $type /out/cl/runs/$name > /out/cl/runs/$name.log 2>&1
  bash -c "source /opt/ros/jazzy/setup.bash; source /root/ws_offboard_control/install/setup.bash; python3 /out/cl/flight_summary.py /out/cl/runs/$name $world" 2>/dev/null | grep '^{' >> "$RES"
  tail -1 "$RES"
done < "$LIST"
echo BATCHDONE
