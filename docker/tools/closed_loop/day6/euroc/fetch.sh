#!/usr/bin/env bash
# fetch.sh: EuRoC from the ETH Research Collection (DSpace bitstreams), politely: one probe every
# 20 min while rate-limited (HTTP 429), then each archive with a single resumable request.
cd /mnt/data/drone_sim_out/euroc/zip
UA='Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/128.0 Safari/537.36'
B=https://www.research-collection.ethz.ch/server/api/core/bitstreams
for p in "vicon_room1 02ecda9a-298f-498b-970c-b7c44334d880 6042263426" "vicon_room2 ea12bc01-3677-4b4c-853d-87c7870b8c44 6013384949" "machine_hall 7b2419c1-62b5-4714-b7f8-485e5fe3e5fe 12683729426"; do
  set -- $p
  while [[ $(stat -c %s $1.zip 2>/dev/null || echo 0) -lt $3 ]]; do
    code=$(curl -sS -L -A "$UA" -r 0-0 -o /dev/null -w '%{http_code}' "$B/$2/content")
    echo "$(date +%T) $1 probe $code"
    if [[ $code == 206 || $code == 200 ]]; then
      curl -sS -L -A "$UA" -C - -o $1.zip "$B/$2/content"; echo "$(date +%T) $1 size $(stat -c %s $1.zip)"
      head -c 4 $1.zip | grep -q PK || { echo "$1 not a zip"; rm -f $1.zip; }
    fi
    [[ $(stat -c %s $1.zip 2>/dev/null || echo 0) -lt $3 ]] && sleep 1200
  done
done
echo FETCHDONE
