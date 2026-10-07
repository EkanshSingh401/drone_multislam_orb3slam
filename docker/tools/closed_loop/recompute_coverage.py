#!/usr/bin/env python3
"""recompute_coverage.py <results.jsonl> <out.jsonl> : redo coverage fields with the fixed cl_report.py."""
import sys, json, subprocess
out = open(sys.argv[2], 'w')
for l in open(sys.argv[1]):
    r = json.loads(l)
    p = subprocess.run(['python3', '/out/cl/cl_report.py', f"/out/cl/runs/{r['flight']}"], capture_output=True, text=True)
    c = json.loads([x for x in p.stdout.splitlines() if x.startswith('{')][-1])
    r = {k: v for k, v in r.items() if not k.startswith('known_m3@')}
    r.update(c)
    r['known_m3_per_m'] = round(c['known_m3_final'] / r['path_m'], 2) if r.get('path_m') else None
    out.write(json.dumps(r) + '\n')
