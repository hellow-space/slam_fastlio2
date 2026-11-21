#!/usr/bin/env bash
# Usage: bash scripts/run_trial.sh <tag> -- <command> [args...]
set -uo pipefail
if (( $# < 3 )) || [[ "$2" != "--" ]]; then
  echo 'Usage: bash scripts/run_trial.sh <tag> -- <command> [args...]' >&2
  exit 2
fi
tag="${1//[^a-zA-Z0-9._-]/_}"
shift 2
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
run_dir="$repo_root/experiments/runs/${stamp}_${tag}_$$"
mkdir -p "$run_dir"
printf '%q ' "$@" > "$run_dir/command.txt"
printf '\n' >> "$run_dir/command.txt"
(git -C "$repo_root" rev-parse HEAD || true) > "$run_dir/git_sha.txt" 2>/dev/null
(git -C "$repo_root" status --porcelain || true) > "$run_dir/git_status.txt" 2>/dev/null
start="$(date +%s)"
start_utc="$(date -u +%FT%TZ)"
"$@" 2>&1 | tee "$run_dir/output.log"
rc=${PIPESTATUS[0]}
end="$(date +%s)"
end_utc="$(date -u +%FT%TZ)"
python3 - "$run_dir/metadata.json" "$start_utc" "$end_utc" "$((end-start))" "$rc" "$tag" <<'PYMETA'
import json,sys
path,start,end,dur,code,tag=sys.argv[1:]
with open(path,'w',encoding='utf-8') as f:
    json.dump({'tag':tag,'started_utc':start,'ended_utc':end,'wall_seconds':int(dur),'exit_code':int(code)},f,indent=2)
PYMETA
echo "Raw evidence: $run_dir"
exit "$rc"
