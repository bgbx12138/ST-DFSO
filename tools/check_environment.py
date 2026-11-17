"""Check included or user-supplied environmental files without solving or downloading."""
from pathlib import Path
import argparse,hashlib,json

def main():
    p=argparse.ArgumentParser();p.add_argument('--historical-hashes',action='store_true')
    p.add_argument('--scenario',choices=[f'S{i}' for i in range(1,7)],help='Check only one paper scenario')
    a=p.parse_args()
    root=Path(__file__).resolve().parents[1];missing=[];different=[];seen=set()
    for scene in json.loads((root/'config/environment_inventory.json').read_text('utf-8')):
        if a.scenario and scene['scenario_id']!=a.scenario:continue
        for item in scene['required_files']:
            file=root/scene['directory']/item['name']
            if file in seen:continue
            seen.add(file)
            if not file.is_file():missing.append(file.relative_to(root).as_posix())
            elif a.historical_hashes and hashlib.sha256(file.read_bytes()).hexdigest()!=item['sha256']:different.append(file.relative_to(root).as_posix())
    print(json.dumps({'required_unique_files':len(seen),'missing':missing,'different_historical_hashes':different},indent=2))
    return 1 if missing or different else 0
if __name__=='__main__':raise SystemExit(main())
