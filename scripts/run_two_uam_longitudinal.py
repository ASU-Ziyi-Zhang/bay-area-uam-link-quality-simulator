"""Run immutable, source-snapshotted two-UAM development experiments."""
import argparse
from dataclasses import asdict
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import traceback
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from uam_simulator.two_uam_longitudinal import resolve, simulate
from uam_simulator.research_provenance import execution_metadata
from verify_two_uam_longitudinal import verify


def write(path, obj):
    path.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


def run(config, output):
    if output.exists():
        raise FileExistsError("run directory already exists")
    spec=json.loads(config.read_text())
    if spec['study']!='two_uam_longitudinal_mechanisms' or spec['schema_version']!=1:
        raise ValueError('unsupported experiment')
    sources=sorted((ROOT/'src').rglob('*.py')) + [Path(__file__),
        ROOT/'scripts/verify_two_uam_longitudinal.py',ROOT/'tests/test_two_uam_longitudinal.py',
        ROOT/'tests/test_aks.py',ROOT/'pyproject.toml',
        ROOT/'research/dynamic-transitions/two-uam-longitudinal-protocol.md',config]
    records=[{'path':str(p.relative_to(ROOT)), 'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sources]
    output.mkdir(parents=True)
    with zipfile.ZipFile(output/'source_snapshot.zip','w',zipfile.ZIP_DEFLATED) as z:
        for p in sources:
            z.write(p,str(p.relative_to(ROOT)))
    execution=execution_metadata(ROOT,[str(Path(__file__).resolve()),'--config',str(config),'--output',str(output)])
    execution['source_snapshot_included']=True
    manifest={'status':'running','scientific_status':'draft_uncommitted_source',
        'execution':execution,'sources':records}
    write(output/'manifest.json',manifest)
    resolved_cases={case['id']:asdict(resolve(spec['parameters'],case.get('vehicle_overrides'))) for case in spec['cases']}
    write(output/'resolved_config.json',{**spec,'resolved_cases':resolved_cases})
    freeze=subprocess.run([sys.executable,'-m','pip','freeze'],capture_output=True,text=True)
    (output/'environment.txt').write_text(freeze.stdout+freeze.stderr)
    try:
        with (output/'run.log').open('w') as log:
            for case in spec['cases']:
                cfg=resolve(spec['parameters'],case.get('vehicle_overrides'))
                for mode in ('acc','aks'):
                    for dt in spec['numerics']['steps_s']:
                        result=simulate(case,cfg,mode,dt,spec['numerics']['post_action_observation_s'])
                        name=f"{case['id']}__{mode}__dt{dt:g}.json.gz"
                        with gzip.open(output/name,'wt') as stream:
                            json.dump(result,stream,allow_nan=False)
                        message=f"{name}: {result['request']['status']}, T={result['request']['duration_s']:.6f}"
                        print(message,flush=True);log.write(message+'\n');log.flush()
        report=verify(output)
        write(output/'validation.json',report)
        write(output/'summary.json',{'status':report['status'],'trace_count':report['trace_count'],
            'results':[r for r in report['results'] if r['dt_s']==min(spec['numerics']['steps_s'])],
            'failures':report['failures'],'refinement':report['refinement']})
        manifest['status']='validated_draft' if report['status']=='passed' else 'failed_validation'
        manifest['outputs']=[{'path':p.name,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
            for p in sorted(output.iterdir()) if p.is_file() and p.name!='manifest.json']
        write(output/'manifest.json',manifest)
        print(json.dumps({'status':manifest['status'],'failures':report['failures']},indent=2))
        return report['status']=='passed'
    except Exception:
        manifest['status']='failed_execution'
        manifest['error']=traceback.format_exc()
        write(output/'manifest.json',manifest)
        raise


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    raise SystemExit(0 if run(args.config.resolve(),args.output.resolve()) else 1)
