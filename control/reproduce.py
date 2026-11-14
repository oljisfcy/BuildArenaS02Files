"""Load the packaged machine and run its exact controller with official BuildArena."""
import argparse,hashlib,json,os,shutil,subprocess,sys,tempfile
from datetime import datetime
from pathlib import Path

HERE=Path(__file__).resolve().parent;PACKAGE=HERE.parent
def sha(path):
 with Path(path).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def find_repo(explicit):
 candidates=[explicit,os.environ.get('BUILDARENA_ROOT')]
 for base in [Path.cwd(),*Path.cwd().parents,PACKAGE,*PACKAGE.parents]:candidates.extend([base,base/'BuildArena-2.0'])
 for candidate in candidates:
  if candidate and (Path(candidate)/'control/besiege_cli/run.py').is_file():return Path(candidate).resolve()
 raise RuntimeError('Use --buildarena-root PATH or BUILDARENA_ROOT for the installed official checkout.')
def main():
 parser=argparse.ArgumentParser(description=__doc__)
 parser.add_argument('--buildarena-root',type=Path);parser.add_argument('--besiege-data',type=Path)
 parser.add_argument('--output',type=Path);parser.add_argument('--check',action='store_true');parser.add_argument('--record',action='store_true')
 opt=parser.parse_args();repo=find_repo(opt.buildarena_root)
 binding=json.loads((HERE/'evidence/submission_binding.json').read_text())
 for name,expected in binding['artifacts'].items():
  if sha(PACKAGE/name)!=expected:raise RuntimeError('Artifact differs from the selected flight: '+name)
 for name,expected in binding['official_sources'].items():
  if sha(repo/name)!=expected:raise RuntimeError('Official dependency differs from tested source: '+name)
 sys.path[:0]=[str(repo),str(repo/'control')]
 from buildarena.paths import load_project_env
 load_project_env()
 from besiege_cli.cli import build_parser
 from besiege_cli.paths import resolve_besiege_data
 from besiege_cli.run import cmd_run
 from besiege_cli.machine import prepare_machine_bsg
 data=resolve_besiege_data(opt.besiege_data)
 cfg=json.loads((HERE/'config.json').read_text())
 target=(opt.output or PACKAGE.parent/'payload30_replays'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')).resolve()
 if target.exists():raise FileExistsError(target)
 with tempfile.TemporaryDirectory(prefix='buildarena_payload30_') as folder:
  layout=Path(folder);controller=layout/'mission/stage/control/controller.py'
  registry=layout/'BuildArena-2.0/blocks/block_registry.generated.toml'
  controller.parent.mkdir(parents=True);registry.parent.mkdir(parents=True)
  shutil.copy2(HERE/'controller.py',controller);shutil.copy2(repo/'blocks/block_registry.generated.toml',registry)
  assert sha(controller)==binding['artifacts']['control/controller.py']
  arguments=['run','--bsg',str(PACKAGE/'machine.bsg'),'--controller',str(controller),'--run-dir',str(target),
   '--sandbox','LONE ORB','--reload-sandbox','--telemetry-hz','25','--telemetry-profile','full',
   '--controller-prestart','--pre-controller-hold','0','--post-completion-hold','2',
   '--controller-timeout','1800','--timeout','45','--besiege-data',str(data)]
  args=build_parser().parse_args(arguments)
  if opt.record:
   args.record=True;args.camera_follow=cfg['roles']['payload_reference_guid'];args.camera_distance=18;args.camera_pitch=18
  if opt.check:
   print(json.dumps({'check':'passed','game_started':False,'selected_run_id':binding['run_id'],'controller_unchanged':True}));return 0
  target.parent.mkdir(parents=True,exist_ok=True)
  os.environ['STAGE3_CONFIG']=json.dumps(cfg)
  code=cmd_run(args)
  if code:return int(code)
 for name in ['build_history.json','build_history_full.json']:shutil.copy2(PACKAGE/name,target/name)
 shutil.copy2(HERE/'evidence/prelaunch_declaration.json',target/'prelaunch_declaration.json')
 manifest=json.loads((target/'input_manifest.json').read_text())
 audit_file=target/'audit_reprepared.bsg'
 prepare_machine_bsg(PACKAGE/'machine.bsg',audit_file,run_id=manifest['run_id'],recorder_enabled=True,recorder_hz=25)
 checks={'machine_matches_selected':manifest['source_sha256']==binding['artifacts']['machine.bsg'],
  'controller_matches_selected':sha(target/'controller_snapshot.py')==binding['artifacts']['control/controller.py'],
  'config_matches_selected':json.loads((target/'config.json').read_text())==cfg,
  'official_reprepare_matches_runtime':sha(audit_file)==sha(target/'machine_prepared.bsg')}
 (target/'source_binding_audit.json').write_text(json.dumps({'checks':checks,'pass':all(checks.values()),'run_id':manifest['run_id']},indent=2))
 if not all(checks.values()):raise RuntimeError('Replay source audit failed')
 return subprocess.run([sys.executable,str(HERE/'verification/verify_mission.py'),str(target)]).returncode
if __name__=='__main__':raise SystemExit(main())
