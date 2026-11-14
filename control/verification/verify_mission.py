"""Independent recorder checks. These do not replace the unavailable S02 scorer."""
import csv,json,math,sys,xml.etree.ElementTree as ET
from pathlib import Path
from collections import deque
import numpy as np
from pose_stationarity import check_pose_window

def unit(x):return x/max(np.linalg.norm(x),1e-12)
def verify(run):
 run=Path(run);cfg=json.loads((run/'config.json').read_text())
 blocks=ET.parse(run/'machine_prepared.bsg').getroot().findall('Blocks/Block')
 guids=[b.get('guid') for b in blocks];types=[int(b.get('id')) for b in blocks]
 payload=set(cfg['roles']['payload_guids']);carrier=set(guids)-payload
 pg=cfg['roles']['payload_reference_guid'];cg=cfg['roles']['carrier_reference_guid']
 center=np.array([0.,-600.,0.]);extents=np.array([cfg['payload_bounds'][g] for g in sorted(payload)])
 bounds=json.loads((Path(__file__).parent/'all_block_bounds.json').read_text())
 carrier_extents=np.array([bounds[g]['radius'] for g in sorted(carrier)])
 phases={};qual=None;release=None;last=None;payload_commands=0;landed_commands=0;glide_thrust=0
 max_drift=0.;tail_drift=deque();terminal_powered_frames=0;terminal_thrust_integral=0.;previous_control_time=None
 for line in (run/'flight.jsonl').open():
  r=json.loads(line);last=r;t=r['t'];phase=r['phase'];phases.setdefault(phase,t)
  dt=0. if previous_control_time is None else t-previous_control_time
  previous_control_time=t
  if phase in ['terminal_brake','powered_return']:
   thrust=sum(v for g,k,v in r['wheel_sliders'] if k=='fthrust')
   terminal_powered_frames+=thrust>0;terminal_thrust_integral+=thrust*dt
  if qual is None and r['qualifying_start'] is not None:qual=r['qualifying_start']
  if release is None and r['release_time'] is not None:release=r['release_time']
  if release is not None:
   payload_commands+=any(g in payload for g,k in r['actions']) or any(g in payload and v!=0 for g,k,v in r['wheel_sliders'])
  if phase=='landed':
   landed_commands+=bool(r['actions']) or any(v!=0 for g,k,v in r['wheel_sliders'])
   tail_drift.append((t,r['anchor_drift']))
   while tail_drift and t-tail_drift[0][0]>12.5:tail_drift.popleft()
  if phase in ['glide','landed']:glide_thrust+=any(k=='fthrust' and v!=0 for g,k,v in r['wheel_sliders'])
  max_drift=max(max_drift,r['anchor_drift'])
 data=[];previous=None;origin=None;endpoint=None;qframes=0;missing_frames=0;total_frames=0
 min_radius=float('inf');min_separation=float('inf');max_gap=0.;max_point_speed=0.;fuel_rows=0;fuel_valid=0
 plane_u=None;plane_w=None;last_angle=None;progress=0.;gate=None;record_start=None;record_end=None
 payload_shape_max=0.;initial_distances=None
 physical_gap=float('inf');closest_gap_time=None;maximum_relative_speed=0.
 def process(rows):
  nonlocal previous,origin,endpoint,qframes,missing_frames,total_frames,min_radius,min_separation,max_gap,max_point_speed,plane_u,plane_w,last_angle,progress,gate,record_start,record_end,payload_shape_max,initial_distances
  nonlocal physical_gap,closest_gap_time,maximum_relative_speed
  t=float(rows[0]['simulation_time']);by={r['block_guid']:r for r in rows if r['x']}
  total_frames+=1;record_start=t if record_start is None else record_start;record_end=t
  if previous is not None:max_gap=max(max_gap,t-previous)
  previous=t
  if set(by)!=set(guids):missing_frames+=1
  def pos(g):return np.array([float(by[g][k]) for k in ['x','y','z']])-center
  if cg in by:
   endpoint=pos(cg)
   if origin is None:origin=unit(endpoint)
  if qual is None or t<qual or not payload.issubset(by) or not carrier.issubset(by):return
  qframes+=1
  ps=np.array([pos(g) for g in sorted(payload)]);cs=np.array([pos(g) for g in sorted(carrier)])
  min_radius=min(min_radius,float(np.min(np.linalg.norm(ps,axis=1)-extents)))
  pair_distances=np.linalg.norm(ps[:,None]-cs[None,:],axis=2)
  min_separation=min(min_separation,float(pair_distances.min()))
  gap=float((pair_distances-extents[:,None]-carrier_extents[None,:]).min())
  if gap<physical_gap:physical_gap=gap;closest_gap_time=t
  def speed(g):return math.sqrt(sum(float(by[g][k])**2 for k in ['vx','vy','vz']))
  maximum_relative_speed=max(maximum_relative_speed,max(speed(g) for g in payload)+max(speed(g) for g in carrier))
  distances=np.linalg.norm(ps[:,None]-ps[None,:],axis=2)
  if initial_distances is None:initial_distances=distances
  payload_shape_max=max(payload_shape_max,float(np.abs(distances-initial_distances).max()))
  for g,e in zip(sorted(payload),extents):
   v=np.array([float(by[g][k]) for k in ['vx','vy','vz']]);w=np.array([float(by[g][k]) for k in ['avx','avy','avz']])
   max_point_speed=max(max_point_speed,float(np.linalg.norm(v)+np.linalg.norm(w)*e))
  p=pos(pg);v=np.array([float(by[pg][k]) for k in ['vx','vy','vz']])
  if plane_u is None:plane_u=unit(p);plane_w=unit(np.cross(unit(np.cross(p,v)),plane_u))
  angle=math.atan2(float(p@plane_w),float(p@plane_u))
  if last_angle is not None:progress+=(angle-last_angle+math.pi)%(2*math.pi)-math.pi
  last_angle=angle
  if progress>=6*math.pi and gate is None:gate=t
 batch=[];sequence=None
 with (run/'telemetry.csv').open(newline='') as f:
  for r in csv.DictReader(f):
   fuel_rows+=1;fuel_valid+=r['fuel_valid']=='1'
   if sequence is not None and r['sample_sequence']!=sequence:process(batch);batch=[]
   sequence=r['sample_sequence'];batch.append(r)
 if batch:process(batch)
 temporal=max(10.,max_point_speed*max_gap/2+10*max_gap**2)
 lower=min_radius-temporal
 separation_temporal=maximum_relative_speed*max_gap/2+.2
 physical_gap_lower=physical_gap-separation_temporal
 landing_error=600*math.atan2(float(np.linalg.norm(np.cross(unit(endpoint),origin))),float(unit(endpoint)@origin)) if endpoint is not None else None
 structure={'pass':False,'reason':'Not landed'}
 if 'landed' in phases:structure=check_pose_window(run/'telemetry.csv',carrier,rotor_guids={g for g,t in zip(guids,types) if t==101})
 checks={'starting_block_is_payload':types[guids.index(pg)]==0 and pg in payload,
  'carrier_reference_excludes_payload':cg in carrier,
  'all_blocks_present_every_frame':missing_frames==0,'telemetry_gap_below_081':max_gap<.081,
  'whole_payload_recorded_radius_with_margin_above_1100':qframes>0 and lower>1100,
  'single_continuous_three_orbit_interval':gate is not None,'payload_shape_retained':qframes>0 and payload_shape_max<1.,
  'no_payload_control_after_release':payload_commands==0,
  'positive_whole_body_separation_with_temporal_margin':physical_gap_lower>0 and qframes>0,
  'landed': 'landed' in phases,'landing_error_below_1':landing_error is not None and landing_error<1.,
  'landed_outputs_all_zero':landed_commands==0,'glide_thrust_zero':glide_thrust==0,
  '30_seconds_after_touchdown':record_end-phases.get('landed',record_end)>=30,
  'all_retained_pose_stationarity':structure['pass'],
  'tail_carrier_geometry_retained':bool(tail_drift) and max(x[1] for x in tail_drift)<.4}
 result={'checks':checks,'local_mission_pass':all(checks.values()),'phase_first_times':phases,'record_start':record_start,
  'record_end':record_end,'frames':total_frames,'missing_block_frames':missing_frames,'maximum_sample_gap':max_gap,
  'payload_orbits_to_endpoint':progress/(2*math.pi),'third_orbit_time':gate,
  'minimum_payload_radius_lower_bound':lower if qframes else None,'temporal_margin':temporal,
  'payload_relative_distance_change':payload_shape_max,'minimum_payload_carrier_anchor_distance':min_separation if qframes else None,
  'original_30_unit_anchor_proxy':min_separation>30 and qframes>0,
  'minimum_whole_body_separation_bound':physical_gap_lower if qframes else None,
  'minimum_sampled_whole_body_separation_bound':physical_gap if qframes else None,
  'separation_temporal_margin':separation_temporal,'closest_geometry_time':closest_gap_time,
  'landing_error':landing_error,'carrier_structure_stationarity':structure,'maximum_carrier_anchor_drift':max_drift,
  'recovery_mode':'glide_then_powered_precision_return' if 'powered_return' in phases else 'glide_with_powered_terminal_braking' if 'terminal_brake' in phases else 'glide',
  'terminal_powered_frames':terminal_powered_frames,'terminal_sum_thrust_time_integral':terminal_thrust_integral,
  'fuel_data_status':'resource_data_missing' if fuel_valid==0 else 'requires_official_conversion',
  'resource_rows':fuel_rows,'fuel_valid_rows':fuel_valid,'F_0':None,'F_returned':None,'F_expended':None,
  'official_score':None,'official_planet_center_verified':False,'center_assumption':center.tolist(),
  'limitations':['Native geometry with conservative extensions; official scorer and mass/fuel tables unavailable.',
   'Continuous collision/joint/contact state is not exposed by the recorder; separation and deformation checks are conservative geometric proxies.']}
 (run/'mission_verification.json').write_text(json.dumps(result,indent=2));return result

if __name__=='__main__':
 result=verify(sys.argv[1]);print(json.dumps(result,indent=2));raise SystemExit(0 if result['local_mission_pass'] else 1)
