import os,json,math,tomllib
from pathlib import Path
from dataclasses import asdict
import numpy as np
import xml.etree.ElementTree as ET
from controller_sdk import ControllerClient
from controller_sdk.protocol import atomic_write_json

def mul(a,b):
    a=np.asarray(a);b=np.asarray(b)
    return np.r_[a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3],b[:3]),a[3]*b[3]-np.dot(a[:3],b[:3])]
def conj(q):return np.r_[-np.asarray(q)[:3],q[3]]
def rotate(q,v):return mul(mul(q,np.r_[v,0]),conj(q))[:3]
def axisq(axis,angle):return np.r_[np.asarray(axis)*math.sin(angle/2),math.cos(angle/2)]
def basisq(right,up,forward):
 m=np.column_stack([right,up,forward]);tr=np.trace(m)
 if tr>0:
  s=math.sqrt(tr+1)*2;q=np.array([(m[2,1]-m[1,2])/s,(m[0,2]-m[2,0])/s,(m[1,0]-m[0,1])/s,s/4])
 else:
  i=int(np.argmax(np.diag(m)));j=(i+1)%3;k=(j+1)%3;s=math.sqrt(1+m[i,i]-m[j,j]-m[k,k])*2
  q=np.zeros(4);q[i]=s/4;q[j]=(m[j,i]+m[i,j])/s;q[k]=(m[k,i]+m[i,k])/s;q[3]=(m[k,j]-m[j,k])/s
 return q/np.linalg.norm(q)

def elements(p,v,mu):
 r=np.linalg.norm(p);h=np.cross(p,v);e=np.linalg.norm(np.cross(v,h)/mu-p/r)
 parameter=np.dot(h,h)/mu
 return float(parameter/(1+e)),float(parameter/(1-e)) if e<1 else None,float(e)

def unit(v):return v/max(np.linalg.norm(v),1e-12)

def great_circle_heading(normal,plane_normal):return unit(np.cross(plane_normal,normal))

def surface_error(radial_position,original_direction):
 normal=unit(radial_position)
 return 600*math.atan2(float(np.linalg.norm(np.cross(normal,original_direction))),float(normal@original_direction))

def terminal_acceleration(error,horizontal_velocity,height,radial_speed,gravity):
 # Cubic endpoint guidance: reach the fixed horizontal target at rest near
 # first contact. Recompute from observed motion; actuator saturation is below.
 remaining=max(height-4.,0.)
 t=max(.12,min(3.,(radial_speed+math.sqrt(radial_speed**2+2*gravity*remaining))/gravity))
 acceleration=6*error/t**2-4*horizontal_velocity/t
 if np.linalg.norm(error)<.15 and np.linalg.norm(horizontal_velocity)<.5:acceleration=np.zeros(3)
 return acceleration,t

def signed_rcs_key(key,sign):
 return key if sign>=0 else 'RightKey' if key=='LeftKey' else 'LeftKey'

def powered_return_acceleration(error,velocity,normal,height,target_height,gravity):
 horizontal=velocity-normal*(velocity@normal)
 goal=unit(error)*min(30.,.35*np.linalg.norm(error))
 lateral=.9*(goal-horizontal)
 lateral*=min(1.,20./max(np.linalg.norm(lateral),1e-9))
 vertical_goal=float(np.clip(.25*(target_height-height),-20.,15.))
 return lateral+normal*max(1.,gravity+1.*(vertical_goal-velocity@normal))

def differential_thrust(collective,positions,torque):
 matrix=np.stack([np.ones(len(positions)),positions[:,1],-positions[:,0]])
 adjustment=matrix.T@np.linalg.pinv(matrix@matrix.T)@np.array([0.,torque[0]/9.,torque[1]/9.])
 return np.clip(collective+adjustment,0.,3.)

def resolve_roles(types,guids,roles):
 payload=[guids.index(g) for g in roles['payload_guids']]
 retained=[i for i in range(len(types)) if i not in payload]
 ref=guids.index(roles['carrier_reference_guid'])
 release=[guids.index(g) for g in roles['release_guids']]
 # Explicit passive assembly allowlist; do not silently include a propulsion part.
 if types.index(0) not in payload or ref not in retained or not release:raise ValueError('Invalid root/carrier roles')
 if any(types[i] not in [0,1,15,24,27,36,7,86] for i in payload):raise ValueError('Non-passive payload block')
 if any(i not in retained or types[i]!=4 for i in release):raise ValueError('Release must be carrier decoupler')
 return payload,retained,ref,release

def whole_payload_lower_bound(positions,extents,temporal_margin):
 return float(np.min(np.linalg.norm(positions,axis=1)-extents)-temporal_margin)

def complete_mission(stable,progress,minimum_radius,error):
 return bool(stable and progress>=6*math.pi+.01 and minimum_radius>1100 and error<1.)

def wait_control_table(client,frame):
 start=frame.simulation_time
 while not client.block_table_path.exists():
  if frame.simulation_time-start>3:raise TimeoutError('Official control table was not published')
  frame=client.next_sample(timeout=3)
 client.load_block_table()
 return frame

def carrier_attitude(records,sensors,relative,reference):
 candidates=np.array([mul(records[i]['rotation'].astype(float),conj(r)) for i,r in zip(sensors,relative)])
 candidates/=np.linalg.norm(candidates,axis=1)[:,None]
 _,vectors=np.linalg.eigh(candidates.T@candidates);q=vectors[:,-1]
 return q if q@reference>=0 else -q

def grabber_release_actions(guids,grab_indices):
 # Native GrabberBlock registers attach first, detach second. This installed
 # official keylist version labels index 1 "attachKey", opposite the native
 # registration order. In native auto-grab mode only index 1 releases.
 return [(guids[i],'attachKey') for i in grab_indices]

def advance_payload_angles(positions,plane_u,plane_w,last_phi,progress):
 phi=np.arctan2(np.sum(positions*plane_w,axis=1),np.sum(positions*plane_u,axis=1))
 delta=(phi-last_phi+math.pi)%(2*math.pi)-math.pi
 return phi,progress+delta

def payload_clearance(payload_positions,carrier_positions,envelope):
 lower=float(np.linalg.norm(payload_positions,axis=1).min()-envelope)
 separation=float(np.linalg.norm(payload_positions[:,None,:]-carrier_positions[None,:,:],axis=2).min())
 pair=float('inf')
 if len(payload_positions)>1:
  distances=np.linalg.norm(payload_positions[:,None,:]-payload_positions[None,:,:],axis=2)
  pair=float(distances[np.triu_indices(len(payload_positions),1)].min())
 return lower,separation,pair

def carrier_encounters(carrier_position,carrier_velocity,payload_positions,payload_velocities):
 # A short linear lookahead is a conservative warning, not an orbit proof.
 # Never act on a departing payload immediately after release.
 result=[]
 for p,v in zip(payload_positions,payload_velocities):
  relative=carrier_position-p;velocity=carrier_velocity-v
  distance=float(np.linalg.norm(relative));rate=float(relative@velocity/max(distance,1e-9))
  closest=float(np.clip(-(relative@velocity)/max(float(velocity@velocity),1e-9),0.,60.))
  miss=float(np.linalg.norm(relative+closest*velocity))
  result.append({'distance':distance,'separation_rate':rate,'closest_time':closest,
                 'closest_distance':miss,'threat':bool(distance<180 and rate<-.3 and miss<40)})
 return result

def bank_offset_estimate(measured,commanded,previous,dt):
 # Native aerodynamic moments require a persistent attitude offset. Estimate
 # the observed offset, rather than waiting for growing lateral miss distance.
 observed=float(np.clip(measured-commanded,-math.radians(25),math.radians(25)))
 return previous+(1-math.exp(-dt/1.))*(observed-previous)

def touchdown_candidate(radius,velocity,drift):
 # Terrain is not a sphere: stop attitude actuation on low-altitude slowdown.
 # This is only a shutdown trigger; the subsequent unpowered stationary
 # interval is independently measured, not claimed as official contact.
 return 597<radius<680 and np.linalg.norm(velocity)<3 and drift<.4

def touchdown_impact(height,previous_radial_speed,radial_speed,dt,drift):
 # Mass-weighted carrier velocity suppresses anchor motion. A fast upward
 # reversal near terrain is an impact cue, including while sliding forward.
 return bool(previous_radial_speed is not None and -3<height<20 and 0<dt<=.3
             and previous_radial_speed<-3 and radial_speed>1.5
             and radial_speed-previous_radial_speed>8 and drift<.4)

def terminal_pitch(height,remaining,speed,radial_speed):
 # Native wings lose range by increasing incidence/drag. The v13 flight
 # demonstrated that pitching down accelerated past the intended landing.
 sink=-speed*max(height-1,0)/max(remaining,25.)
 sink=float(np.clip(sink,-35.,-3.))
 if height<12:sink=max(sink,-5.)
 return math.radians(float(np.clip(7.-.7*(sink-radial_speed),-8.,35.)))

def root_target_remaining(carrier_position,carrier_velocity,pad,root_position):
 # The trajectory model advances the centroid, while the landing reference is
 # the original root. Account for its physical forward lever at touchdown.
 # Measured attitude estimates that lever; the pad itself is never changed.
 normal=unit(carrier_position)
 tangent=unit(carrier_velocity-normal*(carrier_velocity@normal))
 centroid_remaining=600*math.atan2(float(tangent@pad),float(normal@pad))
 return centroid_remaining-float((root_position-carrier_position)@tangent)

def predicted_range_pitch(height,vr,vt,remaining,mu,current_pitch,bank=0.,pitch_bias=0.,wing_scale=1.,drag_scale=1.,aero_table=None,ground_deceleration=None,minimum_command_pitch=-30.):
 # Parallel approximate trajectories; coefficients measured from v10-v13.
 # Prediction is guidance only. Recorder poses decide actual acceptance.
 heights=np.arange(25,500,50)
 wing=np.array([.000374804,.000320795,.000162710,.0000530265,.0000256083,.0000142889,.00000792615,.00000356367,.00000125494,.000000237289])
 drag=np.array([.0000135108,.0000109513,.00000866233,.00000503748,.00000296261,.00000211005,.000001493,.00000100963,.000000589806,.000000241160])
 if aero_table:
  heights=np.array([row['height'] for row in aero_table]);wing=np.array([row['wing'] for row in aero_table],dtype=float);drag=np.array([row['body_drag'] for row in aero_table],dtype=float)
 wing*=wing_scale;drag*=drag_scale
 choices=np.clip(np.radians(np.arange(0,41,5))-pitch_bias,math.radians(minimum_command_pitch),math.radians(45));r=np.full(9,height+600.);u=np.full(9,vr,dtype=float);v=np.full(9,vt,dtype=float);a=np.zeros(9);live=np.ones(9,dtype=bool);dt=.1
 for j in range(500):
  pitch=current_pitch+(choices+pitch_bias-current_pitch)*min(1.,j*dt/.8)
  sa=np.sin(pitch)*math.cos(bank);ca=np.cos(pitch)*math.cos(bank)
  c=np.interp(r-600,heights,wing);d=np.interp(r-600,heights,drag);vn=u*ca-v*sa;s=np.minimum(u*u+v*v,10000.)
  ar=-mu/r**2+v*v/r-c*vn*ca*s-d*u*s;at=-u*v/r+c*vn*sa*s-d*v*s
  u+=ar*dt*live;v+=at*dt*live;r+=u*dt*live;a+=v/r*dt*live
  live &= r>601.
  if not live.any():break
 ranges=a*600
 if ground_deceleration is not None:
  ranges+=np.maximum(v,0.)**2/(2*ground_deceleration)
 best=int(np.argmin(np.abs(ranges-remaining)))
 # Interpolate bracketing outcomes to avoid jumping between 5-degree steps.
 for j in range(len(choices)-1):
  if (ranges[j]-remaining)*(ranges[j+1]-remaining)<=0 and abs(ranges[j+1]-ranges[j])>1e-6:
   f=(remaining-ranges[j])/(ranges[j+1]-ranges[j]);return float(choices[j]*(1-f)+choices[j+1]*f)
 return float(choices[best])

def pose_still(positions,rotations,reference_positions,reference_rotations,rotating_indices=()):
 rotations=rotations/np.linalg.norm(rotations,axis=1)[:,None]
 reference_rotations=reference_rotations/np.linalg.norm(reference_rotations,axis=1)[:,None]
 angles=2*np.arccos(np.clip(np.abs(np.sum(rotations*reference_rotations,axis=1)),-1,1))
 for i in rotating_indices:
  # Internal rotor spin is reported separately; its mounting axis and every
  # hardware anchor must remain stable, just as the external structure does.
  angles[i]=math.acos(float(np.clip(rotate(rotations[i],[0,0,1])@rotate(reference_rotations[i],[0,0,1]),-1,1)))
 return bool(np.linalg.norm(positions-reference_positions,axis=1).max()<.03 and angles.max()<math.radians(1))

def return_prediction(radius,radial_speed,perigee,mu):
 # Instantaneous tangential braking, retaining radial velocity. Predict the
 # descending surface crossing of that orbit, before atmospheric forces.
 tangent_speed=math.sqrt((radial_speed**2+2*mu*(1/perigee-1/radius))/(radius**2/perigee**2-1))
 parameter=(radius*tangent_speed)**2/mu
 ec=parameter/radius-1;es=radial_speed*radius*tangent_speed/mu
 eccentricity=math.hypot(ec,es);anomaly=math.atan2(es,ec)
 crossing=2*math.pi-math.acos(float(np.clip((parameter/600-1)/eccentricity,-1,1)))
 return tangent_speed,(crossing-anomaly)%(2*math.pi)

out=Path(os.environ['BUILDARENA_RUN_DIR']);cfg=json.loads(os.environ['STAGE3_CONFIG'])
blocks=ET.parse(os.environ['BUILDARENA_MACHINE_BSG']).getroot().findall('Blocks/Block')
types=[int(b.get('id')) for b in blocks];guids=[b.get('guid') for b in blocks]
rotations=[np.array([float(b.find('Transform/Rotation').get(k)) for k in ('x','y','z','w')]) for b in blocks]
payload_indices,retained,rootidx,releases=resolve_roles(types,guids,cfg['roles'])
payloadidx=types.index(0);wheels=[i for i in retained if types[i]==101]
frame_offset=np.array(cfg['roles']['carrier_attitude_offset_xyzw'])
rotations[rootidx]=mul(rotations[rootidx],frame_offset)
payload_extents=np.array([cfg['payload_bounds'][guids[i]] for i in payload_indices])
bombs=[i for i,t in enumerate(types) if t==54];rcs=[i for i,t in enumerate(types) if t==91];grabs=[i for i,t in enumerate(types) if t==27]
registry=tomllib.loads((Path(__file__).resolve().parents[3]/'BuildArena-2.0/blocks/block_registry.generated.toml').read_text())['runtime_defs']
attached=[i for i,t in enumerate(types) if t!=54]
weights=np.array([registry[str(types[i])]['weight'] for i in attached]);weights/=weights.sum()
return_weights=np.array([registry[str(types[i])]['weight'] for i in retained]);return_weights/=return_weights.sum()
build_positions=np.array([[float(b.find('Transform/Position').get(k)) for k in 'xyz'] for b in blocks])
build_center=sum(build_positions[i]*w for i,w in zip(retained,return_weights))
body_positions=np.array([rotate(conj(rotations[rootidx]),p-build_center) for p in build_positions])
body_inertia=sum(registry[str(types[i])]['weight']*((p@p+.1)*np.eye(3)-np.outer(p,p)) for i,p in enumerate(body_positions) if i in retained)
axes=[rotate(conj(rotations[rootidx]),rotate(rotations[i],[0,0,1])) for i in wheels]
rcskeys=['LeftKey' if rotate(conj(rotations[rootidx]),rotate(rotations[i],[0,1,0]))[2]>0 else 'RightKey' for i in rcs]
rigid_sensors=[i for i in retained if types[i] in [15,25,91] and i!=rootidx]
sensor_relative=[mul(conj(rotations[rootidx]),rotations[i]) for i in rigid_sensors]
center=np.array([0.,-600.,0.]);mu=cfg.get('mu',8.3e6);rtarget=cfg.get('target_radius',1400.)
launch_azimuth=math.radians(cfg.get('launch_azimuth_degrees',0.))
launch_heading=np.array([math.sin(launch_azimuth),0.,math.cos(launch_azimuth)])
client=ControllerClient.from_environment(poll_interval=.005);client.arm()
ready=os.environ.get('BUILDARENA_CONTROLLER_READY')
if ready:atomic_write_json(Path(ready),{'schema':'buildarena.controller_ready.v1','run_id':os.environ['BUILDARENA_RUN_ID']})
phase='blast';phase_start=0.;coast_samples=[];fit=None;integral=np.zeros(3);filtered=None;target=None;previous=None
direction_states=[0]*len(wheels);last_print=-1;blast_seen=False;initial_local=None;frozen_target=None;feedback_q=None
release_time=None;qual_start=None;progress=0.;last_phi=0.;plane_u=None;plane_w=None
payload_progress=np.zeros(1)
pad=None;original_pad=None;launch_plane_normal=None;return_started=None;deorbit_started=None;landing_stable_start=None
stable_positions=None;stable_rotations=None
filtered_positions=None;filtered_rotations=None
navigation_error=None;navigation_rate=0.
range_guidance_time=-999.;range_guidance_pitch=math.radians(7)
range_pitch_bias=0.
range_bank_bias=0.
range_measured_bank=0.
avoidance_index=None;avoidance_start=None;avoidance_count=0
contact_history=[];payload_local_initial=None;landing_error=None
initial_reference_height=None;settling_target=None;return_bias=np.zeros(3)
minimum_payload_radius=float('inf');on_seconds=np.zeros(len(rcs));command_integral=np.zeros(len(rcs));stop_reason=None
try:
 frame=wait_control_table(client,client.wait_until_running(timeout=30))
 atomic_write_json(out/'runtime_sliders.json',[asdict(s) for s in client.sliders]);atomic_write_json(out/'config.json',cfg)
 ranges={(s.block_guid,s.name):(s.minimum,s.maximum) for s in client.sliders}
 with (out/'flight.jsonl').open('w') as f:
  while True:
   t=frame.simulation_time;rs=frame.records;by={int(r['target_index']):r for r in rs};root=by[rootidx];payload=by[payloadidx]
   if not root['valid'] or any(not by[i]['valid'] for i in payload_indices):stop_reason='root_or_payload_lost';break
   q=mul(root['rotation'].astype(float),frame_offset);q/=np.linalg.norm(q);omega=root['angular_velocity'].astype(float)
   raw_reference_q=q.copy()
   if cfg.get('consensus_frame',False):q=carrier_attitude(by,rigid_sensors,sensor_relative,q)
   pos=root['position'].astype(float);vel=root['velocity'].astype(float);rad=pos-center;radius=np.linalg.norm(rad);normal=rad/radius
   if pad is None:
    initial_reference_height=radius-600.
    original_pad=normal.copy()
    # Navigation aim bias compensates measured approach dispersion. The
    # independent recorder verifier always measures the original launch site.
    aim_offset=np.array(cfg.get('guidance_aim_offset',[0.,0.,0.]),dtype=float)
    aim_offset-=normal*(normal@aim_offset)
    pad=unit(normal+aim_offset/600.)
   if launch_plane_normal is None:launch_plane_normal=unit(np.cross(normal,launch_heading))
   heading=great_circle_heading(normal,launch_plane_normal);right=unit(np.cross(normal,heading))
   vr=float(vel@normal);vt=float(vel@heading);dt=0 if previous is None else min(.1,max(0,t-previous));previous=t
   if target is None:target=q.copy()
   local=np.array([rotate(conj(q),by[i]['position']-pos) for i in retained])
   if initial_local is None:initial_local=local.copy()
   drift=float(np.linalg.norm(local-initial_local,axis=1).max())
   blast_seen=blast_seen or any(not by[i]['valid'] for i in bombs)
   mean_pos=np.sum([by[i]['position']*w for i,w in zip(attached,weights)],axis=0)
   mean_vel=np.sum([by[i]['velocity']*w for i,w in zip(attached,weights)],axis=0)
   perigee,apogee,ecc=elements(mean_pos-center,mean_vel,mu) if release_time is None else elements(rad,vel,mu)
   pp_all=np.array([by[i]['position'].astype(float)-center for i in payload_indices])
   pv_all=np.array([by[i]['velocity'].astype(float) for i in payload_indices])
   pp=pp_all[0];pv=pv_all[0]
   pq=by[payloadidx]['rotation'].astype(float)
   payload_local=np.array([rotate(conj(pq),p-pp) for p in pp_all])
   if payload_local_initial is None:payload_local_initial=payload_local.copy()
   payload_drift=float(np.linalg.norm(payload_local-payload_local_initial,axis=1).max())
   carrier_p=np.sum([by[i]['position']*w for i,w in zip(retained,return_weights)],axis=0)-center
   carrier_v=np.sum([by[i]['velocity']*w for i,w in zip(retained,return_weights)],axis=0)
   carrier_r=np.linalg.norm(carrier_p);carrier_n=unit(carrier_p);carrier_vr=float(carrier_v@carrier_n)
   payload_lower,separation,payload_pair_distance=payload_clearance(pp_all,np.array([by[i]['position']-center for i in retained if by[i]['valid']]),cfg.get('payload_envelope',2.))
   payload_lower=whole_payload_lower_bound(pp_all,payload_extents,cfg.get('payload_temporal_margin',10.))
   landing_error=surface_error(rad,original_pad)
   throttle=0.;thrust_sign=1;terminal_diagnostic=None;desired_forward=rotate(target,[0,0,1]);actions=[]
   avoidance_active=False;encounters=[]
   if cfg.get('fire',True) and t<1:
    actions += [(guids[i],'DetonateKey') for i in bombs if by[i]['valid']]
   if phase=='blast' and t>=cfg.get('assist_start',.6) and (blast_seen or not bombs):
    phase='ascent';phase_start=t
   if phase=='ascent':
    if radius<cfg.get('turn_radius',1000.):
     demand=normal*30.
    else:
     circ=math.sqrt(mu/rtarget)
     vrgoal=float(np.clip((rtarget-radius)*cfg.get('ascent_vertical_gain',.2),-45,cfg.get('ascent_vertical_cap',80)))
     vtgoal=circ*float(np.clip((radius-cfg.get('tangent_ramp_start',950))/cfg.get('tangent_ramp_span',350),0,1))
     demand=.35*((vrgoal-vr)*normal+(vtgoal-vt)*heading-(vel@right)*right)+(mu/radius**2-vt**2/radius)*normal
    desired_forward=unit(demand)
    throttle=min(3.,np.linalg.norm(demand)/cfg.get('accel_per_thrust',3.5))
    if perigee>cfg.get('coast_perigee',1170) and apogee is not None and apogee<cfg.get('coast_apogee_max',1800) and radius>1150:
     phase='coast_check';phase_start=t;coast_samples=[];throttle=0.
   if phase=='coast_check':
    # Weighted anchor motion suppresses rotation about the root. These are
    # native-registry weights, not an official mass or exact center-of-mass claim.
    mean_pos=np.sum([by[i]['position']*w for i,w in zip(attached,weights)],axis=0)
    mean_vel=np.sum([by[i]['velocity']*w for i,w in zip(attached,weights)],axis=0)
    throttle=0.;coast_samples.append((t,mean_pos,mean_vel))
    if t-phase_start>=2.:
     fit_samples=[s for s in coast_samples if s[0]-phase_start>.3]
     ts=np.array([s[0] for s in fit_samples]);vs=np.array([s[2] for s in fit_samples]);ps=np.array([s[1] for s in fit_samples])
     design=np.c_[ts-ts.mean(),np.ones(len(ts))];coeff=np.linalg.lstsq(design,vs,rcond=None)[0];acc=coeff[0]
     rmean=ps.mean(axis=0)-center;estimated_mu=float(-acc@unit(rmean)*np.dot(rmean,rmean))
     residual=float(np.sqrt(np.mean((design@coeff-vs)**2)))
     fit={'time':t,'mu':estimated_mu,'velocity_fit_rms':residual,'samples':len(ts),'method':'native-weighted anchor motion, pre-release only'}
     if 3e6<estimated_mu<2e7 and residual<1.:
      mu=estimated_mu;perigee,apogee,ecc=elements(mean_pos-center,mean_vel,mu)
      payload_perigee=min(elements(p,v,mu)[0] for p,v in zip(pp_all,pv_all))
      if perigee>cfg.get('release_perigee_min',1160) and apogee is not None and apogee<cfg.get('release_apogee_max',1900) and payload_perigee>cfg.get('release_payload_perigee_min',0):
       phase='release';phase_start=t;release_time=t
      else:phase='ascent';phase_start=t
     else:phase='ascent';phase_start=t
   if phase=='release':
    throttle=0.
    if t-phase_start<.16:actions += [(guids[i],'ExplodeKey') for i in releases]
    if t-phase_start>cfg.get('release_wait',.6):phase='escape';phase_start=t
   if phase=='escape':
    throttle=cfg.get('escape_thrust',1.) if t-phase_start<cfg.get('escape_duration',1.5) else 0.
    if t-phase_start>=cfg.get('escape_duration',1.5):
     phase='clearance';phase_start=t
   if phase=='clearance':
    throttle=0.
    if separation>50 and payload_lower>1100 and t-phase_start>1:
     phase='return_coast';phase_start=t;qual_start=t
     plane_u=np.array([unit(pp)]);plane_w=np.array([unit(np.cross(unit(np.cross(pp,pv)),plane_u[0]))])
     last_phi=np.zeros(1);payload_progress=np.zeros(1);progress=0.
    if t-phase_start>cfg.get('clearance_timeout',20.):stop_reason='separation_not_confirmed'
   if qual_start is not None:
    last_phi,payload_progress=advance_payload_angles(np.array([pp]),plane_u,plane_w,last_phi,payload_progress);progress=float(payload_progress.min())
    minimum_payload_radius=min(minimum_payload_radius,payload_lower)
    if payload_lower<=1100:stop_reason='payload_radius_violation'
    if separation<=10:stop_reason='possible_recontact'
    if payload_drift>.5:stop_reason='payload_assembly_fragmentation'
    if phase=='return_coast' and cfg.get('carrier_avoidance',False):
     encounters=carrier_encounters(carrier_p,carrier_v,pp_all,pv_all)
     if avoidance_index is None:
      threats=[i for i,e in enumerate(encounters) if e['threat']]
      if threats:
       avoidance_index=min(threats,key=lambda i:encounters[i]['closest_time'])
       avoidance_start=t;avoidance_count+=1
     if avoidance_index is not None:
      encounter=encounters[avoidance_index]
      if encounter['separation_rate']>=3.:
       avoidance_index=None
      else:
       avoidance_active=True
       desired_forward=unit(carrier_p-pp_all[avoidance_index])
       # Only existing carrier actuators are used; payload channels stay empty.
       candidate_right=np.cross(carrier_n,desired_forward)
       if np.linalg.norm(candidate_right)>.1:right=unit(candidate_right)
       acceleration=float(np.clip((3.5-encounter['separation_rate'])*.5,0.,4.))
       throttle=acceleration/(9*len(rcs)/sum(registry[str(types[i])]['weight'] for i in retained))
       if t-avoidance_start>30.:stop_reason='carrier_avoidance_timeout'
   if phase in ['return_coast','return_align']:
    orbit_axis=unit(np.cross(carrier_p,carrier_v));projected_pad=unit(pad-orbit_axis*(pad@orbit_axis))
    angle_to_pad=math.atan2(orbit_axis@np.cross(carrier_n,projected_pad),carrier_n@projected_pad)%(2*math.pi)
    # Two measured landings put the needed lead near 157 deg at r=1386 and
    # beyond 180 deg at r=1549. Cap at the opposite meridian.
    lead_deg=cfg.get('deorbit_lead_degrees',float(np.clip(157+cfg.get('deorbit_lead_slope',.16)*(radius-1386),145,179)))
    lead=math.radians(lead_deg)
    if cfg.get('predictive_return',False):
     _,ballistic_angle=return_prediction(carrier_r,carrier_vr,cfg.get('return_perigee',540),mu)
     lead=ballistic_angle+math.radians(cfg.get('aero_lead_degrees',18)+cfg.get('burn_lead_degrees',6))
     desired_forward=-unit(carrier_v-carrier_n*carrier_vr)
     right=unit(np.cross(carrier_n,desired_forward))
     if lead<angle_to_pad<lead+math.radians(cfg.get('return_align_lead_degrees',22)):phase='return_align'
    if lead-.07<angle_to_pad<lead:
     phase='deorbit';phase_start=t;return_started=t
   if phase=='deorbit':
    deorbit_started=deorbit_started or t
    desired_forward=-unit(vel-normal*vr)
    right=unit(np.cross(normal,desired_forward))
    throttle=cfg.get('deorbit_thrust',3.)
    perigee,apogee,ecc=elements(rad,vel,mu)
    if cfg.get('predictive_return',False):
     desired_forward=-unit(carrier_v-carrier_n*carrier_vr)
     right=unit(np.cross(carrier_n,desired_forward))
     perigee,apogee,ecc=elements(carrier_p,carrier_v,mu)
    if perigee<cfg.get('return_perigee',540.):
     phase='glide';phase_start=t;throttle=0.
   if phase=='glide':
    throttle=0.
    tangent=vel-normal*vr
    forward_motion=unit(tangent) if np.linalg.norm(tangent)>2 else heading
    right=unit(np.cross(normal,forward_motion))
    alpha=math.radians(15 if radius<640 else 7)
    if cfg.get('terminal_descent_guidance',False) and radius<780:
     remaining=600*math.acos(float(np.clip(normal@pad,-1,1)))
     alpha=terminal_pitch(radius-600,remaining,np.linalg.norm(tangent),vr)
    if cfg.get('predictive_glide_guidance',False) and radius<820:
     pitch=math.asin(float(np.clip(rotate(q,[0,0,1])@carrier_n,-1,1)))
     if cfg.get('compensate_pitch_bias',False):
      commanded_pitch=math.asin(float(np.clip(rotate(target,[0,0,1])@carrier_n,-1,1)))
      bias_limit=math.radians(cfg.get('pitch_bias_limit_degrees',30.))
      observed_bias=float(np.clip(pitch-commanded_pitch,-bias_limit,bias_limit))
      range_pitch_bias+=(1-math.exp(-dt/1.))*(observed_bias-range_pitch_bias)
     if t-range_guidance_time>=.4:
      remaining=600*math.atan2(float(unit(carrier_v-carrier_n*carrier_vr)@pad),float(carrier_n@pad))
      if cfg.get('root_reference_correction',False):
       remaining=root_target_remaining(carrier_p,carrier_v,pad,rad)
      cvt=np.linalg.norm(carrier_v-carrier_n*carrier_vr)
      range_measured_bank=math.asin(float(np.clip(rotate(q,[1,0,0])@carrier_n,-1,1)))
      range_guidance_pitch=predicted_range_pitch(carrier_r-600,carrier_vr,cvt,remaining,mu,pitch,bank=range_measured_bank,pitch_bias=range_pitch_bias,wing_scale=cfg.get('wing_force_scale',1.),drag_scale=cfg.get('body_drag_scale',1.),aero_table=cfg.get('aero_table'),ground_deceleration=cfg.get('ground_deceleration'),minimum_command_pitch=cfg.get('minimum_command_pitch',-30.))
      range_guidance_time=t
     alpha=range_guidance_pitch
    desired_forward=unit(forward_motion*math.cos(alpha)+normal*math.sin(alpha))
    if cfg.get('glide_lateral_guidance',False) and radius<1000 and np.linalg.norm(tangent)>8:
     pad_heading=unit(pad-normal*(pad@normal))
     heading_error=math.atan2(right@pad_heading,forward_motion@pad_heading)
     if navigation_error is not None and dt>0:
      change=(heading_error-navigation_error+math.pi)%(2*math.pi)-math.pi
      navigation_rate+=(1-math.exp(-dt/.4))*(change/dt-navigation_rate)
     navigation_error=heading_error
     bank=float(np.clip(1.4*heading_error+cfg.get('navigation_derivative_gain',0.)*navigation_rate,-math.radians(30),math.radians(30)))
     if cfg.get('compensate_bank_bias',False):
      measured_bank=math.asin(float(np.clip(rotate(q,[1,0,0])@normal,-1,1)))
      commanded_bank=math.asin(float(np.clip(rotate(target,[1,0,0])@normal,-1,1)))
      range_bank_bias=bank_offset_estimate(measured_bank,commanded_bank,range_bank_bias,dt)
      # The navigation bank sign is opposite right-axis radial elevation.
      bank=float(np.clip(bank+range_bank_bias,-math.radians(55),math.radians(55)))
     # Bank the lift direction toward the pad; retain velocity alignment.
     right=unit(right*math.cos(bank)-normal*math.sin(bank))
    contact_history=[s for s in contact_history if 0<t-s[0]<=.24]
    previous_contact_vr=min((s[1] for s in contact_history),default=None)
    contact_dt=t-contact_history[0][0] if contact_history else 0.
    impact=cfg.get('impact_shutdown',False) and touchdown_impact(carrier_r-600,previous_contact_vr,carrier_vr,contact_dt,drift)
    if touchdown_candidate(radius,vel,drift) or impact:
     phase='landed';phase_start=t;throttle=0.
    elif cfg.get('terminal_air_brake',False) and carrier_r<600+cfg.get('terminal_start_height',50) and carrier_vr<-2:
     phase='powered_return' if cfg.get('powered_precision_return',False) else 'terminal_brake';phase_start=t
   if phase=='powered_return':
    error=600*original_pad-rad;error-=normal*(error@normal)
    horizontal=carrier_v-normal*(carrier_v@normal)
    if settling_target is None and landing_error<.4 and np.linalg.norm(horizontal)<.3 and abs(radius-635)<2:
     settling_target=35.
    if settling_target is not None and landing_error<.6:
     settling_target=max(initial_reference_height-.3,settling_target-.6*dt)
    height_goal=35. if settling_target is None else settling_target
    acceleration=powered_return_acceleration(error,carrier_v,normal,radius-600,height_goal,mu/radius**2)
    if landing_error<20 and radius<680:
     return_bias+=.025*dt*(error+normal*(height_goal-(radius-600)))
     return_bias*=min(1.,5./max(np.linalg.norm(return_bias),1e-9))
    acceleration+=return_bias
    desired_forward=unit(acceleration)
    right=unit(np.cross(original_pad,launch_heading))
    accel_unit=9*len(rcs)/sum(registry[str(types[i])]['weight'] for i in retained)
    throttle=min(3.,np.linalg.norm(acceleration)/accel_unit)
    terminal_diagnostic={'position_error':error.tolist(),'acceleration':acceleration.tolist(),'height_goal':height_goal,'settling':settling_target is not None}
    if settling_target is not None and radius-600<initial_reference_height-.12 and np.linalg.norm(vel)<1.5 and landing_error<.7:
     phase='landed';phase_start=t;throttle=0.
   if phase=='terminal_brake':
    error=600*pad-rad;error-=carrier_n*(error@carrier_n)
    horizontal=carrier_v-carrier_n*carrier_vr
    acceleration,contact_time=terminal_acceleration(error,horizontal,carrier_r-600,carrier_vr,mu/carrier_r**2)
    forward_now=rotate(q,[0,0,1])
    if np.linalg.norm(acceleration)>5:
     desired_forward=unit(acceleration)
     if desired_forward@forward_now<0:desired_forward=-desired_forward
    else:desired_forward=unit(forward_now-carrier_n*(forward_now@carrier_n))
    right=unit(np.cross(carrier_n,desired_forward))
    axial=float(acceleration@forward_now);thrust_sign=1 if axial>=0 else -1
    accel_unit=9*len(rcs)/sum(registry[str(types[i])]['weight'] for i in retained)
    throttle=min(3.,abs(axial)/accel_unit)
    terminal_diagnostic={'position_error':error.tolist(),'acceleration':acceleration.tolist(),'time_to_contact':contact_time,'thrust_sign':thrust_sign}
    contact_history=[s for s in contact_history if 0<t-s[0]<=.24]
    previous_contact_vr=min((s[1] for s in contact_history),default=None)
    contact_dt=t-contact_history[0][0] if contact_history else 0.
    impact=touchdown_impact(carrier_r-600,previous_contact_vr,carrier_vr,contact_dt,drift)
    if touchdown_candidate(radius,vel,drift) or impact:
     phase='landed';phase_start=t;throttle=0.
   if phase=='landed':
    throttle=0.
    poses=np.array([by[i]['position'] for i in retained]);rotors=np.array([by[i]['rotation'] for i in retained])
    if filtered_positions is None:filtered_positions=poses.copy();filtered_rotations=rotors.copy()
    smoothing=1-math.exp(-dt/.2)
    filtered_positions+=smoothing*(poses-filtered_positions)
    signs=np.where(np.sum(rotors*filtered_rotations,axis=1)<0,-1.,1.)
    filtered_rotations+=smoothing*(rotors*signs[:,None]-filtered_rotations)
    filtered_rotations/=np.linalg.norm(filtered_rotations,axis=1)[:,None]
    if stable_positions is None or not pose_still(filtered_positions,filtered_rotations,stable_positions,stable_rotations,[retained.index(i) for i in wheels]):
     stable_positions=filtered_positions.copy();stable_rotations=filtered_rotations.copy();landing_stable_start=t
    stable=(-3<radius-600<80 and drift<.4 and all(by[i]['valid'] for i in retained) and t-landing_stable_start>=12.6 and t-phase_start>=30)
    if complete_mission(stable,progress,minimum_payload_radius,landing_error):
     stop_reason='three_orbit_and_return_proxy_pass'
    elif stable and landing_error>=1.:
     stop_reason='landing_error_exceeded'
   if phase in ['return_coast','return_align','deorbit','glide','landed'] and payload_lower<=1100:
    stop_reason='payload_radius_violation_after_three_orbits'
   controls_off=(phase in ['clearance','return_coast','landed'] or (phase=='qualifying' and not avoidance_active)) or t>=cfg.get('diagnostic_off_at',1e9)
   if cfg.get('ascent_free_roll',False) and phase in ['ascent','coast_check','release','escape']:
    right=rotate(q,[1,0,0])
   desired_right=unit(right-desired_forward*(right@desired_forward))
   desired=basisq(desired_right,unit(np.cross(desired_forward,desired_right)),desired_forward)
   if desired@target<0:desired=-desired
   delta_angle=2*math.acos(float(np.clip(abs(desired@target),-1,1)))
   slew=cfg.get('terminal_slew_rate',1.2) if phase=='powered_return' else cfg.get('target_slew_rate',.65)
   blend=min(1.,slew*dt/max(delta_angle,1e-9))
   target=unit((1-blend)*target+blend*desired)
   if t>=cfg.get('diagnostic_hold_at',1e9):
    if frozen_target is None:frozen_target=q.copy()
    target=frozen_target.copy();throttle=0.
   raw_err=mul(target,conj(q));raw_err*=1 if raw_err[3]>=0 else -1;angle=2*math.acos(float(np.clip(raw_err[3],-1,1)))
   measured_q=q;measured_omega=omega
   if cfg.get('spatial_attitude',False):
    candidates=np.array([mul(by[i]['rotation'].astype(float),conj(relative)) for i,relative in zip(rigid_sensors,sensor_relative)])
    candidates/=np.linalg.norm(candidates,axis=1)[:,None]
    _,vectors=np.linalg.eigh(candidates.T@candidates);measured_q=vectors[:,-1]
    if measured_q@q<0:measured_q=-measured_q
    measured_omega=np.mean([by[i]['angular_velocity'] for i in rigid_sensors],axis=0)
   tau=cfg.get('attitude_filter_tau',0.)
   if feedback_q is None or tau<=0:feedback_q=measured_q.copy()
   else:
    observed_q=measured_q if measured_q@feedback_q>=0 else -measured_q
    alpha=1-math.exp(-dt/tau)
    feedback_q=unit((1-alpha)*feedback_q+alpha*observed_q)
   err=mul(target,conj(feedback_q));err*=1 if err[3]>=0 else -1
   body_rate=rotate(conj(q),measured_omega)
   if filtered is None:filtered=body_rate.copy()
   filtered+=(1-math.exp(-dt/.16))*(body_rate-filtered)
   body_rate[1:]=filtered[1:];feedback_rate=rotate(q,body_rate)
   rate_gain=cfg.get('glide_rate_gain',1.) if phase in ['glide','terminal_brake','powered_return'] else 1.
   attitude_gain=16.
   if phase=='powered_return':attitude_gain=cfg.get('powered_attitude_gain',8.);rate_gain=cfg.get('powered_rate_gain',4.)
   command=attitude_gain*err[:3]-rate_gain*feedback_rate
   differential_torque=None
   if phase=='powered_return' and cfg.get('differential_landing_thrust',False):
    body_error=rotate(conj(q),2*err[:3]);body_omega=rotate(conj(q),feedback_rate)
    angular_acceleration=np.clip(2.*body_error-2.5*body_omega,-1.,1.)
    differential_torque=body_inertia@angular_acceleration
   sliders=[(guids[i],'fusedelay',0.) for i in bombs]
   for j,(i,axis) in enumerate(zip(wheels,axes)):
    u=-float(command@rotate(q,axis));active=j in cfg.get('enabled_wheels',[0,1,2])
    if differential_torque is not None and abs(axis[2])<.5:active=False
    if not active or controls_off:u=0.
    engage=cfg.get('axis_engage_threshold',[.5,.5,.05,.05])[j];release=cfg.get('axis_release_threshold',[.2,.2,.05,.05])[j]
    direction=(1 if u>0 else -1) if abs(u)>engage else direction_states[j] if direction_states[j]*u>release else 0
    direction_states[j]=direction
    scales=cfg.get('glide_axis_speed_scale',cfg.get('axis_speed_scale',[.25,.25,1,1])) if phase in ['glide','terminal_brake','powered_return'] else cfg.get('axis_speed_scale',[.25,.25,1,1])
    speed=min(.8,abs(u)*.2)*scales[j] if direction else 0.
    if direction:actions.append((guids[i],'LeftKey' if direction>0 else 'RightKey'))
    damping=cfg.get('landing_passive_damping',0.) if phase=='landed' else 0. if controls_off else 1.
    sliders += [(guids[i],'speed',speed),(guids[i],'damper',damping)]
   alignment=float(rotate(q,[0,0,1])@desired_forward)
   if differential_torque is not None:throttle=max(.3,throttle*max(.1,alignment))
   elif alignment<math.cos(math.radians(cfg.get('thrust_alignment_degrees',35))):throttle=0.
   if controls_off:throttle=0.;actions=[]
   jet_throttles=differential_thrust(throttle,body_positions[rcs],differential_torque) if differential_torque is not None else np.full(len(rcs),throttle)
   for j,i in enumerate(rcs):
    jet=float(jet_throttles[j]);sliders.append((guids[i],'fthrust',jet))
    if jet>0:
     actions.append((guids[i],signed_rcs_key(rcskeys[j],thrust_sign)));on_seconds[j]+=dt;command_integral[j]+=dt*jet
   for g,k,v in sliders:
    if not ranges[(g,k)][0]<=v<=ranges[(g,k)][1]:raise ValueError('Illegal slider request')
   client.send_sliders(sliders=sliders);sent=client.send_channels(channels=actions)
   records=[{n:r[n].tolist() for n in rs.dtype.names} for r in rs]
   row={'t':t,'sequence':frame.sequence,'applied':frame.sequence_applied,'sent':sent,'phase':phase,'actions':actions,'wheel_sliders':sliders,'error_deg':math.degrees(angle),'target':target.tolist(),'estimated_height':radius-600,'radial_speed':vr,'tangential_speed':vt,'controls_off':controls_off,'blast_seen':blast_seen,'mu_estimate':mu,'gravity_fit':fit,'perigee_estimate':perigee,'apogee_estimate':apogee,'eccentricity_estimate':ecc,'anchor_drift':drift,'payload_min_radius_lower_bound':payload_lower,'payload_carrier_min_anchor_distance':separation,'release_time':release_time,'qualifying_start':qual_start,'orbital_progress_radians':progress,'minimum_qualifying_radius_lower_bound':minimum_payload_radius if math.isfinite(minimum_payload_radius) else None,'auxiliary_on_seconds':on_seconds.tolist(),'auxiliary_thrust_time_integral':command_integral.tolist(),'machine':frame.machine.as_dict(),'records':records}
   row['range_pitch_bias_degrees']=math.degrees(range_pitch_bias)
   row['range_guidance_pitch_degrees']=math.degrees(range_guidance_pitch)
   row['range_bank_bias_degrees']=math.degrees(range_bank_bias)
   row['range_measured_bank_degrees']=math.degrees(range_measured_bank)
   row['payload_progress_radians_each']=payload_progress.tolist()
   row['payload_guids']=[guids[i] for i in payload_indices]
   row['payload_pair_distance']=payload_pair_distance if math.isfinite(payload_pair_distance) else None
   row['carrier_avoidance_active']=avoidance_active
   row['carrier_avoidance_count']=avoidance_count
   row['carrier_avoidance_payload_index']=avoidance_index
   row['carrier_encounters']=encounters
   row['carrier_reference_guid']=guids[rootidx]
   row['landing_error']=landing_error
   row['payload_assembly_drift']=payload_drift
   row['terminal_guidance']=terminal_diagnostic
   row['thrust_sign']=thrust_sign
   row['reference_to_consensus_degrees']=math.degrees(2*math.acos(float(np.clip(abs(raw_reference_q@q),-1,1))))
   f.write(json.dumps(row)+'\n');f.flush()
   if int(t/5)!=last_print:
    last_print=int(t/5);print(json.dumps({k:row[k] for k in ['t','phase','estimated_height','radial_speed','tangential_speed','error_deg','perigee_estimate','apogee_estimate','orbital_progress_radians']}),flush=True)
   if any(not by[i]['valid'] for i in retained) or drift>1.5:stop_reason='carrier_fragmentation'
   if t>1. and payload_drift>2.:stop_reason='payload_assembly_fragmentation'
   if t>8 and radius<605 and phase in ['blast','ascent','coast_check','release','escape']:stop_reason='ascent_failed'
   if t>=cfg.get('duration',450):stop_reason=stop_reason or 'time_budget'
   if (out/'stop_requested.json').exists():stop_reason='diagnostic_stop_requested'
   if stop_reason:break
   if phase in ['glide','terminal_brake']:contact_history.append((t,carrier_vr))
   frame=client.next_sample(timeout=3)
 atomic_write_json(out/'outcome.json',{'stop_reason':stop_reason,'phase':phase,'orbital_progress_radians':progress,'minimum_qualifying_radius_lower_bound':minimum_payload_radius if math.isfinite(minimum_payload_radius) else None,'auxiliary_on_seconds':on_seconds.tolist(),'auxiliary_thrust_time_integral':command_integral.tolist(),'official_propellant':None,'official_qualification':None,'gravity_fit':fit})
finally:client.close()

