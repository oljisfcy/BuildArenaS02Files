"""Independent end-window check from recorder poses, including every rotor.

Velocity fields contain constraint-solver impulses even on stationary anchors.
This check bounds actual position and quaternion excursions for 12 seconds.
It does not implement or claim the unavailable official ground detector.
"""
import csv, math
import numpy as np

def check_pose_window(csv_path,guids,duration=12.,rotor_guids=()):
    samples={}
    with open(csv_path,newline='') as f:
        for r in csv.DictReader(f):
            if r['block_guid'] not in guids or not r['x']:continue
            t=float(r['simulation_time'])
            samples.setdefault(t,{})[r['block_guid']]=[float(r[k]) for k in ('x','y','z','qx','qy','qz','qw')]
            # Only the endpoint window is needed; prune older data periodically.
            if len(samples)>500:
                cutoff=t-duration-1
                samples={k:v for k,v in samples.items() if k>=cutoff}
    times=sorted(samples);end=times[-1]
    times=[t for t in times if t>=end-duration-(.24 if rotor_guids else 0)-.021]
    complete=all(set(samples[t])==set(guids) for t in times)
    if not complete:return {'pass':False,'reason':'missing retained block poses'}
    a=np.array([[samples[t][g] for g in sorted(guids)] for t in times])
    positions=a[:,:,:3];q=a[:,:,3:];q/=np.linalg.norm(q,axis=2)[:,:,None]
    displacement=np.linalg.norm(positions-positions[0],axis=2)
    angles=np.degrees(2*np.arccos(np.clip(np.abs(np.sum(q*q[0],axis=2)),-1,1)))
    rotor_spin=float(angles[:,[i for i,g in enumerate(sorted(guids)) if g in rotor_guids]].max()) if rotor_guids else None
    for i,g in enumerate(sorted(guids)):
        if g not in rotor_guids:continue
        x,y,z,w=q[:,i].T
        axes=np.column_stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
        angles[:,i]=np.degrees(np.arccos(np.clip(axes@axes[0],-1,1)))
    raw_position=float(displacement.max());raw_rotation=float(angles.max())
    measured_span=times[-1]-times[0]
    if rotor_guids:
        # Independent 6-sample (0.24 s at 25 Hz) box average. The controller
        # uses a 0.2 s exponential observer; this is not its internal timer.
        smoothed_p=[];smoothed_q=[];smoothed_t=[]
        for j in range(len(times)-5):
            smoothed_p.append(positions[j:j+6].mean(axis=0))
            window=q[j:j+6].copy();sign=np.where(np.sum(window*window[0],axis=2)<0,-1.,1.)
            mean=(window*sign[:,:,None]).mean(axis=0);mean/=np.linalg.norm(mean,axis=1)[:,None]
            smoothed_q.append(mean);smoothed_t.append(np.mean(times[j:j+6]))
        sp=np.array(smoothed_p);sq=np.array(smoothed_q)
        # Use the window's center pose, not an arbitrarily jittered first frame.
        displacement=np.linalg.norm(sp-sp.mean(axis=0),axis=2)
        sign=np.where(np.sum(sq*sq[0],axis=2)<0,-1.,1.)
        reference=(sq*sign[:,:,None]).mean(axis=0);reference/=np.linalg.norm(reference,axis=1)[:,None]
        angles=np.degrees(2*np.arccos(np.clip(np.abs(np.sum(sq*reference,axis=2)),-1,1)))
        for i,g in enumerate(sorted(guids)):
            if g not in rotor_guids:continue
            x,y,z,w=sq[:,i].T;axis=np.column_stack([2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)])
            reference_axis=axis.mean(axis=0);reference_axis/=np.linalg.norm(reference_axis)
            angles[:,i]=np.degrees(np.arccos(np.clip(axis@reference_axis,-1,1)))
        measured_span=smoothed_t[-1]-smoothed_t[0]
    result={'start':times[0],'end':end,'duration':measured_span,
            'max_position_excursion':float(displacement.max()),
            'max_rotation_excursion_degrees':float(angles.max()),
            'max_sample_gap':float(np.diff(times).max()),'blocks':len(guids),
            'internal_rotor_rotation_degrees':rotor_spin,
            'raw_max_position_excursion':raw_position,'raw_max_rotation_excursion_degrees':raw_rotation,
            'pose_averaging_seconds':.24 if rotor_guids else 0,
            'reference':'window centroid' if rotor_guids else 'first sample',
            'orientation_check':'all rigid poses and rotor mounting axes' if rotor_guids else 'all poses including rotor spin'}
    result['pass']=bool(result['duration']>=duration-.001 and result['max_position_excursion']<.03
                        and result['max_rotation_excursion_degrees']<1 and result['max_sample_gap']<.081
                        and raw_position<.1 and raw_rotation<3)
    return result
