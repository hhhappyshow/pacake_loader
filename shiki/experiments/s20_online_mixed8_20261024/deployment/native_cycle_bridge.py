"""Execute current online plans with controller-native paths, not ServoJ ticks.

Contact and vertical clearance use MoveL. Other accepted joint paths are
uploaded to MovePathJ. Native interpolation differs from the source curve.
--backend movej_chain replays the same waypoints as blended MoveJ commands
instead of MovePathJ (fallback when the controller rejects MovePathJ starts).
--backend servoj_replay streams the source trajectory itself at 20 ms via
StartServo/PushServoJ (smoothest; requires a stall-free 50 Hz host loop).
"""
import argparse
import json
import math
from pathlib import Path
import time

from jetson_interfaces import SDKInterfaces

SERVO_PERIOD_S=.02
SERVO_LOOKAHEAD_S=.2


def distinct_joint_points(points):
    result=[]
    for point in points:
        if not result or point!=result[-1]:result.append(point)
    return result

def quintic_hermite_waypoints(points,key_step,samples):
    """Resample a joint trajectory through C2 quintic Hermite interpolation.

    Keys are taken every key_step source points; each interval is bridged by a
    quintic polynomial matching position, velocity and acceleration at both
    keys (finite-difference boundary conditions), sampled at `samples`
    interior waypoints. Returns (waypoints, max deviation from the source
    polyline in degrees)."""
    n=len(points)
    if n<3:return list(points),0.
    key_step=max(2,int(key_step))
    idx=list(range(0,n,key_step))
    if idx[-1]!=n-1:idx.append(n-1)
    keys=[points[i] for i in idx]
    vel=[];acc=[]
    for k,i in enumerate(idx):
        i0=max(i-1,0);i1=min(i+1,n-1);span=max(i1-i0,1)
        vel.append([(points[i1][j]-points[i0][j])/span for j in range(6)])
        acc.append([(points[i+1][j]-2*points[i][j]+points[i-1][j]) for j in range(6)] if 0<i<n-1 else [0.]*6)
    waypoints=[list(keys[0])];deviation=0.
    for k in range(len(keys)-1):
        width=idx[k+1]-idx[k]
        q0,q1=keys[k],keys[k+1]
        v0=[v*width for v in vel[k]];v1=[v*width for v in vel[k+1]]
        a0=[a*width*width for a in acc[k]];a1=[a*width*width for a in acc[k+1]]
        for s in range(1,samples+1):
            u=s/(samples+1);u2=u*u;u3=u2*u;u4=u3*u;u5=u4*u
            h00=1-10*u3+15*u4-6*u5
            h10=u-6*u3+8*u4-3*u5
            h20=.5*u2-1.5*u3+1.5*u4-.5*u5
            h01=10*u3-15*u4+6*u5
            h11=-4*u3+7*u4-3*u5
            h21=.5*u3-u4+.5*u5
            waypoint=[h00*q0[j]+h10*v0[j]+h20*a0[j]+h01*q1[j]+h11*v1[j]+h21*a1[j] for j in range(6)]
            waypoints.append(waypoint)
            source=float(idx[k])+u*width;i0=int(source);frac=source-i0;i1=min(i0+1,n-1)
            reference=[points[i0][j]*(1-frac)+points[i1][j]*frac for j in range(6)]
            deviation=max(deviation,max(abs(waypoint[j]-reference[j]) for j in range(6)))
    waypoints.append(list(keys[-1]))
    return waypoints,deviation



def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cycle',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--action')  # Same session dispatcher as the ServoJ variant.
    p.add_argument('--place-only',action='store_true')
    p.add_argument('--initial-holding',action='store_true')
    p.add_argument('--vacuum-feedback',choices=('digital','operator'),default='digital')
    p.add_argument('--speed-scale',type=float,default=1.)
    p.add_argument('--backend',choices=('native','movej_chain','movej_quintic','servoj_replay'),default='native',
        help='native uploads MovePathJ paths; movej_chain replays the same waypoints as '
             'blended MoveJ commands; movej_quintic resamples the segment through C2 '
             'quintic Hermite interpolation first (smoothest MoveJ-based fallback)')
    p.add_argument('--movej-chunk',type=int,default=20,
        help='Waypoints per MoveJ_chain burst; each burst ends with a brief stop')
    p.add_argument('--movej-speed',type=float,default=15.,
        help='Base joint speed in deg/s before speed-scale for MoveJ_chain')
    p.add_argument('--place-speed-scale',type=float,default=None,
        help='Overrides --speed-scale for place-only executions (place + empty return); '
             'use a lower value when the box-carrying phases trip collision detection')
    p.add_argument('--return-speed-scale',type=float,default=None,
        help='Overrides the speed scale for the final empty-return stream only '
             '(place-only executions); the empty tool tolerates higher speed than '
             'the box-carrying place stretch')
    p.add_argument('--servo-taper-fraction',type=float,default=None,
        help='Fraction of a streamed trajectory after which the per-tick speed drops to '
             '--servo-taper-scale. Default: disabled (1.0); pass explicitly to enable')
    p.add_argument('--servo-taper-scale',type=float,default=None,
        help='Absolute speed scale used beyond --servo-taper-fraction; the effective '
             'scale is min(speed-scale, this value). Default: 0.3 when the taper is enabled')
    p.add_argument('--servo-head-taper-points',type=int,default=0,
        help='Source points at the head of every stream that run at '
             '--servo-head-taper-scale; 0 disables (used to crawl through a '
             'known-bad stretch such as the elbow-straightening separation lift)')
    p.add_argument('--servo-head-taper-scale',type=float,default=.15,
        help='Absolute speed scale for the head taper region')
    p.add_argument('--quintic-key-step',type=int,default=50,
        help='Quintic key every N source points; halved automatically until the '
             'interpolation deviation is acceptable')
    p.add_argument('--quintic-samples',type=int,default=6,
        help='Interior waypoints sampled per quintic interval')
    a=p.parse_args();recipe=json.loads(a.cycle.read_text())
    if a.place_speed_scale is not None and a.place_only:
        a.speed_scale=a.place_speed_scale
    if a.return_speed_scale is not None and (
            not math.isfinite(a.return_speed_scale) or a.return_speed_scale<=0):
        raise ValueError('return-speed-scale must be positive and finite')
    if not math.isfinite(a.speed_scale) or a.speed_scale<=0:
        raise ValueError('speed-scale must be positive and finite')
    if a.servo_taper_fraction is not None and (
            not math.isfinite(a.servo_taper_fraction) or not 0.<a.servo_taper_fraction<=1.):
        raise ValueError('servo-taper-fraction must be in (0,1]')
    if a.servo_taper_scale is not None and (
            not math.isfinite(a.servo_taper_scale) or a.servo_taper_scale<=0):
        raise ValueError('servo-taper-scale must be positive and finite')
    # The place stretch keeps a protected final quarter at the place speed
    # scale's 0.3x; pick-side streams stay at full speed. Disable explicitly
    # with --servo-taper-fraction 1.0.
    servo_taper_fraction=a.servo_taper_fraction if a.servo_taper_fraction is not None \
        else (.75 if a.place_only else 1.)
    servo_taper_scale=a.servo_taper_scale if a.servo_taper_scale is not None else .3
    if a.servo_head_taper_points<0:raise ValueError('servo-head-taper-points must be >=0')
    if a.servo_head_taper_points and (
            not math.isfinite(a.servo_head_taper_scale) or a.servo_head_taper_scale<=0):
        raise ValueError('servo-head-taper-scale must be positive and finite')
    if not a.execute:
        print(json.dumps(dict(operations=recipe['operations'],motion_sent=False)));return
    a.output.mkdir(parents=True,exist_ok=False)
    log=(a.output/'events.jsonl').open('x')
    report=dict(completed=False,motion_requested=False,io_requested=False,
        backend='native_MoveL_MovePathJ',occupancy_committed=False,
        speed_scale=a.speed_scale,
        vacuum_confirmation_source=a.vacuum_feedback,
        pressure_sensor_verified=False,
        automatic_holding_loss_monitoring=a.vacuum_feedback=='digital',
        native_interpolation_is_source_curve=False,settled_placement_observed=False)
    holding=False;moving=False
    def record(event,**data):
        log.write(json.dumps(dict(event=event,monotonic_s=time.monotonic(),**data))+'\n');log.flush()
    def confirm(token,message):
        record('operator_confirmation_requested',token=token,message=message)
        if input(message+' Type '+token+' to continue, anything else stops: ').strip()!=token:
            raise RuntimeError('Operator did not confirm; no following motion sent')
        record('operator_confirmed',token=token,pressure_measured=False)
    def error_if_fault(sdk):
        fsm=int(sdk.read('HRIF_ReadCurFSM',0)[0])
        if fsm in (21,22,24):raise RuntimeError('Controller stopped: '+str(fsm))
        return fsm
    with SDKInterfaces() as sdk:
        def valve(bit,value):
            report['io_requested']=True
            sdk.call('HRIF_SetBoxDO',0,bit,int(value))
            record('valve_result',bit=bit,on=value)
        def wait_vacuum(value):
            if a.vacuum_feedback=='operator':
                if value:
                    began=time.monotonic()
                    while True:
                        io=sdk.read_vacuum()
                        if io['DO0']==1 and io['DO1']==0:break
                        if time.monotonic()-began>=2.:
                            raise RuntimeError('Suction output readback did not reach DO0=1/DO1=0: '+str(io))
                        time.sleep(.05)
                    record('suction_output_confirmed',value=io,
                        readback_wait_s=time.monotonic()-began,pressure_measured=False)
                    confirm('2','Operator mode: confirm actual suction and supervise the following motion.')
                else:
                    confirm('3','Confirm the box is supported and the suction cup has physically released.')
                return
            began=time.monotonic();stable=None;last=None
            while time.monotonic()-began<30:
                io=sdk.read_vacuum();now=time.monotonic()
                if io!=last:record('vacuum_io_changed',value=io);last=io
                matched=io['DI0']==value and (not value or io['DO0']==1 and io['DO1']==0)
                if matched:
                    if stable is None:stable=now
                    if now-stable>=(1. if value else 0.):
                        record('vacuum_feedback',value=io,stable_s=now-stable);return
                else:stable=None
                time.sleep(.05)
            raise RuntimeError('Vacuum feedback did not confirm; no following motion sent')
        def wait_motion(goal):
            nonlocal moving
            began=time.monotonic()
            while time.monotonic()-began<180:
                q=list(map(float,sdk.read('HRIF_ReadActJointPos',0)[:6]))
                fsm=error_if_fault(sdk)
                if holding and a.vacuum_feedback=='digital':
                    io=sdk.read_vacuum()
                    if io['DI0']!=1:raise RuntimeError('Holding feedback lost; vacuum retained: '+str(io))
                if fsm==33 and max(abs(x-y) for x,y in zip(q,goal))<.05:
                    moving=False;record('trajectory_result',completed=True,q_deg=q,wall_s=time.monotonic()-began);return
                time.sleep(.05)
            raise RuntimeError('Native motion did not finish')
        def motion(points,linear=False,period_s=.02,scale=None):
            nonlocal moving
            scale=a.speed_scale if scale is None else scale
            current=list(map(float,sdk.read('HRIF_ReadActJointPos',0)[:6]))
            if max(abs(x-y) for x,y in zip(current,points[0]))>.05:
                raise RuntimeError('Native path start differs from actual joints; replan required')
            if len(distinct_joint_points(points))==1:
                record('native_segment_skipped',reason='identical_reference_points',source_points=len(points))
                return
            if max(abs(x-y) for x,y in zip(current,points[-1]))<.01 and len(points)<5:return
            if error_if_fault(sdk)!=33:raise RuntimeError('Native segment requires StandBy')
            if linear:
                pose=[]
                sdk.call('HRIF_GetForwardKin',0,0,points[-1],[0.]*6,[0.]*6,pose)
                record('native_move_command',method='MoveL',target_pose=pose,
                    velocity_mm_s=30.*scale,acceleration_mm_s2=50.*scale)
                report['motion_requested']=True;moving=True
                sdk.call('HRIF_MoveL',0,0,list(map(float,pose[:6])),points[-1],
                    'TCP','Base',30.*scale,50.*scale,0.,0,0,0,'native_contact')
            else:
                if a.backend=='servoj_replay':
                    # Stream the source trajectory itself: StartServo, then one
                    # PushServoJ per 20 ms slot. Beyond --servo-taper-fraction
                    # of the trajectory the per-tick advance drops to
                    # --servo-taper-scale (protects the final reach, where the
                    # box-carrying torque peaks). The final target is held for
                    # the lookahead plus three slots, then GrpStop. Deadline-
                    # based; every overrun is recorded. Any fault or vacuum
                    # loss aborts the stream, which always exits via GrpStop.
                    record('servoj_stream_start',source_points=len(points),period_s=period_s,
                        speed_scale=scale,lookahead_s=SERVO_LOOKAHEAD_S,
                        servo_taper_fraction=stream_taper_fraction,servo_taper_scale=servo_taper_scale,
                        servo_head_taper_points=stream_head_taper_points,
                        servo_head_taper_scale=a.servo_head_taper_scale,
                        speed_is_not_measured_profile=True)
                    report['motion_requested']=True;moving=True
                    report['backend']='servoj_replay_MoveL'
                    began=time.monotonic();tick=0;index=0.;lateness=0.
                    last=len(points)-1
                    hold_ticks=int(math.ceil(SERVO_LOOKAHEAD_S/SERVO_PERIOD_S))+3
                    end_tick=None
                    try:
                        sdk.call('HRIF_StartServo',0,0,SERVO_PERIOD_S,SERVO_LOOKAHEAD_S)
                        while True:
                            deadline=began+tick*SERVO_PERIOD_S
                            delay=deadline-time.monotonic()
                            if delay>0:time.sleep(delay)
                            lateness=max(lateness,time.monotonic()-deadline)
                            progress=index/last if last else 1.
                            local_scale=scale
                            if stream_head_taper_points and index<stream_head_taper_points:
                                local_scale=min(local_scale,a.servo_head_taper_scale)
                            elif progress>=stream_taper_fraction:
                                local_scale=min(local_scale,servo_taper_scale)
                            index=min(index+local_scale,last)
                            i0=min(int(index),last);frac=index-i0;i1=min(i0+1,last)
                            target=[points[i0][j]*(1-frac)+points[i1][j]*frac for j in range(6)]
                            code=sdk.client.HRIF_PushServoJ(0,0,[float(v) for v in target])
                            if code:raise RuntimeError('HRIF_PushServoJ returned %d'%code)
                            if tick%10==0:
                                error_if_fault(sdk)
                                if holding and a.vacuum_feedback=='digital':
                                    io=sdk.read_vacuum()
                                    if io['DI0']!=1:raise RuntimeError(
                                        'Holding feedback lost during stream; vacuum retained: '+str(io))
                            if index>=last:
                                if end_tick is None:end_tick=tick
                                if tick-end_tick>=hold_ticks:break
                            tick+=1
                    finally:
                        try:
                            stop_code=sdk.client.HRIF_GrpStop(0,0)
                            record('servoj_stream_stop',return_code=stop_code,
                                max_deadline_lateness_s=round(lateness,4),
                                wall_s=round(time.monotonic()-began,3),ticks=tick+1)
                        except Exception as stop_error:
                            record('servoj_stop_failed',error=str(stop_error))
                else:
                    chosen=points[::5]
                    if chosen[-1]!=points[-1]:chosen.append(points[-1])
                    chosen=distinct_joint_points(chosen)
                    if len(chosen)<5:chosen=distinct_joint_points(points)
                    while len(chosen)<5:
                        k=max(range(len(chosen)-1),key=lambda k:sum((a-b)**2 for a,b in zip(chosen[k],chosen[k+1])))
                        chosen.insert(k+1,[(a+b)/2 for a,b in zip(chosen[k],chosen[k+1])])
                    def movej_burst(waypoints,chunk_size,label_extra):
                        # Queue blended MoveJ commands: the controller keeps
                        # executing while further waypoints arrive, and
                        # dRadius>0 rounds the corners. If the controller
                        # refuses a queued command (state 20018) the burst
                        # degrades to stop-and-go for that waypoint.
                        nonlocal moving
                        speed=a.movej_speed*scale;acc=2.*speed
                        record('native_move_command',method='MoveJ_chain',waypoints=len(waypoints),
                            chunk_size=chunk_size,speed_deg_s=speed,acc_deg_s2=acc,blend_radius_mm=1.,
                            speed_is_not_measured_profile=True,**label_extra)
                        report['motion_requested']=True;moving=True
                        report['backend']='movej_chain_MoveL'
                        ignored=[0.]*6;seq=0;previous=None
                        for begin in range(0,len(waypoints),chunk_size):
                            chunk=waypoints[begin:begin+chunk_size]
                            for waypoint in chunk:
                                seq+=1;command_id='mc%04d'%seq
                                try:
                                    sdk.call('HRIF_MoveJ',0,0,ignored,waypoint,'TCP','Base',
                                        speed,acc,1.,1,0,0,0,command_id)
                                except RuntimeError as exc:
                                    if '20018' not in str(exc):raise
                                    if previous is not None:wait_motion(previous)
                                    sdk.call('HRIF_MoveJ',0,0,ignored,waypoint,'TCP','Base',
                                        speed,acc,0.,1,0,0,0,command_id)
                                previous=waypoint
                            wait_motion(chunk[-1])
                    if a.backend=='movej_quintic':
                        key_step=max(5,a.quintic_key_step)
                        while True:
                            waypoints,deviation=quintic_hermite_waypoints(points,key_step,a.quintic_samples)
                            record('quintic_resampling',keys_every_source_points=key_step,
                                samples_per_interval=a.quintic_samples,waypoints=len(waypoints),
                                max_deviation_deg=round(deviation,4))
                            if deviation<=.5 or key_step<=5:break
                            key_step//=2
                        movej_burst(waypoints,len(waypoints),
                            dict(quintic_keys_every=key_step,max_deviation_deg=round(deviation,4),
                                single_burst=True))
                    elif a.backend=='movej_chain':
                        movej_burst(chosen,a.movej_chunk,dict(source_sampling='every_5th'))
                    else:
                        name='p'+str(time.time_ns())
                        ratio=min(1.,a.speed_scale*min(12./max(report['before']['joint_max_velocity_deg_s']),
                            10./max(report['before']['joint_max_acceleration_deg_s2'])))
                        sdk.call('HRIF_StartPushMovePathJ',0,0,name,ratio,1.)
                        for q in chosen:sdk.call('HRIF_PushMovePathJ',0,0,name,q)
                        sdk.call('HRIF_EndPushMovePathJ',0,0,name)
                        began=time.monotonic()
                        while time.monotonic()-began<30:
                            state=sdk.read('HRIF_ReadPathState',0,name)
                            if int(state[0])==3 and not int(state[1]):break
                            if int(state[0])==5 or int(state[1]):raise RuntimeError('Native path calculation failed: '+str(state))
                            time.sleep(.05)
                        else:raise RuntimeError('Native path calculation timeout')
                        record('native_move_command',method='MovePathJ',path_name=name,source_points=len(points),
                            uploaded_points=len(chosen),speed_ratio=ratio,
                            ratio_is_not_a_measured_velocity_or_acceleration=True,
                            path_info=sdk.read('HRIF_ReadPathInfo',0,name))
                    report['motion_requested']=True;moving=True
                    sdk.call('HRIF_MovePathJ',0,0,name)
            wait_motion(points[-1])
        try:
            report['before']=sdk.snapshot()
            if report['before']['fsm_name']!='StandBy':raise RuntimeError('Controller is not StandBy')
            for method,name in (('HRIF_ReadTCPByName','TCP'),('HRIF_ReadUCSByName','Base')):
                if any(float(v)!=0. for v in sdk.read(method,0,name)):
                    raise RuntimeError('Native mode requires the verified zero TCP/Base definitions')
            if any(float(v)!=0. for key in ('tcp_config','ucs_config') for v in report['before'][key]):
                raise RuntimeError('Controller current coordinates differ from verified native comparison')
            if not a.initial_holding and a.vacuum_feedback=='operator':
                confirm('1','Confirm the tool is empty and separated from the supported box before motion.')
            if a.vacuum_feedback=='digital' and recipe.get('role')=='empty_observation_return' and report['before']['vacuum']['DI0']:
                raise RuntimeError('Empty return cannot start with holding feedback')
            if a.initial_holding:wait_vacuum(1);holding=True
            operations=recipe['operations']
            if a.place_only:
                if not a.initial_holding:raise ValueError('place-only requires initial-holding')
                operations=operations[next(i for i,o in enumerate(operations) if o['type']=='bind_async_result'):]
            servoj_indices=[i for i,o in enumerate(operations) if o['type']=='servoj']
            last_servoj_index=servoj_indices[-1] if servoj_indices else None
            stream_head_taper_points=0
            stream_taper_fraction=1.
            for op_index,op in enumerate(operations):
                # The head taper exists for the final exit stream only (the
                # elbow-straightening separation lift after release); every
                # other stream keeps its full speed. The tail taper protects
                # the box-carrying place stretch only — the empty return
                # keeps its own scale to the end.
                stream_head_taper_points=a.servo_head_taper_points \
                    if (a.place_only and op_index==last_servoj_index) else 0
                stream_taper_fraction=servo_taper_fraction \
                    if (a.place_only and op_index!=last_servoj_index) else 1.
                # The final empty-return stream may run faster than the
                # box-carrying place stretch: the tool is empty again.
                stream_speed_scale=a.speed_scale
                if a.return_speed_scale is not None and op_index==last_servoj_index:
                    stream_speed_scale=a.return_speed_scale
                kind=op['type'];record('operation_start',operation=op)
                if kind=='servoj':
                    ref=json.loads((a.cycle.parent/op['reference']).read_text())
                    points=ref.get('source_positions_deg',ref['positions_deg'])
                    period_s=float(ref.get('period_s',.02))
                    labels=ref.get('source_stages')
                    start=0
                    if labels is not None:
                        i=0
                        while i<len(labels):
                            j=i+1
                            while j<len(labels) and labels[j]==labels[i]:j+=1
                            if (labels[i] in ('CONTACT','VERTICAL_CLEARANCE') or labels[i].endswith(
                                    ('_EMPTY_SEPARATE','_INSERT_DEPTH','_SHIFT_LATERAL','_LOWER_FINAL'))) and j-i>1:
                                if i>start:motion(points[start:i],period_s=period_s,scale=stream_speed_scale)
                                motion(points[max(0,i-1):j],linear=True);start=j-1
                            i=j
                    if start<len(points)-1:motion(points[start:],period_s=period_s,scale=stream_speed_scale)
                elif kind=='vacuum_on_and_confirm':
                    valve(1,False);valve(0,True);wait_vacuum(1);holding=True
                elif kind=='bind_async_result':record('entry_feedback',scene_version=op['scene_version'])
                elif kind in ('release_and_confirm_support','release_and_confirm_vacuum_off'):
                    holding=False;valve(0,False);valve(1,True)
                    time.sleep(.2);valve(1,False);wait_vacuum(0)
                elif kind=='commit_planned_occupancy':
                    (a.output/'scene_state.json').write_text(json.dumps(recipe['next_scene_after_execution'],indent=2))
                    report.update(occupancy_committed=True,occupancy_source='planned_release_pose_assumption')
                else:raise ValueError('Unknown operation '+kind)
                record('operation_end',type=kind)
            report.update(completed=True,motion_and_io_sequence_finished=True)
        except BaseException as exc:
            report['error']=f'{type(exc).__name__}: {exc}';record('stopped',error=report['error'])
            if moving:report['stop_return_code']=sdk.client.HRIF_GrpStop(0,0)
        finally:
            (a.output/'result.json').write_text(json.dumps(report,indent=2));log.close()
            if a.vacuum_feedback=='operator':
                print(json.dumps(dict(completed=report['completed'],error=report.get('error'),
                    confirmation_source='operator_without_pressure_sensor',output=str(a.output))))
            else:print(json.dumps(report))
    return 0 if report['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
