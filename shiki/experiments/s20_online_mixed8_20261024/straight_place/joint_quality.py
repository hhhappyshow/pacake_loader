"""Simulation trajectory constraints; never changes controller limits or IO.

Angles are unwrapped physical joint coordinates. Modulo 2*pi would hide a
real cable/limit excursion and is deliberately not used here.
"""
import numpy as np
from scipy.interpolate import PchipInterpolator


def assess(q, lower, upper, dt=.02):
    q=np.asarray(q,float)
    lower=np.asarray(lower,float)[:q.shape[1]]
    upper=np.asarray(upper,float)[:q.shape[1]]
    if q.ndim!=2 or len(q)==0 or not np.isfinite(q).all():
        raise ValueError('Expected finite, nonempty joint samples')
    delta=np.diff(q,axis=0)
    velocity=delta/dt
    acceleration=np.diff(np.vstack([np.zeros((1,q.shape[1])),velocity,
                                    np.zeros((1,q.shape[1]))]),axis=0)/dt
    margin=np.minimum(q-lower,upper-q).min(axis=0)
    travel=np.abs(delta).sum(axis=0)
    return dict(min_margin_deg=np.rad2deg(margin).tolist(),
        travel_deg=np.rad2deg(travel).tolist(),
        max_speed_deg_s=np.rad2deg(np.max(abs(velocity),axis=0) if len(velocity) else travel).tolist(),
        max_acceleration_deg_s2=np.rad2deg(abs(acceleration).max(axis=0)).tolist(),
        # Penalise concentration on one joint as well as total motion.
        score=float(travel.sum()+2*travel.max()+.1*np.sum(1/np.maximum(margin,.01))))


def check_margin(q, model, policy):
    if not policy:return
    q=np.atleast_2d(np.asarray(q,float))
    lower=model.joint_limit_lower.numpy()[:6]
    upper=model.joint_limit_upper.numpy()[:6]
    margin=np.minimum(q-lower,upper-q)
    if not np.isfinite(q).all() or np.any(margin < np.radians(policy['margin_deg'])):
        index,joint=np.unravel_index(np.argmin(margin),margin.shape)
        raise ValueError(f'joint_margin: J{joint+1}, sample {index}, '
                         f'{np.degrees(margin[index,joint]):.2f} deg remaining')


def retime(q, duration, policy, dt=.02):
    """Stop at each segment endpoint; bound sampled speed and acceleration.

    Returns the whole path including its starting point. Interpolated samples
    MUST still undergo FK and collision validation by the caller.
    """
    q=np.asarray(q,float)
    if not policy:return q
    if len(q)<2:raise ValueError('Need at least two samples')
    curve=PchipInterpolator(np.linspace(0,1,len(q)),q,axis=0)
    count=max(2,int(np.ceil(duration/dt)))
    for _ in range(12):
        u=np.linspace(0,1,count+1)
        smooth=u**3*(10-15*u+6*u*u)
        path=curve(smooth)
        v=np.diff(path,axis=0)/dt
        a=np.diff(np.vstack([np.zeros((1,6)),v,np.zeros((1,6))]),axis=0)/dt
        ratio=max(abs(v).max()/np.radians(policy['speed_deg_s']),
                  np.sqrt(abs(a).max()/np.radians(policy['acceleration_deg_s2'])))
        if ratio<=1.000001:return path
        count=int(np.ceil(count*ratio*1.03))
        if count>100000:raise ValueError('Trajectory cannot be retimed within 2000 seconds')
    raise ValueError('Joint time scaling failed to converge')
