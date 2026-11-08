"""Synthetic recordings only; no field data or current-device claims."""
import copy,math
from pathlib import Path
from raw_odom_mode import RawTracker,RawChassisSession
from test_raw_mode import identity,snap,update

def session_fixture(tmp):
    t=RawTracker();t.bind(identity());now=10.
    for _ in range(45):now+=.05;update(t,now)
    session=RawChassisSession('metadata',Path(tmp)/'start.json',Path(tmp),snap(t,now),now,t.since)
    return t,now,session


def move(t,now,session,v,w,count):
    p=list(t.latest['pose'])
    for _ in range(count):
        now+=.05;p=[p[0]+v*.05*math.cos(p[2]),p[1]+v*.05*math.sin(p[2]),p[2]+w*.05]
        update(t,now,p,(v,0.,w));session.update(snap(t,now),'metadata',now)
    return now


def finish(t,now,session):
    at=now;p=list(t.latest['pose'])
    for _ in range(46):
        now+=.05;update(t,now,p)
        s=snap(t,now);s.update(goal_status=4,navigation_id=session.goal_id,goal_finished_at=at)
        session.update(s,'metadata',now)
    return now
