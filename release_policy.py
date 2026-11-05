"""Independent output release policy. Limits do not depend on panel checkboxes."""
from run_lease import RunLeaseGate
import math,struct


def feedback_speed_exceeded(value,limit):
    # Driver feedback is promoted float32. Permit the representation of the
    # nominal boundary, but reject the next representable value above it.
    if type(value) not in (int,float) or type(limit) not in (int,float) or not math.isfinite(value) or not math.isfinite(limit) or limit<=0:return True
    encoded=struct.unpack('!f',struct.pack('!f',limit))[0]
    return abs(value)>max(limit,encoded)


class ReleasePolicy:
    def __init__(self, config, *, isolated_test=False):
        if not isinstance(config,dict):config={}
        self.profile=config.get('profile','disabled')
        self.isolated_test=isolated_test
        self.errors=[]
        self.start_mode=config.get('return_start_mode','hold')
        if self.start_mode not in ('hold','click'):self.errors.append('unsupported return start mode')
        self.return_speed=config.get('return_speed_m_s',.05)
        if type(self.return_speed) not in (int,float) or self.return_speed not in (.05,.10,.15):
            self.errors.append('unsupported return speed');self.return_speed=.05
        if self.profile=='short_only' and self.return_speed!=.05:self.errors.append('short_only speed must remain 0.05')
        self.feedback_speed_limit=config.get('feedback_speed_limit_m_s',self.return_speed*1.2)
        if type(self.feedback_speed_limit) not in (int,float) or not math.isfinite(self.feedback_speed_limit) or not self.return_speed<=self.feedback_speed_limit<=.40:
            self.errors.append('invalid measured return speed limit');self.feedback_speed_limit=self.return_speed*1.2
        if self.profile=='short_only' and self.feedback_speed_limit>.06:self.errors.append('short_only feedback limit must remain 0.06')
        if self.profile not in ('disabled','short_only','supervised_route','supervised_trial'):
            self.errors.append('unknown release profile')
        if self.profile!='disabled' and not isolated_test:
            if not config.get('deployment_id'):self.errors.append('missing deployment identity')
            evidence=config.get('physical_stop_evidence',{})
            if not isinstance(evidence,dict):evidence={}
            site=config.get('site_authorization',{})
            if not isinstance(site,dict):site={}
            if evidence.get('user_confirmed') is not True:
                self.errors.append('user-managed physical stop acknowledgement missing')
            for field in ('operator_present','route_cleared','arm_stowed','other_controls_handed_over'):
                if site.get(field) is not True:
                    self.errors.append('site authorization missing: '+field)
            if self.profile=='supervised_route':
                receipt=config.get('full_route_acceptance_receipt') or {}
                if not isinstance(receipt,dict):receipt={}
                if receipt.get('synthetic') is True:self.errors.append('synthetic acceptance is not field evidence')
                if receipt.get('deployment_id')!=config.get('deployment_id'):
                    self.errors.append('field acceptance is for another deployment')
                for field in ('short_trial_passed','hold_release_passed','software_stop_passed',
                              'producer_loss_passed','command_silence_stop_passed','operator_approved'):
                    if receipt.get(field) is not True:self.errors.append('field acceptance missing: '+field)
        self.trial_authorization=config.get('full_trial_authorization') or {}
        self.trial_max_route_m=0.;self.trial_max_time_s=0.
        if self.profile=='supervised_trial':
            a=self.trial_authorization
            if not isinstance(a,dict):a={};self.trial_authorization=a
            for field in ('operator_approved','one_run','allow_full_route'):
                if a.get(field) is not True:self.errors.append('full trial authorization missing: '+field)
            ident=a.get('authorization_id')
            if not isinstance(ident,str) or not 8<=len(ident)<=80 or any(ch not in '0123456789abcdef-' for ch in ident):self.errors.append('invalid full trial authorization identity')
            if a.get('user_selected_speed_m_s')!=self.return_speed:self.errors.append('full trial speed not explicitly authorized')
            for name,ceiling in [('max_route_m',4.),('max_time_s',90.)]:
                value=a.get(name)
                if type(value) not in (float,int) or not math.isfinite(value) or not 0<value<=ceiling:self.errors.append('invalid full trial '+name)
                else:setattr(self,'trial_'+name,value)
        self.enabled=self.profile!='disabled' and not self.errors


class TrialLimiter:
    """Latches one return trial at 3s or 0.15m in short_only, at the arbiter."""
    MAX_TIME=3.
    MAX_DISTANCE=.15
    MAX_SPEED=.05
    MIN_RADIUS=1.78

    def __init__(self,policy):
        self.policy=policy;self.max_speed=policy.return_speed;self.claim_trial=None;self.claimed=False;self.full_trial_consumed=False;self.active_route_length=None
        self.fault=None
        self.run_lease=RunLeaseGate()
        self.used=set()
        self.active=None
        self.started_at=None
        self.last_tick=None
        self.last_odom=None
        self.distance=0.

    def fail(self,reason):
        self.fault=reason
        return 0.,0.

    def reset(self,*,quiet,explicit_reset):
        if not quiet or not explicit_reset:return False
        self.run_lease.close()
        self.fault=None;self.active=None;self.started_at=None;self.last_tick=None
        self.last_odom=None;self.distance=0.
        # A panel or arbiter reset must never make a consumed trial reusable.
        return True

    def filter(self,mode,packet,command,now,*,odom_pose,odom_stamp,odom_age,measured_velocity):
        if not self.policy.enabled:return 0.,0.
        if self.fault:return 0.,0.
        if mode!='return':
            self.run_lease.close()
            self.active=None;self.started_at=None;self.last_tick=None;self.last_odom=None
            return command
        if not isinstance(packet,dict):return self.fail('invalid trial packet')
        key=tuple(packet.get(k) for k in ('owner','trial_id','cycle_id','goal_id'))
        if not all(isinstance(k,str) and k for k in key):return self.fail('return trial identity missing')
        if packet.get('site_confirmed') is not True:return self.fail('return site confirmation absent')
        if self.policy.start_mode=='click':
            if packet.get('activation_mode')!='click':return self.fail('explicit click mode required')
            try:self.run_lease.check(packet,now)
            except ValueError as e:return self.fail(str(e))
        elif packet.get('activation_mode','hold')!='hold' or packet.get('held') is not True:
            return self.fail('return hold/site confirmation absent')
        if self.policy.profile=='short_only' and packet.get('short_test') is not True:
            return self.fail('short_only rejects full-route requests')
        if self.policy.profile=='supervised_trial':
            length=packet.get('route_length_m')
            if packet.get('short_test') is not False:return self.fail('full trial requires full-route mode')
            if type(length) not in (int,float) or not math.isfinite(length):return self.fail('full trial route length is invalid')
            if length<.05:return self.fail('full trial route is too short')
            if length>self.policy.trial_max_route_m:return self.fail('full trial route {:.3f}m exceeds authorization {:.3f}m'.format(length,self.policy.trial_max_route_m))
        if self.active!=key:
            if self.active is not None:return self.fail('return trial changed while active')
            if key in self.used:return self.fail('consumed return trial cannot resume')
            if self.policy.profile=='supervised_trial' and self.full_trial_consumed:return self.fail('one-time full trial already consumed')
            self.active=key;self.used.add(key);self.distance=0.;self.started_at=None;self.claimed=False;self.active_route_length=packet.get('route_length_m')
            self.last_tick=now;self.last_odom=None
        if self.policy.profile=='supervised_trial' and length!=self.active_route_length:return self.fail('full trial route length changed')
        if not isinstance(now,(int,float)) or not math.isfinite(now) or not 0<=now-self.last_tick<=.25:
            return self.fail('independent return loop stalled')
        self.last_tick=now
        if (not isinstance(odom_pose,(tuple,list)) or len(odom_pose)!=2 or
                not isinstance(measured_velocity,(tuple,list)) or len(measured_velocity)!=2 or
                not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (*odom_pose,odom_stamp,odom_age,*measured_velocity)) or
                not 0<=odom_age<=.25):return self.fail('independent odometry unavailable')
        if measured_velocity[0]>.01 or feedback_speed_exceeded(measured_velocity[0],self.policy.feedback_speed_limit):
            return self.fail('independent measured return speed exceeded')
        if self.last_odom:
            previous_stamp,previous_pose=self.last_odom
            if odom_stamp<previous_stamp:return self.fail('odometry clock reversed')
            if odom_stamp==previous_stamp and tuple(odom_pose)!=previous_pose:return self.fail('same odometry stamp changed')
            if odom_stamp>previous_stamp:
                step=math.dist(odom_pose,previous_pose)
                if step>self.max_speed*(odom_stamp-previous_stamp)+.025:
                    return self.fail('independent odometry jump')
                if self.started_at is not None:self.distance+=step
        self.last_odom=(odom_stamp,tuple(odom_pose))
        v,w=command
        if not all(math.isfinite(x) for x in (v,w)) or v>0 or abs(v)>self.max_speed+1e-9 or abs(w)>abs(v)/self.MIN_RADIUS+1e-9:
            return self.fail('independent velocity envelope exceeded')
        if self.started_at is None and (v!=0 or w!=0):self.started_at=now
        if self.policy.profile=='short_only' and self.started_at is not None:
            if now-self.started_at>=self.MAX_TIME or self.distance>=self.MAX_DISTANCE:
                return self.fail('independent short trial limit reached')
        if self.policy.profile=='supervised_trial':
            if self.started_at is not None and (now-self.started_at>=min(self.policy.trial_max_time_s,length/.03+15.) or self.distance>=length+.20):return self.fail('full trial route/time watchdog reached')
            if (v!=0 or w!=0) and not self.claimed:
                if self.claim_trial is None:return self.fail('full trial persistent claim unavailable')
                try:accepted=self.claim_trial(key,packet,now)
                except Exception:return self.fail('full trial claim storage failed')
                if not accepted:return self.fail('one-time full trial already consumed')
                self.claimed=True;self.full_trial_consumed=True
        return v,w
