"""Six second-round contracts, saved first-round tests are not repeated."""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent/'source'))
from stack.core.gap_events import forecast_events,generate_gap_events
from stack.core.candidates import Candidate,generate
from stack.core.geometry import Path as RoutePath
from stack.core.arbiter import Arbiter,ArbiterConfig,EgoState
from stack.core.control import ControlAdapter,LateralTracker
from stack.core.native_execution import rollout_projected


class EventV2Contracts(unittest.TestCase):
    def setup(self,dt=.1,speed=10.):
        arb=Arbiter(ArbiterConfig(dt=dt))
        x=np.arange(201.);y=6.*np.clip((x-30.)/60.,0.,1.)
        path=RoutePath(np.column_stack((x,y)))
        ego=EgoState(10.,0.,0.,speed,0.,4.5,1.8)
        t=arb.t
        pos=np.zeros((3,1,len(t),2));pos[:,:,:,0]=(-20.+20.*t)[None,None,:];pos[:,:,:,1]=3.
        ev=SimpleNamespace(times=t,positions=pos,headings=np.zeros(pos.shape[:-1]),
            enabled=np.ones(pos.shape[:-1],bool),
            prediction=dict(ids=['vehicle/a'],now=np.array([[-20.,3.]]),L=np.array([4.5]),W=np.array([1.8])))
        c=[Candidate('progress','envelope_progress',np.zeros(len(t)),original_reference=False)]
        a=SimpleNamespace(first_hit=np.array([3.]),eta=np.array([9.]),collision_actor=np.array([0]),
            collision_branch=np.array([0]),stations=np.full((1,len(t)),65.))
        return arb,path,ego,ev,c,a

    def output(self,args,cap=10.,held=None,timestamp=0.):
        arb,path,ego,ev,c,a=args
        return generate_gap_events(ev,a,c,arb,ego,10.,np.arange(201.),np.full(201,cap),15.,
            path,0.,ControlAdapter(),LateralTracker(),timestamp,committed=held)

    def test_01_after_event_uses_actual_conflict_station(self):
        arb,path,e,ev,c,a=self.setup()
        events=forecast_events(ev,a,c,path,10.,e,0.,arb.cfg)
        after=next(x for x in events if x['side']=='after')
        self.assertEqual(after['station'],65.)
        self.assertTrue(any(abs(after['deadline_abs']-t)<1e-7 for t in arb.t))

    def test_02_disabled_unknown_not_extrapolated(self):
        arb,path,e,ev,c,a=self.setup();ev.enabled[:]=False
        self.assertEqual(forecast_events(ev,a,c,path,10.,e,0.,arb.cfg),[])

    def test_03_limit_is_applied_through_official_next_state(self):
        args=self.setup(speed=9.)
        out=self.output(args,cap=9.1234)
        arb,path,ego,_,_,_=args
        commands=np.stack([c.acceleration for c in out])
        states,_,_=rollout_projected(np.array([10.,0.,0.,9.,0.,0.]),commands,path,10.,
            ControlAdapter(),LateralTracker(),.1,4.5)
        self.assertLessEqual(states[:,1:,3].max(),9.1234+1e-7)

    def test_04_physically_impossible_state_not_clipped(self):
        args=self.setup(speed=20.)
        out=self.output(args,cap=5.)
        arb,path,ego,_,_,_=args
        states,_,_=rollout_projected(np.array([10.,0.,0.,20.,0.,0.]),np.stack([c.acceleration for c in out]),
            path,10.,ControlAdapter(),LateralTracker(),.1,4.5)
        self.assertTrue(np.all(states[:,0,3]==20.))
        self.assertTrue(np.all(states[:,1,3]>5.))

    def test_05_absolute_deadline_and_short_grid(self):
        args=self.setup(dt=.04)
        held=dict(side='after',actor='vehicle/a',station=65.,deadline_abs=4.,created_at=0.)
        out=self.output(args,held=held,timestamp=1.)
        self.assertEqual(out[0].event['deadline_abs'],4.)
        self.assertEqual(len(out[0].acceleration),150)

    def test_06_original_candidates_and_bound_preserved(self):
        args=self.setup();arb,path,e,_,_,_=args
        ss,vv=np.arange(201.),np.full(201,10.)
        old=generate(arb,e,10.,ss,vv,15.)
        out=self.output(args)
        again=generate(arb,e,10.,ss,vv,15.)
        self.assertLessEqual(len(out),4)
        self.assertTrue(all(not c.original_reference for c in out))
        self.assertEqual([x.id for x in old],[x.id for x in again])
        self.assertTrue(all(np.array_equal(x.acceleration,y.acceleration) for x,y in zip(old,again)))


if __name__=='__main__':unittest.main(verbosity=2)
