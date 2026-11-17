package Navigation.Navigation3D.fsa;
import Navigation.Navigation3D.VoyageSpeedModel;
import Navigation.object.*;
import java.util.*;

/** Analytic invariants and an independently integrated midnight split; no physical provider is mocked in real smoke runs. */
public class VoyageSpeedChecks {
    static int checks=0;
    static void check(boolean b,String name){checks++;if(!b)throw new AssertionError(name);}
    static void near(double a,double b,double tol,String name){check(Math.abs(a-b)<=tol*Math.max(1,Math.abs(b)),name+": "+a+" != "+b);}
    static ShipParameters ship(){return new ShipParameters(6.15,9.256149419710122/.5144,142,19.06,8431.8);}
    static double[][] ar(double v){return new double[][]{{v,v}};}
    static EnvironmentFrame frame(double we,double wn,double ce,double cn){return new EnvironmentFrame(ar(we),ar(wn),ar(ce),ar(cn),ar(0),ar(8),ar(1));}
    public static void main(String[] args) throws Exception {
        ShipParameters s=ship();double u0=s.shipSpeed*.5144;
        VoyageSpeedModel.State calm=VoyageSpeedModel.solve(s,90,0,0,0,0),head=VoyageSpeedModel.solve(s,90,-10,0,0,0),tail=VoyageSpeedModel.solve(s,90,10,0,0,0);
        near(calm.waterSpeed,u0,1e-9,"calm equilibrium");check(head.waterSpeed<u0&&tail.waterSpeed>u0,"head and following wind");
        double last=u0;
        for(int w=0;w<=40;w++) {
            VoyageSpeedModel.State v=VoyageSpeedModel.solve(s,90,-w,0,0,0);
            check(v.valid()&&v.waterSpeed<=last+1e-8,"stronger headwind");last=v.waterSpeed;
        }
        for(int bearing=0;bearing<360;bearing+=15)for(double w:new double[]{0,5,15,30})for(double c:new double[]{-2,0,2}) {
            VoyageSpeedModel.State v=VoyageSpeedModel.solve(s,bearing,w,0,c,1);
            check(v.valid(),"supported state");near(v.balanceResidualN,0,0.001,"force equilibrium");
            double h=Math.toRadians(v.headingDeg),b=Math.toRadians(bearing);
            near(v.waterSpeed*Math.sin(h)+c,v.groundSpeed*Math.sin(b),1e-8,"east vector closure");
            near(v.waterSpeed*Math.cos(h)+1,v.groundSpeed*Math.cos(b),1e-8,"north vector closure");
            near(VoyageSpeedModel.energyJ(s,v.waterSpeed,1),VoyageSpeedModel.waterResistance(s,v.waterSpeed)*v.waterSpeed*3600,1e-12,"energy consistency");
        }
        near(VoyageSpeedModel.solve(s,90,0,10,0,0).waterSpeed,VoyageSpeedModel.solve(s,90,0,-10,0,0).waterSpeed,1e-10,"port-starboard symmetry");
        check(VoyageSpeedModel.solve(s,90,0,0,2,0).groundSpeed>calm.groundSpeed,"following current");
        check(VoyageSpeedModel.solve(s,90,0,0,-2,0).groundSpeed<calm.groundSpeed,"opposing current");
        check(!VoyageSpeedModel.solve(s,90,0,0,0,50).valid(),"impossible cross current");
        check(!VoyageSpeedModel.solve(s,90,Double.NaN,0,0,0).valid(),"missing wind remains unavailable");
        NavigationConfig cfg=new NavigationConfig();cfg.shipParameters=s;
        FsaContext ctx=new FsaContext(cfg,s,1,2,2,23,Arrays.asList(frame(0,0,0,0),frame(-15,0,1,0)),new boolean[1][2],new boolean[1][2],60,60,1,1,null,null);
        List<ResearchResponsePort.Request> requests=new ArrayList<>();
        // Test-only constant cost permits exact exposure and request-speed assertions.
        ResearchResponsePort.Provider provider=r->{requests.add(r);return new ResearchResponsePort.Response(ResearchResponsePort.Status.OK,"TEST",new double[]{1,1,1,1},"TEST","TEST","TEST","TEST","false");};
        try(ResearchResponsePort.Session scope=new ResearchResponsePort.Session(provider)){
            EdgeSampler.EdgeResult all=EdgeSampler.sample(ctx,0,0,0,1,0,EdgeSampler.NeedComponents.all(),true);
            VoyageSpeedModel.State second=VoyageSpeedModel.solve(s,90,-15,0,1,0);
            double remain=60*1852-calm.groundSpeed*3600,expected=1+remain/second.groundSpeed/3600;
            check(all.valid,"cross-midnight route");near(all.timeHours,expected,1e-10,"split at midnight using actual speed");
            near(all.fuelEnergyJ,VoyageSpeedModel.energyJ(s,calm.waterSpeed,1)+VoyageSpeedModel.energyJ(s,second.waterSpeed,expected-1),1e-10,"split energy");
            check(requests.stream().anyMatch(r->r.windE==0)&&requests.stream().anyMatch(r->r.windE==-15),"both daily frames used");
            for(ResearchResponsePort.Request r:requests)near(r.U,r.windE==0?calm.waterSpeed:second.waterSpeed,1e-10,"response uses actual water speed");
            EdgeSampler.EdgeResult onlyTime=EdgeSampler.sample(ctx,0,0,0,1,0,EdgeSampler.NeedComponents.timeOnly(),true);
            near(onlyTime.timeHours,all.timeHours,1e-12,"objective-independent clock");
        }
        System.out.printf(Locale.ROOT,"PASS %d checks; calm=%.9f head10=%.9f following10=%.9f m/s; nominal_RPM=%.6f%n",checks,calm.waterSpeed,head.waterSpeed,tail.waterSpeed,60*calm.rps);
    }
}
