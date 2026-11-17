package Navigation.benchmark;
import Navigation.Navigation3D.VoyageSpeedModel;
import Navigation.Navigation3D.fsa.*;
import Navigation.object.NavigationConfig;
import java.util.*;
public class VoyageEnvironmentChecks {
    public static void main(String[] args)throws Exception {
        for(BenchmarkLauncher.Scenario scenario:BenchmarkLauncher.SCENARIOS) {
            NavigationConfig cfg=BenchmarkLauncher.buildConfig(scenario);cfg.showVisualization=false;
            cfg.shipParameters=new Navigation.object.ShipParameters(6.15,9.256149419710122/.5144,142,19.06,8431.8);
            cfg.enableCompositeNormalization=false;
            long tested=0,unavailable=0;double low=Double.POSITIVE_INFINITY,high=0,maxResidual=0;
            try(PathEvaluator p=new PathEvaluator(cfg)){
                FsaContext c=IntegrationAudit.context(p);
                for(EnvironmentFrame f:c.frames)for(int i=0;i<c.ny;i++)for(int j=0;j<c.nx;j++)if(!c.blocked(i,j))
                for(double b:FsaContext.BEARINGS){
                    double we=f.windUData[i][j],wn=f.windVData[i][j],ce=f.currentUData[i][j],cn=f.currentVData[i][j];
                    if(!Double.isFinite(we+wn+ce+cn))continue;
                    VoyageSpeedModel.State v=VoyageSpeedModel.solve(c.ship,b,we,wn,ce,cn);
                    if(!v.valid()){unavailable++;continue;}
                    tested++;low=Math.min(low,v.waterSpeed);high=Math.max(high,v.waterSpeed);maxResidual=Math.max(maxResidual,Math.abs(v.balanceResidualN));
                    double ratio=v.waterSpeed/(c.ship.shipSpeed*.5144);
                    if(ratio<.95||ratio>1.05)throw new AssertionError("OUTSIDE_RESPONSE_DOMAIN");
                }
            }
            System.out.printf(Locale.ROOT,"%s states=%d unavailable=%d water_speed_min=%.9f max=%.9f max_balance_residual_N=%.9f%n",scenario.id,tested,unavailable,low,high,maxResidual);
        }
    }
}
