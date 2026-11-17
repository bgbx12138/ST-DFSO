package Navigation.benchmark;

import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.FsaContext;
import Navigation.Navigation3D.fsa.IntegrationAudit;
import Navigation.Navigation3D.fsa.SearchWorkLedger;
import Navigation.model.GridPoint;
import Navigation.model.GridRouteGraph;
import Navigation.object.NavigationConfig;
import java.io.IOException;
import java.lang.reflect.Field;
import java.util.List;

/** Independent251 overlay. Existing algorithms keep their constructor and geometry API. */
public final class FairPathEvaluator166 extends PathEvaluator {
    private final SearchWorkLedger work;
    private final NavigationConfig config;
    private final ExternalEvaluationScope251 scope;
    // Same native object, not another context: IntegrationAudit.context uses declared fields.
    private final FsaContext ctxComposite;

    public FairPathEvaluator166(NavigationConfig config,SearchWorkLedger work)throws IOException{
        super(ExternalEvaluationScope251.beforeConstruction(config,work),work);
        this.work=work;this.config=config;scope=ExternalEvaluationScope251.current(work);
        try{
            Field field=PathEvaluator.class.getDeclaredField("ctxComposite");
            field.setAccessible(true);ctxComposite=(FsaContext)field.get(this);
        }catch(ReflectiveOperationException error){throw new IOException("NATIVE251_CONTEXT_BINDING",error);}
        scope.verifyEvaluator(config,ctxComposite,work);
    }
    public boolean extensionIsPractical(GridPoint a,GridPoint b,GridPoint c){
        if(a==null)return true;
        double x=(b.j-a.j)*ctxComposite.dLonNm,y=(b.i-a.i)*ctxComposite.dLatNm;
        double u=(c.j-b.j)*ctxComposite.dLonNm,v=(c.i-b.i)*ctxComposite.dLatNm;
        return !(x*u+y*v<0&&Math.abs(x*v-y*u)<1e-10*Math.max(1,Math.hypot(x,y)*Math.hypot(u,v)));
    }
    @Override public double evaluateStrict(List<GridPoint> route,HybridFSA3D.ObjectiveMode goal,
            ObjectiveWeights weights,GridRouteGraph graph,double max){
        if(ExternalEvaluationScope251.current(work)!=scope)throw new IllegalStateException("EVALUATOR251_DIFFERENT_SCOPE");
        RouteIncumbent249.Identity actual=scope.identityFor(goal,weights,config,graph,getRouteGraph(),max);
        return scope.binding().evaluate(actual,scope.nextCandidateId(),route,()->{
            if(!IntegrationAudit.practical(route,ctxComposite)){
                work.recordRouteObjectiveEvaluation();work.recordOperatorEvent("common_practical_rejections166");
                return Double.POSITIVE_INFINITY;
            }
            return super.evaluateStrict(route,goal,weights,graph,max);
        });
    }
}
