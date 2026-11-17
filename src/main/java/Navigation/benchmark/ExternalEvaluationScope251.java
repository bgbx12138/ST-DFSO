package Navigation.benchmark;

import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.FsaContext;
import Navigation.Navigation3D.fsa.IntegrationAudit;
import Navigation.Navigation3D.fsa.SearchWorkLedger;
import Navigation.model.GridRouteGraph;
import Navigation.object.NavigationConfig;
import com.fasterxml.jackson.databind.MapperFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import java.security.MessageDigest;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/** Explicit launcher-owned context for unchanged external constructor signatures.
 * This scope neither starts a clock nor finishes an incumbent or response session.
 */
public final class ExternalEvaluationScope251 implements AutoCloseable {
    private static final ThreadLocal<ExternalEvaluationScope251> ACTIVE=new ThreadLocal<>();
    private static final ObjectMapper JSON=new ObjectMapper()
        .configure(MapperFeature.SORT_PROPERTIES_ALPHABETICALLY,true)
        .configure(SerializationFeature.ORDER_MAP_ENTRIES_BY_KEYS,true);
    private final Thread owner;
    private final RouteIncumbent249.Identity identity;
    private final SearchWorkLedger ledger;
    private final CandidateBinding250 binding;
    private final ObjectiveWeights compositeWeights;
    private final ObjectiveWeights fingerprintWeights;
    private final String configHash,normalizerHash,providerRevision;
    private final double maxDistance;
    private long sequence;
    private boolean closed;

    private ExternalEvaluationScope251(RouteIncumbent249.Identity identity,RouteIncumbent249 incumbent,
            NavigationConfig config,FsaContext completeSeedContext,SearchWorkLedger ledger,
            ObjectiveWeights compositeWeights,double maxDistance,String providerRevision){
        this.identity=Objects.requireNonNull(identity,"IDENTITY251_NULL");
        this.ledger=Objects.requireNonNull(ledger,"LEDGER251_NULL");
        this.compositeWeights=normalized(compositeWeights);
        // Keep raw explicit weights for repeatable one-pass fingerprint normalization.
        fingerprintWeights=new ObjectiveWeights(compositeWeights.safety,compositeWeights.time,compositeWeights.fuel);
        if(Double.isNaN(maxDistance)||maxDistance<=0.)throw new IllegalArgumentException("DISTANCE251_INVALID");
        this.maxDistance=maxDistance;
        this.providerRevision=text(providerRevision,"PROVIDER251_REVISION");
        Objects.requireNonNull(config,"CONFIG251_NULL");
        Objects.requireNonNull(completeSeedContext,"COMPLETE_SEED_CONTEXT251_NULL");
        configHash=configFingerprint(config);
        if(!configHash.equals(configFingerprint(completeSeedContext.config)))throw new IllegalArgumentException("SEED251_CONFIG_MISMATCH");
        if(!vesselFingerprint(config).equals(identity.fields().get("vessel")))throw new IllegalArgumentException("VESSEL251_IDENTITY");
        normalizerHash=normalizerFingerprint(completeSeedContext,fingerprintWeights);
        if(!normalizerHash.equals(identity.fields().get("normalizer")))throw new IllegalArgumentException("NORMALIZER251_IDENTITY");
        checkRuntime();
        binding=new CandidateBinding250(Objects.requireNonNull(incumbent,"INCUMBENT251_NULL"),identity,ledger);
        owner=Thread.currentThread();
    }
    public static ExternalEvaluationScope251 open(RouteIncumbent249.Identity identity,RouteIncumbent249 incumbent,
            NavigationConfig config,FsaContext completeSeedContext,SearchWorkLedger ledger,
            ObjectiveWeights compositeWeights,double maxDistance,String providerRevision){
        if(ACTIVE.get()!=null)throw new IllegalStateException("NESTED251_SCOPE");
        ExternalEvaluationScope251 scope=new ExternalEvaluationScope251(identity,incumbent,config,completeSeedContext,
            ledger,compositeWeights,maxDistance,providerRevision);
        ACTIVE.set(scope);return scope;
    }
    /** Called from the super-constructor argument so unbound runs do not load weather first. */
    public static NavigationConfig beforeConstruction(NavigationConfig config,SearchWorkLedger ledger){
        current(ledger).checkConfig(config);return config;
    }
    public static ExternalEvaluationScope251 current(SearchWorkLedger ledger){
        ExternalEvaluationScope251 scope=ACTIVE.get();
        if(scope==null)throw new IllegalStateException("UNBOUND251_EXTERNAL_EVALUATOR");
        scope.requireLive();
        if(scope.ledger!=ledger)throw new IllegalArgumentException("LEDGER251_DIFFERENT_RUN");
        return scope;
    }
    private void requireLive(){
        if(closed||owner!=Thread.currentThread()||ACTIVE.get()!=this)throw new IllegalStateException("INACTIVE251_SCOPE");
        checkRuntime();
    }
    private void checkRuntime(){
        if(!"false".equals(System.getProperty("nav220.screen")))throw new IllegalStateException("R2251_REQUIRES_EXPLICIT_SCREEN_FALSE");
        if(!providerRevision.equals(System.getProperty("nav.r1.tableRevision")))throw new IllegalStateException("PROVIDER251_REVISION_CHANGED");
    }
    private void checkConfig(NavigationConfig config){
        if(!configHash.equals(configFingerprint(config)))throw new IllegalArgumentException("CONFIG251_CHANGED");
    }
    /** Confirm the independently constructed native contexts share the accepted seed's bounds. */
    public void verifyEvaluator(NavigationConfig config,FsaContext context,SearchWorkLedger actualLedger){
        requireLive();checkConfig(config);
        if(ledger!=actualLedger||context.searchWorkLedger!=actualLedger)throw new IllegalArgumentException("CONTEXT251_LEDGER");
        if(!normalizerHash.equals(normalizerFingerprint(context,fingerprintWeights)))throw new IllegalArgumentException("CONTEXT251_NORMALIZER");
    }
    public RouteIncumbent249.Identity identityFor(HybridFSA3D.ObjectiveMode goal,ObjectiveWeights actualWeights,
            NavigationConfig config,GridRouteGraph actualGraph,GridRouteGraph ownedGraph,double actualDistance){
        requireLive();checkConfig(config);
        if(goal==null||!identity.objective.equals(goal.name()))throw new IllegalArgumentException("GOAL251_DIFFERENT_RUN");
        if(actualGraph!=ownedGraph)throw new IllegalArgumentException("GRAPH251_DIFFERENT_OWNER");
        if(Double.doubleToLongBits(actualDistance)!=Double.doubleToLongBits(maxDistance))throw new IllegalArgumentException("DISTANCE251_DIFFERENT_RUN");
        if(goal==HybridFSA3D.ObjectiveMode.COMPOSITE){
            ObjectiveWeights actual=normalized(actualWeights);
            if(!same(actual.safety,compositeWeights.safety)||!same(actual.time,compositeWeights.time)||!same(actual.fuel,compositeWeights.fuel))
                throw new IllegalArgumentException("COMPOSITE251_WEIGHT_MISMATCH");
        }
        // Use this call's actual goal; never replace it with a fixed COMPOSITE constant.
        return new RouteIncumbent249.Identity(goal.name(),identity.fields());
    }
    public CandidateBinding250 binding(){requireLive();return binding;}
    public String nextCandidateId(){requireLive();return "external251-"+(++sequence);}
    @Override public void close(){
        if(closed)return;
        if(owner!=Thread.currentThread()||ACTIVE.get()!=this)throw new IllegalStateException("SCOPE251_CLOSE_OWNER");
        ACTIVE.remove();closed=true;
    }

    private static String text(String value,String label){
        if(value==null||value.trim().isEmpty())throw new IllegalArgumentException(label);return value;
    }
    private static boolean same(double a,double b){return Double.doubleToLongBits(a)==Double.doubleToLongBits(b);}
    private static ObjectiveWeights normalized(ObjectiveWeights weights){
        Objects.requireNonNull(weights,"EXPLICIT_COMPOSITE251_WEIGHTS_REQUIRED");
        if(!Double.isFinite(weights.safety)||!Double.isFinite(weights.time)||!Double.isFinite(weights.fuel)
                ||weights.safety<0||weights.time<0||weights.fuel<0)throw new IllegalArgumentException("COMPOSITE251_INVALID_WEIGHTS");
        return weights.normalized();
    }
    private static String digest(Object value){
        try{
            byte[] hash=MessageDigest.getInstance("SHA-256").digest(JSON.writeValueAsBytes(value));
            StringBuilder text=new StringBuilder();for(byte b:hash)text.append(String.format("%02x",b&0xff));return text.toString();
        }catch(Exception error){throw new IllegalStateException("IDENTITY251_SERIALIZATION",error);}
    }
    public static String configFingerprint(NavigationConfig config){return digest(Objects.requireNonNull(config,"CONFIG251_NULL"));}
    public static String vesselFingerprint(NavigationConfig config){return digest(Objects.requireNonNull(config.shipParameters,"SHIP251_NULL"));}
    public static String normalizerFingerprint(FsaContext context,ObjectiveWeights compositeWeights){
        try{
            ObjectiveWeights weights=normalized(compositeWeights);Map<String,Object> payload=new LinkedHashMap<>();
            payload.put("native_normalizers",IntegrationAudit.normalizers(context));
            payload.put("composite_weights",new double[]{weights.safety,weights.time,weights.fuel});
            return digest(payload);
        }catch(Exception error){throw new IllegalStateException("NORMALIZER251_SERIALIZATION",error);}
    }
}
