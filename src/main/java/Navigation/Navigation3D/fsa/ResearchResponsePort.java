package Navigation.Navigation3D.fsa;

import Navigation.Navigation3D.VoyageSpeedModel;
import Navigation.Navigation3D.safety.ResolvedDynamicResponseCost;
import Navigation.object.ShipParameters;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;

/** Scoped research cost. Frozen dynamics; explicit supported approximation, no finite failure penalty or safety verdict. */
public final class ResearchResponsePort {
    private ResearchResponsePort() {}
    public static final String CONFIG = "5415M_MARIN35.48_active_coursekeeping";
    private static final InheritableThreadLocal<Session> ACTIVE = new InheritableThreadLocal<>();
    public enum Status { OK, APPROXIMATED, SOLVER_FAILURE, TIMEOUT, MISSING_RESPONSE, UNSUPPORTED_INPUT, CONTRACT_FAILURE }
    public static boolean isPriced(Status s){return s==Status.OK||s==Status.APPROXIMATED;}
    public static double wrap(double x) { double y=x%360.;return y<0?y+360:y; }
    public static final class Request {
        public final double U,heading,windE,windN,currentE,currentN,H,waveFrom,Tp;
        public final String configuration;
        public Request(double U,double heading,double windE,double windN,double currentE,double currentN,double H,double waveFrom,double Tp,String configuration) {
            for(double v:new double[]{U,heading,windE,windN,currentE,currentN,H,waveFrom,Tp})
                if(!Double.isFinite(v))throw new IllegalArgumentException("NONFINITE_REQUEST");
            if(U<=0||H<0||Tp<=0)throw new IllegalArgumentException("INVALID_REQUEST");
            this.U=U;this.heading=wrap(heading);this.windE=windE;this.windN=windN;
            this.currentE=currentE;this.currentN=currentN;this.H=H;this.waveFrom=wrap(waveFrom);this.Tp=Tp;
            this.configuration=required(configuration,"configuration");
        }
        /** Exact double-bit key; neither heading tolerance nor rounded nearest-state lookup. */
        public String key() {
            StringBuilder b=new StringBuilder(configuration);
            for(double x:new double[]{U,heading,windE,windN,currentE,currentN,H,waveFrom,Tp})
                b.append('|').append(Long.toHexString(Double.doubleToLongBits(x==0?0.:x)));
            return b.toString();
        }
        public static Request from(Map<String,String> r) {
            return new Request(num(r,"U_m_s"),num(r,"heading_through_water_deg"),num(r,"wind_east_mps"),num(r,"wind_north_mps"),
                num(r,"current_east_mps"),num(r,"current_north_mps"),num(r,"Hm0_m"),num(r,"wave_from_trueNorth_deg"),num(r,"Tp_s"),required(r.get("configuration"),"configuration"));
        }
        public String csv() {return U+","+heading+","+windE+","+windN+","+currentE+","+currentN+","+H+","+waveFrom+","+Tp+","+configuration;}
    }
    public static final class Response {
        public final Status status;public final String reason,inputSha,solverSha,seed,sourceAnchor,conditionalTransfer;
        public final Double cost;public final double[] sd;public final String provenance;
        Response(Status status,String reason,double[] sd,String inputSha,String solverSha,String seed,String sourceAnchor,String transfer) {
            this(status,reason,sd,inputSha,solverSha,seed,sourceAnchor,transfer,"{}");
        }
        Response(Status status,String reason,double[] sd,String inputSha,String solverSha,String seed,String sourceAnchor,String transfer,String provenance) {
            this.provenance=provenance;
            this.status=status;this.reason=required(reason,"reason_code");this.inputSha=required(inputSha,"input_sha256");
            this.solverSha=required(solverSha,"solver_sha256");this.seed=required(seed,"seed");
            this.sourceAnchor=required(sourceAnchor,"source_anchor");this.conditionalTransfer=required(transfer,"conditional_transfer");
            if(!Arrays.asList("true","false").contains(transfer))throw new IllegalArgumentException("conditional_transfer boolean");
            if(!isPriced(status)) {if(sd!=null)throw new IllegalArgumentException("FAILED_RESPONSE_HAS_MEASURES");this.sd=null;cost=null;return;}
            if(sd==null||sd.length!=4)throw new IllegalArgumentException("FOUR_SD_REQUIRED");
            this.sd=sd.clone();
            ResolvedDynamicResponseCost.Measure[] m=new ResolvedDynamicResponseCost.Measure[4];
            ResolvedDynamicResponseCost.Statistic[] types=ResolvedDynamicResponseCost.Statistic.values();
            for(int i=0;i<4;i++)m[i]=new ResolvedDynamicResponseCost.Measure(sd[i],types[i],types[i].unit);
            ResolvedDynamicResponseCost.Evidence e=new ResolvedDynamicResponseCost.Evidence(inputSha,"SOURCE_BACKED_CONDITIONAL_RESEARCH",
                ResolvedDynamicResponseCost.EvidenceStatus.NOT_ADMITTED,ResolvedDynamicResponseCost.DomainStatus.UNKNOWN,false);
            cost=ResolvedDynamicResponseCost.score(m,e,ResolvedDynamicResponseCost.Mode.DIAGNOSTIC).rawTaskCost;
        }
        static Response unavailable(Status s,String reason) {return new Response(s,reason,null,"UNKNOWN","UNKNOWN","UNKNOWN","NONE","true");}
    }
    public interface Provider { Response response(Request r); }
    public static final class Registry implements Provider {
        final Map<String,Response> values=new LinkedHashMap<>();
        public final String sha256;public final Set<String> solverVersions=new LinkedHashSet<>();
        public Registry(Path file,String expectedSha) throws Exception {
            sha256=sha(file);if(!sha256.equals(expectedSha))throw new IllegalArgumentException("REGISTRY_HASH_MISMATCH");
            for(Map<String,String> row:table(file)) {
                Request r=Request.from(row);Status status=Status.valueOf(required(row.get("response_status"),"response_status"));
                double[] sd=null;String[] names={"roll_centered_sd_deg","heading_centered_sd_deg","sway_centered_sd_m_s","surge_centered_sd_m_s"};
                if(isPriced(status)) {sd=new double[4];for(int i=0;i<4;i++)sd[i]=num(row,names[i]);}
                else for(String n:names)if(!row.getOrDefault(n,"").isEmpty())throw new IllegalArgumentException("FAILED_RESPONSE_LEAKED_SD");
                Response value=new Response(status,row.get("reason_code"),sd,row.get("input_sha256"),row.get("solver_sha256"),row.get("seed"),row.get("source_anchor"),row.get("conditional_transfer"));
                if(!value.inputSha.matches("[0-9a-f]{64}"))throw new IllegalArgumentException("INPUT_SHA_REQUIRED");
                if(values.putIfAbsent(r.key(),value)!=null)throw new IllegalArgumentException("DUPLICATE_EXACT_REQUEST");
                solverVersions.add(value.solverSha);
            }
        }
        public Response response(Request r) {
            if(!CONFIG.equals(r.configuration))return Response.unavailable(Status.UNSUPPORTED_INPUT,"CONFIGURATION_UNSUPPORTED");
            Response response=values.get(r.key());
            return response==null?Response.unavailable(Status.MISSING_RESPONSE,"EXACT_REQUEST_NOT_IN_REGISTRY"):response;
        }
        public int size(){return values.size();}
    }
    public static final class Coverage {
        public double validNm,failedNm,missingNm,unsupportedNm,validCostDistance,totalNm;
        public final Map<String,Double> reasons=new LinkedHashMap<>();
        public final Set<String> sourceAnchors=new LinkedHashSet<>(),transfers=new LinkedHashSet<>();
        public void add(double nm,Response r) {
            if(!Double.isFinite(nm)||nm<0)throw new IllegalArgumentException("INVALID_DISTANCE");
            IntegrationAudit.observe(nm,r);
            totalNm+=nm;sourceAnchors.add(r.sourceAnchor);transfers.add(r.conditionalTransfer);
            switch(r.status) {
                case OK:case APPROXIMATED:validNm+=nm;validCostDistance+=r.cost*nm;break;
                case MISSING_RESPONSE:missingNm+=nm;break;
                case UNSUPPORTED_INPUT:unsupportedNm+=nm;break;
                default:failedNm+=nm;
            }
            if(!isPriced(r.status))reasons.merge(r.status.name()+":"+r.reason,nm,Double::sum);
        }
        public boolean complete(){return failedNm==0&&missingNm==0&&unsupportedNm==0;}
        public String json(String route) {
            StringBuilder reason=new StringBuilder("{");for(Map.Entry<String,Double> e:reasons.entrySet()) {if(reason.length()>1)reason.append(',');reason.append(j(e.getKey())).append(':').append(e.getValue());}reason.append('}');
            return "{\"route\":"+j(route)+",\"valid_nm\":"+validNm+",\"failed_nm\":"+failedNm+",\"missing_nm\":"+missingNm+",\"unsupported_nm\":"+unsupportedNm+
                ",\"total_nm\":"+totalNm+",\"priced_partial_cost_distance\":"+validCostDistance+",\"task_cost_distance\":"+(complete()?Double.toString(validCostDistance):"null")+
                ",\"task_cost_density\":"+(complete()&&totalNm>0?Double.toString(validCostDistance/totalNm):"null")+",\"complete\":"+complete()+",\"reason_nm\":"+reason+
                ",\"source_anchor_labels\":"+strings(sourceAnchors)+",\"conditional_transfer_labels\":"+strings(transfers)+",\"physical_unsafe_inferred\":false}";
        }
    }
    public static final class Session implements AutoCloseable {
        final Provider provider;public Coverage lastEdgeCoverage=new Coverage();public final Map<String,Request> missingRequests=new LinkedHashMap<>();
        public Session(Provider provider) {if(ACTIVE.get()!=null)throw new IllegalStateException("NESTED_RESEARCH_SESSION");this.provider=Objects.requireNonNull(provider);ACTIVE.set(this);}
        public void close(){if(ACTIVE.get()!=this)throw new IllegalStateException("WRONG_RESEARCH_SESSION");ACTIVE.remove();}
    }
    public static boolean installed(){return ACTIVE.get()!=null;}
    public static Response requestResponse(Request request){
        Session s=ACTIVE.get();if(s==null)throw new IllegalStateException("RESEARCH_SESSION_REQUIRED");
        Response r=s.provider.response(Objects.requireNonNull(request));
        return r==null?Response.unavailable(Status.CONTRACT_FAILURE,"PROVIDER_RETURNED_NULL"):r;
    }
    public static EdgeSampler.EdgeResult sample(FsaContext c,int ai,int aj,int bi,int bj,double start,EdgeSampler.NeedComponents need,boolean correction) {
        Session s=ACTIVE.get();if(s==null)throw new IllegalStateException("RESEARCH_SESSION_REQUIRED");s.lastEdgeCoverage=new Coverage();
        if(!loading(c.ship)) {s.lastEdgeCoverage.add(Math.max(0,c.gridDistanceNm(ai,aj,bi,bj)),Response.unavailable(Status.UNSUPPORTED_INPUT,"VESSEL_LOADING_UNSUPPORTED"));return EdgeSampler.EdgeResult.invalid();}
        if(!c.inBounds(ai,aj)||!c.inBounds(bi,bj)||c.blocked(ai,aj)||c.blocked(bi,bj))return EdgeSampler.EdgeResult.invalid();
        if(!Double.isFinite(start)||start<0)throw new IllegalArgumentException("INVALID_ELAPSED_TIME");
        if(c.searchWorkLedger!=null)c.searchWorkLedger.recordEdgeSampleInvocation();
        double distance=c.gridDistanceNm(ai,aj,bi,bj),bearing=c.bearingOf(ai,aj,bi,bj),rad=Math.toRadians(bearing),U=c.ship.shipSpeed*FsaContext.KNOT_TO_MS;
        if(distance<=0||!Double.isFinite(distance)||U<=0)return EdgeSampler.EdgeResult.invalid();
        int count=Math.max(1,(int)Math.ceil(Math.max(Math.abs(bi-ai),Math.abs(bj-aj))/.5));
        double elapsed=start,hours=0,fuel=0,risk=0;
        for(int k=0;k<count;k++) {
            int i=Math.max(0,Math.min(c.ny-1,(int)Math.round(ai+(bi-ai)*(k+.5)/count))),q=Math.max(0,Math.min(c.nx-1,(int)Math.round(aj+(bj-aj)*(k+.5)/count)));
            if(c.blocked(i,q))return EdgeSampler.EdgeResult.invalid();double remaining=distance/count;
            while(remaining>1e-12) {
                if(!c.hasWeatherAt(elapsed))return EdgeSampler.EdgeResult.invalid();
                if(c.searchWorkLedger!=null&&!c.searchWorkLedger.tryRecordEdgeSubsegmentAttempt())return EdgeSampler.EdgeResult.invalid();
                int day=c.clampDay(elapsed);EnvironmentFrame f=c.frames.get(day);
                double we=f.windUData[i][q],wn=f.windVData[i][q],ce=f.currentUData[i][q],cn=f.currentVData[i][q];
                if(!Double.isFinite(we+wn+ce+cn))return EdgeSampler.EdgeResult.invalid();
                VoyageSpeedModel.State speed=VoyageSpeedModel.solve(c.ship,bearing,we,wn,ce,cn);
                if(!speed.valid())return EdgeSampler.EdgeResult.invalid();
                double vg=speed.groundSpeed;
                if(vg<=0||!Double.isFinite(vg))return EdgeSampler.EdgeResult.invalid();
                double dt=Math.min(remaining*1852/vg/3600,(day+1)*24.-c.departureOffsetHours-elapsed);
                if(dt<=0)throw new IllegalStateException("CLOCK_NO_PROGRESS");double nm=vg*dt*3600/1852;
                if(need.needSafety) {
                    double H=f.waveHeightData[i][q],wd=f.waveDirData[i][q],Tp=f.wavePeriodData[i][q];
                    Response r;Request request=null;
                    try {
                        request=new Request(speed.waterSpeed,speed.headingDeg,we,wn,ce,cn,H,wd,Tp,CONFIG);
                    } catch(IllegalArgumentException e) {r=Response.unavailable(Status.UNSUPPORTED_INPUT,"INVALID_ENVIRONMENT_REQUEST");}
                    if(request==null)r=Response.unavailable(Status.UNSUPPORTED_INPUT,"INVALID_ENVIRONMENT_REQUEST");
                    else {
                        try {r=s.provider.response(request);if(r==null)r=Response.unavailable(Status.CONTRACT_FAILURE,"PROVIDER_RETURNED_NULL");}
                        catch(RuntimeException e) {throw e;} // Resource control and genuine provider faults are not physical response labels.
                    }
                    s.lastEdgeCoverage.add(nm,r);
                    if(r.status==Status.MISSING_RESPONSE&&request!=null)s.missingRequests.putIfAbsent(request.key(),request);
                    if(isPriced(r.status))risk+=r.cost*nm;
                }
                if(need.needFuel) {
                    double energy=VoyageSpeedModel.energyJ(c.ship,speed.waterSpeed,dt);
                    if(!Double.isFinite(energy)||energy<0)return EdgeSampler.EdgeResult.invalid();fuel+=energy;
                }
                elapsed+=dt;hours+=dt;remaining=Math.max(0,remaining-nm);
            }
        }
        if(!s.lastEdgeCoverage.complete())return EdgeSampler.EdgeResult.invalid();
        // Legacy safetyWeighted is deliberately unavailable: no 1-cost interpretation.
        return new EdgeSampler.EdgeResult(distance,hours,fuel,Double.NaN,risk,null,elapsed,true);
    }
    private static boolean loading(ShipParameters s){return Math.abs(s.shipLength-142)<1e-7&&Math.abs(s.shipWidth-19.06)<1e-7&&Math.abs(s.shipDraft-6.15)<1e-7&&Math.abs(s.shipDisplacement-8431.8)<1e-7;}
    static String required(String v,String name){if(v==null||v.isEmpty())throw new IllegalArgumentException("MISSING_"+name);return v;}
    static double num(Map<String,String> m,String name){double v=Double.parseDouble(required(m.get(name),name));if(!Double.isFinite(v))throw new IllegalArgumentException("NONFINITE_"+name);return v;}
    static String sha(Path p)throws Exception{StringBuilder b=new StringBuilder();for(byte x:MessageDigest.getInstance("SHA-256").digest(Files.readAllBytes(p)))b.append(String.format("%02x",x&255));return b.toString();}
    static List<String> fields(String line) {
        List<String> out=new ArrayList<>();StringBuilder b=new StringBuilder();boolean quoted=false;
        for(int i=0;i<line.length();i++){char c=line.charAt(i);if(c=='"'){if(quoted&&i+1<line.length()&&line.charAt(i+1)=='"'){b.append('"');i++;}else quoted=!quoted;}else if(c==','&&!quoted){out.add(b.toString());b.setLength(0);}else b.append(c);}
        if(quoted)throw new IllegalArgumentException("MULTILINE_CSV_UNSUPPORTED");out.add(b.toString());return out;
    }
    static List<Map<String,String>> table(Path p)throws IOException {
        List<String> lines=Files.readAllLines(p,StandardCharsets.UTF_8);if(lines.isEmpty())throw new IOException("EMPTY_CSV");
        String first=lines.get(0);if(first.startsWith("\ufeff"))first=first.substring(1);List<String> names=fields(first);
        if(new HashSet<>(names).size()!=names.size())throw new IOException("DUPLICATE_CSV_HEADER");
        List<Map<String,String>> out=new ArrayList<>();
        for(int i=1;i<lines.size();i++){if(lines.get(i).isEmpty())continue;List<String> cells=fields(lines.get(i));if(cells.size()!=names.size())throw new IOException("CSV_FIELD_COUNT:"+(i+1));Map<String,String> row=new LinkedHashMap<>();for(int k=0;k<names.size();k++)row.put(names.get(k),cells.get(k));out.add(row);}return out;
    }
    static String j(String s){return "\""+s.replace("\\","\\\\").replace("\"","\\\"").replace("\n","\\n").replace("\r","\\r")+"\"";}
    static String strings(Collection<String> values){List<String> s=new ArrayList<>();for(String v:values)s.add(j(v));return "["+String.join(",",s)+"]";}
    public static void main(String[] args)throws Exception {
        if(args.length!=5)throw new IllegalArgumentException("REGISTRY EXPECTED_SHA SEGMENTS_CSV NEW_OUTPUT_JSON NEW_MISSING_REQUESTS_CSV");
        Registry registry=new Registry(Paths.get(args[0]),args[1]);Map<String,Coverage> routes=new LinkedHashMap<>();Map<String,Request> missing=new LinkedHashMap<>();
        for(Map<String,String> row:table(Paths.get(args[2]))) {
            String route=required(row.get("route"),"route");Coverage c=routes.computeIfAbsent(route,k->new Coverage());Request r=null;Response response;
            try {r=Request.from(row);response=registry.response(r);}catch(IllegalArgumentException e){response=Response.unavailable(Status.UNSUPPORTED_INPUT,"INVALID_SEGMENT_REQUEST");}
            c.add(num(row,"distance_m")/1852.,response);if(response.status==Status.MISSING_RESPONSE)missing.putIfAbsent(r.key(),r);
        }
        List<String> reports=new ArrayList<>();for(Map.Entry<String,Coverage> e:routes.entrySet())reports.add(e.getValue().json(e.getKey()));
        String result="{\"schema\":\"R1_RESEARCH_ROUTE_COST_V1\",\"registry_sha256\":"+j(registry.sha256)+",\"solver_versions\":"+strings(registry.solverVersions)+
            ",\"uniform_lookup_and_policy_for_all_algorithm_labels\":true,\"strict_unpriced_failure\":true,\"heading_tolerance_used_for_admission\":false,\"physical_validation\":false,\"overall_safe_verdict\":null,\"routes\":["+String.join(",",reports)+"]}";
        Files.write(Paths.get(args[3]),result.getBytes(StandardCharsets.UTF_8),StandardOpenOption.CREATE_NEW);
        List<String> requests=new ArrayList<>();requests.add("U_m_s,heading_through_water_deg,wind_east_mps,wind_north_mps,current_east_mps,current_north_mps,Hm0_m,wave_from_trueNorth_deg,Tp_s,configuration");for(Request r:missing.values())requests.add(r.csv());
        Files.write(Paths.get(args[4]),requests,StandardCharsets.UTF_8,StandardOpenOption.CREATE_NEW);System.out.println("routes="+routes.size()+";missing_requests="+missing.size()+";dynamics_runs=0");
    }
}
