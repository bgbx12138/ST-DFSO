package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import java.util.*;

/** v5 first increment: TIME-only, real-prefix, two-edit conditional cooperation.
 * Independent of environmental/safety formulas. All replay goes through Host.
 */
public final class ConditionalCooperation {
    public static final class Options {
        public final boolean demand, memory, relay;
        public final String mode;
        public Options(String bits) {
            if (bits == null || !(bits.matches("[01]{3}") || bits.equals("ARC_RAW") || bits.equals("ARC_CTX")))
                throw new IllegalArgumentException("expected M1M2M3 bits, ARC_RAW or ARC_CTX");
            mode=bits;
            demand = bits.matches("1[01]{2}");
            memory = bits.startsWith("ARC_") || bits.charAt(1)=='1';
            relay = !bits.startsWith("ARC_") && bits.charAt(2)=='1';
        }
    }
    public interface Host {
        Trace replay(List<GridPoint> path);
        List<GridPoint> propose(List<GridPoint> path, int left, int right, Random random);
        double distance(GridPoint a, GridPoint b);
        default double heading(GridPoint a,GridPoint b) {return Math.toDegrees(Math.atan2(b.j-a.j,b.i-a.i));}
        boolean allowed();
        long work();
        void event(String event);
        default double incumbentCost() {return Double.POSITIVE_INFINITY;}
    }
    public static final class Trace {
        public final List<GridPoint> path;
        public final double cost;
        public final double[] arrival;
        public Trace(List<GridPoint> path, double cost, double[] arrival) {
            if (!Double.isFinite(cost) || arrival.length != path.size()) throw new IllegalArgumentException("incomplete trace");
            this.path=copy(path); this.cost=cost; this.arrival=arrival.clone();
        }
    }
    private static final double EPS=1e-9;
    private final Options options;
    private final Host host;
    private final Random random;
    private final Map<Integer,Deque<List<GridPoint>>> recent=new HashMap<>();
    private final Map<Integer,List<Suggestion>> archive=new HashMap<>();
    private final Map<Integer,Request> requests=new HashMap<>();
    private final LinkedHashMap<String,double[]> feedback=new LinkedHashMap<>();
    private final List<Map<String,Object>> witnesses=new ArrayList<>();
    private final List<Map<String,Object>> attribution=new ArrayList<>();
    private final Map<Integer,History> history=new HashMap<>();
    // Guarded by this: the ordinary worker records evaluations while the coordinator
    // collects earlier futures. The remaining cooperation state is coordinator-only.
    private final LinkedHashMap<String,Double> sourceCosts=new LinkedHashMap<>();
    private final Set<String> strictStructures=new HashSet<>();
    private double knownBest=Double.POSITIVE_INFINITY; // guarded by this, including reads
    private double stepKnownBefore;
    private Trace stepBest;
    private String stepStage;
    private Suggestion pendingReward;
    private double pendingRewardGain;
    private String pendingRewardRoute;
    private long requestSerial;
    private static final class History {
        final String version;
        final LinkedHashMap<String,Set<String>> tried=new LinkedHashMap<>();
        History(String version){this.version=version;}
    }
    private long turn;
    private int round;
    private static final class Suggestion {
        final GridPoint left,right;
        final List<GridPoint> replacement;
        final String lineage;
        String evidenceVersion;
        double negativeGain=Double.NaN, positiveGain=Double.NaN, targetArrival=Double.NaN;
        Trace positiveBase, positiveEdit;
        double targetHeading;
        double sourceCost=Double.POSITIVE_INFINITY;
        int born, used;
        Suggestion(List<GridPoint> fragment,int round) {
            replacement=copy(fragment);left=replacement.get(0);right=replacement.get(replacement.size()-1);
            lineage=key(replacement);born=used=round;
        }
    }
    private static final class Request {
        final Trace base, singleB;
        final Suggestion b;
        final int born;
        final long id;
        final Set<String> tried=new HashSet<>();
        int attempts, services;
        Request(Trace base,Trace singleB,Suggestion b,int born,long id) {
            this.base=base;this.singleB=singleB;this.b=b;this.born=born;this.id=id;
        }
    }
    public ConditionalCooperation(Options options,Host host,long seed) {
        this.options=options;this.host=host;this.random=new Random(seed);
    }
    /** Only ordinary, already-evaluated routes enter the four-item working cache. */
    public void observe(int owner,List<GridPoint> path) {
        observe(owner,path,Double.POSITIVE_INFINITY);
    }
    public void observe(int owner,List<GridPoint> path,double cost) {
        if (path==null || path.size()<2) return;
        recordEvaluation(path,cost);
        Deque<List<GridPoint>> q=recent.computeIfAbsent(owner,x->new ArrayDeque<>());
        if (!q.isEmpty() && key(q.peekLast()).equals(key(path))) return;
        q.addLast(copy(path)); while(q.size()>4) q.removeFirst();
    }
    public void beginRound(int currentRound) {
        round=currentRound;
        for(Map.Entry<Integer,List<Suggestion>> e:archive.entrySet()) {
            Request active=requests.get(e.getKey());
            e.getValue().removeIf(s->(active==null || active.b!=s) && (!options.memory ||
                    (Double.isNaN(s.positiveGain) && round-s.born>=2)));
        }
    }
    public List<Map<String,Object>> witnesses() { return new ArrayList<>(witnesses); }
    public List<Map<String,Object>> attribution() {return new ArrayList<>(attribution);}
    /** Completed ordinary evaluations count as known even if not admitted to a personal archive. */
    public synchronized void recordEvaluation(List<GridPoint> path,double cost) {
        if(path==null || !Double.isFinite(cost))return;
        knownBest=Math.min(knownBest,cost);
        sourceCosts.put(key(path),cost);
        while(sourceCosts.size()>4096)sourceCosts.remove(sourceCosts.keySet().iterator().next());
    }
    private synchronized double knownEvaluatedBest() {return knownBest;}
    private synchronized double sourceCost(List<GridPoint> path) {
        return sourceCosts.getOrDefault(key(path),Double.POSITIVE_INFINITY);
    }
    public static int ownerIndex(int population,int iteration,int slot) {
        if(population<1 || iteration<1 || slot<0 || slot>=Math.max(1,population/3))throw new IllegalArgumentException();
        return (int)(((long)(iteration-1)*Math.max(1,population/3)+slot)%population);
    }
    private Set<String> tried(int owner,String version,String lineage) {
        History h=history.get(owner);
        if(h==null || !h.version.equals(version)){h=new History(version);history.put(owner,h);}
        // Failure bookkeeping is common to all switches, not positive conditional memory.
        Set<String> set=h.tried.get(lineage);
        if(set==null) {
            if(h.tried.size()>=64){host.event("retry_history_capacity");return null;}
            set=new HashSet<>();h.tried.put(lineage,set);
        }
        return set;
    }
    private Trace replay(List<GridPoint> path,String stage) {
        if(path==null || !host.allowed())return null;
        double before=Math.min(knownEvaluatedBest(),host.incumbentCost());
        Trace t=host.replay(path);
        if(t==null || !host.allowed())return null;
        recordEvaluation(t.path,t.cost);
        if(stepBest==null || better(t.cost,stepBest.cost)){stepBest=t;stepStage=stage;}
        if(better(t.cost,before) && !stage.equals("current")) {
            host.event("evaluated_discoveries");host.event(stage+"_discoveries");
        }
        return t;
    }
    /** Called only after the coordinator accepts the returned route within its deadline. */
    public void accepted(int owner,double personalBefore,double globalBefore,Trace result) {
        host.event("accepted_individual_improvements");
        boolean global=better(result.cost,globalBefore), novel=better(result.cost,stepKnownBefore);
        if(global)host.event("accepted_global_improvements");
        host.event(novel?"accepted_novel_evaluated_best":"accepted_known_route_transfer");
        if(pendingReward!=null && key(result.path).equals(pendingRewardRoute))reward(pendingReward,pendingRewardGain);
        if(attribution.size()<256) {
            Map<String,Object> row=new LinkedHashMap<>();row.put("owner",owner);row.put("round",round);
            row.put("source",stepStage);row.put("route",key(result.path));row.put("cost",result.cost);
            row.put("personal_before",personalBefore);row.put("global_before",globalBefore);
            row.put("evaluated_best_before_slot",Double.isFinite(stepKnownBefore)?stepKnownBefore:null);
            row.put("individual_gain",Math.max(0,personalBefore-result.cost));
            row.put("global_gain",global?globalBefore-result.cost:0);
            row.put("novel_evaluated_gain",novel && Double.isFinite(stepKnownBefore)?stepKnownBefore-result.cost:0);
            row.put("known_transfer",!novel);attribution.add(row);
        }
        pendingReward=null;pendingRewardGain=0;
    }
    public int storedSuggestions() {return archive.values().stream().mapToInt(List::size).sum();}
    public int activeRequests() {return requests.size();}
    long requestId(int owner){return requests.containsKey(owner)?requests.get(owner).id:-1;}
    int requestServices(int owner){return requests.containsKey(owner)?requests.get(owner).services:0;}

    /** One bounded transaction. Caller charges all time including maintenance to cooperation. */
    public Trace step(int owner,List<GridPoint> current,List<List<GridPoint>> population) {
        if(!host.allowed() || current==null || current.size()<2) return null;
        stepKnownBefore=Math.min(knownEvaluatedBest(),host.incumbentCost());stepBest=null;stepStage=null;
        pendingReward=null;pendingRewardGain=0;pendingRewardRoute=null;
        Trace base=replay(current,"current");if(base==null)return null;
        transact(owner,current,population,base);
        // All valid full-route probes are eligible, equally for archive and demand modes.
        if(host.allowed() && stepBest!=null && better(stepBest.cost,base.cost)) {
            if(requests.remove(owner)!=null)host.event("request_closed_by_probe_improvement");
            return stepBest;
        }
        return null;
    }
    private Trace transact(int owner,List<GridPoint> current,List<List<GridPoint>> population,Trace base) {
        host.event("slots");turn++;
        String version=key(current);
        Request request=requests.get(owner);
        if(request!=null && !key(request.base.path).equals(version)) {
            requests.remove(owner);host.event("request_version_invalidated");request=null;
        }
        List<Suggestion> bank=archive.computeIfAbsent(owner,x->new ArrayList<>());
        for(Suggestion s:bank) if(s.evidenceVersion!=null && !s.evidenceVersion.equals(version)) {
            s.evidenceVersion=null;s.positiveGain=s.negativeGain=s.targetArrival=Double.NaN;
            s.positiveBase=s.positiveEdit=null;
            host.event("evidence_version_invalidated");
        }
        List<int[]> windows=windows(current,host);
        if(windows.isEmpty()) return null;
        bank.removeIf(s->uniqueIndex(current,s.left)<0 || uniqueIndex(current,s.right)<=uniqueIndex(current,s.left));
        while(bank.size()>windows.size()+1) {
            Suggestion victim=null;
            for(Suggestion s:bank)if(request==null || request.b!=s)
                if(victim==null || s.used<victim.used)victim=s;
            if(victim==null)break;bank.remove(victim);host.event("archive_window_resize_evictions");
        }
        List<List<GridPoint>> materials=new ArrayList<>();
        for(List<GridPoint> p:population) if(p!=null && p.size()>1) materials.add(p);
        for(Integer id:new TreeSet<>(recent.keySet())) materials.addAll(recent.get(id));
        if(materials.isEmpty()) materials.add(current);
        // Bounded geometry-only intake, never a Cartesian scoring prescreen.
        for(int n=0;n<Math.min(4,windows.size());n++) {
            int[] w=windows.get((int)((turn+n)%windows.size()));
            List<GridPoint> donor=materials.get((int)((turn+n)%materials.size()));
            int l=uniqueIndex(donor,current.get(w[0])),h=uniqueIndex(donor,current.get(w[1]));
            if(l>=0 && h>l) {
                Suggestion suggestion=new Suggestion(donor.subList(l,h+1),round);
                suggestion.sourceCost=sourceCost(donor);
                add(bank,suggestion,current,windows.size()+1,request);
            }
        }
        if(bank.isEmpty()) {
            int[] w=windows.get((int)(turn%windows.size()));
            List<GridPoint> generated=host.propose(current,w[0],w[1],random);
            int l=uniqueIndex(generated,current.get(w[0])),h=uniqueIndex(generated,current.get(w[1]));
            if(l>=0 && h>l)add(bank,new Suggestion(generated.subList(l,h+1),round),current,windows.size()+1,request);
        }
        if(request!=null) {
            host.event("request_continuations");
            long used=host.work();try{return fulfill(owner,request,materials,windows);}
            finally{charge(request.b,used);}
        }
        if(bank.isEmpty()) {host.event("no_eligible_suggestion");return null;}
        Suggestion b=choose(bank,base);b.used=round;
        if(options.memory && b.born<round) host.event("cross_round_retrievals");
        long startWork=host.work();
        try {
            Trace p=base;
            Trace pb=replay(apply(current,b),"single_b");
            if(pb==null || !host.allowed()) return null;
            if(better(pb.cost,p.cost)) {host.event("single_b_improvements");return pb;}
            int entry=uniqueIndex(current,b.left);
            if(entry<1)return null;
            // Use ONE actually observed prefix at a time; replay it with the SAME suffix.
            List<GridPoint> other=materials.get((int)(turn%materials.size()));
            int anchor=uniqueIndex(other,b.left);
            boolean flip=version.equals(b.evidenceVersion) && b.positiveGain>EPS && b.negativeGain<=EPS;
            if(!options.mode.equals("ARC_RAW") && !flip && anchor>=1 && other.get(0).equals(current.get(0))) {
                List<GridPoint> q=new ArrayList<>(copy(other.subList(0,anchor)));
                q.addAll(copy(current.subList(entry,current.size())));
                if(!key(q).equals(version)) {
                    host.event("condition_probes");
                    Trace tq=replay(q,"condition_base"),tqb=replay(apply(q,b),"condition_edit");
                    if(tq!=null && tqb!=null && host.allowed()) {
                        double negative=p.cost-pb.cost,positive=tq.cost-tqb.cost;
                        if(negative<=EPS && positive>EPS) {
                            b.evidenceVersion=version;b.negativeGain=negative;b.positiveGain=positive;
                            b.targetArrival=tq.arrival[uniqueIndex(tq.path,b.left)];
                            int at=uniqueIndex(tq.path,b.left);b.targetHeading=host.heading(tq.path.get(at-1),tq.path.get(at));
                            b.positiveBase=tq;b.positiveEdit=tqb;flip=true;host.event("sign_flips");
                        }
                    }
                }
            }
            if(!host.allowed())return null;
            if(!options.demand) {
                host.event("passive_pairs");return passive(p,pb,b,materials,windows);
            }
            if(flip) {
                // Qualifying evidence alone is not a new recruitment: require actual fresh supply.
                List<List<GridPoint>> fresh=freshSupply(owner,p,b,materials,windows);
                if(fresh.isEmpty()){host.event("recruitment_suppressed_no_fresh_supply");return null;}
                request=new Request(p,pb,b,round,++requestSerial);requests.put(owner,request);
                host.event("requests_created");
                return fulfillPrepared(owner,request,fresh);
            }
            return null;
        } finally {charge(b,startWork);}
    }
    private Trace fulfill(int owner,Request req,List<List<GridPoint>> materials,List<int[]> windows) {
        return fulfillPrepared(owner,req,freshSupply(owner,req.base,req.b,materials,windows));
    }
    private List<List<GridPoint>> freshSupply(int owner,Trace base,Suggestion b,List<List<GridPoint>> materials,List<int[]> windows) {
        List<List<GridPoint>> fresh=new ArrayList<>();
        Set<String> old=tried(owner,key(base.path),b.lineage),local=new HashSet<>();
        if(old==null || old.size()>=64){host.event("supply_history_exhausted");return fresh;}
        int entry=uniqueIndex(base.path,b.left);
        for(int k=0;k<4 && host.allowed();k++) {
            List<GridPoint> pa=upstream(base.path,entry,materials,windows,k);
            if(pa==null){host.event("supply_generation_empty");continue;}
            String identity=key(pa);
            if(old.contains(identity) || !local.add(identity)){host.event("supply_duplicate_skipped");continue;}
            fresh.add(pa);
        }
        return fresh;
    }
    private Trace fulfillPrepared(int owner,Request req,List<List<GridPoint>> fresh) {
        req.services++;host.event("request_service_opportunities");
        if(fresh.isEmpty())host.event("request_service_without_supply");
        {
            List<Trace> supply=new ArrayList<>();
            Set<String> old=tried(owner,key(req.base.path),req.b.lineage);
            // Validate bounded actual single-upstream edits before ordering on actual arrival.
            for(int k=0;k<Math.min(2,fresh.size()) && req.attempts<4 && host.allowed();k++) {
                List<GridPoint> pa=fresh.get(k);
                if(old==null || old.size()>=64 || old.contains(key(pa)))continue;
                req.attempts++;host.event("supply_trials");
                Trace ta=replay(pa,"supply_a");
                if(host.allowed()){old.add(key(pa));req.tried.add(key(pa));}
                if(ta!=null)supply.add(ta);else host.event("supply_replay_incomplete_or_invalid");
            }
            supply.sort(Comparator.comparingDouble((Trace t)->Math.abs(t.arrival[uniqueIndex(t.path,req.b.left)]-req.b.targetArrival))
                    .thenComparingDouble(t->headingGap(t,req.b))
                    .thenComparingDouble(t->t.cost).thenComparing(t->key(t.path)));
            Trace best=null;
            for(Trace a:supply) {
                if(!host.allowed())break;
                if(better(a.cost,req.base.cost)) {if(best==null || better(a.cost,best.cost))best=a;host.event("single_a_improvements");}
                double knownBeforeJoint=Math.min(knownEvaluatedBest(),host.incumbentCost());
                Trace ab=replay(apply(a.path,req.b),"joint");
                if(ab==null || !host.allowed())continue;
                boolean strict=!better(a.cost,req.base.cost) && !better(req.singleB.cost,req.base.cost) && better(ab.cost,req.base.cost);
                witness(owner,req,a,ab,strict,knownBeforeJoint);
                if(better(ab.cost,req.base.cost) && (best==null || better(ab.cost,best.cost)))best=ab;
                if(strict)host.event("strict_complement_improvements");
                double bestSingle=Math.min(req.base.cost,req.singleB.cost);
                for(Trace single:supply)bestSingle=Math.min(bestSingle,single.cost);
                double discoveryBaseline=Math.min(knownBeforeJoint,bestSingle);
                if(better(ab.cost,discoveryBaseline)) {
                    pendingReward=req.b;pendingRewardGain+=discoveryBaseline-ab.cost;pendingRewardRoute=key(ab.path);
                    host.event("joint_discoveries_beyond_known_and_singles");
                } else if(better(ab.cost,bestSingle)) host.event("joint_known_route_transfers");
            }
            if(best!=null && host.allowed()) {
                requests.remove(owner);
                // Credit only a joint result beyond the best single, once per transaction.
                double bestSingle=req.base.cost;
                for(Trace a:supply)bestSingle=Math.min(bestSingle,a.cost);
                bestSingle=Math.min(bestSingle,req.singleB.cost);
                double jointGain=Math.max(0,bestSingle-best.cost);
                host.event(jointGain>EPS?"request_fulfilled":"request_single_only_exit");return best;
            }
            if(req.attempts>=4) {requests.remove(owner);host.event("request_exhausted");}
            else if(req.services>=3) {requests.remove(owner);host.event("request_service_expired");}
            return null;
        }
    }
    private Trace passive(Trace p,Trace pb,Suggestion b,List<List<GridPoint>> materials,List<int[]> windows) {
        int entry=uniqueIndex(p.path,b.left);
        for(int k=0;k<4 && host.allowed();k++) {
            List<GridPoint> pa=upstream(p.path,entry,materials,windows,k);
            if(pa==null)continue;
            Trace a=replay(pa,"passive_a");if(a==null)continue;
            double knownBeforeJoint=Math.min(knownEvaluatedBest(),host.incumbentCost());
            Trace ab=replay(apply(a.path,b),"passive_joint");if(ab==null || !host.allowed())continue;
            Trace best=better(a.cost,ab.cost)?a:ab;
            if(better(best.cost,p.cost)) {
                double baseline=Math.min(knownBeforeJoint,Math.min(p.cost,Math.min(pb.cost,a.cost)));
                if(better(best.cost,baseline)) {pendingReward=b;pendingRewardGain=baseline-best.cost;pendingRewardRoute=key(best.path);}
                return best;
            }
        }
        return null;
    }
    private List<GridPoint> upstream(List<GridPoint> p,int entry,List<List<GridPoint>> materials,List<int[]> windows,int attempt) {
        List<int[]> available=new ArrayList<>();
        for(int[] w:windows)if(w[1]<=entry)available.add(w);
        if(available.isEmpty())return null;
        available.sort((a,b)->Integer.compare(b[1],a[1]));
        int[] w=available.get(attempt%available.size());
        List<GridPoint> donor=materials.get((int)((turn+attempt)%materials.size()));
        int a=uniqueIndex(donor,p.get(w[0])),b=uniqueIndex(donor,p.get(w[1]));
        if(a>=0 && b>a) {
            List<GridPoint> trial=apply(p,new Suggestion(donor.subList(a,b+1),round));
            if(trial!=null && !key(trial).equals(key(p)))return trial;
        }
        List<GridPoint> trial=host.propose(p,w[0],w[1],random);
        return trial==null || key(trial).equals(key(p))?null:trial;
    }
    private Suggestion choose(List<Suggestion> bank,Trace base) {
        if(options.mode.startsWith("ARC_")) {
            host.event(options.mode.equals("ARC_RAW")?"archive_raw_selections":"archive_context_selections");
            if((turn&1)==0)return bank.get((int)((turn/2)%bank.size()));
            Comparator<Suggestion> quality=Comparator.comparingDouble((Suggestion s)->s.sourceCost).thenComparing(s->s.lineage);
            if(options.mode.equals("ARC_CTX"))quality=Comparator.comparingDouble((Suggestion s)-> {
                int at=uniqueIndex(base.path,s.left);
                return at<0 || !Double.isFinite(s.targetArrival)?Double.POSITIVE_INFINITY:Math.abs(base.arrival[at]-s.targetArrival);
            }).thenComparing(quality);
            return Collections.min(bank,quality);
        }
        List<Suggestion> untried=new ArrayList<>();
        for(Suggestion s:bank)if(!feedback.containsKey(s.lineage))untried.add(s);
        if((turn&1)==0) {
            // Explore even after every suggestion has a cost record. Otherwise all-zero
            // feedback ties repeatedly select one lexical winner and starve the rest.
            host.event("suggestion_exploration_selections");
            List<Suggestion> explore=untried.isEmpty()?bank:untried;
            return explore.get((int)((turn/2)%explore.size()));
        }
        if(options.relay) {
            host.event("relay_priority_selections");
            return Collections.max(bank,Comparator.comparingDouble((Suggestion s)->efficiency(s)).thenComparing(s->s.lineage,Comparator.reverseOrder()));
        }
        return bank.get((int)(turn%bank.size()));
    }
    private double efficiency(Suggestion s) {double[] f=feedback.get(s.lineage);return f==null?0:f[0]/Math.max(1,f[1]);}
    private double headingGap(Trace t,Suggestion s) {
        int i=uniqueIndex(t.path,s.left);double angle=Math.abs(host.heading(t.path.get(i-1),t.path.get(i))-s.targetHeading)%360;
        return Math.min(angle,360-angle);
    }
    private void reward(Suggestion b,double gain) {
        if(gain>EPS){feedback.computeIfAbsent(b.lineage,k->new double[2])[0]+=gain;host.event("recipient_rewards");}
    }
    private void charge(Suggestion b,long before) {
        feedback.computeIfAbsent(b.lineage,k->new double[2])[1]+=Math.max(1,host.work()-before);
        while(feedback.size()>2048)feedback.remove(feedback.keySet().iterator().next());
    }
    private void add(List<Suggestion> bank,Suggestion s,List<GridPoint> p,int capacity,Request active) {
        List<GridPoint> changed=apply(p,s);
        if(changed==null || key(changed).equals(key(p)) || bank.stream().anyMatch(x->x.lineage.equals(s.lineage)))return;
        while(bank.size()>=capacity) {
            Suggestion victim=bank.stream().filter(x->active==null || active.b!=x)
                    .min(Comparator.comparingInt((Suggestion x)->Double.isNaN(x.positiveGain)?0:1).thenComparingInt(x->x.used)).orElse(null);
            if(victim==null){host.event("archive_admission_rejected");return;}bank.remove(victim);
        }
        bank.add(s);host.event("suggestions_admitted");
    }
    private void witness(int owner,Request req,Trace a,Trace ab,boolean strict,double knownBeforeJoint) {
        String structure=polylineKey(req.base.path)+"|"+polylineKey(a.path)+"|"+polylineKey(req.singleB.path)+"|"+polylineKey(ab.path);
        boolean first=false;
        if(strict && strictStructures.size()<2048)first=strictStructures.add(structure);
        if(first)host.event("strict_unique_grid_structures");
        // Bounded diagnostic records, not an optimizer-accessible candidate archive.
        if(witnesses.size()>=64)return;
        Map<String,Object> w=new LinkedHashMap<>();w.put("round",round);w.put("lineage",req.b.lineage);
        w.put("owner",owner);w.put("request_id",req.id);w.put("request_services",req.services);
        w.put("base_version",key(req.base.path));w.put("supply_identity",key(a.path));
        w.put("grid_structure",structure);w.put("first_grid_structure",first);
        w.put("evaluated_best_before_joint",Double.isFinite(knownBeforeJoint)?knownBeforeJoint:null);
        w.put("new_evaluated_best",better(ab.cost,knownBeforeJoint));
        w.put("incumbent_before_acceptance",Double.isFinite(host.incumbentCost())?host.incumbentCost():null);
        w.put("P",req.base.path);w.put("Pa",a.path);w.put("Pb",req.singleB.path);w.put("Pab",ab.path);
        w.put("costs",Arrays.asList(req.base.cost,a.cost,req.singleB.cost,ab.cost));
        w.put("negative_gain",req.b.negativeGain);w.put("positive_gain",req.b.positiveGain);
        w.put("positive_background",req.b.positiveBase.path);w.put("positive_background_edited",req.b.positiveEdit.path);
        w.put("positive_background_costs",Arrays.asList(req.b.positiveBase.cost,req.b.positiveEdit.cost));
        w.put("strict",strict);witnesses.add(w);
    }
    public static List<GridPoint> copy(List<GridPoint> p) {
        List<GridPoint> c=new ArrayList<>();for(GridPoint g:p)c.add(new GridPoint(g.i,g.j));return c;
    }
    public static String key(List<GridPoint> p) {StringBuilder b=new StringBuilder();for(GridPoint g:p)b.append(g.i).append(',').append(g.j).append(';');return b.toString();}
    /** Diagnostic normalization only; never changes evaluated routes or deduplicates sampling. */
    public static String polylineKey(List<GridPoint> p) {
        List<GridPoint> q=new ArrayList<>();
        for(GridPoint c:p) {
            while(q.size()>=2) {
                GridPoint a=q.get(q.size()-2),b=q.get(q.size()-1);
                long x=b.i-a.i,y=b.j-a.j,u=c.i-b.i,v=c.j-b.j;
                if(x*v!=y*u || x*u+y*v<=0)break;q.remove(q.size()-1);
            }
            q.add(c);
        }
        return key(q);
    }
    private static boolean better(double a,double b){return Double.isFinite(a) && (!Double.isFinite(b) || a<b-EPS*Math.max(1,Math.abs(b)));}
    private static int uniqueIndex(List<GridPoint> p,GridPoint g) {
        if(p==null)return -1;int found=-1;
        for(int i=0;i<p.size();i++)if(p.get(i).equals(g)){if(found>=0)return -1;found=i;}return found;
    }
    private static List<GridPoint> apply(List<GridPoint> p,Suggestion s) {
        int a=uniqueIndex(p,s.left),b=uniqueIndex(p,s.right);if(a<0 || b<=a)return null;
        List<GridPoint> q=copy(p.subList(0,a));q.addAll(copy(s.replacement));q.addAll(copy(p.subList(b+1,p.size())));return q;
    }
    private static int baseWindowCount(List<GridPoint> p){return Math.max(1,Math.min(8,p.size()-1));}
    public static List<int[]> windows(List<GridPoint> p,Host host) {
        List<int[]> out=new ArrayList<>();if(p.size()<2)return out;
        double[] length=new double[p.size()];for(int i=1;i<p.size();i++)length[i]=length[i-1]+host.distance(p.get(i-1),p.get(i));
        List<Integer> edges=new ArrayList<>();edges.add(0);int n=baseWindowCount(p);
        for(int k=1;k<n;k++){double target=length[p.size()-1]*k/n;int at=1;while(at<p.size()-1 && length[at]<target)at++;
            if(at>edges.get(edges.size()-1) && at<p.size()-1)edges.add(at);}
        edges.add(p.size()-1);
        for(int k=0;k<edges.size()-1;k++){out.add(new int[]{edges.get(k),edges.get(k+1)});
            if(k+2<edges.size())out.add(new int[]{edges.get(k),edges.get(k+2)});}
        return out;
    }
}
