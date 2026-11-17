package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import java.util.*;

/** A bounded transaction, not an alarm queue. The random control observes exactly the same
 * proposal family and gate measurements conditional on the same parent/window. */
public final class SelectiveFollowOperator {
    private static final double EPS=1e-9;
    private final String mode;
    private final AlarmWrapOperator.Host host;
    private final long seed;
    private final LinkedHashMap<String,TimeEvaluator.Trace> cache=new LinkedHashMap<>();
    private final List<Map<String,Object>> witnesses=new ArrayList<>(),transactions=new ArrayList<>();
    public SelectiveFollowOperator(String mode,AlarmWrapOperator.Host host,long seed){
        if(!AlarmWrapOperator.isSelectiveMode(mode))throw new IllegalArgumentException("selective mode");
        this.mode=mode;this.host=host;this.seed=seed;
    }
    public List<Map<String,Object>> witnesses(){return Collections.unmodifiableList(witnesses);}
    public List<Map<String,Object>> transactions(){return Collections.unmodifiableList(transactions);}
    private void count(String k){host.count("sf_"+k);}
    private boolean aborted(TimeEvaluator.Trace t){return "ABORTED".equals(t.status)||!host.completedAllowed();}
    private static String exactKey(List<GridPoint> p){StringBuilder b=new StringBuilder();for(GridPoint q:p)b.append(q.i).append(',').append(q.j).append(';');return b.toString();}

    AlarmWrapOperator.Result apply(int owner,long event,List<GridPoint> original,double cost,int a,int b,List<List<GridPoint>> chains){
        if(!host.allowed())return null;
        List<GridPoint> parent=AlarmWrapOperator.copy(original);String key=exactKey(parent);
        count("transactions_started");
        TimeEvaluator.Trace base=cache.get(key);
        if(base==null){
            count("parent_trace_evaluations");base=host.evaluate(parent);
            if(aborted(base)||!base.valid()){count("aborted");return null;}
            if(cache.size()>=64)cache.remove(cache.keySet().iterator().next());cache.put(key,base);
        }else count("parent_trace_hits");
        if(Math.abs(base.cost-cost)>1e-7*Math.max(1,Math.abs(cost)))throw new IllegalStateException("parent score mismatch");
        int m=b-a-1;
        TimeEvaluator.Trace[] traces=new TimeEvaluator.Trace[2];
        AlarmWrapOperator.Packed[] packed=new AlarmWrapOperator.Packed[2];
        List<List<GridPoint>> rawProposals=new ArrayList<>();
        for(int q=0;q<2;q++){
            if(!host.allowed()){count("aborted");return null;}
            List<GridPoint> raw=AlarmWrapOperator.copy(parent);
            for(int j=0;j<m;j++){
                int d=chains.get(j).size()-1;boolean tight=(j<(m+1)/2)!=(q==1);
                raw.set(a+j+1,chains.get(j).get(tight?d/4:(3*d+3)/4));
            }
            rawProposals.add(raw);packed[q]=AlarmWrapOperator.pack(raw);
            // Deliberately matched slots, even when rounding produced a geometric duplicate.
            count("proposal_evaluations");traces[q]=host.evaluate(packed[q].path);
            if(aborted(traces[q])){count("aborted");return null;}
            if(traces[q].valid())count("valid_proposals");
        }
        List<GridPoint> tentative=AlarmWrapOperator.copy(parent);
        int[] source=new int[parent.size()];Arrays.fill(source,-1);
        double[] estimates=new double[parent.size()];
        for(int j=a+1;j<b;j++){
            int selected=-1;double best=Double.POSITIVE_INFINITY;
            for(int q=0;q<2;q++){
                int left=packed[q].map[j-1],right=packed[q].map[j+1];
                double delay=traces[q].arrival[right]-traces[q].arrival[left]-(base.arrival[j+1]-base.arrival[j-1]);
                if(Double.isFinite(delay) && delay<best-EPS){best=delay;selected=q;}
            }
            if(selected>=0){tentative.set(j,rawProposals.get(selected).get(j));source[j]=selected;estimates[j]=-best;}
        }
        List<int[]> pool=new ArrayList<>();
        for(int j=a+1;j<b;){
            if(tentative.get(j).equals(parent.get(j))){j++;continue;}
            int left=j;while(j<b&&!tentative.get(j).equals(parent.get(j)))j++;
            pool.add(new int[]{left,j-1});
        }
        pool.sort(Comparator.<int[]>comparingDouble(x->{double sum=0;for(int j=x[0];j<=x[1];j++)sum+=estimates[j];return -sum;}).thenComparingInt(x->x[0]));
        if(pool.size()>2)pool=new ArrayList<>(pool.subList(0,2));
        AlarmWrapOperator.Packed mixed=AlarmWrapOperator.pack(tentative);
        List<Map<String,Object>> gateEvidence=new ArrayList<>();boolean[] localKeep=new boolean[pool.size()];
        int keepCount=0;
        for(int i=0;i<pool.size();i++){
            if(!host.allowed()){count("aborted");return null;}
            int[] block=pool.get(i);int left=block[0]-1,right=block[1]+1;
            List<GridPoint> segment=new ArrayList<>(mixed.path.subList(mixed.map[left],mixed.map[right]+1));
            double entry=base.arrival[left],reference=base.arrival[right]-entry;
            count("block_evaluations");double duration=segmentDuration(segment,entry);
            if(!host.completedAllowed()){count("aborted");return null;}
            localKeep[i]=Double.isFinite(duration)&&duration<reference-EPS;if(localKeep[i]){keepCount++;count("blocks_passed");}else count("blocks_rejected");
            Map<String,Object> g=new LinkedHashMap<>();g.put("from",block[0]);g.put("to",block[1]);g.put("entry_hours",entry);
            g.put("parent_duration",reference);g.put("candidate_duration",Double.isFinite(duration)?duration:null);
            g.put("segment",AlarmWrapOperator.copy(segment));g.put("local_keep",localKeep[i]);gateEvidence.add(g);
        }
        // Cardinality-matched random subset: no extra candidates and no different observation cost.
        // This intentionally tests WHICH blocks to retain, not the keep-rate itself.
        boolean[] randomKeep=randomMask(pool.size(),keepCount,seed^key.hashCode()^((long)a<<32)^b);
        boolean choice=keepCount>0&&keepCount<pool.size();
        if(choice)count("mask_choice_opportunities");else count("no_mask_choice");
        if(!Arrays.equals(localKeep,randomKeep))count("shadow_masks_differ");
        boolean[] keep=mode.equals("LOCAL_FOLLOW")?localKeep:randomKeep;
        List<GridPoint> candidate=AlarmWrapOperator.copy(parent);Set<Integer> sources=new HashSet<>();int points=0;
        for(int i=0;i<pool.size();i++){
            gateEvidence.get(i).put("chosen",keep[i]);
            if(keep[i])for(int j=pool.get(i)[0];j<=pool.get(i)[1];j++){candidate.set(j,tentative.get(j));sources.add(source[j]);points++;}
        }
        AlarmWrapOperator.Packed finalPacked=AlarmWrapOperator.pack(candidate);
        boolean novel=!AlarmWrapOperator.key(finalPacked.path).equals(AlarmWrapOperator.key(parent));
        for(AlarmWrapOperator.Packed p:packed)novel&=!AlarmWrapOperator.key(finalPacked.path).equals(AlarmWrapOperator.key(p.path));
        if(novel)count("new_selection_geometry");else count("selection_duplicate_geometry");
        if(sources.size()>1)count("multi_source_selection");
        if(!host.allowed()){count("aborted");return null;}
        count("selection_full_evaluations");TimeEvaluator.Trace selected=host.evaluate(finalPacked.path);
        if(aborted(selected)){count("aborted");return null;}
        double bestProposal=base.cost;TimeEvaluator.Trace bestTrace=base;String stage="parent";
        for(int q=0;q<2;q++)if(traces[q].valid()&&traces[q].cost<bestProposal-EPS){bestProposal=traces[q].cost;bestTrace=traces[q];stage=q==0?"proposal_a":"proposal_b";}
        boolean selectionGain=selected.valid()&&selected.cost<bestProposal-EPS;
        if(selectionGain){bestTrace=selected;stage="selection";count("selection_beyond_proposals");count(choice?"selection_gain_with_choice":"selection_gain_without_choice");}
        Map<String,Object> w=new LinkedHashMap<>();w.put("mode",mode);w.put("event",event);w.put("source_owner",owner);
        w.put("from",a);w.put("to",b);w.put("parent",AlarmWrapOperator.copy(parent));w.put("parent_cost",base.cost);
        w.put("proposal_a",AlarmWrapOperator.copy(traces[0].path));w.put("proposal_b",AlarmWrapOperator.copy(traces[1].path));
        w.put("proposal_a_cost",traces[0].valid()?traces[0].cost:null);w.put("proposal_b_cost",traces[1].valid()?traces[1].cost:null);
        w.put("tentative",tentative);w.put("gate_evidence",gateEvidence);w.put("local_mask",localKeep);w.put("random_mask",randomKeep);
        w.put("choice_opportunity",choice);w.put("shadow_masks_differ",!Arrays.equals(localKeep,randomKeep));w.put("selection",AlarmWrapOperator.copy(selected.path));w.put("selection_cost",selected.valid()?selected.cost:null);
        w.put("best_without_selection",bestProposal);w.put("selection_beyond_proposals",selectionGain);w.put("selected_points",points);w.put("selected_blocks",keepCount);
        w.put("selection_sources",sources.size());w.put("stage",stage);w.put("candidate",AlarmWrapOperator.copy(bestTrace.path));w.put("candidate_cost",bestTrace.cost);
        w.put("committed_personal",false);w.put("committed_incumbent",false);
        count("transactions_completed");if(transactions.size()<256)transactions.add(w);
        if(bestTrace.cost>=base.cost-EPS)return null;
        count("strict_parent_improvements");if(witnesses.size()<256)witnesses.add(w);
        return new AlarmWrapOperator.Result(bestTrace,w);
    }

    double segmentDuration(List<GridPoint> segment,double entry){
        double elapsed=entry;Set<GridPoint> seen=new HashSet<>();seen.add(segment.get(0));
        for(int i=1;i<segment.size();i++){
            GridPoint a=segment.get(i-1),b=segment.get(i);
            if(!host.allowed()||!seen.add(b)||!host.edgeLegal(a,b))return Double.POSITIVE_INFINITY;
            double arrival=host.edgeArrival(a,b,elapsed);
            if(!Double.isFinite(arrival)||arrival<=elapsed)return Double.POSITIVE_INFINITY;
            elapsed=arrival;
        }
        return elapsed-entry;
    }
    static boolean[] randomMask(int n,int k,long seed){
        List<Integer> indices=new ArrayList<>();for(int i=0;i<n;i++)indices.add(i);
        // Decorrelate adjacent run seeds before the first small-bound Random draw.
        long z=seed+0x9E3779B97F4A7C15L;z=(z^(z>>>30))*0xBF58476D1CE4E5B9L;z=(z^(z>>>27))*0x94D049BB133111EBL;z^=z>>>31;
        Collections.shuffle(indices,new Random(z));boolean[] mask=new boolean[n];for(int i=0;i<k;i++)mask[indices.get(i)]=true;return mask;
    }
}
