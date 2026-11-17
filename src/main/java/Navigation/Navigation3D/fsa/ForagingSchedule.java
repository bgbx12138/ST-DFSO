package Navigation.Navigation3D.fsa;
import Navigation.model.GridPoint;
import java.util.*;

/** Physical-arclength departure strata; no objective/environment evaluations. */
public final class ForagingSchedule {
    private ForagingSchedule(){}
    public static int[] build(List<GridPoint> seed,int population,double dRowNm,double dColNm){
        if(population<1||!Double.isFinite(dRowNm)||!Double.isFinite(dColNm)||dRowNm<=0||dColNm<=0)throw new IllegalArgumentException("schedule dimensions");
        int[] out=new int[population];if(seed==null||seed.size()<3)return out;
        int count=seed.size()-1;double[] distance=new double[count];
        for(int k=1;k<count;k++)distance[k]=distance[k-1]+Math.hypot((seed.get(k).i-seed.get(k-1).i)*dRowNm,(seed.get(k).j-seed.get(k-1).j)*dColNm);
        int[] uses=new int[count];
        for(int rank=0;rank<population;rank++){
            double target=(rank+.5)/population*distance[count-1];int best=0;
            for(int k=1;k<count;k++)if(uses[k]<uses[best]||(uses[k]==uses[best]&&Math.abs(distance[k]-target)<Math.abs(distance[best]-target)))best=k;
            out[rank]=best;uses[best]++;
        }
        Arrays.sort(out);return out;
    }
    public static int departure(int[] schedule,int fishId,int iteration){
        return schedule[Math.floorMod(fishId+iteration-1,schedule.length)];
    }
}
