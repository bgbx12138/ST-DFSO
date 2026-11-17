package Navigation.Navigation3D.fsa;
/** A resource-control signal, carrying no physical response or objective value. */
public final class TruthBudgetStop211 extends RuntimeException {
    public TruthBudgetStop211(String reason){super(reason);}
    public static TruthBudgetStop211 find(Throwable e){for(Throwable c=e;c!=null;c=c.getCause())if(c instanceof TruthBudgetStop211)return (TruthBudgetStop211)c;return null;}
}
