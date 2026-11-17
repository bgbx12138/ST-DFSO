package Navigation.Navigation3D.fsa;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.LongAdder;

/** Run-scoped accounting for objective work performed by an optimizer. */
public final class SearchWorkLedger {
    private final LongAdder routeObjectiveEvaluations = new LongAdder();
    private final LongAdder edgeSampleInvocations = new LongAdder();
    private final LongAdder edgeSubsegmentAttempts = new LongAdder();
    private final ConcurrentHashMap<String, LongAdder> operatorCounts = new ConcurrentHashMap<>();
    private final long maxEdgeSubsegmentAttempts;
    private volatile boolean active = true;
    private volatile boolean budgetExhausted;
    private volatile boolean budgetLimitReached;
    private volatile long clockStartNanos;
    private volatile long clockStopNanos;
    private volatile long deadlineNanos = Long.MAX_VALUE;
    private volatile long phaseDeadlineNanos = Long.MAX_VALUE;

    /** Opt-in wall budget. Legacy callers keep the unlimited clock. */
    public void startWallClock(long milliseconds) {
        if (milliseconds <= 0 || milliseconds > Long.MAX_VALUE / 1_000_000L)
            throw new IllegalArgumentException("positive finite wall budget required");
        clockStartNanos = System.nanoTime();
        clockStopNanos = 0;
        deadlineNanos = clockStartNanos + milliseconds * 1_000_000L;
    }

    public boolean wallLimitReached() {
        return active && deadlineNanos != Long.MAX_VALUE && System.nanoTime() - deadlineNanos >= 0;
    }

    public boolean evaluationAllowed() {
        return !active || (!shouldStopSearch() && timingAllowed());
    }
    public boolean timingAllowed() {
        return !active || (!wallLimitReached() &&
                (phaseDeadlineNanos == Long.MAX_VALUE || System.nanoTime() - phaseDeadlineNanos < 0));
    }

    /** Called only with all ordinary workers joined; phase exhaustion is not global exhaustion. */
    public void beginPhase(long remainingNanos) {
        if (remainingNanos <= 0) throw new IllegalArgumentException("empty phase budget");
        phaseDeadlineNanos = System.nanoTime() + remainingNanos;
    }

    public void endPhase() { phaseDeadlineNanos = Long.MAX_VALUE; }
    public long elapsedWallNanos() { return clockStartNanos == 0 ? 0 :
            (clockStopNanos == 0 ? System.nanoTime() : clockStopNanos) - clockStartNanos; }

    /** Unlimited subsegment work, retained for legacy callers and calibration runs. */
    public SearchWorkLedger() {
        this(0L);
    }

    /** A zero limit means unlimited; a positive limit is enforced before every sample. */
    public SearchWorkLedger(long maxEdgeSubsegmentAttempts) {
        if (maxEdgeSubsegmentAttempts < 0L) {
            throw new IllegalArgumentException("subsegment budget cannot be negative");
        }
        this.maxEdgeSubsegmentAttempts = maxEdgeSubsegmentAttempts;
    }

    public void setActive(boolean active) {
        if (!active && this.active && clockStartNanos != 0) clockStopNanos=System.nanoTime();
        this.active = active;
    }

    public void recordRouteObjectiveEvaluation() {
        if (active) routeObjectiveEvaluations.increment();
    }

    public void recordEdgeSampleInvocation() {
        if (active) edgeSampleInvocations.increment();
    }

    public synchronized boolean tryRecordEdgeSubsegmentAttempt() {
        if (!active) return true;
        if (!timingAllowed()) return false;
        long used = edgeSubsegmentAttempts.sum();
        if (maxEdgeSubsegmentAttempts > 0L && used >= maxEdgeSubsegmentAttempts) {
            budgetExhausted = true;
            return false;
        }
        edgeSubsegmentAttempts.increment();
        used++;
        if (maxEdgeSubsegmentAttempts > 0L && used >= maxEdgeSubsegmentAttempts) {
            budgetLimitReached = true;
        }
        return true;
    }

    /** Compatibility helper for call sites that only record and cannot reject a sample. */
    public void recordEdgeSubsegmentAttempt() {
        tryRecordEdgeSubsegmentAttempt();
    }

    public long getMaxEdgeSubsegmentAttempts() {
        return maxEdgeSubsegmentAttempts;
    }

    /** True only while search accounting is active; final reporting runs outside the cap. */
    public boolean shouldStopSearch() {
        return wallLimitReached() || (active && maxEdgeSubsegmentAttempts > 0L
                && (budgetLimitReached || budgetExhausted));
    }

    public void recordOperatorEvent(String name) {
        if (active && name != null && !name.isBlank()) {
            operatorCounts.computeIfAbsent(name, ignored -> new LongAdder()).increment();
        }
    }

    public Snapshot snapshot() {
        Map<String, Long> operators = new HashMap<>();
        for (Map.Entry<String, LongAdder> entry : operatorCounts.entrySet()) {
            operators.put(entry.getKey(), entry.getValue().sum());
        }
        return new Snapshot(routeObjectiveEvaluations.sum(), edgeSampleInvocations.sum(),
                edgeSubsegmentAttempts.sum(), maxEdgeSubsegmentAttempts,
                budgetLimitReached, budgetExhausted, operators);
    }

    public static final class Snapshot {
        public final long routeObjectiveEvaluations;
        public final long edgeSampleInvocations;
        public final long edgeSubsegmentAttempts;
        public final long maxEdgeSubsegmentAttempts;
        public final boolean budgetLimitReached;
        public final boolean budgetExhausted;
        private final Map<String, Long> operatorCounts;

        private Snapshot(long routeObjectiveEvaluations, long edgeSampleInvocations,
                         long edgeSubsegmentAttempts, long maxEdgeSubsegmentAttempts,
                         boolean budgetLimitReached, boolean budgetExhausted,
                         Map<String, Long> operatorCounts) {
            this.routeObjectiveEvaluations = routeObjectiveEvaluations;
            this.edgeSampleInvocations = edgeSampleInvocations;
            this.edgeSubsegmentAttempts = edgeSubsegmentAttempts;
            this.maxEdgeSubsegmentAttempts = maxEdgeSubsegmentAttempts;
            this.budgetLimitReached = budgetLimitReached;
            this.budgetExhausted = budgetExhausted;
            this.operatorCounts = Collections.unmodifiableMap(new HashMap<>(operatorCounts));
        }

        public long operatorCount(String name) {
            return operatorCounts.getOrDefault(name, 0L);
        }
        public Map<String, Long> getOperatorCounts() { return operatorCounts; }
    }
}
