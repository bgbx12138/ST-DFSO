package Navigation.Navigation3D;

import Navigation.Navigation3D.Normalization.Normalizers;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.CompositeEvaluator;
import Navigation.Navigation3D.fsa.ConditionalCooperation;
import Navigation.Navigation3D.fsa.ArrivalActionTransfer;
import Navigation.Navigation3D.fsa.AlarmWrapOperator;
import Navigation.Navigation3D.fsa.EdgeSampler;
import Navigation.Navigation3D.fsa.EliteArchive;
import Navigation.Navigation3D.fsa.EnvironmentFrame;
import Navigation.Navigation3D.fsa.Fish;
import Navigation.Navigation3D.fsa.FsaContext;
import Navigation.Navigation3D.fsa.FuelEvaluator;
import Navigation.Navigation3D.fsa.ObjectiveEvaluator;
import Navigation.Navigation3D.fsa.SafetyDensityEvaluator;
import Navigation.Navigation3D.fsa.SafetyEvaluator;
import Navigation.Navigation3D.fsa.SearchWorkLedger;
import Navigation.Navigation3D.fsa.TimeEvaluator;
import Navigation.PathVisualizer;
import Navigation.model.GridPoint;
import Navigation.model.GridRouteGraph;
import Navigation.object.NavigationConfig;
import Navigation.object.NavigationResult;
import Navigation.object.RoutePoint;
import Navigation.preprocess.object.CurrentOutput;
import Navigation.preprocess.object.ObstacleOutput;
import Navigation.preprocess.object.WaveOutput;
import Navigation.preprocess.object.WindOutput;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDate;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeParseException;
import java.util.*;
import java.util.concurrent.*;

/**
 * HybridFSA3D：Dijkstra 播种 + 跳跃式候选生成的轻量鱼群路径规划器。
 *
 * 设计要点：
 *  - 跳跃式：每步在 visualRange 内采样候选点，直接跳（而非逐格走）
 *  - 播种：将 Dijkstra 最优路径注入种群，提高初始解质量
 *  - FSPO 概率公式：乘法整合 self/social/heuristic 线索；使用固定阈值，不含单独线性加权系数
 *  - 不改动现有 3D Dijkstra 四个实验
 */
public class HybridFSA3D {

    private static final DateTimeFormatter DATE_FORMATTER = DateTimeFormatter.BASIC_ISO_DATE;
    private static final double EPSILON = 1e-9;

    // ==================== 超参 ====================
    private int populationSize = 50;           // 鱼群规模
    private int maxIterations = 50;            // 迭代轮数
    private double visualRange = 8.0;          // 跳跃视野（网格单位）
    private double initialVisualRange = 8.0;
    private int eliteCapacity = 10;            // 精英解容量
    private int deferSocialIter = 3;           // 延迟社会学习轮数
    private int numCandidatesPerStep = 20;     // 每步候选点数
    private long baseSeed = 42L;
    /**
     * 内部鱼群评估线程池大小。**固定值**，不随 `availableProcessors()` 变化：
     * 既保证结果与机器无关（可复现），也避免与外层多 run 并行叠加造成线程超订
     * （2026-09-24：原为 max(2, availableProcessors())，本机 24 线程 × 外层 6 run = 144 线程抢 24 核）。
     */
    private static final int DEFAULT_THREAD_POOL_SIZE = 4;
    private int threadPoolSize = DEFAULT_THREAD_POOL_SIZE;
    private boolean staticSingleDayMode = false;

    // ==================== 消融开关 ====================
    private boolean enableLateralLine = true;     // 侧线感知（5×5 窗口障碍密度）
    private boolean enableAdaptiveVision = true;  // 自适应视野（Area-Restricted Search）
    private boolean enableSwarmFusion = true;     // 分群融合（Fission-Fusion）
    private boolean enableEscapeResponse = true;  // 逃逸反应（Escape Response）
    private boolean enableScatteredForaging = true; // 分散搜索（Scattered Foraging）
    private boolean enableForagingImitation = true;   // 觅食模仿（Foraging Imitation）
    private boolean enableForagingExploration = true; // 觅食队列的随机邻居探索
    private boolean enablePreyInspection = true;      // 猎物侦察（Prey Inspection）
    private boolean enableMultiplicativeSuccessorSelection = true;
    private Navigation.Navigation3D.fsa.FspoProbability.Mode successorFormula = Navigation.Navigation3D.fsa.FspoProbability.Mode.LEGACY;
    public HybridFSA3D setSuccessorFormula(String mode) {
        successorFormula = Navigation.Navigation3D.fsa.FspoProbability.Mode.valueOf(mode);
        return this;
    }

    // Opt-in, independently attributable refinements. Defaults preserve the frozen reference.
    private boolean legalSuccessorCandidates,segmentLateralLine,completeCycleVision,arclengthForaging,reachableEscape;
    private String fusionRefinement="OFF";
    private Navigation.Navigation3D.fsa.StaticRouteGeometry staticGeometry;
    private final List<Map<String,Object>> refinementWitnesses=new ArrayList<>();
    private final List<Map<String,Object>> foragingWitnesses=new ArrayList<>();
    public HybridFSA3D setLegalSuccessorCandidates(boolean enabled){legalSuccessorCandidates=enabled;return this;}
    public HybridFSA3D setSegmentLateralLine(boolean enabled){segmentLateralLine=enabled;return this;}
    public HybridFSA3D setCompleteCycleVision(boolean enabled){completeCycleVision=enabled;return this;}
    public HybridFSA3D setArclengthForaging(boolean enabled){arclengthForaging=enabled;return this;}
    public HybridFSA3D setReachableEscape(boolean enabled){reachableEscape=enabled;return this;}
    public HybridFSA3D setFusionRefinement(String mode){
        if(!Arrays.asList("OFF","LOCAL","GLOBAL_CONTROL").contains(mode))throw new IllegalArgumentException("fusion refinement");
        fusionRefinement=mode;return this;
    }
    public Map<String,Object> getRefinementStatistics(){
        Map<String,Object>x=new LinkedHashMap<>();x.put("legal_candidates",legalSuccessorCandidates);
        x.put("segment_lateral_line",segmentLateralLine);x.put("complete_cycle_vision",completeCycleVision);
        x.put("arclength_foraging",arclengthForaging);x.put("reachable_escape",reachableEscape);
        x.put("fusion_mode",fusionRefinement);x.put("geometry",staticGeometry==null?Collections.emptyMap():staticGeometry.statistics());return x;
    }
    public List<Map<String,Object>> getRefinementWitnesses(){return new ArrayList<>(refinementWitnesses);}
    public List<Map<String,Object>> getForagingWitnesses(){return new ArrayList<>(foragingWitnesses);}

    // [DISABLED] 时空经验记忆（v8.1）— 消融实验显示无显著贡献，已禁用
    // private boolean enableExperienceMemory = true;
    // private double experienceMemoryDecay = 0.95;
    // private double[][][] riskMemory;
    // private double[][][] timeMemory;
    // private double[][][] fuelMemory;
    /**
     * 强制指定 frames 起始天索引（用于 FSA-Easy-逐日：第 d 段强制使用 frames[d]）。
     * - 默认 -1：保持原行为（按时间正常索引）。
     * - >=0 且 staticSingleDayMode=true：所有 frames 复制为 frames[forceDayIndex]，
     *   等价于"锁定单日环境为第 d 天"。
     */
    private int forceDayIndex = -1;

    // FSPO 权重（来自鱼群路径优化文档）
    // FSPO 权重（v7.0：已改为乘法模型，这些字段保留但不再使用）
    @SuppressWarnings("unused") private double heuristicWeight = 0.5;
    @SuppressWarnings("unused") private double socialWeight = 0.3;
    @SuppressWarnings("unused") private double individualWeight = 0.2;

    // 自适应视野（Area-Restricted Search）
    private static final double VR_MIN = 4.0;
    private static final double VR_MAX = 12.0;

    // 分群-融合（Fission-Fusion）
    private static final int FUSION_STAGNATION_THRESHOLD = 3;
    private static final int FUSION_SPLIT = 3;
    private static final int FUSION_ROUNDS = 5;

    // 目标模式
    public enum ObjectiveMode { TIME, SAFETY_DENSITY, SAFETY_EXPOSURE, @Deprecated SAFETY, FUEL, COMPOSITE }
    private ObjectiveMode objectiveMode = ObjectiveMode.TIME;
    private ObjectiveWeights objectiveWeights;

    // ==================== 内部状态 ====================
    private final NavigationConfig config;
    private final ObjectMapper objectMapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);

    private LocalDate baseDate;
    private Path dataFolderPath;
    private int numDays;
    private double departureOffsetHours;
    private boolean[][] obstacleMap;
    private boolean[][] obstacleBufferMap;
    private final List<EnvironmentFrame> frames = new ArrayList<>();
    private double deltaLat, deltaLon, dLatNm, dLonNm;

    // 种子路径（外部注入的 Dijkstra 结果）
    private List<GridPoint> seedPath;

    // 结果
    private List<GridPoint> bestPath = new ArrayList<>();
    private double bestCost = Double.POSITIVE_INFINITY;
    private double seedCost = Double.POSITIVE_INFINITY;
    private final List<Double> convergenceHistory = new ArrayList<>();
    private long wallClockMs;
    private ObjectiveEvaluator lastEvaluator;  // execute() 后缓存，供外部保底比较复用
    private SearchWorkLedger searchWorkLedger;
    private ConditionalCooperation.Options conditionalOptions;
    private ConditionalCooperation conditionalCooperation;
    private String actionTransferMode;
    private ArrivalActionTransfer actionTransfer;
    private String headingMode;
    private boolean measureSearchCpu;
    private long searchCpuNanos=-1;
    private final List<Map<String,Object>> searchMilestones=new ArrayList<>();
    public List<Map<String,Object>> getSearchMilestones(){return new ArrayList<>(searchMilestones);}
    private void recordSearchMilestone(boolean force){
        if(!measureSearchCpu||searchWorkLedger==null||!Double.isFinite(bestCost))return;
        if(!force&&!searchMilestones.isEmpty()&&bestCost>=((Number)searchMilestones.get(searchMilestones.size()-1).get("cost")).doubleValue()-EPSILON)return;
        Map<String,Object> row=new LinkedHashMap<>();row.put("cost",bestCost);row.put("wall_nanos",searchWorkLedger.elapsedWallNanos());
        row.put("subsegments",searchWorkLedger.snapshot().edgeSubsegmentAttempts);row.put("checkpoint","iteration_boundary");
        if(searchMilestones.size()<2048)searchMilestones.add(row);
    }
    public HybridFSA3D setMeasureSearchCpu(boolean enabled){measureSearchCpu=enabled;return this;}
    public long getSearchCpuNanos(){return searchCpuNanos;}
    private static long processCpuNanos(){
        java.lang.management.OperatingSystemMXBean bean=java.lang.management.ManagementFactory.getOperatingSystemMXBean();
        return bean instanceof com.sun.management.OperatingSystemMXBean?((com.sun.management.OperatingSystemMXBean)bean).getProcessCpuTime():-1;
    }
    private Navigation.Navigation3D.fsa.MigrationFeedback.Mode migrationMode;
    private Navigation.Navigation3D.fsa.MigrationFeedback migration;
    private Navigation.Navigation3D.fsa.RequestMigration.Mode requestMigrationMode;
    private Navigation.Navigation3D.fsa.RequestMigration requestMigration;
    private Navigation.Navigation3D.fsa.MigrationRenewal.Mode renewalMode;
    private Navigation.Navigation3D.fsa.MigrationRenewal renewal;
    private boolean coreRepairProfile;
    public HybridFSA3D setCoreRepairProfile(boolean enabled){coreRepairProfile=enabled;return this;}
    private final Map<List<GridPoint>,Navigation.Navigation3D.fsa.MigrationRenewal.Plan> renewalPending=Collections.synchronizedMap(new IdentityHashMap<>());
    public HybridFSA3D setMigrationRenewal(String mode){renewalMode=mode==null?null:Navigation.Navigation3D.fsa.MigrationRenewal.Mode.valueOf(mode);return this;}
    public List<Map<String,Object>> getMigrationRenewalWitnesses(){return renewal==null?Collections.emptyList():renewal.witnesses();}
    public Map<String,Long> getMigrationRenewalStatistics(){return renewal==null?Collections.emptyMap():renewal.statistics();}
    private long requestMigrationWork,requestMigrationNanos;
    private final Map<List<GridPoint>,Map<String,Object>> requestMigrationPending=Collections.synchronizedMap(new IdentityHashMap<>());
    public HybridFSA3D setRequestMigration(String mode){requestMigrationMode=mode==null?null:Navigation.Navigation3D.fsa.RequestMigration.Mode.valueOf(mode);return this;}
    public List<Map<String,Object>> getRequestMigrationWitnesses(){return requestMigration==null?Collections.emptyList():requestMigration.witnesses();}
    public long getRequestMigrationWork(){return requestMigrationWork;}
    public long getRequestMigrationNanos(){return requestMigrationNanos;}
    // The coordinator and the single executor worker are still two concurrent threads.
    // Publish the route and its score together; separate volatile fields would not suffice.
    static final class MigrationIncumbent {
        final List<GridPoint> path;
        final double cost;
        MigrationIncumbent(List<GridPoint> path,double cost){
            this.path=Collections.unmodifiableList(ConditionalCooperation.copy(path));this.cost=cost;
        }
    }
    private volatile MigrationIncumbent migrationIncumbent;
    private void publishMigrationIncumbent(){
        if(migrationMode!=null||requestMigrationMode!=null)migrationIncumbent=new MigrationIncumbent(bestPath,bestCost);
    }
    MigrationIncumbent migrationIncumbentSnapshot(){return migrationIncumbent;}
    private final Map<List<GridPoint>,Map<String,Object>> migrationPending=Collections.synchronizedMap(new IdentityHashMap<>());
    public HybridFSA3D setMigrationFeedback(String mode){migrationMode=mode==null?null:Navigation.Navigation3D.fsa.MigrationFeedback.Mode.valueOf(mode);return this;}
    public List<Map<String,Object>> getMigrationWitnesses(){return migration==null?Collections.emptyList():migration.witnesses();}
    private Navigation.Navigation3D.fsa.PassageRejoin.Mode passageMode;
    private Navigation.Navigation3D.fsa.PassageRejoin passage;
    private final Map<List<GridPoint>,Map<String,Object>> passagePending=Collections.synchronizedMap(new IdentityHashMap<>());
    public HybridFSA3D setPassageRejoin(String mode){
        passageMode=mode==null?null:Navigation.Navigation3D.fsa.PassageRejoin.Mode.valueOf(mode);return this;
    }
    public List<Map<String,Object>> getPassageWitnesses(){return passage==null?Collections.emptyList():passage.witnesses();}
    private final List<Map<String,Object>> headingWitnesses = new ArrayList<>();
    private final Map<List<GridPoint>,Map<String,Object>> headingPending = Collections.synchronizedMap(new IdentityHashMap<>());
    public HybridFSA3D setInformedHeading(String mode) {
        if(mode!=null && !Arrays.asList("PLAIN_HEADING","SOCIAL_HEADING").contains(mode))
            throw new IllegalArgumentException("unknown heading mode");
        headingMode=mode;return this;
    }
    public List<Map<String,Object>> getHeadingWitnesses(){return headingWitnesses;}
    private String alarmWrapMode;
    private AlarmWrapOperator alarmWrap;
    public HybridFSA3D setAlarmWrap(String mode) {
        if(mode!=null && !Arrays.asList("ALARM_WRAP","SINGLE_SECTION","LOCAL_REPLAN","LOCAL_FOLLOW","RANDOM_SECTION").contains(mode))
            throw new IllegalArgumentException("unknown alarm wrap/control mode");
        alarmWrapMode=mode;return this;
    }
    public List<Map<String,Object>> getAlarmWrapWitnesses() {
        return alarmWrap==null?Collections.emptyList():alarmWrap.witnesses();
    }
    public List<Map<String,Object>> getSelectiveTransactions(){return alarmWrap==null?Collections.emptyList():alarmWrap.transactions();}
    private long actionOpportunities;
    public HybridFSA3D setActionTransfer(String mode) {
        if(mode!=null && !Arrays.asList("A0","A1","G0","G1","NF_A1").contains(mode))
            throw new IllegalArgumentException("unknown action transfer mode");
        actionTransferMode=mode;return this;
    }
    public List<Map<String,Object>> getActionTransferWitnesses() {
        return actionTransfer==null?Collections.emptyList():actionTransfer.witnesses();
    }
    private long improvementWallMillis;
    private long cooperationNanos;
    private boolean v5ExperimentContract;
    private boolean nonSwarmControl;
    public HybridFSA3D setNonSwarmControl(boolean enabled) {nonSwarmControl=enabled;return this;}
    public List<Map<String,Object>> getCooperationAttribution() {
        return conditionalCooperation==null?Collections.emptyList():conditionalCooperation.attribution();
    }

    /** Opt-in research increment; no changes to legacy/default or safety objectives. */
    public HybridFSA3D setConditionalCooperation(String bits) {
        conditionalOptions = bits == null ? null : new ConditionalCooperation.Options(bits); return this;
    }
    public HybridFSA3D setImprovementWallMillis(long millis) {
        if (millis <= 0) throw new IllegalArgumentException("positive wall budget required");
        improvementWallMillis=millis; return this;
    }
    public HybridFSA3D setV5ExperimentContract(boolean enabled) { v5ExperimentContract=enabled;return this; }
    public long getCooperationNanos() {return cooperationNanos;}
    public double getBestCost() {return bestCost;}
    public List<Map<String,Object>> getConditionalWitnesses() {
        return conditionalCooperation==null ? Collections.emptyList() : conditionalCooperation.witnesses();
    }
    private double routeDistanceBudgetMultiplier = Double.NaN;
    private double activeRouteDistanceBudgetNm = Double.POSITIVE_INFINITY;
    /** 跨日标记点：与 NavigationObjective3D.getDaySwitchPoints() 口径一致，用于 PathVisualizer 可视化。 */
    private List<PathVisualizer.MarkerPoint> daySwitchMarkers = new ArrayList<>();

    // ==================== 构造 & 设参 ====================
    public HybridFSA3D(NavigationConfig config) {
        this.config = config;
    }

    public HybridFSA3D setPopulationSize(int v) { populationSize = Math.max(4, v); return this; }
    public HybridFSA3D setMaxIterations(int v) { maxIterations = Math.max(1, v); return this; }
    public HybridFSA3D setVisualRange(double v) { visualRange = initialVisualRange = Math.max(2.0, v); return this; }
    public HybridFSA3D setEliteCapacity(int v) { eliteCapacity = Math.max(1, v); return this; }
    public HybridFSA3D setDeferSocialIter(int v) { deferSocialIter = Math.max(0, v); return this; }
    public HybridFSA3D setNumCandidatesPerStep(int v) { numCandidatesPerStep = Math.max(5, v); return this; }
    public HybridFSA3D setBaseSeed(long v) { baseSeed = v; return this; }

    /** 覆盖内部线程池大小（默认 4，固定值）。仅用于实验对照，正常运行不要改。 */
    public HybridFSA3D setThreadPoolSize(int v) { threadPoolSize = Math.max(1, v); return this; }

    /**
     * 辅助机制（猎物侦察 / 逃逸反应）的确定性随机源。
     *
     * 2026-09-24 修正：原为 `new Random(baseSeed + System.nanoTime())`，种子取自系统纳秒时钟，
     * 导致同一份代码在不同批次/不同机器上给出不同结果（实测同一格 S5-FUEL 30 次均值 7 月 24.86
     * vs 9 月 26.49）。改为 baseSeed + 单调递增计数：每 run 新建实例、计数从 0 开始，
     * 且调用顺序由单线程主循环决定 ⇒ 结果可逐位复现，且与线程数/机器无关。
     * 目前两处调用都在主协调线程收集完鱼群结果后执行，因而调用顺序可复现；
     * AtomicLong 保留作防御性保护，避免该辅助方法未来移入工作线程池后发生竞态。
     */
    private final java.util.concurrent.atomic.AtomicLong auxRngSeq =
            new java.util.concurrent.atomic.AtomicLong();

    private java.util.Random auxRng() {
        if (Boolean.getBoolean("fsa.auxRng.time")) {
            return new java.util.Random(baseSeed + System.nanoTime());
        }
        long n = auxRngSeq.getAndIncrement();
        return new java.util.Random(baseSeed * 1_000_003L + n * 0x9E3779B97F4A7C15L);
    }
    public HybridFSA3D setStaticSingleDayMode(boolean v) { staticSingleDayMode = v; return this; }
    public HybridFSA3D setForceDayIndex(int v) { forceDayIndex = v; return this; }
    public HybridFSA3D setSeedPath(List<GridPoint> path) { seedPath = path; return this; }
    public HybridFSA3D setHeuristicWeight(double v) { heuristicWeight = Math.max(0.0, v); return this; }
    public HybridFSA3D setSocialWeight(double v) { socialWeight = Math.max(0.0, v); return this; }
    public HybridFSA3D setIndividualWeight(double v) { individualWeight = Math.max(0.0, v); return this; }
    public HybridFSA3D setObjectiveMode(ObjectiveMode mode) { objectiveMode = mode; return this; }
    public HybridFSA3D setObjectiveWeights(ObjectiveWeights weights) { objectiveWeights = weights; return this; }
    public HybridFSA3D setSearchWorkLedger(SearchWorkLedger ledger) { searchWorkLedger = ledger; return this; }
    public HybridFSA3D setRouteDistanceBudgetMultiplier(double kappa) {
        if (!Double.isFinite(kappa) || kappa < 1.0) {
            throw new IllegalArgumentException("detour kappa must be finite and at least 1.0");
        }
        routeDistanceBudgetMultiplier = kappa;
        return this;
    }
    public HybridFSA3D setEnableLateralLine(boolean v) { enableLateralLine = v; return this; }
    public HybridFSA3D setEnableAdaptiveVision(boolean v) { enableAdaptiveVision = v; return this; }
    public HybridFSA3D setEnableSwarmFusion(boolean v) { enableSwarmFusion = v; return this; }
    public HybridFSA3D setEnableEscapeResponse(boolean v) { enableEscapeResponse = v; return this; }
    public HybridFSA3D setEnableScatteredForaging(boolean v) { enableScatteredForaging = v; return this; }
    public HybridFSA3D setEnableForagingImitation(boolean v) { enableForagingImitation = v; return this; }
    public HybridFSA3D setEnableForagingExploration(boolean v) { enableForagingExploration = v; return this; }
    public HybridFSA3D setEnablePreyInspection(boolean v) { enablePreyInspection = v; return this; }
    public HybridFSA3D setEnableMultiplicativeSuccessorSelection(boolean v) {
        enableMultiplicativeSuccessorSelection = v;
        return this;
    }
    // [REMOVED] 时空经验记忆不再参与搜索；保留旧接口以兼容外部调用。
    @Deprecated
    public HybridFSA3D setEnableExperienceMemory(boolean v) { return this; }
    @Deprecated
    public HybridFSA3D setExperienceMemoryDecay(double v) { return this; }

    public List<Double> getConvergenceHistory() { return new ArrayList<>(convergenceHistory); }
    public double getSeedCost() { return seedCost; }
    public List<GridPoint> getBestPath() { return new ArrayList<>(bestPath); }
    /** execute() 后返回内部评估器，供外部做口径一致的保底比较。 */
    public ObjectiveEvaluator getObjectiveEvaluator() { return lastEvaluator; }
    public List<EnvironmentFrame> getFrames() { return Collections.unmodifiableList(frames); }
    /** 跨日标记点（仅多日真实环境下才非空）。由 evaluatePathFull 顺带计算并缓存。 */
    public List<PathVisualizer.MarkerPoint> getDaySwitchPoints() { return new ArrayList<>(daySwitchMarkers); }

    // ==================== 主入口 ====================
    public NavigationResult execute() throws Exception {
        if(alarmWrapMode!=null && (objectiveMode!=ObjectiveMode.TIME || threadPoolSize!=1
                || improvementWallMillis<=0 || !v5ExperimentContract || seedPath==null || seedPath.size()<2
                || conditionalOptions!=null || actionTransferMode!=null || nonSwarmControl))
            throw new IllegalArgumentException("alarm increment requires TIME, one worker, seed, strict contract, wall budget and a separate arm");
        if ((conditionalOptions != null || actionTransferMode != null || nonSwarmControl) && objectiveMode != ObjectiveMode.TIME)
            throw new IllegalArgumentException("v5 first increment supports TIME only; safety model is not adapted");
        if ((conditionalOptions != null || actionTransferMode != null) && (threadPoolSize != 1 || improvementWallMillis <= 0))
            throw new IllegalArgumentException("v5 requires one worker and an explicit wall budget");
        if(actionTransferMode!=null && conditionalOptions!=null)
            throw new IllegalArgumentException("action transfer and v5 cooperation are separate experiment arms");
        if(actionTransferMode!=null && (!v5ExperimentContract || seedPath==null || seedPath.size()<2))
            throw new IllegalArgumentException("action transfer requires a common seed and strict experiment contract");
        if (improvementWallMillis>0 && searchWorkLedger==null) searchWorkLedger=new SearchWorkLedger();
        long t0 = System.currentTimeMillis();
        validateConfig();
        loadAllEnvironmentData();
        calculateGridResolution();

        FsaContext ctx = buildContext();
        ObjectiveEvaluator evaluator = buildEvaluator(ctx);
        this.lastEvaluator = evaluator;  // 缓存供外部复用

        long cpuBefore=measureSearchCpu?processCpuNanos():-1;
        try {
            runFsa(ctx, evaluator);
        } finally {
            long cpuAfter=measureSearchCpu?processCpuNanos():-1;
            searchCpuNanos=cpuBefore>=0&&cpuAfter>=cpuBefore?cpuAfter-cpuBefore:-1;
            recordSearchMilestone(true);
            // Full-result evaluation and JSON/report construction are outside optimizer search work.
            if (searchWorkLedger != null) searchWorkLedger.setActive(false);
        }

        wallClockMs = System.currentTimeMillis() - t0;
        return buildResult(ctx, evaluator);
    }

    // ==================== 数据加载 ====================
    private void validateConfig() throws IOException {
        if (config == null) throw new IllegalArgumentException("config 为空");
        if (config.area == null || config.grid == null || config.shipParameters == null)
            throw new IllegalArgumentException("area/grid/shipParameters 不能为空");
        if (config.navigationDate == null || config.navigationDate.isBlank())
            throw new IllegalArgumentException("navigationDate 不能为空");
        if (config.dataFolder == null || config.dataFolder.isBlank())
            throw new IllegalArgumentException("dataFolder 不能为空");

        departureOffsetHours = config.resolveDepartureOffsetHours();
        try { baseDate = LocalDate.parse(config.navigationDate, DATE_FORMATTER); }
        catch (DateTimeParseException e) {
            throw new IllegalArgumentException("日期格式 yyyyMMdd: " + config.navigationDate, e);
        }
        dataFolderPath = Path.of(config.dataFolder);
        if (!Files.isDirectory(dataFolderPath))
            throw new IOException("数据目录不存在: " + dataFolderPath.toAbsolutePath());
    }

    private void loadAllEnvironmentData() throws IOException {
        numDays = 0;
        for (int off = 0; off < 30; off++) {
            String txt = DATE_FORMATTER.format(baseDate.plusDays(off));
            if (Files.exists(dataFolderPath.resolve(txt + "_wind.json")) &&
                Files.exists(dataFolderPath.resolve(txt + "_cur.json")) &&
                Files.exists(dataFolderPath.resolve(txt + "_wave.json"))) numDays++;
            else break;
        }
        if (numDays == 0)
            throw new IOException("未找到环境数据: " + config.navigationDate);
        frames.clear();
        for (int off = 0; off < numDays; off++)
            frames.add(loadFrame(baseDate.plusDays(off)));

        if (staticSingleDayMode && frames.size() >= 1) {
            int srcDay = (forceDayIndex >= 0 && forceDayIndex < frames.size()) ? forceDayIndex : 0;
            EnvironmentFrame ds = frames.get(srcDay);
            for (int i = 0; i < frames.size(); i++) frames.set(i, ds);
        }

        Path obFile = dataFolderPath.resolve((config.obstacleFileName == null || config.obstacleFileName.isBlank())
                ? "obstacle_map.json" : config.obstacleFileName);
        ObstacleOutput obOut = objectMapper.readValue(obFile.toFile(), ObstacleOutput.class);
        if (obOut.gridXY.nx != config.grid.nx || obOut.gridXY.ny != config.grid.ny)
            throw new IllegalStateException("障碍网格尺寸不一致");
        obstacleMap = obOut.obstacleMap;
        obstacleBufferMap = copyBoolMap(obstacleMap);
        GridPoint s = startP(), e = endP();
        if (obstacleMap[s.i][s.j]) throw new IllegalStateException("起点在障碍上");
        if (obstacleMap[e.i][e.j]) throw new IllegalStateException("终点在障碍上");
    }

    private EnvironmentFrame loadFrame(LocalDate date) throws IOException {
        String txt = DATE_FORMATTER.format(date);
        WindOutput w = objectMapper.readValue(dataFolderPath.resolve(txt + "_wind.json").toFile(), WindOutput.class);
        CurrentOutput c = objectMapper.readValue(dataFolderPath.resolve(txt + "_cur.json").toFile(), CurrentOutput.class);
        WaveOutput v = objectMapper.readValue(dataFolderPath.resolve(txt + "_wave.json").toFile(), WaveOutput.class);
        int ny = config.grid.ny, nx = config.grid.nx;
        return new EnvironmentFrame(to2D(w.windDataU, ny, nx), to2D(w.windDataV, ny, nx),
                to2D(c.currentDataU, ny, nx), to2D(c.currentDataV, ny, nx),
                to2D(v.waveDataD, ny, nx), to2D(v.waveDataP, ny, nx), to2D(v.waveDataH, ny, nx));
    }

    private double[][] to2D(List<Double> data, int rows, int cols) {
        if (data == null || data.size() != rows * cols)
            throw new IllegalStateException("数据长度不匹配: " + (data == null ? "null" : data.size()) + " vs " + rows * cols);
        double[][] r = new double[rows][cols];
        int k = 0;
        for (int i = 0; i < rows; i++)
            for (int j = 0; j < cols; j++)
                r[i][j] = data.get(k++) == null ? Double.NaN : data.get(k - 1);
        return r;
    }

    private boolean[][] copyBoolMap(boolean[][] src) {
        boolean[][] d = new boolean[src.length][src[0].length];
        for (int i = 0; i < src.length; i++) System.arraycopy(src[i], 0, d[i], 0, src[i].length);
        return d;
    }

    private void calculateGridResolution() {
        deltaLat = (config.area.latMax - config.area.latMin) / Math.max(1, config.grid.ny - 1);
        deltaLon = (config.area.lonMax - config.area.lonMin) / Math.max(1, config.grid.nx - 1);
        double lc = (config.area.latMin + config.area.latMax) / 2.0;
        dLatNm = deltaLat * 60.0;
        dLonNm = deltaLon * 60.0 * Math.cos(Math.toRadians(lc));
    }

    // ==================== FSA 主体 ====================
    private FsaContext buildContext() {
        Normalizers.Bundle bundle = new Normalizers.Bundle(null, null);
        if (objectiveMode == ObjectiveMode.COMPOSITE || (Boolean.getBoolean("nav220.screen")
                && (objectiveMode==ObjectiveMode.SAFETY_DENSITY || objectiveMode==ObjectiveMode.SAFETY_EXPOSURE))) {
            bundle = Normalizers.build(
                    config, config.shipParameters,
                    FsaContext.DIRECTIONS, FsaContext.BEARINGS,
                    new Normalizers.EnvironmentSampleProvider() {
                        @Override public int numDays() { return numDays; }
                        @Override public int ny() { return config.grid.ny; }
                        @Override public int nx() { return config.grid.nx; }
                        @Override public boolean isObstacle(int i, int j) {
                            return obstacleMap != null && obstacleMap[i][j];
                        }
                        // Preserve unsupported cells as NaN so Normalizers.build skips
                        // incomplete vectors, matching PathEvaluator and the shared
                        // fail-closed environmental policy. Zero-filling here changes
                        // the COMPOSITE objective's global time/fuel normalizers.
                        @Override public double windU(int d, int i, int j) { return frames.get(d).windUData[i][j]; }
                        @Override public double windV(int d, int i, int j) { return frames.get(d).windVData[i][j]; }
                        @Override public double currentU(int d, int i, int j) { return frames.get(d).currentUData[i][j]; }
                        @Override public double currentV(int d, int i, int j) { return frames.get(d).currentVData[i][j]; }
                        @Override public double waveHeight(int d, int i, int j) { return frames.get(d).waveHeightData[i][j]; }
                        @Override public double wavePeriod(int d, int i, int j) { return frames.get(d).wavePeriodData[i][j]; }
                        @Override public double waveDir(int d, int i, int j) { return frames.get(d).waveDirData[i][j]; }
                    }
            );
        }
        return new FsaContext(config, config.shipParameters,
                config.grid.ny, config.grid.nx, numDays, departureOffsetHours,
                Collections.unmodifiableList(new ArrayList<>(frames)),
                obstacleMap, obstacleBufferMap,
                dLatNm, dLonNm, deltaLat, deltaLon,
                bundle.timeNormalizer(), bundle.fuelNormalizer(), searchWorkLedger);
    }

    private ObjectiveEvaluator buildEvaluator(FsaContext ctx) {
        boolean shipCorr = config.enableCompositeShipClassCorrection || objectiveMode != ObjectiveMode.COMPOSITE;
        switch (objectiveMode) {
            case TIME: return new TimeEvaluator(ctx, shipCorr);
            case SAFETY_DENSITY: return new SafetyDensityEvaluator(ctx, shipCorr);
            case SAFETY_EXPOSURE:
            case SAFETY: return new SafetyEvaluator(ctx, shipCorr);
            case FUEL: return new FuelEvaluator(ctx, shipCorr);
            case COMPOSITE:
                if (objectiveWeights == null)
                    throw new IllegalStateException("COMPOSITE 模式需要设置 objectiveWeights");
                return new CompositeEvaluator(ctx, objectiveWeights);
            default: throw new IllegalStateException("未知目标: " + objectiveMode);
        }
    }

    private double scoreCandidateRoute(FsaContext ctx, ObjectiveEvaluator evaluator,
                                       List<GridPoint> path) {
        if(alarmWrap!=null)alarmWrap.ordinaryEvaluation();
        if(passage!=null)passage.ordinaryEvaluation();
        if(migration!=null)migration.ordinaryEvaluation();
        if(requestMigration!=null)requestMigration.ordinaryEvaluation();
        if (ctx.searchWorkLedger != null) ctx.searchWorkLedger.recordRouteObjectiveEvaluation();
        if (ctx.searchWorkLedger != null && ctx.searchWorkLedger.shouldStopSearch())
            return Double.POSITIVE_INFINITY;
        if (path == null || !ctx.routeGraph.validate(path, null, null,
                activeRouteDistanceBudgetNm).valid) return Double.POSITIVE_INFINITY;
        if (v5ExperimentContract && (!path.get(0).equals(seedPath.get(0))
                || !path.get(path.size()-1).equals(seedPath.get(seedPath.size()-1))
                || new HashSet<>(path).size()!=path.size())) return Double.POSITIVE_INFINITY;
        Navigation.Navigation3D.fsa.CandidateEvaluation217.Result outcome217=Navigation.Navigation3D.fsa.CandidateEvaluation217.evaluate(path,()->Navigation.Navigation3D.fsa.CandidateScreen220.evaluate(ctx,path,objectiveMode,objectiveMode==ObjectiveMode.COMPOSITE?objectiveWeights:null,Math.min(bestCost,seedCost),()->evaluator.pathFinalCost(path)));
        if(outcome217.unpriced())return Double.NaN; // Explicit UNPRICED result; no response or penalty value.
        double cost=outcome217.legacyScalar();
        if(searchWorkLedger!=null && !searchWorkLedger.timingAllowed())return Double.POSITIVE_INFINITY;
        if(conditionalCooperation!=null)conditionalCooperation.recordEvaluation(path,cost);
        if(requestMigration!=null)requestMigration.observe(path,cost);
        return cost;
    }

    private void runFsa(FsaContext ctx, ObjectiveEvaluator evaluator) throws Exception {
        if(segmentLateralLine && (!legalSuccessorCandidates || successorFormula==Navigation.Navigation3D.fsa.FspoProbability.Mode.LEGACY))
            throw new IllegalArgumentException("segment lateral line requires legal candidates and a refined FSPO formula");
        if(reachableEscape&&!legalSuccessorCandidates)throw new IllegalArgumentException("reachable escape requires common legal geometry");
        if((legalSuccessorCandidates||completeCycleVision||arclengthForaging||reachableEscape||!fusionRefinement.equals("OFF"))
                && (renewalMode!=null||requestMigrationMode!=null||migrationMode!=null||passageMode!=null||headingMode!=null
                    ||alarmWrapMode!=null||conditionalOptions!=null||actionTransferMode!=null||coreRepairProfile||nonSwarmControl))
            throw new IllegalArgumentException("mechanism refinements must be isolated from other experimental profiles");
        if(renewalMode!=null && (objectiveMode!=ObjectiveMode.TIME || threadPoolSize!=1 || !v5ExperimentContract
                || seedPath==null || !enableScatteredForaging || (alarmWrapMode!=null&&!"SINGLE_SECTION".equals(alarmWrapMode))
                || (requestMigrationMode!=null&&requestMigrationMode!=Navigation.Navigation3D.fsa.RequestMigration.Mode.PLAIN3)
                || migrationMode!=null || passageMode!=null || headingMode!=null || conditionalOptions!=null || actionTransferMode!=null || nonSwarmControl))
            throw new IllegalArgumentException("migration renewal requires full TIME FSA, one worker, strict seed contract, and no other experimental increment");
        renewal=null;renewalPending.clear();
        if(requestMigrationMode!=null && (objectiveMode!=ObjectiveMode.TIME || threadPoolSize!=1 || !v5ExperimentContract
                || improvementWallMillis<=0 || seedPath==null || !"SINGLE_SECTION".equals(alarmWrapMode)
                || headingMode!=null || conditionalOptions!=null || actionTransferMode!=null || passageMode!=null || migrationMode!=null))
            throw new IllegalArgumentException("request migration requires isolated TIME/SINGLE_SECTION, one worker, wall/seed");
        requestMigrationPending.clear();requestMigrationWork=0;requestMigrationNanos=0;
        if(migrationMode!=null && (objectiveMode!=ObjectiveMode.TIME || threadPoolSize!=1 || !v5ExperimentContract
                || improvementWallMillis<=0 || seedPath==null || !"SINGLE_SECTION".equals(alarmWrapMode)
                || headingMode!=null || conditionalOptions!=null || actionTransferMode!=null || passageMode!=null))
            throw new IllegalArgumentException("migration requires isolated TIME/SINGLE_SECTION, one worker, wall/seed");
        migrationPending.clear();
        migrationIncumbent=null;
        if(passageMode!=null && (objectiveMode!=ObjectiveMode.TIME || threadPoolSize!=1 || !v5ExperimentContract
                || improvementWallMillis<=0 || seedPath==null || !"SINGLE_SECTION".equals(alarmWrapMode)
                || headingMode!=null || conditionalOptions!=null || actionTransferMode!=null))
            throw new IllegalArgumentException("passage requires TIME, one worker, wall/seed, preserved SINGLE_SECTION and separate controls");
        passagePending.clear();
        if(headingMode!=null && (objectiveMode!=ObjectiveMode.TIME || threadPoolSize!=1 || !v5ExperimentContract
                || improvementWallMillis<=0 || seedPath==null || alarmWrapMode==null || !alarmWrapMode.equals("SINGLE_SECTION")
                || conditionalOptions!=null || actionTransferMode!=null))
            throw new IllegalArgumentException("heading increment requires TIME, one worker, explicit wall/seed and preserved SINGLE_SECTION only");
        headingWitnesses.clear();headingPending.clear();
        cooperationNanos=0;
        if(improvementWallMillis>0) searchWorkLedger.startWallClock(improvementWallMillis);
        refinementWitnesses.clear();foragingWitnesses.clear();
        staticGeometry=legalSuccessorCandidates?new Navigation.Navigation3D.fsa.StaticRouteGeometry(
                ctx.routeGraph,ctx::blocked,segmentLateralLine&&enableLateralLine):null;
        conditionalCooperation=conditionalOptions==null?null:buildConditionalCooperation(ctx,evaluator);
        actionOpportunities=0;
        actionTransfer=actionTransferMode==null?null:new ArrivalActionTransfer(actionTransferMode,
                buildTransferHost(ctx,evaluator,"a1_"),ctx.dLonNm,ctx.dLatNm);
        alarmWrap=alarmWrapMode==null?null:buildAlarmWrap(ctx,(TimeEvaluator)evaluator);
        passage=passageMode==null?null:buildPassage(ctx,(TimeEvaluator)evaluator);
        migration=migrationMode==null?null:buildMigration(ctx,(TimeEvaluator)evaluator);
        requestMigration=requestMigrationMode==null?null:buildRequestMigration(ctx,(TimeEvaluator)evaluator);
        // 起终点：优先用种子路径的起终点（与 PSO/GWO/ACO 一致）
        GridPoint start, end;
        if (seedPath != null && seedPath.size() >= 2) {
            start = seedPath.get(0);
            end = seedPath.get(seedPath.size() - 1);
        } else {
            start = startP();
            end = endP();
        }
        activeRouteDistanceBudgetNm = Double.POSITIVE_INFINITY;
        if (seedPath != null && seedPath.size() >= 2) {
            GridRouteGraph.Validation seedValidation = ctx.routeGraph.validate(
                    seedPath, start, end, Double.POSITIVE_INFINITY);
            if (!seedValidation.valid) {
                throw new IllegalArgumentException("FSA common seed violates the shared route graph: "
                        + seedValidation.reason);
            }
            if (Double.isFinite(routeDistanceBudgetMultiplier)) {
                activeRouteDistanceBudgetNm = seedValidation.distanceNm * routeDistanceBudgetMultiplier;
                if (!Double.isFinite(activeRouteDistanceBudgetNm)) {
                    throw new IllegalArgumentException("FSA route-distance budget is not finite");
                }
            }
        }
        List<Fish> fishes = new ArrayList<>(populationSize);
        for (int k = 0; k < populationSize; k++)
            fishes.add(new Fish(k, baseSeed * 1_000_003L + k));

        // ====== 播种：将种子路径注入初始种群（FSAO 风格：每条鱼都播种种子扰动版） ======
        if (seedPath != null && seedPath.size() >= 2) {
            seedCost = scoreCandidateRoute(ctx, evaluator, seedPath);
            if (Double.isFinite(seedCost)) {
                // 鱼 0：精确复制；部分鱼用随机游走增加种群多样性；其余：种子扰动版
                for (int i = 0; i < populationSize
                        && (searchWorkLedger == null || !searchWorkLedger.shouldStopSearch()); i++) {
                    Fish f = fishes.get(i);
                    List<GridPoint> init;
                    if (i == 0) {
                        init = new ArrayList<>(seedPath);
                    } else if (i % 10 == 0) {
                        // 随机游走（10%）：产生与 Dijkstra 不同的路径，增加搜索空间
                        init = randomWalkPath(start, end, f, ctx);
                    } else {
                        init = perturbPath(seedPath, f.rng);
                    }
                    double c = scoreCandidateRoute(ctx, evaluator, init);
                    // Initialization deletion has a known evaluated parent; random walk does not.
                    if(actionTransfer!=null && i!=0 && i%10!=0)
                        actionTransfer.observe(f.id,seedPath,seedCost,init,c);
                    if (!Double.isFinite(c)) {
                        init = new ArrayList<>(seedPath);
                        c = seedCost;
                    }
                    f.update(init, c);
                }
                bestCost = seedCost;
                bestPath = new ArrayList<>(seedPath);
                publishMigrationIncumbent();
                System.out.printf("[HybridFSA] 种子路径注入（全群播种）: 代价=%.4f, 点数=%d%n",
                        seedCost, seedPath.size());
            }
        }

        EliteArchive elite = new EliteArchive(eliteCapacity,coreRepairProfile);
        // 初始精英：放入种子路径
        if (seedPath != null && Double.isFinite(seedCost)) {
            List<List<GridPoint>> seedList = new ArrayList<>();
            List<Double> seedCosts = new ArrayList<>();
            seedList.add(new ArrayList<>(seedPath));
            seedCosts.add(seedCost);
            elite.rebuildFrom(seedList, seedCosts, ctx.ny, ctx.nx);
        }

        convergenceHistory.clear();
        searchMilestones.clear();recordSearchMilestone(true);

        // [DISABLED] 时空经验记忆初始化
        // if (enableExperienceMemory) {
        //     int emNy = ctx.ny, emNx = ctx.nx;
        //     riskMemory = new double[emNy][emNx][numDays];
        //     timeMemory = new double[emNy][emNx][numDays];
        //     fuelMemory = new double[emNy][emNx][numDays];
        // }

        // 跟踪变量
        double prevBestCost = bestCost;
        int noImproveCount = 0;
        double prevEliteBest = Double.POSITIVE_INFINITY;
        int eliteStagnation = 0;
        int escapeStagnation = 0;  // 逃逸反应独立计数器
        Navigation.Navigation3D.fsa.CycleVisionControl cycleVision=new Navigation.Navigation3D.fsa.CycleVisionControl();

        // 分散搜索参数
        int seedLen = (seedPath != null) ? seedPath.size() : 0;
        int segLen = Math.max(1, Math.min(seedLen - 2, seedLen / populationSize));
        int[] scatterSchedule=arclengthForaging&&enableScatteredForaging&&seedLen>2
                ?Navigation.Navigation3D.fsa.ForagingSchedule.build(seedPath,populationSize,ctx.dLatNm,ctx.dLonNm):null;
        if(renewalMode!=null)renewal=new Navigation.Navigation3D.fsa.MigrationRenewal(renewalMode,seedPath,seedCost,ctx.dLonNm,ctx.dLatNm);

        ExecutorService pool = Executors.newFixedThreadPool(threadPoolSize);
        try {
            for (int iter = 1; iter <= maxIterations
                    && (searchWorkLedger == null || !searchWorkLedger.shouldStopSearch()); iter++) {
                final double cycleStartBest=bestCost;
                if(scatterSchedule!=null){
                    ctx.recordOperatorEvent("refinement_foraging_schedule_cycles");
                    if(foragingWitnesses.size()<64){
                        Map<String,Object>w=new LinkedHashMap<>();w.put("iteration",iter);List<Integer> assignments=new ArrayList<>();
                        for(int id=0;id<populationSize;id++)assignments.add(Navigation.Navigation3D.fsa.ForagingSchedule.departure(scatterSchedule,id,iter));
                        w.put("departure_index_by_stable_fish_id",assignments);w.put("distinct_departures",new HashSet<>(assignments).size());foragingWitnesses.add(w);
                    }
                }
                // [DISABLED] 时空经验记忆衰减
                // if (enableExperienceMemory) {
                //     for (int emI = 0; emI < ctx.ny; emI++)
                //         for (int emJ = 0; emJ < ctx.nx; emJ++)
                //             for (int emD = 0; emD < numDays; emD++) {
                //                 riskMemory[emI][emJ][emD] *= experienceMemoryDecay;
                //                 timeMemory[emI][emJ][emD] *= experienceMemoryDecay;
                //                 fuelMemory[emI][emJ][emD] *= experienceMemoryDecay;
                //             }
                // }
                final boolean socialActive = !nonSwarmControl && iter > deferSocialIter;
                if(renewal!=null)renewal.beginGeneration(iter);
                elite.setActive(socialActive);
                List<Future<FishResult>> futures = new ArrayList<>(populationSize);

                // 分散搜索（Scattered Foraging）：仿生鱼群分散-觅食-回归-信息共享
                // 鱼从种子路径不同散播点出发，用 FSPO 探索到终点。
                // 固定 30% 鱼进入觅食队列；精英模仿与随机邻居探索是两个独立算子。
                for (int fi = 0; fi < fishes.size(); fi++) {
                    Fish fish = fishes.get(fi);
                    final Fish f = fish;
                    final GridPoint scatterPt;
                    final List<GridPoint> prefix;
                    final int scatterIdx;
                    // 固定 30% 鱼构成觅食队列；精英模仿和随机邻居探索分别开关。
                    final boolean foragingCohort = ((arclengthForaging?f.id:fi) % 10 < 3);
                    final boolean exploreMode = foragingCohort && enableForagingExploration;
                    final Navigation.Navigation3D.fsa.MigrationRenewal.Plan renewalPlan=renewal==null?null:renewal.plan(f);
                    if(renewalPlan!=null){
                        scatterIdx=renewalPlan.cut;scatterPt=renewalPlan.start;prefix=new ArrayList<>(renewalPlan.prefix);
                    } else if (enableScatteredForaging && seedLen > 2) {
                        scatterIdx = scatterSchedule==null?Math.min(fi * segLen, Math.max(0, seedLen - 2))
                                :Navigation.Navigation3D.fsa.ForagingSchedule.departure(scatterSchedule,f.id,iter);
                        scatterPt = seedPath.get(scatterIdx);
                        prefix = (scatterIdx > 0)
                                ? new ArrayList<>(seedPath.subList(0, scatterIdx + 1))
                                : new ArrayList<>();
                        if (scatterIdx > 0) ctx.recordOperatorEvent("scattered_foraging_departures");
                    } else {
                        scatterIdx = 0;
                        scatterPt = start;
                        prefix = new ArrayList<>();
                    }

                    futures.add(pool.submit(() -> {
                        FishResult alarmResult=runAlarmSlot(f);
                        if(alarmResult!=null)return alarmResult;
                        if(renewalPlan!=null){
                            ctx.recordOperatorEvent("renewal_long_departures");
                            if("SOCIAL_PREVIOUS_GENERATION".equals(renewalPlan.witness.get("source_kind")))ctx.recordOperatorEvent("renewal_social_departures");
                        }
                        // 觅食模仿：部分鱼直接复制精英路径段并微调，其余用 FSPO 构造
                        if (renewalPlan==null && foragingCohort && enableForagingImitation
                                && elite.size() > 0 && seedPath != null) {
                            ctx.recordOperatorEvent(nonSwarmControl?"nf_local_perturbation_attempts":"foraging_imitation_attempts");
                            FishResult imitationResult = foragingImitation(f, elite, ctx, evaluator);
                            if (imitationResult != null) {
                                if(arclengthForaging)imitationResult.foragingOrigin="imitation";
                                ctx.recordOperatorEvent(nonSwarmControl?"nf_local_perturbation_returns":"foraging_imitation_successful_returns");
                                return imitationResult;
                            }
                        }
                        // Construction returns the complete route, including the seed prefix.
                        FishResult built=constructCandidateRoute(ctx, evaluator, elite, f,
                                scatterPt, end, socialActive, prefix, exploreMode, departureOffsetHours);
                        if(arclengthForaging&&built!=null&&built.foragingOrigin==null)built.foragingOrigin=exploreMode?"exploration":"guided";
                        if(renewalPlan!=null&&built!=null)renewalPending.put(built.path,renewalPlan);
                        return built;
                    }));
                }

                List<List<GridPoint>> iterPaths = new ArrayList<>();
                List<Double> iterCosts = new ArrayList<>();
                // [DISABLED] iterBestPath tracking for experience memory
                // List<GridPoint> iterBestPath = null;
                // double iterBestCost = Double.POSITIVE_INFINITY;
                for (Future<FishResult> fut : futures) {
                    FishResult r = fut.get();
                    if (r.path != null && r.path.size() >= 2 && Double.isFinite(r.cost)) {
                        if (searchWorkLedger!=null && searchWorkLedger.wallLimitReached()) continue;
                        if (conditionalCooperation!=null) conditionalCooperation.observe(r.fish.id,r.path,r.cost);
                        if(r.actionTransferred && r.cost<r.fish.personalBestCost-EPSILON)
                            ctx.recordOperatorEvent("a1_committed_personal_improvements");
                        if(r.actionTransferred && r.cost<bestCost-EPSILON)
                            ctx.recordOperatorEvent("a1_committed_incumbent_improvements");
                        if(r.alarmWitness!=null) {
                            boolean personal=r.cost<r.fish.personalBestCost-EPSILON;
                            boolean incumbent=r.cost<bestCost-EPSILON;
                            r.alarmWitness.put("recipient_before",r.fish.personalBestCost);
                            r.alarmWitness.put("incumbent_before",bestCost);
                            r.alarmWitness.put("recipient_owner",r.fish.id);
                            r.alarmWitness.put("committed_personal",personal);
                            r.alarmWitness.put("committed_incumbent",incumbent);
                            if(personal)ctx.recordOperatorEvent("aw_committed_personal_improvements");
                            if(incumbent)ctx.recordOperatorEvent("aw_committed_incumbent_improvements");
                            if(personal)ctx.recordOperatorEvent("aw_committed_personal_"+r.alarmWitness.get("stage"));
                            if(incumbent)ctx.recordOperatorEvent("aw_committed_incumbent_"+r.alarmWitness.get("stage"));
                            if(AlarmWrapOperator.isSelectiveMode(alarmWrapMode)){
                                if(personal)ctx.recordOperatorEvent("sf_committed_personal_"+r.alarmWitness.get("stage"));
                                if(incumbent)ctx.recordOperatorEvent("sf_committed_incumbent_"+r.alarmWitness.get("stage"));
                                if("selection".equals(r.alarmWitness.get("stage"))){
                                    String suffix=Boolean.TRUE.equals(r.alarmWitness.get("choice_opportunity"))?"with_choice":"without_choice";
                                    if(personal)ctx.recordOperatorEvent("sf_committed_personal_selection_"+suffix);
                                    if(incumbent)ctx.recordOperatorEvent("sf_committed_incumbent_selection_"+suffix);
                                }
                            }
                        }
                        Map<String,Object> mw=migrationPending.remove(r.path);
                        Map<String,Object> rw=requestMigrationPending.remove(r.path);
                        if(rw!=null){
                            boolean incumbent=r.cost<bestCost-EPSILON;
                            rw.put("incumbent_before",bestCost);rw.put("committed_incumbent",incumbent);
                            rw.put("incumbent_gain",incumbent?bestCost-r.cost:0);
                            if(incumbent)ctx.recordOperatorEvent("rm_committed_"+rw.get("origin"));
                        }
                        if(mw!=null){
                            boolean incumbent=r.cost<bestCost-EPSILON;
                            mw.put("incumbent_before",bestCost);mw.put("committed_incumbent",incumbent);
                            mw.put("incumbent_gain",incumbent?bestCost-r.cost:0);
                            if(incumbent)ctx.recordOperatorEvent("mf_committed_"+mw.get("origin"));
                            if(incumbent&&"continuation".equals(mw.get("origin")))ctx.recordOperatorEvent("mf_committed_action_"+mw.get("action"));
                        }
                        Map<String,Object> pw=passagePending.remove(r.path);
                        if(pw!=null){
                            boolean personal=r.cost<r.fish.personalBestCost-EPSILON,incumbent=r.cost<bestCost-EPSILON;
                            pw.put("recipient_before",r.fish.personalBestCost);pw.put("incumbent_before",bestCost);
                            pw.put("committed_personal",personal);pw.put("committed_incumbent",incumbent);
                            pw.put("incumbent_gain",incumbent?bestCost-r.cost:0);
                            pw.put("incumbent_extra_gain_over_shortcut",incumbent?Math.max(0,
                                    Math.min(bestCost,((Number)pw.get("shortcut_best")).doubleValue())-r.cost):0);
                            if(personal)ctx.recordOperatorEvent("pr_committed_personal_"+pw.get("origin"));
                            if(incumbent)ctx.recordOperatorEvent("pr_committed_incumbent_"+pw.get("origin"));
                        }
                        Map<String,Object> hw=headingPending.remove(r.path);
                        if(hw!=null){
                            boolean personal=r.cost<r.fish.personalBestCost-EPSILON,incumbent=r.cost<bestCost-EPSILON;
                            hw.put("recipient_before",r.fish.personalBestCost);hw.put("incumbent_before",bestCost);
                            hw.put("candidate_cost",r.cost);hw.put("committed_personal",personal);hw.put("committed_incumbent",incumbent);
                            if(personal)ctx.recordOperatorEvent("ih_committed_personal");
                            if(incumbent){ctx.recordOperatorEvent("ih_committed_incumbent");
                                ctx.recordOperatorEvent(((Number)hw.get("social_selected_steps")).intValue()>0?"ih_incumbent_with_social_direction":"ih_incumbent_without_social_direction");
                                if(((Number)hw.get("surviving_social_edges")).intValue()>0)ctx.recordOperatorEvent("ih_incumbent_social_edge_survived");}
                        }
                        recordRenewal(r,ctx);
                        if(arclengthForaging&&r.foragingOrigin!=null){
                            String refinementPrefix229=("imitation".equals(r.foragingOrigin)||"exploration".equals(r.foragingOrigin))?"refinement_fg_":"refinement_common_";
                            ctx.recordOperatorEvent(refinementPrefix229+r.foragingOrigin+"_finite_returns");
                            if(r.cost<r.fish.personalBestCost)ctx.recordOperatorEvent(refinementPrefix229+r.foragingOrigin+"_personal_improvements");
                            if(r.cost<bestCost)ctx.recordOperatorEvent(refinementPrefix229+r.foragingOrigin+"_global_improvements");
                        }
                        r.fish.update(r.path, r.cost);
                        iterPaths.add(r.path);
                        iterCosts.add(r.cost);
                        if (r.cost < bestCost) {
                            bestCost = r.cost;
                            bestPath = new ArrayList<>(r.path);
                            publishMigrationIncumbent();
                        }
                        // [DISABLED] iterBestPath tracking
                        // if (r.cost < iterBestCost) {
                        //     iterBestCost = r.cost;
                        //     iterBestPath = r.path;
                        // }
                    }
                }
                headingPending.clear();passagePending.clear();migrationPending.clear();renewalPending.clear();
                // All ordinary workers are joined before a cooperation-only sampling deadline.
                runConditionalSlots(fishes,iter);
                // [DISABLED] 时空经验记忆回写
                // if (enableExperienceMemory && iterBestPath != null) {
                //     updateExperienceMemory(ctx, iterBestPath);
                // }

                // 重建精英
                if (iter >= deferSocialIter && !iterPaths.isEmpty()) {
                    List<List<GridPoint>> candidates = new ArrayList<>(iterPaths);
                    List<Double> candCosts = new ArrayList<>(iterCosts);
                    for (Fish f : fishes) {
                        if (Double.isFinite(f.personalBestCost) && f.personalBestPath != null && f.personalBestPath.size() >= 2) {
                            candidates.add(f.personalBestPath);
                            candCosts.add(f.personalBestCost);
                        }
                    }
                    elite.rebuildFrom(candidates, candCosts, ctx.ny, ctx.nx);
                }
                // ── 自适应视野（Area-Restricted Search）──
                // 改善 → 缩小视野精细挖掘；停滞 → 扩大视野跳出局部
                final double beforeLateOperators=bestCost;
                if (enableAdaptiveVision && !completeCycleVision) {
                    ctx.recordOperatorEvent("adaptive_vision_checks");
                    double priorVisualRange = visualRange;
                    if (bestCost < prevBestCost * 0.999) {
                        noImproveCount = 0;
                        visualRange = Math.max(VR_MIN, visualRange * 0.85);
                    } else {
                        noImproveCount++;
                    }
                    if (noImproveCount >= 3) {
                        visualRange = Math.min(VR_MAX, visualRange * 1.5);
                        noImproveCount = 0;
                    }
                    if (Double.compare(priorVisualRange, visualRange) != 0) {
                        ctx.recordOperatorEvent("adaptive_vision_adjustments");
                    }
                }
                // prevBestCost 在逃逸反应之后更新（L527）
                // 精英档案连续 3 代最优不变 → 种群分裂探索，再合并
                double currentEliteBest = (elite.best() != null) ? elite.best().cost : Double.POSITIVE_INFINITY;
                if (Double.isFinite(currentEliteBest) && Math.abs(currentEliteBest - prevEliteBest) < 1e-9) {
                    eliteStagnation++;
                } else {
                    eliteStagnation = 0;
                }
                prevEliteBest = currentEliteBest;

                if (!nonSwarmControl && enableSwarmFusion && eliteStagnation >= FUSION_STAGNATION_THRESHOLD && iter < maxIterations - FUSION_ROUNDS) {
                    ctx.recordOperatorEvent("swarm_fusion_events");
                    eliteStagnation = 0;
                    if(!fusionRefinement.equals("OFF")){
                        runRefinedFusion(fishes,start,end,ctx,evaluator,elite,socialActive,seedLen,segLen,iter,scatterSchedule);
                    } else {
                    // 随机打乱并分成子群
                    List<Fish> splitOrder=(coreRepairProfile||arclengthForaging)?new ArrayList<>(fishes):fishes;
                    Collections.shuffle(splitOrder, new Random(baseSeed + iter));
                    int subSize = populationSize / FUSION_SPLIT;
                    List<EliteArchive> fusionArchives = new ArrayList<>(FUSION_SPLIT);
                    for (int g = 0; g < FUSION_SPLIT; g++) {
                        List<Fish> subGroup = splitOrder.subList(g * subSize,
                                (g == FUSION_SPLIT - 1) ? populationSize : (g + 1) * subSize);
                        // 每个子群从共同的分裂前精英和自身个体最优解出发，随后维护独立档案。
                        EliteArchive groupElite = new EliteArchive(eliteCapacity,coreRepairProfile);
                        groupElite.setActive(socialActive);
                        List<List<GridPoint>> initialGroupPaths = new ArrayList<>();
                        List<Double> initialGroupCosts = new ArrayList<>();
                        for (EliteArchive.Entry entry : elite.snapshotEntries()) {
                            initialGroupPaths.add(entry.path);
                            initialGroupCosts.add(entry.cost);
                        }
                        for (Fish f : subGroup) {
                            if (Double.isFinite(f.personalBestCost) && f.personalBestPath != null
                                    && f.personalBestPath.size() >= 2) {
                                initialGroupPaths.add(f.personalBestPath);
                                initialGroupCosts.add(f.personalBestCost);
                            }
                        }
                        groupElite.rebuildFrom(initialGroupPaths, initialGroupCosts, ctx.ny, ctx.nx);
                        for (int r = 0; r < FUSION_ROUNDS && (!coreRepairProfile||searchWorkLedger==null||!searchWorkLedger.shouldStopSearch()); r++) {
                            ctx.recordOperatorEvent("swarm_fusion_group_rounds");
                            final boolean sa = socialActive;
                            final EliteArchive activeGroupElite = groupElite;
                            List<Future<FishResult>> subFutures = new ArrayList<>(subGroup.size());
                            for (Fish fish : subGroup) {
                                final Fish f = fish;
                                int fi = fishes.indexOf(f);
                                int sIdx = (enableScatteredForaging && fi >= 0)
                                        ?(scatterSchedule==null?Math.min(fi * segLen, Math.max(0, seedLen - 2))
                                            :Navigation.Navigation3D.fsa.ForagingSchedule.departure(scatterSchedule,f.id,iter)):0;
                                final Navigation.Navigation3D.fsa.MigrationRenewal.Plan renewalPlan=renewal==null?null:renewal.plan(f);
                                GridPoint sPt = renewalPlan!=null?renewalPlan.start:(enableScatteredForaging && seedPath != null && sIdx < seedLen)
                                        ? seedPath.get(sIdx) : start;
                                List<GridPoint> pfx = renewalPlan!=null?new ArrayList<>(renewalPlan.prefix):(enableScatteredForaging && seedPath != null && sIdx > 0)
                                        ? new ArrayList<>(seedPath.subList(0, sIdx + 1)) : new ArrayList<>();
                                if (sIdx > 0) ctx.recordOperatorEvent("scattered_foraging_departures");
                                // 与主循环使用同一觅食队列；随机探索独立受开关控制。
                                final boolean exploreMode = ((arclengthForaging?f.id:fi) % 10 < 3) && enableForagingExploration;
                                subFutures.add(pool.submit(() -> {
                                    if(renewalPlan!=null){ctx.recordOperatorEvent("renewal_long_departures");if("SOCIAL_PREVIOUS_GENERATION".equals(renewalPlan.witness.get("source_kind")))ctx.recordOperatorEvent("renewal_social_departures");}
                                    // Use the same complete-route contract as the main swarm.
                                    FishResult built=constructCandidateRoute(ctx, evaluator, activeGroupElite, f,
                                            sPt, end, sa, pfx, exploreMode, departureOffsetHours);
                                    if(renewalPlan!=null&&built!=null)renewalPending.put(built.path,renewalPlan);
                                    return built;
                                }));
                            }
                            List<List<GridPoint>> subPaths = new ArrayList<>();
                            List<Double> subCosts = new ArrayList<>();
                            for (Future<FishResult> fut : subFutures) {
                                FishResult res = fut.get();
                                if (res.path != null && res.path.size() >= 2 && Double.isFinite(res.cost)
                                        && (searchWorkLedger==null || searchWorkLedger.timingAllowed())) {
                                    recordRenewal(res,ctx);
                                    res.fish.update(res.path, res.cost);
                                    subPaths.add(res.path);
                                    subCosts.add(res.cost);
                                    if (res.cost < bestCost) {
                                        bestCost = res.cost;
                                        bestPath = new ArrayList<>(res.path);
                                        publishMigrationIncumbent();
                                    }
                                }
                            }
                            // 子群内保留此前档案，并合入本轮路径和个体最优解。
                            if (!subPaths.isEmpty()) {
                                List<List<GridPoint>> cands = new ArrayList<>();
                                List<Double> costs = new ArrayList<>();
                                for (EliteArchive.Entry entry : groupElite.snapshotEntries()) {
                                    cands.add(entry.path);
                                    costs.add(entry.cost);
                                }
                                cands.addAll(subPaths);
                                costs.addAll(subCosts);
                                for (Fish f : subGroup) {
                                    if (Double.isFinite(f.personalBestCost) && f.personalBestPath != null && f.personalBestPath.size() >= 2) {
                                        cands.add(f.personalBestPath);
                                        costs.add(f.personalBestCost);
                                    }
                                }
                                groupElite.rebuildFrom(cands, costs, ctx.ny, ctx.nx);
                            }
                        }
                        fusionArchives.add(groupElite);
                    }
                    // 三个独立档案在分裂搜索结束后汇合，保留原全局精英和全部子群成果。
                    List<List<GridPoint>> mergedPaths = new ArrayList<>();
                    List<Double> mergedCosts = new ArrayList<>();
                    for (EliteArchive.Entry entry : elite.snapshotEntries()) {
                        mergedPaths.add(entry.path);
                        mergedCosts.add(entry.cost);
                    }
                    for (EliteArchive groupArchive : fusionArchives) {
                        for (EliteArchive.Entry entry : groupArchive.snapshotEntries()) {
                            mergedPaths.add(entry.path);
                            mergedCosts.add(entry.cost);
                        }
                    }
                    for (Fish f : fishes) {
                        if (Double.isFinite(f.personalBestCost) && f.personalBestPath != null
                                && f.personalBestPath.size() >= 2) {
                            mergedPaths.add(f.personalBestPath);
                            mergedCosts.add(f.personalBestCost);
                        }
                    }
                    elite.rebuildFrom(mergedPaths, mergedCosts, ctx.ny, ctx.nx);
                    // 融合后恢复视野到初始值
                    if (enableAdaptiveVision && !completeCycleVision) visualRange = initialVisualRange;
                    }
                }

                // ── 猎物侦察 + 逃逸反应（Prey Inspection + Escape Response）──
                // 停滞计数（独立于开关，两个机制都依赖）
                if (bestCost >= prevBestCost * 0.999) {
                    escapeStagnation++;
                } else {
                    escapeStagnation = 0;
                }
                // 猎物侦察：停滞 3 轮时触发，在最优路径附近采样 K 个点
                if (enablePreyInspection && escapeStagnation == 3) {
                    ctx.recordOperatorEvent("prey_inspection_events");
                    if (preyInspection(ctx, evaluator, elite)) {
                        escapeStagnation = 0;
                    }
                }
                // escape response → 大幅弹射
                if (enableEscapeResponse && escapeStagnation >= 5) {
                    ctx.recordOperatorEvent("escape_response_events");
                    escapeStagnation = 0;
                    escapePerturb(fishes, start, end, ctx, evaluator, elite, socialActive);
                }

                if(enableAdaptiveVision && completeCycleVision){
                    ctx.recordOperatorEvent("adaptive_vision_checks");
                    ctx.recordOperatorEvent("refinement_complete_cycle_vision_checks");
                    double oldRadius=visualRange;
                    visualRange=cycleVision.next(visualRange,cycleStartBest,bestCost);
                    if(oldRadius!=visualRange)ctx.recordOperatorEvent("adaptive_vision_adjustments");
                    if(beforeLateOperators>=cycleStartBest*.999 && bestCost<cycleStartBest*.999)
                        ctx.recordOperatorEvent("refinement_vision_late_improvement_credited");
                }

                // Log the incumbent at the end of the complete search cycle, after
                // all enabled operators have had an opportunity to improve it.
                convergenceHistory.add(Double.isFinite(bestCost) ? bestCost : Double.NaN);
                recordSearchMilestone(false);
                prevBestCost = bestCost;  // 在逃逸反应之后更新

                if (iter % 5 == 0 || iter == maxIterations) {
                    System.out.printf("  [迭代 %d/%d] best=%.4f vis=%.1f%n", iter, maxIterations, bestCost, visualRange);
                }
            }
        } finally {
            pool.shutdown();
            pool.awaitTermination(60, TimeUnit.SECONDS);
        }
        if (bestPath.isEmpty())
            throw new IllegalStateException("HybridFSA3D 未找到可行路径");

        // ── 起终点校验 + 种子保底 ──
        GridPoint pathStart = bestPath.get(0);
        GridPoint pathEnd = bestPath.get(bestPath.size() - 1);
        boolean startOk = (pathStart.i == start.i && pathStart.j == start.j);
        boolean endOk = (pathEnd.i == end.i && pathEnd.j == end.j);
        if (!startOk) {
            System.out.printf("[HybridFSA] WARNING: bestPath 起点(%d,%d)≠配置起点(%d,%d)%n",
                    pathStart.i, pathStart.j, start.i, start.j);
        }
        if (!endOk) {
            System.out.printf("[HybridFSA] WARNING: bestPath 终点(%d,%d)≠配置终点(%d,%d)%n",
                    pathEnd.i, pathEnd.j, end.i, end.j);
        }
        // 种子保底：仅在 FSA 完全失败时回退（不再因代价略差而覆盖 FSA 结果）
        // 分散搜索的鱼从种子路径不同位置出发，构造的路径可能不连回种子起点，
        // 但这些路径的代价可能远优于种子。起终点强制回退会扼杀分散搜索的探索能力。
        if (seedPath != null && seedPath.size() >= 2) {
            if (bestPath == null || bestPath.size() < 2) {
                System.out.printf("[HybridFSA] FSA 未找到有效路径，回退到种子路径 (代价 %.6f)%n", seedCost);
                bestPath = new ArrayList<>(seedPath);
                bestCost = seedCost;
                publishMigrationIncumbent();
            } else if (!startOk || !endOk) {
                System.out.printf("[HybridFSA] 起终点不同但代价更优: 代价 %.6f vs 种子 %.6f, 保留FSA路径%n",
                        bestCost, seedCost);
            }
        }
    }

    /** Local paid experience, round-robin subgroup work, same strict objective and common ledger. */
    private void runRefinedFusion(List<Fish> fishes,GridPoint start,GridPoint end,FsaContext ctx,
                                 ObjectiveEvaluator evaluator,EliteArchive elite,boolean socialActive,
                                 int seedLen,int segLen,int iteration,int[] scatterSchedule)throws Exception{
        ctx.recordOperatorEvent("refinement_fusion_events");
        List<Fish> order=new ArrayList<>(fishes);
        order.sort(Comparator.comparingInt((Fish f)->scatterSchedule==null?Math.min(f.id*segLen,Math.max(0,seedLen-2))
                :Navigation.Navigation3D.fsa.ForagingSchedule.departure(scatterSchedule,f.id,iteration)).thenComparingInt(f->f.id));
        List<List<Fish>> groups=new ArrayList<>();List<EliteArchive> archives=new ArrayList<>();
        List<EliteArchive.Entry> global=elite.snapshotEntries();
        for(int g=0;g<FUSION_SPLIT;g++){
            List<Fish> group=new ArrayList<>(order.subList(g*fishes.size()/FUSION_SPLIT,(g+1)*fishes.size()/FUSION_SPLIT));
            groups.add(group);
            EliteArchive archive=new EliteArchive(eliteCapacity,true);archive.setActive(socialActive);
            List<List<GridPoint>> paths=new ArrayList<>();List<Double> costs=new ArrayList<>();
            int anchorCount=fusionRefinement.equals("LOCAL")?Math.min(1,global.size()):global.size();
            for(int i=0;i<anchorCount;i++){paths.add(global.get(i).path);costs.add(global.get(i).cost);}
            for(Fish f:group)if(Double.isFinite(f.personalBestCost)&&f.personalBestPath!=null&&f.personalBestPath.size()>=2){
                paths.add(f.personalBestPath);costs.add(f.personalBestCost);
            }
            archive.rebuildFrom(paths,costs,ctx.ny,ctx.nx);archives.add(archive);
        }
        Set<String> distinctInitial=new HashSet<>();
        for(EliteArchive archive:archives){
            List<String> shapes=new ArrayList<>();
            for(EliteArchive.Entry e:archive.snapshotEntries())shapes.add(Navigation.Navigation3D.fsa.RequestMigration.shapeKey(e.path));
            Collections.sort(shapes);distinctInitial.add(shapes.toString());
        }
        Map<String,Object>w=new LinkedHashMap<>();w.put("iteration",iteration);w.put("mode",fusionRefinement);
        w.put("distinct_initial_archives",distinctInitial.size());w.put("social_active",socialActive);
        w.put("start_cost",bestCost);w.put("start_work",searchWorkLedger==null?-1:searchWorkLedger.snapshot().edgeSubsegmentAttempts);
        List<Map<String,Object>> rounds=new ArrayList<>();w.put("rounds",rounds);
        boolean[] active=new boolean[FUSION_SPLIT];Arrays.fill(active,true);
        for(int round=0;round<FUSION_ROUNDS;round++)for(int g=0;g<FUSION_SPLIT;g++){
            if(!active[g])continue;
            if(searchWorkLedger!=null&&searchWorkLedger.shouldStopSearch()){active[g]=false;continue;}
            ctx.recordOperatorEvent("swarm_fusion_group_rounds");ctx.recordOperatorEvent("refinement_fusion_group_rounds");
            long beforeWork=searchWorkLedger==null?-1:searchWorkLedger.snapshot().edgeSubsegmentAttempts;
            double beforeGlobal=bestCost;int submitted=0,personalImprovements=0;
            EliteArchive archive=archives.get(g);
            for(Fish fish:groups.get(g)){
                if(searchWorkLedger!=null&&searchWorkLedger.shouldStopSearch())break;
                int fi=fish.id,sIdx=enableScatteredForaging?(scatterSchedule==null?Math.min(fi*segLen,Math.max(0,seedLen-2))
                        :Navigation.Navigation3D.fsa.ForagingSchedule.departure(scatterSchedule,fish.id,iteration)):0;
                GridPoint sPt=enableScatteredForaging&&seedPath!=null&&sIdx<seedLen?seedPath.get(sIdx):start;
                List<GridPoint> prefix=enableScatteredForaging&&seedPath!=null&&sIdx>0?new ArrayList<>(seedPath.subList(0,sIdx+1)):new ArrayList<>();
                if(sIdx>0)ctx.recordOperatorEvent("scattered_foraging_departures");
                boolean explore=(fi%10<3)&&enableForagingExploration;
                // Freeze each group's social archive for a complete round, as in the reference worker batch.
                FishResult result=constructCandidateRoute(ctx,evaluator,archive,fish,sPt,end,socialActive,prefix,explore,departureOffsetHours);
                submitted++;
                if(result.path==null||result.path.size()<2||!Double.isFinite(result.cost)
                        ||(searchWorkLedger!=null&&!searchWorkLedger.timingAllowed()))continue;
                double prior=fish.personalBestCost;fish.update(result.path,result.cost);
                if(fish.personalBestCost<prior){personalImprovements++;ctx.recordOperatorEvent("refinement_fusion_personal_improvements");}
                if(result.cost<bestCost){bestCost=result.cost;bestPath=new ArrayList<>(result.path);publishMigrationIncumbent();ctx.recordOperatorEvent("refinement_fusion_global_improvements");}
            }
            List<List<GridPoint>> paths=new ArrayList<>();List<Double> costs=new ArrayList<>();
            for(EliteArchive.Entry e:archive.snapshotEntries()){paths.add(e.path);costs.add(e.cost);}
            for(Fish fish:groups.get(g))if(Double.isFinite(fish.personalBestCost)&&fish.personalBestPath!=null&&fish.personalBestPath.size()>=2){
                paths.add(fish.personalBestPath);costs.add(fish.personalBestCost);
            }
            archive.rebuildFrom(paths,costs,ctx.ny,ctx.nx);
            active[g]=personalImprovements>0;
            if(!active[g])ctx.recordOperatorEvent("refinement_fusion_group_stopped_no_gain");
            Map<String,Object>row=new LinkedHashMap<>();row.put("group",g);row.put("round",round);row.put("submitted",submitted);
            row.put("personal_improvements",personalImprovements);row.put("global_improved",bestCost<beforeGlobal);
            row.put("work",searchWorkLedger==null?-1:searchWorkLedger.snapshot().edgeSubsegmentAttempts-beforeWork);rounds.add(row);
        }
        List<List<GridPoint>> merged=new ArrayList<>();List<Double> costs=new ArrayList<>();
        for(EliteArchive.Entry e:global){merged.add(e.path);costs.add(e.cost);}
        for(EliteArchive archive:archives)for(EliteArchive.Entry e:archive.snapshotEntries()){merged.add(e.path);costs.add(e.cost);}
        for(Fish fish:fishes)if(Double.isFinite(fish.personalBestCost)&&fish.personalBestPath!=null&&fish.personalBestPath.size()>=2){merged.add(fish.personalBestPath);costs.add(fish.personalBestCost);}
        elite.rebuildFrom(merged,costs,ctx.ny,ctx.nx);
        w.put("final_cost",bestCost);w.put("end_work",searchWorkLedger==null?-1:searchWorkLedger.snapshot().edgeSubsegmentAttempts);
        if(refinementWitnesses.size()<256)refinementWitnesses.add(w);
        // This operator never overwrites the main swarm's adaptive radius.
    }

    /**
     * Shared main-swarm/subgroup entry point. The constructor returns a complete
     * route; callers must never prepend the prefix again after simplification.
     */
    private FishResult constructCandidateRoute(FsaContext ctx, ObjectiveEvaluator evaluator,
                                              EliteArchive elite, Fish fish,
                                              GridPoint scatterStart, GridPoint end,
                                              boolean socialActive, List<GridPoint> prefix,
                                              boolean exploreMode, double elapsed) {
        List<GridPoint> completeRoute = constructCompleteRoute(ctx, evaluator, elite, fish,
                scatterStart, end, socialActive, prefix, exploreMode, elapsed);
        if (completeRoute == null || completeRoute.size() < 2) {
            if (seedPath != null){
                FishResult fallback=new FishResult(fish,new ArrayList<>(seedPath),seedCost);
                if(arclengthForaging)fallback.foragingOrigin="seed_fallback";
                return fallback;
            }
            return new FishResult(fish, null, Double.POSITIVE_INFINITY);
        }
        double cost = scoreCandidateRoute(ctx, evaluator, completeRoute);
        return new FishResult(fish, completeRoute, cost);
    }

    private List<GridPoint> constructHeadingRoute(FsaContext ctx,TimeEvaluator evaluator,EliteArchive elite,Fish fish,
                                                   GridPoint start,GridPoint end,List<GridPoint> prefix,boolean socialActive){
        Navigation.Navigation3D.fsa.InformedHeading.Host host=new Navigation.Navigation3D.fsa.InformedHeading.Host(){
            public boolean allowed(){return searchWorkLedger.evaluationAllowed();}
            public boolean completedAllowed(){return searchWorkLedger.timingAllowed();}
            public boolean legal(GridPoint a,GridPoint b){return ctx.routeGraph.isValidEdge(a,b);}
            public double distance(GridPoint a,GridPoint b){return ctx.gridDistanceNm(a.i,a.j,b.i,b.j);}
            public double bearing(GridPoint a,GridPoint b){return ctx.bearingOf(a.i,a.j,b.i,b.j);}
            public double sample(GridPoint a,GridPoint b,double elapsed){
                EdgeSampler.EdgeResult e=evaluator.sampleEdge(a.i,a.j,b.i,b.j,elapsed);
                return e.valid && completedAllowed()?e.endElapsedHours:Double.POSITIVE_INFINITY;
            }
            public List<GridPoint> candidates(GridPoint a,Set<GridPoint> visited){
                Set<Long> keys=new HashSet<>();for(GridPoint p:visited)keys.add(key(p.i,p.j));
                List<GridPoint> out=new ArrayList<>();
                for(int[] p:getVisualCandidates(a.i,a.j,ctx,keys)){
                    if(!allowed())break;
                    GridPoint b=new GridPoint(p[0],p[1]);if(legal(a,b))out.add(b);
                }
                return out;
            }
            public void count(String name){ctx.recordOperatorEvent(name);}
        };
        Navigation.Navigation3D.fsa.InformedHeading op=new Navigation.Navigation3D.fsa.InformedHeading(host,headingMode.equals("SOCIAL_HEADING"));
        double initial=host.bearing(seedPath.get(0),seedPath.get(1));
        List<EliteArchive.Entry> leaders=socialActive?elite.snapshotEntries():new ArrayList<>();
        leaders.removeIf(e->e.path.equals(fish.personalBestPath));
        List<GridPoint> result=op.build(prefix,start,end,initial,leaders,
                visualRange*Math.max(ctx.dLatNm,ctx.dLonNm),Math.max(ctx.ship.shipSpeed*1.944,0.1),
                Math.min(400,Math.max(100,(fish.personalBestPath==null?seedPath.size():fish.personalBestPath.size())*2)));
        op.witness.put("owner",fish.id);op.witness.put("social_active",socialActive);
        if(headingWitnesses.size()<128)headingWitnesses.add(op.witness);
        if(result==null)return null;
        result=postProcessPath(result,ctx);op.witness.put("final_path",new ArrayList<>(result));
        int survived=0;
        for(Object edge:(List<?>)op.witness.get("social_edges")){
            List<?> pair=(List<?>)edge;
            for(int i=1;i<result.size();i++)if(result.get(i-1).equals(pair.get(0)) && result.get(i).equals(pair.get(1))){survived++;break;}
        }
        op.witness.put("surviving_social_edges",survived);
        headingPending.put(result,op.witness);return result;
    }

    // ==================== FSAO 风格路径构造（参考 FSAO/鱼群路径优化1215.docx） ====================
    private void recordRenewal(FishResult r,FsaContext ctx){
        if(renewal==null)return;
        Navigation.Navigation3D.fsa.MigrationRenewal.Plan plan=renewalPending.remove(r.path);
        renewal.record(plan,r.path,r.cost,r.fish.personalBestCost,bestCost);
        renewal.observe(r.fish.id,r.path,r.cost);
        if(plan!=null){
            if(r.cost<bestCost-EPSILON)ctx.recordOperatorEvent("renewal_committed_incumbent");
            if(!r.path.equals(seedPath)&&r.cost>r.fish.personalBestCost+EPSILON)ctx.recordOperatorEvent("renewal_valid_transient_departure");
        }
    }
    /**
     * 返回包含 prefix 的完整路线；prefix 为空时先加入 scatterStart。
     * prefix 非空时必须以 scatterStart 结尾。简化后的路线仍保留起终点。
     * 单条鱼从 scatterStart 到 end 继续构造路线：
     *   1) 若可直连终点且距离在视野内，直接连接
     *   2) 枚举视觉范围内所有未访问、非障碍候选点
     *   3) 用 FSPO 乘法公式计算概率（self × social × heuristic，零参数），轮盘赌选下一点
     *      exploreMode=true 时跳过 FSPO，纯随机选择（扩大搜索空间）
     *   4) 视线不通则找第一个视线通的安全替代
     *   5) 重复直到抵达终点或超步数
     *   6) 最后用视线 funnel 做线性简化
     */
    private List<GridPoint> constructCompleteRoute(FsaContext ctx, ObjectiveEvaluator evaluator,
                                          EliteArchive elite, Fish fish,
                                          GridPoint scatterStart, GridPoint end,
                                          boolean socialActive,
                                          List<GridPoint> prefix,
                                          boolean exploreMode, double elapsed) {
        if(headingMode!=null){
            List<GridPoint> guided=constructHeadingRoute(ctx,(TimeEvaluator)evaluator,elite,fish,scatterStart,end,prefix,socialActive);
            if(guided!=null || !searchWorkLedger.evaluationAllowed())return guided;
            ctx.recordOperatorEvent("ih_legacy_fallbacks");
        }
        Random rng = fish.rng;
        List<GridPoint> path = new ArrayList<>(prefix);
        if (path.isEmpty()) {
            path.add(new GridPoint(scatterStart.i, scatterStart.j));
        } else if (!path.get(path.size() - 1).equals(scatterStart)) {
            // A supplied prefix must end at the actual construction origin.
            return null;
        }
        Set<Long> visited = new HashSet<>();
        for (GridPoint point : path) visited.add(key(point.i, point.j));

        int ci = scatterStart.i, cj = scatterStart.j;
        int personalLen = (fish.personalBestPath != null) ? fish.personalBestPath.size() : 200;
        int maxSteps = Math.min(400, Math.max(100, personalLen * 2));
        int step = 0;
        double emElapsed = elapsed;  // 经验记忆用

        while (!(ci == end.i && cj == end.j) && step < maxSteps) {
            if(coreRepairProfile&&searchWorkLedger!=null&&searchWorkLedger.shouldStopSearch())return null;
            step++;

            // 1) 若终点在视野内且视线通，直接连接
            double distToEndCells = Math.hypot(end.i - ci, end.j - cj);
            if (distToEndCells <= visualRange * 1.5 && hasLineOfSight(ci, cj, end.i, end.j, ctx)) {
                path.add(new GridPoint(end.i, end.j));
                ci = end.i; cj = end.j;
                break;
            }

            // 2) 枚举视觉范围内所有候选
            List<int[]> candidates = getVisualCandidates(ci, cj, ctx, visited);
            if (candidates.isEmpty()) {
                // ghost-edge guard: only connect if end is within visual reach
                if (distToEndCells <= visualRange * 1.5
                        && hasLineOfSight(ci, cj, end.i, end.j, ctx)) {
                    path.add(new GridPoint(end.i, end.j));
                    ci = end.i; cj = end.j;
                }
                break;
            }

            // 3) FSPO 概率轮盘赌
            int[] next = selectNextByFSPO(ci, cj, candidates, end, ctx, fish, elite, socialActive, rng, exploreMode, emElapsed);
            if (next == null) break;
            int ni = next[0], nj = next[1];

            // 4) 视线不通则找最先视线通的替代
            if (!hasLineOfSight(ci, cj, ni, nj, ctx)) {
                int[] alt = findSafeAlternative(ci, cj, candidates, ctx);
                if (alt == null) break;
                ni = alt[0]; nj = alt[1];
            }

            path.add(new GridPoint(ni, nj));
            visited.add(key(ni, nj));
            // 更新经验记忆时间估算
            double stepDistNm = ctx.gridDistanceNm(ci, cj, ni, nj);
            emElapsed += stepDistNm / Math.max(ctx.ship.shipSpeed * 1.944, 0.1);
            ci = ni; cj = nj;
        }

        // 兜底：未到终点但能直连（距离受 visualRange 限制，防止 ghost edge）
        if (!(ci == end.i && cj == end.j)) {
            double finalDist = Math.hypot(end.i - ci, end.j - cj);
            if (finalDist <= visualRange * 1.5 && hasLineOfSight(ci, cj, end.i, end.j, ctx)) {
                path.add(new GridPoint(end.i, end.j));
            }
        }

        if (path.size() < 2) return null;
        GridPoint last = path.get(path.size() - 1);
        if (last.i != end.i || last.j != end.j) return null;

        // 后处理：视线 funnel 简化冗余点
        return postProcessPath(path, ctx);
    }

    /**
     * FSAO 风格候选生成：在 visualRange 内枚举所有未访问、非障碍格点（欧氏距离过滤）。
     */
    private List<int[]> getVisualCandidates(int ci, int cj, FsaContext ctx, Set<Long> visited) {
        List<int[]> candidates = new ArrayList<>();
        int range = (int) Math.ceil(visualRange);
        for (int di = -range; di <= range; di++) {
            for (int dj = -range; dj <= range; dj++) {
                if (di == 0 && dj == 0) continue;
                int ni = ci + di, nj = cj + dj;
                if (ni < 0 || ni >= ctx.ny || nj < 0 || nj >= ctx.nx) continue;
                double d = Math.sqrt(di * di + dj * dj);
                if (d > visualRange) continue;
                if (visited.contains(key(ni, nj))) continue;
                if (ctx.blocked(ni, nj)) continue;
                if(legalSuccessorCandidates && !staticGeometry.legal(new GridPoint(ci,cj),new GridPoint(ni,nj))){
                    ctx.recordOperatorEvent("refinement_common_illegal_candidates_removed");continue;
                }
                candidates.add(new int[]{ni, nj});
            }
        }
        return candidates;
    }

    /**
     * FSPO 概率公式（乘法多线索整合，零参数）：
     *   仿生依据：鱼类整合多重感觉线索（视觉/侧线/记忆）为独立调制过程，
     *   各因素以乘法方式组合（类似多线索整合 Mulit-cue Integration）。
     *   p ∝ max(selfAtt,ε) × max(socialAtt,ε) × max(heurAtt,ε)
     *   优势：零参数（论文无需解释权重）；"最弱环节"效应自动淘汰单因素极差的候选。
     *   视线不通 p *= 0.1；跳跃过长 p *= 0.1
     */
    private int[] selectNextByFSPO(int ci, int cj, List<int[]> candidates, GridPoint end,
                                   FsaContext ctx, Fish fish, EliteArchive elite,
                                   boolean socialActive, Random rng, boolean exploreMode,
                                   double elapsed) {
        int n = candidates.size();
        if (n == 0) return null;
        // 探索模式：纯随机选择，跳过 FSPO 概率计算（扩大搜索空间）
        if (exploreMode) {
            ctx.recordOperatorEvent("foraging_random_selection_events");
            return candidates.get(rng.nextInt(n));
        }
        if (!enableMultiplicativeSuccessorSelection) {
            ctx.recordOperatorEvent("fspo_uniform_successor_draws");
            return candidates.get(rng.nextInt(n));
        }
        ctx.recordOperatorEvent("fspo_multiplicative_successor_draws");
        if (successorFormula != Navigation.Navigation3D.fsa.FspoProbability.Mode.LEGACY)
            return selectNextRefinedFSPO(ci,cj,candidates,end,ctx,fish,elite,socialActive,rng);
        double[] probs = new double[n];
        double total = 0.0;
        double maxJumpNm = 3.0 * ctx.diagDist;

        for (int k = 0; k < n; k++) {
            int ni = candidates.get(k)[0], nj = candidates.get(k)[1];

            // 1) 个体经验：personalBestPath 中是否存在 (ci,cj)→(ni,nj) 这条边
            double selfAtt = selfAttraction(fish, ci, cj, ni, nj);

            // 2) 社会学习：精英解集中含该边的路径占比（默认 0.1）
            double socialAtt = !nonSwarmControl && socialActive ? elite.edgeFrequency(ci, cj, ni, nj, ctx.nx) : 0.1;
            if(nonSwarmControl)ctx.recordOperatorEvent("nf_constant_social_cues");
            else if(socialActive)ctx.recordOperatorEvent("swarm_elite_social_cues");

            // 3) 启发式：候选到终点距离越近越优，1/(1+dist)
            double distToGoalNm = ctx.gridDistanceNm(ni, nj, end.i, end.j);
            double heurAtt = 1.0 / (1.0 + distToGoalNm);

            // 4) 侧线感知：候选点周围 5×5 窗口内障碍越多，吸引力越低
            //    模拟鱼通过侧线感知水压变化避开障碍密集区，无需额外评估开销
            if (enableLateralLine) {
                ctx.recordOperatorEvent("lateral_line_candidate_cues");
                int blockedCount = 0;
                for (int di = -2; di <= 2; di++) {
                    for (int dj = -2; dj <= 2; dj++) {
                        int ri = ni + di, rj = nj + dj;
                        if (ri < 0 || ri >= ctx.ny || rj < 0 || rj >= ctx.nx || ctx.blocked(ri, rj))
                            blockedCount++;
                    }
                }
                heurAtt *= (1.0 - blockedCount / 25.0);
            }

            // [DISABLED] 时空经验记忆（v8.1）：候选点累积经验惩罚
            // if (enableExperienceMemory) {
            //     int memDay = ctx.clampDay(elapsed);
            //     double memPenalty;
            //     if (objectiveMode == ObjectiveMode.COMPOSITE && objectiveWeights != null) {
            //         double rM = riskMemory[ni][nj][memDay] / (1.0 + riskMemory[ni][nj][memDay]);
            //         double tM = timeMemory[ni][nj][memDay] / (1.0 + timeMemory[ni][nj][memDay]);
            //         double fM = fuelMemory[ni][nj][memDay] / (1.0 + fuelMemory[ni][nj][memDay]);
            //         double wS = objectiveWeights.safety;
            //         double wT = objectiveWeights.time;
            //         double wF = objectiveWeights.fuel;
            //         double wTot = wS + wT + wF;
            //         memPenalty = (wS * rM + wT * tM + wF * fM) / Math.max(wTot, 1e-9);
            //     } else {
            //         double raw;
            //         switch (objectiveMode) {
            //             case TIME:  raw = timeMemory[ni][nj][memDay]; break;
            //             case FUEL:  raw = fuelMemory[ni][nj][memDay]; break;
            //             default:    raw = riskMemory[ni][nj][memDay]; break;
            //         }
            //         memPenalty = raw / (1.0 + raw);
            //     }
            //     heurAtt *= (1.0 - 0.5 * memPenalty);
            // }

            // 乘法多线索整合：三因素独立调制，零参数
            double prob = Math.max(selfAtt, 0.01)
                    * Math.max(socialAtt, 0.01)
                    * Math.max(heurAtt, 0.01);

            // 视线不通 → 大幅惩罚
            if (!hasLineOfSight(ci, cj, ni, nj, ctx)) prob *= 0.1;

            // 跳跃过长 → 惩罚
            double edgeNm = ctx.gridDistanceNm(ci, cj, ni, nj);
            if (edgeNm > maxJumpNm) prob *= 0.1;

            probs[k] = Math.max(0.001, prob);
            total += probs[k];
        }

        if (total <= 0) return candidates.get(rng.nextInt(n));

        // 轮盘赌
        double r = rng.nextDouble() * total;
        double sum = 0;
        for (int k = 0; k < n; k++) {
            sum += probs[k];
            if (r <= sum) return candidates.get(k);
        }
        return candidates.get(n - 1);
    }

    private static double selfAttraction(Fish fish, int fi, int fj, int ti, int tj) {
        if (fish.personalBestPath == null || fish.personalBestPath.size() < 2) return 0.2;
        List<GridPoint> p = fish.personalBestPath;
        for (int k = 0; k < p.size() - 1; k++) {
            GridPoint a = p.get(k), b = p.get(k + 1);
            if (a.i == fi && a.j == fj && b.i == ti && b.j == tj) return 1.0;
        }
        return 0.2;
    }

    /** Same candidates and cues as FSPO. Refines only their numerical combination/goal scaling. */
    private int[] selectNextRefinedFSPO(int ci,int cj,List<int[]> candidates,GridPoint end,
                                      FsaContext ctx,Fish fish,EliteArchive elite,boolean socialActive,Random rng) {
        ctx.recordOperatorEvent("fspo_formula_"+successorFormula.name());
        double current=ctx.gridDistanceNm(ci,cj,end.i,end.j),maxJump=3*ctx.diagDist;
        double[] logs=new double[candidates.size()];double legacyMin=Double.POSITIVE_INFINITY,legacyMax=0;
        for(int k=0;k<candidates.size();k++) {
            int ni=candidates.get(k)[0],nj=candidates.get(k)[1];
            double self=selfAttraction(fish,ci,cj,ni,nj);
            double social=!nonSwarmControl&&socialActive?elite.edgeFrequency(ci,cj,ni,nj,ctx.nx):.1;
            if(nonSwarmControl)ctx.recordOperatorEvent("nf_constant_social_cues");
            else if(socialActive)ctx.recordOperatorEvent("swarm_elite_social_cues");
            double clearance=1;
            if(enableLateralLine) {
                ctx.recordOperatorEvent("lateral_line_candidate_cues");
                if(segmentLateralLine){
                    ctx.recordOperatorEvent("refinement_segment_lateral_line_cues");
                    clearance=staticGeometry.segmentCue(new GridPoint(ci,cj),new GridPoint(ni,nj));
                }else{
                    int blocked=0;
                    for(int di=-2;di<=2;di++)for(int dj=-2;dj<=2;dj++){
                        int ri=ni+di,rj=nj+dj;
                        if(ri<0||ri>=ctx.ny||rj<0||rj>=ctx.nx||ctx.blocked(ri,rj))blocked++;
                    }
                    clearance=1-blocked/25.0;
                }
            }
            double edge=ctx.gridDistanceNm(ci,cj,ni,nj),next=ctx.gridDistanceNm(ni,nj,end.i,end.j);
            double penalty=(hasLineOfSight(ci,cj,ni,nj,ctx)?1:.1)*(edge>maxJump?.1:1);
            logs[k]=Navigation.Navigation3D.fsa.FspoProbability.logWeight(successorFormula,self,social,current,next,edge,ctx.diagDist,clearance,penalty);
            double old=Navigation.Navigation3D.fsa.FspoProbability.legacyWeight(self,social,next,clearance,penalty);
            if(old==.001)ctx.recordOperatorEvent("fspo_legacy_floor_candidate");
            legacyMin=Math.min(legacyMin,old);legacyMax=Math.max(legacyMax,old);
            ctx.recordOperatorEvent("fspo_refined_candidates");
        }
        if(legacyMax-legacyMin<=1e-15)ctx.recordOperatorEvent("fspo_legacy_flat_at_refined_draw");
        double[] p=Navigation.Navigation3D.fsa.FspoProbability.normalize(logs);double draw=rng.nextDouble(),sum=0;
        for(int k=0;k<p.length;k++){sum+=p[k];if(draw<=sum)return candidates.get(k);}
        return candidates.get(candidates.size()-1);
    }

    private int[] findSafeAlternative(int ci, int cj, List<int[]> candidates, FsaContext ctx) {
        for (int[] c : candidates) {
            if (hasLineOfSight(ci, cj, c[0], c[1], ctx)) return c;
        }
        return null;
    }

    /**
     * FSAO 风格 funnel 简化：从 i 出发尝试直连 path 中最远的 j，视线通且跨度 ≤ visualRange 才跳过中间冗余点。
     * 限制单跳跨度是为了防止把 Dijkstra 稠密种子路径一口气简化到 [start,end] 两点。
     */
    private List<GridPoint> postProcessPath(List<GridPoint> path, FsaContext ctx) {
        if (path == null || path.size() < 3) return path;
        int maxJumpCells = Math.max(2, (int) Math.ceil(visualRange));
        List<GridPoint> simp = new ArrayList<>();
        simp.add(path.get(0));
        int i = 0;
        while (i < path.size() - 1) {
            int j = path.size() - 1;
            while (j > i + 1) {
                GridPoint a = path.get(i), b = path.get(j);
                int span = Math.max(Math.abs(a.i - b.i), Math.abs(a.j - b.j));
                if (span <= maxJumpCells && hasLineOfSight(a.i, a.j, b.i, b.j, ctx)) break;
                j--;
            }
            if (j > i + 1) {
                simp.add(path.get(j));
                i = j;
            } else {
                simp.add(path.get(i + 1));
                i++;
            }
        }
        return simp;
    }

    /**
     * Bresenham 视线检查：线段经过的所有格子不可被阻塞。
     */
    private boolean hasLineOfSight(int i0, int j0, int i1, int j1, FsaContext ctx) {
        return ctx.routeGraph.isValidEdge(new GridPoint(i0, j0), new GridPoint(i1, j1));
    }

    // ==================== 路径扰动（FSAO 风格：随机删除中间点） ====================
    private List<GridPoint> perturbPath(List<GridPoint> path, Random rng) {
        if (path.size() < 5) return new ArrayList<>(path);
        List<GridPoint> p = new ArrayList<>(path);
        int removeCount = 1 + rng.nextInt(3);
        for (int r = 0; r < removeCount; r++) {
            if (p.size() < 4) break;
            int idx = 1 + rng.nextInt(p.size() - 2);
            p.remove(idx);
        }
        return p;
    }

    /**
     * 随机游走路径构造：用于初始化部分鱼，产生与 Dijkstra 不同的路径以增加搜索空间。
     * 从 start 开始，以 50% 概率随机选择邻居，50% 概率偏向终点方向。
     */
    private List<GridPoint> randomWalkPath(GridPoint start, GridPoint end, Fish fish, FsaContext ctx) {
        Random rng = fish.rng;
        List<GridPoint> path = new ArrayList<>();
        Set<Long> visited = new HashSet<>();
        int ci = start.i, cj = start.j;
        path.add(new GridPoint(ci, cj));
        visited.add(key(ci, cj));
        int maxSteps = 200;

        for (int step = 0; step < maxSteps; step++) {
            if (ci == end.i && cj == end.j) break;
            // 直连检查
            if (Math.hypot(end.i - ci, end.j - cj) <= visualRange * 1.5
                    && hasLineOfSight(ci, cj, end.i, end.j, ctx)) {
                path.add(new GridPoint(end.i, end.j));
                return postProcessPath(path, ctx);
            }
            // 枚举非障碍邻居
            List<int[]> neighbors = new ArrayList<>();
            int range = (int) Math.ceil(visualRange);
            for (int di = -range; di <= range; di++) {
                for (int dj = -range; dj <= range; dj++) {
                    if (di == 0 && dj == 0) continue;
                    int ni = ci + di, nj = cj + dj;
                    if (ni < 0 || ni >= ctx.ny || nj < 0 || nj >= ctx.nx) continue;
                    if (Math.sqrt(di * di + dj * dj) > visualRange) continue;
                    if (visited.contains(key(ni, nj))) continue;
                    if (ctx.blocked(ni, nj)) continue;
                    neighbors.add(new int[]{ni, nj});
                }
            }
            if (neighbors.isEmpty()) break;
            // 50% 随机，50% 偏向终点
            int[] next;
            if (rng.nextDouble() < 0.5) {
                next = neighbors.get(rng.nextInt(neighbors.size()));
            } else {
                neighbors.sort((a, b) -> {
                    double da = Math.hypot(a[0] - end.i, a[1] - end.j);
                    double db = Math.hypot(b[0] - end.i, b[1] - end.j);
                    return Double.compare(da, db);
                });
                next = neighbors.get(0);
            }
            ci = next[0]; cj = next[1];
            path.add(new GridPoint(ci, cj));
            visited.add(key(ci, cj));
        }
        // 兜底直连
        if (!(ci == end.i && cj == end.j)) {
            if (Math.hypot(end.i - ci, end.j - cj) <= visualRange * 1.5
                    && hasLineOfSight(ci, cj, end.i, end.j, ctx)) {
                path.add(new GridPoint(end.i, end.j));
            }
        }
        if (path.size() < 2) return null;
        GridPoint last = path.get(path.size() - 1);
        if (last.i != end.i || last.j != end.j) return null;
        return postProcessPath(path, ctx);
    }

    /**
     * 觅食模仿（Foraging Imitation）：scrounging 策略。
     * 仿生依据：鱼群约 20-30% 个体是「跟随者」（scroungers），不自行觅食，
     * 而是跟随成功觅食者的路径，在其发现的食物源附近做局部搜索（Giraldeau & Caraco 2000）。
     * 算法：从精英档案随机选一条路径，复制其中一段（3-5 个航点），
     * 对这段做小幅扰动（±2 格），用扰动后的段替换原路径对应段。
     * @return 首条有限代价的扰动路径；三次尝试均未生成有效路径时返回 null
     */
    private FishResult foragingImitation(Fish fish, EliteArchive elite,
                                          FsaContext ctx, ObjectiveEvaluator evaluator) {
        // Replace one in four imitation opportunities. No extra outer search loop.
        if(actionTransfer!=null && actionOpportunities++%4==0) {
            long begun=System.nanoTime();
            ConditionalCooperation.Trace transferred=actionTransfer.step(fish.id,fish.personalBestPath,
                    fish.personalBestCost,actionOpportunities/4);
            cooperationNanos+=System.nanoTime()-begun;
            if(transferred!=null)return new FishResult(fish,transferred.path,transferred.cost,true);
        }
        // Preserve the same local perturbation operator; NF uses its own route, never elite copying.
        List<List<GridPoint>> elitePaths = nonSwarmControl
                ? Collections.singletonList(fish.personalBestPath==null?seedPath:fish.personalBestPath) : elite.allPaths();
        if (elitePaths.isEmpty()) return null;

        // 随机选一条精英路径
        List<GridPoint> elitePath = elitePaths.get(fish.rng.nextInt(elitePaths.size()));
        if (elitePath==null || elitePath.size() < 6) return null;
        double observedParentCost=Double.POSITIVE_INFINITY;
        if(actionTransfer!=null) {
            if(nonSwarmControl)observedParentCost=fish.personalBestCost;
            else for(EliteArchive.Entry entry:elite.snapshotEntries())
                if(entry.path.equals(elitePath)){observedParentCost=entry.cost;break;}
        }

        Random rng = fish.rng;
        int maxJumpCells = Math.max(2, (int) Math.ceil(visualRange));

        for (int attempt = 0; attempt < 3; attempt++) {
            ctx.recordOperatorEvent(nonSwarmControl?"nf_local_perturbation_trials":"foraging_imitation_perturbation_trials");
            // 复制精英路径
            List<GridPoint> newPath = new ArrayList<>(elitePath);

            // 选一段（3-5 个航点），在路径中间区域
            int segLen = 3 + rng.nextInt(3); // 3, 4, or 5
            int maxStart = newPath.size() - 1 - segLen;
            if (maxStart < 1) continue;
            int segStart = 1 + rng.nextInt(maxStart);

            // 对段内每个航点做小幅扰动（±2 格）
            boolean changed = false;
            for (int k = segStart; k < segStart + segLen && k < newPath.size() - 1; k++) {
                GridPoint old = newPath.get(k);
                int di = rng.nextInt(5) - 2; // [-2, 2]
                int dj = rng.nextInt(5) - 2;
                int ni = old.i + di, nj = old.j + dj;
                if (ni >= 0 && ni < ctx.ny && nj >= 0 && nj < ctx.nx && !ctx.blocked(ni, nj)) {
                    if (ni != old.i || nj != old.j) {
                        newPath.set(k, new GridPoint(ni, nj));
                        changed = true;
                    }
                }
            }
            if (!changed) continue;

            // 边长检查：扰动后的段与前后连接不能有超长边
            boolean valid = true;
            int checkStart = Math.max(0, segStart - 1);
            int checkEnd = Math.min(newPath.size() - 1, segStart + segLen);
            for (int k = checkStart; k < checkEnd; k++) {
                int span = Math.max(
                        Math.abs(newPath.get(k).i - newPath.get(k + 1).i),
                        Math.abs(newPath.get(k).j - newPath.get(k + 1).j));
                if (span > maxJumpCells) { valid = false; break; }
            }
            if (!valid) continue;

            // 评估
            double cost = scoreCandidateRoute(ctx, evaluator, newPath);
            if (Double.isFinite(cost)) {
                if(actionTransfer!=null)actionTransfer.observe(fish.id,elitePath,observedParentCost,newPath,cost);
                ctx.recordOperatorEvent(nonSwarmControl?"nf_local_finite_candidates":"foraging_imitation_finite_candidate_routes");
                // Return the finite route as this iteration's candidate even
                // when it is worse than the personal best. The coordinator
                // updates personal best only when the route is cheaper.
                return new FishResult(fish, newPath, cost);
            }
        }
        return null;
    }

    /**
     * 猎物侦察（Prey Inspection）：在弹射之前先评估目标区域。
     * 仿生依据：鱼发现潜在食物源时，先派侦察鱼在附近探查，确认后再跟进。
     * 算法：在当前最优路径的航点附近随机采样 K 个点，评估每个点构成的新路径。
     * 如果发现更好的解 → 更新最优解（鱼群迁移）；否则 → 等待逃逸反应。
     * @return true 如果侦察发现了更好的解
     */
    /** Enumerate the original perturbation window intersected with both legal incident-edge neighborhoods. */
    private List<GridPoint> jointReplacementCandidates(List<GridPoint> path,int index,int radius,int maxJump,FsaContext ctx){
        ctx.recordOperatorEvent("refinement_joint_replacement_queries");
        GridPoint old=path.get(index),prev=path.get(index-1),next=path.get(index+1);
        Set<Long> used=new HashSet<>();for(int k=0;k<path.size();k++)if(k!=index)used.add(key(path.get(k).i,path.get(k).j));
        int r0=Math.max(0,Math.max(old.i-radius,Math.max(prev.i-maxJump,next.i-maxJump)));
        int r1=Math.min(ctx.ny-1,Math.min(old.i+radius,Math.min(prev.i+maxJump,next.i+maxJump)));
        int c0=Math.max(0,Math.max(old.j-radius,Math.max(prev.j-maxJump,next.j-maxJump)));
        int c1=Math.min(ctx.nx-1,Math.min(old.j+radius,Math.min(prev.j+maxJump,next.j+maxJump)));
        List<GridPoint> choices=new ArrayList<>();
        for(int r=r0;r<=r1;r++)for(int c=c0;c<=c1;c++){
            if((r==old.i&&c==old.j)||ctx.blocked(r,c)||used.contains(key(r,c)))continue;
            ctx.recordOperatorEvent("refinement_joint_replacement_candidate_checks");
            GridPoint p=new GridPoint(r,c);
            if(staticGeometry.legal(prev,p)&&staticGeometry.legal(p,next))choices.add(p);
        }
        if(choices.isEmpty())ctx.recordOperatorEvent("refinement_joint_replacement_empty");
        return choices;
    }

    private boolean preyInspection(FsaContext ctx, ObjectiveEvaluator evaluator, EliteArchive elite) {
        if (bestPath == null || bestPath.size() < 4) return false;
        Random rng = auxRng();
        int inspectRadius = Math.max(3, (int)(visualRange * 1.5));
        int numSamples = 10;

        for (int s = 0; s < numSamples; s++) {
            if(reachableEscape&&searchWorkLedger!=null&&searchWorkLedger.shouldStopSearch())break;
            ctx.recordOperatorEvent("prey_inspection_waypoint_trials");
            // 随机选一个航点
            int idx = 1 + rng.nextInt(bestPath.size() - 2);
            GridPoint anchor = bestPath.get(idx);

            // 在锚点附近采样
            int ni,nj;
            if(reachableEscape){
                List<GridPoint> choices=jointReplacementCandidates(bestPath,idx,inspectRadius,Math.max(2,(int)Math.ceil(visualRange)),ctx);
                if(choices.isEmpty())continue;
                GridPoint p=choices.get(rng.nextInt(choices.size()));ni=p.i;nj=p.j;
                ctx.recordOperatorEvent("refinement_prey_inspection_reachable_samples");
            }else{
                ni=anchor.i+rng.nextInt(inspectRadius*2+1)-inspectRadius;
                nj=anchor.j+rng.nextInt(inspectRadius*2+1)-inspectRadius;
            }
            if (ni < 0 || ni >= ctx.ny || nj < 0 || nj >= ctx.nx || ctx.blocked(ni, nj)) continue;

            // 构成新路径：前缀 + 采样点 + 后缀
            List<GridPoint> newPath = new ArrayList<>();
            for (int k = 0; k < idx; k++) newPath.add(bestPath.get(k));
            newPath.add(new GridPoint(ni, nj));
            for (int k = idx + 1; k < bestPath.size(); k++) newPath.add(bestPath.get(k));

            // 边长检查
            int maxJumpCells = Math.max(2, (int) Math.ceil(visualRange));
            boolean valid = true;
            for (int k = Math.max(0, idx - 1); k < Math.min(newPath.size() - 1, idx + 1); k++) {
                int span = Math.max(
                        Math.abs(newPath.get(k).i - newPath.get(k + 1).i),
                        Math.abs(newPath.get(k).j - newPath.get(k + 1).j));
                if (span > maxJumpCells) { valid = false; break; }
            }
            if (!valid) continue;

            newPath = postProcessPath(newPath, ctx);
            if (newPath == null || newPath.size() < 2) continue;

            ctx.recordOperatorEvent("prey_inspection_route_probes");
            double newCost = scoreCandidateRoute(ctx, evaluator, newPath);
            if (Double.isFinite(newCost) && newCost < bestCost * 0.999
                    && (searchWorkLedger==null || searchWorkLedger.timingAllowed())) {
                bestCost = newCost;
                bestPath = new ArrayList<>(newPath);
                publishMigrationIncumbent();
                // 让侦察发现的改进进入后续 FSPO 社会学习，而不只更新最终 incumbent。
                elite.offer(newPath, newCost, ctx.ny, ctx.nx);
                ctx.recordOperatorEvent("prey_inspection_improvements");
                return true;
            }
        }
        return false;
    }

    /**
     * 逃逸反应：替换最差鱼为经过局部几何校验的扰动路径。
     * 仿生学依据：鱼群遭遇捕食者时的爆发性弹射行为。
     */
    private void escapePerturb(List<Fish> fishes, GridPoint start, GridPoint end,
                               FsaContext ctx, ObjectiveEvaluator evaluator,
                               EliteArchive elite, boolean socialActive) {
        if (bestPath == null || bestPath.size() < 4) return;
        if(reachableEscape&&searchWorkLedger!=null&&searchWorkLedger.shouldStopSearch())return;
        Random rng = auxRng();
        int maxJumpCells = Math.max(2, (int) Math.ceil(visualRange));
        int jumpRadius = Math.max(4, maxJumpCells);

        // 找最差的非精英鱼
        int worstIdx = 0;
        double worstCost = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < fishes.size(); i++) {
            Fish f = fishes.get(i);
            if (Double.isFinite(f.personalBestCost) && f.personalBestCost > worstCost) {
                worstCost = f.personalBestCost;
                worstIdx = i;
            }
        }

        // 对 bestPath 做大幅扰动
        List<GridPoint> escPath = new ArrayList<>(bestPath);
        int numJumps = Math.min(3, escPath.size() - 2);
        boolean changed = false;
        for (int j = 0; j < numJumps; j++) {
            ctx.recordOperatorEvent("escape_response_waypoint_jumps");
            int idx = 1 + rng.nextInt(escPath.size() - 2);
            GridPoint old = escPath.get(idx);
            GridPoint prev = escPath.get(idx - 1);
            GridPoint next = escPath.get(idx + 1);
            boolean accepted = false;
            if(reachableEscape){
                List<GridPoint> choices=jointReplacementCandidates(escPath,idx,jumpRadius,maxJumpCells,ctx);
                if(!choices.isEmpty()){
                    escPath.set(idx,choices.get(rng.nextInt(choices.size())));accepted=true;changed=true;
                    ctx.recordOperatorEvent("refinement_escape_reachable_samples");
                    ctx.recordOperatorEvent("escape_response_waypoint_jump_accepts");
                }
            }else for (int attempt = 0; attempt < 40; attempt++) {
                ctx.recordOperatorEvent("escape_response_waypoint_jump_trials");
                int ni = old.i + rng.nextInt(jumpRadius * 2 + 1) - jumpRadius;
                int nj = old.j + rng.nextInt(jumpRadius * 2 + 1) - jumpRadius;
                if (ni == old.i && nj == old.j) continue;
                if (!ctx.inBounds(ni, nj) || ctx.blocked(ni, nj)) continue;
                if (Math.max(Math.abs(ni - prev.i), Math.abs(nj - prev.j)) > maxJumpCells
                        || Math.max(Math.abs(ni - next.i), Math.abs(nj - next.j)) > maxJumpCells) continue;
                if (!hasLineOfSight(prev.i, prev.j, ni, nj, ctx)
                        || !hasLineOfSight(ni, nj, next.i, next.j, ctx)) continue;
                escPath.set(idx, new GridPoint(ni, nj));
                accepted = true;
                changed = true;
                ctx.recordOperatorEvent("escape_response_waypoint_jump_accepts");
                break;
            }
            if (!accepted) ctx.recordOperatorEvent("escape_response_waypoint_jump_failures");
        }
        if (!changed) {
            ctx.recordOperatorEvent("escape_response_reject_no_accepted_jump");
            return;
        }

        for (int k = 0; k < escPath.size() - 1; k++) {
            GridPoint a = escPath.get(k), b = escPath.get(k + 1);
            if (!hasLineOfSight(a.i, a.j, b.i, b.j, ctx)) {
                ctx.recordOperatorEvent("escape_response_reject_geometry");
                return;
            }
        }

        // funnel 简化 + 障碍检查
        escPath = postProcessPath(escPath, ctx);
        if (escPath == null || escPath.size() < 2) {
            ctx.recordOperatorEvent("escape_response_reject_postprocess");
            return;
        }

        // 评估并注入
        ctx.recordOperatorEvent("escape_response_route_candidates");
        double escCost = scoreCandidateRoute(ctx, evaluator, escPath);
        if (Double.isFinite(escCost) && (searchWorkLedger==null || searchWorkLedger.timingAllowed())) {
            fishes.get(worstIdx).update(escPath, escCost);
            if (escCost < bestCost) {
                ctx.recordOperatorEvent("escape_response_improvements");
                bestCost = escCost;
                bestPath = new ArrayList<>(escPath);
                publishMigrationIncumbent();
            }
        }
    }

    // [DISABLED] 时空经验记忆回写方法 — 消融实验显示无显著贡献
    // private void updateExperienceMemory(FsaContext ctx, List<GridPoint> path) {
    //     double elapsed = departureOffsetHours;
    //     boolean shipCorr = config.enableCompositeShipClassCorrection || objectiveMode != ObjectiveMode.COMPOSITE;
    //     for (int k = 0; k < path.size() - 1; k++) {
    //         GridPoint a = path.get(k), b = path.get(k + 1);
    //         EdgeSampler.EdgeResult edge = EdgeSampler.sample(
    //                 ctx, a.i, a.j, b.i, b.j, elapsed,
    //                 EdgeSampler.NeedComponents.all(), shipCorr
    //         );
    //         if (!edge.valid) continue;
    //         int day = ctx.clampDay(elapsed);
    //         int cellsSpan = Math.max(1, Math.max(Math.abs(b.i - a.i), Math.abs(b.j - a.j)));
    //         for (int s = 0; s <= cellsSpan; s++) {
    //             double t = (cellsSpan > 0) ? (double) s / cellsSpan : 0;
    //             int si = a.i + (int) Math.round((b.i - a.i) * t);
    //             int sj = a.j + (int) Math.round((b.j - a.j) * t);
    //             if (si >= 0 && si < ctx.ny && sj >= 0 && sj < ctx.nx) {
    //                 double subRisk = edge.riskWeighted / (cellsSpan + 1);
    //                 double subTime = edge.timeHours / (cellsSpan + 1);
    //                 double subFuel = edge.fuelEnergyJ / (cellsSpan + 1);
    //                 riskMemory[si][sj][day] += subRisk;
    //                 timeMemory[si][sj][day] += subTime;
    //                 fuelMemory[si][sj][day] += subFuel;
    //             }
    //         }
    //         elapsed += edge.timeHours;
    //     }
    // }

    // ==================== 工具方法 ====================
    private ConditionalCooperation buildConditionalCooperation(FsaContext ctx,ObjectiveEvaluator evaluator) {
        return new ConditionalCooperation(conditionalOptions,buildTransferHost(ctx,evaluator,"v5_"),baseSeed ^ 0x56354C4F4E47L);
    }
    private ArrivalActionTransfer.Host buildTransferHost(FsaContext ctx,ObjectiveEvaluator evaluator,String eventPrefix) {
        return new ArrivalActionTransfer.Host() {
            public ConditionalCooperation.Trace replay(List<GridPoint> path) {
                if(path==null || path.size()<2 || !allowed())return null;
                searchWorkLedger.recordRouteObjectiveEvaluation();
                if(!ctx.routeGraph.validate(path,seedPath.get(0),seedPath.get(seedPath.size()-1),activeRouteDistanceBudgetNm).valid
                        || new HashSet<>(path).size()!=path.size())return null;
                double[] arrival=new double[path.size()];double elapsed=0;
                for(int k=1;k<path.size();k++) {
                    if(!allowed())return null;
                    GridPoint a=path.get(k-1),b=path.get(k);
                    EdgeSampler.EdgeResult edge=evaluator.sampleEdge(a.i,a.j,b.i,b.j,elapsed);
                    if(edge==null || !edge.valid)return null;
                    elapsed=edge.endElapsedHours;arrival[k]=elapsed;
                }
                if(!allowed() || !ctx.meetsArrivalDeadline(elapsed))return null;
                return new ConditionalCooperation.Trace(path,elapsed,arrival);
            }
            public List<GridPoint> propose(List<GridPoint> path,int left,int right,Random rng) {
                // Reuse the existing deletion perturbation, including its minimum-window rule.
                List<GridPoint> local=perturbPath(path.subList(left,right+1),rng);
                List<GridPoint> result=ConditionalCooperation.copy(path.subList(0,left));
                result.addAll(ConditionalCooperation.copy(local));
                result.addAll(ConditionalCooperation.copy(path.subList(right+1,path.size())));return result;
            }
            public double distance(GridPoint a,GridPoint b){return ctx.gridDistanceNm(a.i,a.j,b.i,b.j);}
            public double heading(GridPoint a,GridPoint b){return ctx.bearingOf(a.i,a.j,b.i,b.j);}
            public boolean allowed(){return searchWorkLedger.evaluationAllowed();}
            public long work(){return searchWorkLedger.snapshot().edgeSubsegmentAttempts;}
            public void event(String name){ctx.recordOperatorEvent(eventPrefix+name);}
            public double incumbentCost(){return bestCost;}
            public double[] environment(ConditionalCooperation.Trace trace,int left,int right) {
                // Read-only proposal descriptors. Actual evaluation remains EdgeSampler's midpoint sampling.
                double[] lengths=new double[right-left+1];
                for(int k=left+1;k<=right;k++)lengths[k-left]=lengths[k-left-1]+distance(trace.path.get(k-1),trace.path.get(k));
                GridPoint a=trace.path.get(left),b=trace.path.get(right);
                double x=(b.j-a.j)*ctx.dLonNm,y=(b.i-a.i)*ctx.dLatNm,len=Math.hypot(x,y);
                if(len<=1e-9)return null;
                double tx=x/len,ty=y/len,speed=Math.max(0.1,ctx.ship.shipSpeed*FsaContext.KNOT_TO_MS);
                double[] values=new double[27];int at=0;
                for(double fraction:new double[]{.25,.5,.75}) {
                    int near=0;double target=fraction*lengths[lengths.length-1];
                    for(int k=1;k<lengths.length;k++)if(Math.abs(lengths[k]-target)<Math.abs(lengths[near]-target))near=k;
                    int index=left+near;GridPoint p=trace.path.get(index);double elapsed=trace.arrival[index];
                    if(!allowed() || !ctx.hasWeatherAt(elapsed) || !ctx.inBounds(p.i,p.j))return null;
                    int day=ctx.clampDay(elapsed);if(day+1>=ctx.frames.size())return null;
                    EnvironmentFrame now=ctx.frames.get(day),next=ctx.frames.get(day+1);
                    double[] raw={now.windUData[p.i][p.j],now.windVData[p.i][p.j],now.currentUData[p.i][p.j],now.currentVData[p.i][p.j]};
                    double[] future={next.windUData[p.i][p.j],next.windVData[p.i][p.j],next.currentUData[p.i][p.j],next.currentVData[p.i][p.j]};
                    for(int k=0;k<4;k++)if(!Double.isFinite(raw[k]) || !Double.isFinite(future[k]))return null;
                    for(int k=0;k<4;k+=2){values[at++]=(raw[k]*tx+raw[k+1]*ty)/speed;values[at++]=(-raw[k]*ty+raw[k+1]*tx)/speed;}
                    for(int k=0;k<4;k+=2){double du=future[k]-raw[k],dv=future[k+1]-raw[k+1];values[at++]=(du*tx+dv*ty)/speed;values[at++]=(-du*ty+dv*tx)/speed;}
                    values[at++]=((day+1)*24-(ctx.departureOffsetHours+elapsed))/24;
                    event("context_vertex_queries");
                }return values;
            }
        };
    }
    private void runConditionalSlots(List<Fish> fishes,int iteration) {
        if(conditionalCooperation==null)return;
        long remaining=improvementWallMillis*1_000_000L/4-cooperationNanos;
        if(remaining<=0 || !searchWorkLedger.evaluationAllowed())return;
        long start=System.nanoTime();searchWorkLedger.beginPhase(remaining);
        try {
            conditionalCooperation.beginRound(iteration);
            List<Fish> ordered=new ArrayList<>(fishes);ordered.sort(Comparator.comparingInt(f->f.id));
            List<List<GridPoint>> population=new ArrayList<>();
            for(Fish f:ordered)population.add(f.personalBestPath);
            for(int slot=0;slot<Math.max(1,fishes.size()/3) && searchWorkLedger.evaluationAllowed();slot++) {
                Fish fish=ordered.get(ConditionalCooperation.ownerIndex(ordered.size(),iteration,slot));
                if(nonSwarmControl)searchWorkLedger.recordOperatorEvent("nf_global_queue_slots");
                ConditionalCooperation.Trace result=conditionalCooperation.step(fish.id,fish.personalBestPath,population);
                if(result!=null && searchWorkLedger.evaluationAllowed()) {
                    conditionalCooperation.accepted(fish.id,fish.personalBestCost,bestCost,result);
                    fish.update(result.path,result.cost);
                    if(result.cost<bestCost){bestCost=result.cost;bestPath=ConditionalCooperation.copy(result.path);publishMigrationIncumbent();}
                }
            }
        } finally {searchWorkLedger.endPhase();cooperationNanos+=System.nanoTime()-start;}
    }

    private AlarmWrapOperator buildAlarmWrap(FsaContext ctx,TimeEvaluator evaluator) {
        return new AlarmWrapOperator(alarmWrapMode,new AlarmWrapOperator.Host() {
            public boolean allowed(){return searchWorkLedger.evaluationAllowed();}
            public boolean completedAllowed(){return searchWorkLedger.timingAllowed();}
            public TimeEvaluator.Trace evaluate(List<GridPoint> path) {
                searchWorkLedger.recordRouteObjectiveEvaluation();
                if(!allowed() || path==null || path.size()<2 || !path.get(0).equals(seedPath.get(0))
                        || !path.get(path.size()-1).equals(seedPath.get(seedPath.size()-1))) {
                    TimeEvaluator.Trace t=new TimeEvaluator.Trace(path);t.status=allowed()?"ENDPOINT":"ABORTED";return t;
                }
                return evaluator.tracePath(path,activeRouteDistanceBudgetNm);
            }
            public boolean edgeLegal(GridPoint a,GridPoint b){return ctx.routeGraph.isValidEdge(a,b);}
            public double edgeArrival(GridPoint a,GridPoint b,double elapsed) {
                if(!allowed())return Double.POSITIVE_INFINITY;
                EdgeSampler.EdgeResult r=evaluator.sampleEdge(a.i,a.j,b.i,b.j,elapsed);
                return r.valid && completedAllowed()?r.endElapsedHours:Double.POSITIVE_INFINITY;
            }
            public List<GridPoint> neighbors(GridPoint p){return ctx.routeGraph.neighbors(p);}
            public double dx(){return ctx.dLonNm;} public double dy(){return ctx.dLatNm;}
            public double nominalSpeed(){return ctx.ship.shipSpeed;}
            public void count(String name){ctx.recordOperatorEvent(name);}
        },baseSeed);
    }

    private Navigation.Navigation3D.fsa.PassageRejoin buildPassage(FsaContext ctx,TimeEvaluator evaluator){
        return new Navigation.Navigation3D.fsa.PassageRejoin(passageMode,new Navigation.Navigation3D.fsa.PassageRejoin.Host(){
            public boolean allowed(){return searchWorkLedger.evaluationAllowed();}
            public boolean completedAllowed(){return searchWorkLedger.timingAllowed();}
            public TimeEvaluator.Trace evaluate(List<GridPoint> path){
                searchWorkLedger.recordRouteObjectiveEvaluation();
                if(!allowed() || path==null || path.size()<2 || !path.get(0).equals(seedPath.get(0))
                        || !path.get(path.size()-1).equals(seedPath.get(seedPath.size()-1))){
                    TimeEvaluator.Trace t=new TimeEvaluator.Trace(path);t.status=allowed()?"ENDPOINT":"ABORTED";return t;
                }
                return evaluator.tracePath(path,activeRouteDistanceBudgetNm);
            }
            public boolean edgeLegal(GridPoint a,GridPoint b){return ctx.routeGraph.isValidEdge(a,b);}
            public double edgeArrival(GridPoint a,GridPoint b,double elapsed){
                if(!allowed())return Double.POSITIVE_INFINITY;
                EdgeSampler.EdgeResult e=evaluator.sampleEdge(a.i,a.j,b.i,b.j,elapsed);
                return e.valid&&completedAllowed()?e.endElapsedHours:Double.POSITIVE_INFINITY;
            }
            public List<GridPoint> neighbors(GridPoint p){return ctx.routeGraph.neighbors(p);}
            public double dx(){return ctx.dLonNm;} public double dy(){return ctx.dLatNm;}
            public double nominalSpeed(){return ctx.ship.shipSpeed;}
            public double distance(GridPoint a,GridPoint b){return ctx.gridDistanceNm(a.i,a.j,b.i,b.j);}
            public double radius(){return visualRange*Math.min(ctx.dLatNm,ctx.dLonNm);}
            public long work(){return searchWorkLedger.snapshot().edgeSubsegmentAttempts;}
            public void count(String name){ctx.recordOperatorEvent(name);}
        },baseSeed^0x50415353414745L);
    }

    private Navigation.Navigation3D.fsa.MigrationFeedback buildMigration(FsaContext ctx,TimeEvaluator evaluator){
        return new Navigation.Navigation3D.fsa.MigrationFeedback(migrationMode,new Navigation.Navigation3D.fsa.MigrationFeedback.Host(){
            public boolean allowed(){return searchWorkLedger.evaluationAllowed();}
            public boolean completedAllowed(){return searchWorkLedger.timingAllowed();}
            public TimeEvaluator.Trace evaluate(List<GridPoint> path){
                searchWorkLedger.recordRouteObjectiveEvaluation();
                if(!allowed() || path==null || path.size()<2 || !path.get(0).equals(seedPath.get(0))
                        || !path.get(path.size()-1).equals(seedPath.get(seedPath.size()-1))){
                    TimeEvaluator.Trace t=new TimeEvaluator.Trace(path);t.status=allowed()?"ENDPOINT":"ABORTED";return t;
                }
                return evaluator.tracePath(path,activeRouteDistanceBudgetNm);
            }
            public boolean edgeLegal(GridPoint a,GridPoint b){return ctx.routeGraph.isValidEdge(a,b);}
            public double edgeArrival(GridPoint a,GridPoint b,double elapsed){
                if(!allowed())return Double.POSITIVE_INFINITY;
                EdgeSampler.EdgeResult e=evaluator.sampleEdge(a.i,a.j,b.i,b.j,elapsed);
                return e.valid&&completedAllowed()?e.endElapsedHours:Double.POSITIVE_INFINITY;
            }
            public List<GridPoint> neighbors(GridPoint p){return ctx.routeGraph.neighbors(p);}
            public double dx(){return ctx.dLonNm;} public double dy(){return ctx.dLatNm;}
            public double nominalSpeed(){return ctx.ship.shipSpeed;}
            public double distance(GridPoint a,GridPoint b){return ctx.gridDistanceNm(a.i,a.j,b.i,b.j);}
            public double radius(){return visualRange*Math.min(ctx.dLatNm,ctx.dLonNm);}
            public double distanceBudget(){return activeRouteDistanceBudgetNm;}
            public long work(){return searchWorkLedger.snapshot().edgeSubsegmentAttempts;}
            public void count(String name){ctx.recordOperatorEvent(name);}
        },baseSeed^0x4D494752415445L);
    }

    private long requestWorkRemaining(){
        long global=searchWorkLedger.snapshot().edgeSubsegmentAttempts;
        long ordinary=Math.max(0,global-requestMigrationWork);
        long funded=Math.max(0,ordinary/9-requestMigrationWork);
        long cap=searchWorkLedger.getMaxEdgeSubsegmentAttempts();
        return cap>0?Math.min(funded,Math.max(0,cap/10-requestMigrationWork)):funded;
    }
    private Navigation.Navigation3D.fsa.RequestMigration buildRequestMigration(FsaContext ctx,TimeEvaluator evaluator){
        return new Navigation.Navigation3D.fsa.RequestMigration(requestMigrationMode,new Navigation.Navigation3D.fsa.RequestMigration.Host(){
            public boolean allowed(){return searchWorkLedger.evaluationAllowed()&&requestWorkRemaining()>0;}
            public boolean completedAllowed(){return searchWorkLedger.timingAllowed();}
            public long remainingWork(){return requestWorkRemaining();}
            public long work(){return searchWorkLedger.snapshot().edgeSubsegmentAttempts;}
            public TimeEvaluator.Trace evaluate(List<GridPoint> path){
                long before=work();searchWorkLedger.recordRouteObjectiveEvaluation();
                try {
                    if(!allowed()||path==null||path.size()<2||!path.get(0).equals(seedPath.get(0))||!path.get(path.size()-1).equals(seedPath.get(seedPath.size()-1))){
                        TimeEvaluator.Trace t=new TimeEvaluator.Trace(path);t.status=allowed()?"ENDPOINT":"ABORTED";return t;
                    }
                    return evaluator.tracePath(path,activeRouteDistanceBudgetNm);
                }finally{requestMigrationWork+=work()-before;}
            }
            public double edgeArrival(GridPoint a,GridPoint b,double elapsed){
                if(!allowed())return Double.POSITIVE_INFINITY;long before=work();
                try {EdgeSampler.EdgeResult e=evaluator.sampleEdge(a.i,a.j,b.i,b.j,elapsed);return e.valid&&completedAllowed()?e.endElapsedHours:Double.POSITIVE_INFINITY;}
                finally{requestMigrationWork+=work()-before;}
            }
            public boolean edgeLegal(GridPoint a,GridPoint b){return ctx.routeGraph.isValidEdge(a,b);}
            public List<GridPoint> neighbors(GridPoint p){return ctx.routeGraph.neighbors(p);}
            public double distance(GridPoint a,GridPoint b){return ctx.gridDistanceNm(a.i,a.j,b.i,b.j);}
            public double dx(){return ctx.dLonNm;}public double dy(){return ctx.dLatNm;}
            public double radius(){return visualRange*Math.min(ctx.dLatNm,ctx.dLonNm);}
            public double nominalSpeed(){return ctx.ship.shipSpeed;}
            public double distanceBudget(){return activeRouteDistanceBudgetNm;}
            public int weatherDay(double elapsed){return ctx.clampDay(elapsed);}
            public double maxEdgeSpanCells(){return ctx.routeGraph.getMaxEdgeSpanCells();}
            public void count(String name){ctx.recordOperatorEvent(name);}
        });
    }
    private FishResult runRequestMigrationSlot(Fish fish){
        if(requestMigration==null||!searchWorkLedger.evaluationAllowed())return null;
        long remaining=Math.min(improvementWallMillis*1_000_000L/10-requestMigrationNanos,
                improvementWallMillis*1_000_000L/4-cooperationNanos);
        if(remaining<=0){searchWorkLedger.recordOperatorEvent("rm_phase_cap");return null;}
        long start=System.nanoTime();searchWorkLedger.beginPhase(remaining);
        try {
            MigrationIncumbent parent=migrationIncumbentSnapshot();
            Navigation.Navigation3D.fsa.RequestMigration.Result r=parent==null?null:requestMigration.poll(parent.path,parent.cost);
            if(r!=null&&searchWorkLedger.timingAllowed()){
                requestMigrationPending.put(r.path,r.witness);return new FishResult(fish,r.path,r.cost);
            }
            return null;
        }finally{searchWorkLedger.endPhase();long elapsed=System.nanoTime()-start;requestMigrationNanos+=elapsed;cooperationNanos+=elapsed;}
    }
    private FishResult runAlarmSlot(Fish fish) {
        if(requestMigration!=null){FishResult r=runRequestMigrationSlot(fish);if(r!=null)return r;}
        if(alarmWrap==null || !searchWorkLedger.evaluationAllowed())return null;
        long remaining=improvementWallMillis*1_000_000L/4-cooperationNanos;
        if(remaining<=0){searchWorkLedger.recordOperatorEvent("aw_phase_cap");return null;}
        long start=System.nanoTime();searchWorkLedger.beginPhase(remaining);
        try {
            List<GridPoint> parent=fish.personalBestPath==null?seedPath:fish.personalBestPath;
            double cost=fish.personalBestPath==null?seedCost:fish.personalBestCost;
            if(migration!=null){
                MigrationIncumbent incumbent=migrationIncumbentSnapshot();
                Navigation.Navigation3D.fsa.MigrationFeedback.Result mr=incumbent==null?null:migration.poll(incumbent.path,incumbent.cost);
                if(mr!=null&&searchWorkLedger.timingAllowed()){
                    migrationPending.put(mr.path,mr.witness);return new FishResult(fish,mr.path,mr.cost);
                }
            }
            if(passage!=null){
                Navigation.Navigation3D.fsa.PassageRejoin.Result pr=passage.poll(fish.id,parent,cost);
                if(pr!=null && searchWorkLedger.timingAllowed() && pr.cost<fish.personalBestCost-EPSILON){
                    passagePending.put(pr.path,pr.witness);
                    return new FishResult(fish,pr.path,pr.cost);
                }
            }
            AlarmWrapOperator.Result r=alarmWrap.poll(fish.id,parent,cost);
            if(r!=null && searchWorkLedger.timingAllowed() && r.cost<fish.personalBestCost-EPSILON)
                return new FishResult(fish,r.path,r.cost,false,r.witness);
            return null;
        } finally {searchWorkLedger.endPhase();cooperationNanos+=System.nanoTime()-start;}
    }
    private GridPoint startP() {
        return new GridPoint(config.grid.startY - 1, config.grid.startX - 1);
    }

    private GridPoint endP() {
        return new GridPoint(config.grid.endY - 1, config.grid.endX - 1);
    }

    private static long key(int i, int j) { return ((long) i << 32) | (j & 0xFFFFFFFFL); }

    private static double personalFrequency(Fish fish, int i, int j) {
        if (fish.personalBestPath == null || fish.personalBestPath.isEmpty()) return 0.0;
        int cnt = 0;
        for (GridPoint g : fish.personalBestPath)
            if (g.i == i && g.j == j) cnt++;
        return (double) cnt / Math.max(fish.personalBestPath.size(), 1);
    }

    // ==================== 结果构造 ====================
    private NavigationResult buildResult(FsaContext ctx, ObjectiveEvaluator evaluator) {
        NavigationResult r = new NavigationResult();
        r.projectName = "HybridFSA3D_" + objectiveMode;
        r.calculationType = "hybridFSA3D_" + objectiveMode;
        r.navigationDate = config.navigationDate;
        r.area = config.area;
        r.grid = config.grid;
        r.shipParameters = config.shipParameters;

        // 先做完整评估获取实际海里距离与实际小时数
        PathMetrics pm = evaluatePathFull(ctx, bestPath);

        r.totalDistance = round2(pm.totalDistNm);
        // totalTime 始终为 EdgeSampler 累积得到的真实小时数，与 Dijkstra 口径可直接对比；
        // bestCost 可能是目标代价（SAFETY/FUEL/COMPOSITE 语义非小时），不再直接写入 totalTime。
        r.totalTime = round2(pm.totalTimeHours);
        r.pathPointCount = bestPath.size();
        r.obstacleFile = (config.obstacleFileName == null || config.obstacleFileName.isBlank())
                ? "obstacle_map.json" : config.obstacleFileName;
        r.pathPoints = buildRoutePoints(bestPath);
        r.daySwitchPoints = buildDaySwitchPoints(daySwitchMarkers, r.pathPoints);

        // 安全 & 油耗（通过完整评估）
        r.totalFuelEnergyJ = round6(pm.totalFuelJ);
        r.totalSafetyScore = round6(pm.avgSafety);
        r.totalCumulativeRisk = round6(pm.cumulativeRisk);
        // 综合代价：仅 COMPOSITE 模式下 bestCost 语义 = 综合加权代价，写入以便汇总对比。
        // 其它模式（TIME/SAFETY/FUEL）的 bestCost 是目标本身量纲，不写 compositeCost。
        if (objectiveMode == ObjectiveMode.COMPOSITE) {
            r.totalCompositeCost = round6(bestCost);
        }

        System.out.printf("[HybridFSA] %s 目标代价 bestCost=%.4f | 实际时间=%.2fh, 距离=%.2fnm, 节点=%d%n",
                objectiveMode, bestCost, pm.totalTimeHours, pm.totalDistNm, bestPath.size());

        return r;
    }

    private static double pathDistance(List<GridPoint> path) {
        double d = 0;
        for (int k = 1; k < path.size(); k++) {
            GridPoint a = path.get(k - 1), b = path.get(k);
            d += Math.sqrt(Math.pow(a.i - b.i, 2) + Math.pow(a.j - b.j, 2));
        }
        return d;
    }

    /** 将内部的跨日标记（网格索引 0-based）转成 JSON 持久化口径（1-based 栅格坐标 + 经纬度 + 最近航点索引）。
     *  与 NavigationTime3D#buildDaySwitchPoints 保持完全一致，用于 FSA 路径图可视化读取。 */
    private List<NavigationResult.DaySwitchPoint> buildDaySwitchPoints(
            List<PathVisualizer.MarkerPoint> markers,
            List<RoutePoint> routePoints) {
        List<NavigationResult.DaySwitchPoint> points = new ArrayList<>();
        if (markers == null || markers.isEmpty()) {
            return points;
        }
        for (PathVisualizer.MarkerPoint marker : markers) {
            NavigationResult.DaySwitchPoint pt = new NavigationResult.DaySwitchPoint();
            pt.label = marker.label;
            pt.dayNumber = parseDayNumberFromLabel(marker.label);
            pt.gridX = roundToSixDecimals(marker.gridX + 1.0);
            pt.gridY = roundToSixDecimals(marker.gridY + 1.0);
            pt.lon = roundToSixDecimals(config.area.lonMin + marker.gridX * deltaLon);
            pt.lat = roundToSixDecimals(config.area.latMin + marker.gridY * deltaLat);
            pt.nearestPathIndex = findNearestPathIndex(marker, routePoints);
            points.add(pt);
        }
        return points;
    }

    private static Integer parseDayNumberFromLabel(String label) {
        if (label == null || label.isBlank() || !label.startsWith("D")) return null;
        try {
            return Integer.parseInt(label.substring(1));
        } catch (NumberFormatException e) {
            return null;
        }
    }

    private static double roundToSixDecimals(double v) {
        return Math.round(v * 1_000_000.0) / 1_000_000.0;
    }

    private static Integer findNearestPathIndex(PathVisualizer.MarkerPoint marker,
                                                List<RoutePoint> routePoints) {
        if (routePoints == null || routePoints.isEmpty()) return null;
        double targetX = marker.gridX + 1.0;
        double targetY = marker.gridY + 1.0;
        double bestDist = Double.MAX_VALUE;
        Integer bestIdx = null;
        for (RoutePoint rp : routePoints) {
            double dx = rp.x - targetX;
            double dy = rp.y - targetY;
            double d = dx * dx + dy * dy;
            if (d < bestDist) { bestDist = d; bestIdx = rp.index; }
        }
        return bestIdx;
    }

    private List<RoutePoint> buildRoutePoints(List<GridPoint> path) {
        List<RoutePoint> pts = new ArrayList<>();
        double lat0 = config.area.latMin, lon0 = config.area.lonMin;
        for (int k = 0; k < path.size(); k++) {
            GridPoint g = path.get(k);
            double lat = lat0 + g.i * deltaLat;
            double lon = lon0 + g.j * deltaLon;
            RoutePoint rp = new RoutePoint();
            rp.lon = round6(lon);
            rp.lat = round6(lat);
            rp.x = g.j + 1;
            rp.y = g.i + 1;
            rp.index = k + 1;
            pts.add(rp);
        }
        return pts;
    }

    private static class PathMetrics {
        double totalFuelJ = 0.0;
        double avgSafety = 0.0;
        double cumulativeRisk = 0.0;
        double totalDistNm = 0.0;
        double totalTimeHours = 0.0;
    }

    private PathMetrics evaluatePathFull(FsaContext ctx, List<GridPoint> path) {
        PathMetrics pm = new PathMetrics();
        // 清空跨日标记（每次完整评估都重建）
        daySwitchMarkers = new ArrayList<>();
        if (path == null || path.size() < 2) return pm;
        double elapsed = 0.0;
        double safetyWeighted = 0.0;
        double riskWeighted = 0.0;
        final double DAY_HOURS = 24.0;
        // 多日模式下才生成跨日点；staticSingleDayMode 或只有 1 天数据时跳过
        boolean multiDayForSwitch = !staticSingleDayMode && numDays > 1;
        for (int k = 0; k < path.size() - 1; k++) {
            GridPoint a = path.get(k), b = path.get(k + 1);
            EdgeSampler.EdgeResult e = EdgeSampler.sample(ctx, a.i, a.j, b.i, b.j, elapsed,
                    EdgeSampler.NeedComponents.all(), true);
            if (e == null || !e.valid) continue;
            pm.totalDistNm += e.distanceNm;
            pm.totalFuelJ += e.fuelEnergyJ;
            safetyWeighted += e.safetyWeighted;
            riskWeighted += e.riskWeighted;
            // 检测本边是否跨越整 24h 边界（绝对时间 = departureOffsetHours + elapsed）
            if (multiDayForSwitch) {
                double absStart = departureOffsetHours + elapsed;
                double absEnd = departureOffsetHours + e.endElapsedHours;
                int startDay = (int) Math.floor(absStart / DAY_HOURS);
                int endDay = (int) Math.floor(absEnd / DAY_HOURS);
                if (endDay > startDay) {
                    double edgeHours = e.endElapsedHours - elapsed;
                    if (edgeHours > EPSILON) {
                        for (int d = startDay + 1; d <= endDay; d++) {
                            double ratio = (d * DAY_HOURS - absStart) / edgeHours;
                            if (ratio < 0.0) ratio = 0.0;
                            if (ratio > 1.0) ratio = 1.0;
                            double gx = a.j + (b.j - a.j) * ratio;
                            double gy = a.i + (b.i - a.i) * ratio;
                            daySwitchMarkers.add(new PathVisualizer.MarkerPoint(gx, gy, "D" + (d + 1)));
                        }
                    }
                }
            }
            elapsed = e.endElapsedHours;
        }
        pm.totalTimeHours = elapsed;
        if (pm.totalDistNm > 0) {
            pm.avgSafety = Math.min(1.0, safetyWeighted / pm.totalDistNm);
            pm.cumulativeRisk = riskWeighted;
        }
        return pm;
    }

    // ==================== 内部类 ====================
    private static class FishResult {
        final Fish fish;
        final List<GridPoint> path;
        final double cost;
        final boolean actionTransferred;
        final Map<String,Object> alarmWitness;
        String foragingOrigin;
        FishResult(Fish fish, List<GridPoint> path, double cost) {
            this(fish,path,cost,false);
        }
        FishResult(Fish fish, List<GridPoint> path, double cost, boolean actionTransferred) {
            this(fish,path,cost,actionTransferred,null);
        }
        FishResult(Fish fish, List<GridPoint> path, double cost, boolean actionTransferred,Map<String,Object> alarmWitness) {
            this.fish = fish; this.path = path; this.cost = cost; this.actionTransferred=actionTransferred;
            this.alarmWitness=alarmWitness;
        }
    }

    // ==================== 格式化 ====================
    private static double round2(double v) { return Math.round(v * 100.0) / 100.0; }
    private static double round6(double v) { return Math.round(v * 1_000_000.0) / 1_000_000.0; }
}
