package Navigation.Navigation3D.fsa;

import Navigation.Navigation3D.FuelCostCalculator;
import Navigation.Navigation3D.SafetyCostCalculator;
import Navigation.object.ShipParameters;
import Navigation.utils.ShipUtils;

/**
 * 边采样器：把鱼的一次视野跳跃 (from → cand) 切成若干个 0.5 格子段，
 * 子段内按所在格查当日环境、按连续 bearing 现算 safety/fuel。
 *
 * 关键硬约束（用户需求）：
 *   - 鱼群视野不影响安全性计算：不论视野多大，dsCells 固定 = 0.5 格；
 *   - 连续 bearing 直接调用 SafetyCostCalculator / FuelCostCalculator / ShipUtils，
 *     **不查 safetyCache**，避免视野 > 1 时缓存查不到。
 */
public final class EdgeSampler {

    private EdgeSampler() {
    }

    public static final class EdgeResult {
        public final double distanceNm;
        public final double timeHours;
        public final double fuelEnergyJ;
        public final double safetyWeighted;   // Σ safetyIndex_k · lenSub_k
        public final double riskWeighted;     // Σ (1 - safetyIndex_k) · lenSub_k
        public final double[] riskWeightedVariants;
        public final double endElapsedHours;
        public final boolean valid;

        EdgeResult(double distanceNm, double timeHours, double fuelEnergyJ,
                   double safetyWeighted, double riskWeighted,
                   double[] riskWeightedVariants, double endElapsedHours, boolean valid) {
            this.distanceNm = distanceNm;
            this.timeHours = timeHours;
            this.fuelEnergyJ = fuelEnergyJ;
            this.safetyWeighted = safetyWeighted;
            this.riskWeighted = riskWeighted;
            this.riskWeightedVariants = riskWeightedVariants;
            this.endElapsedHours = endElapsedHours;
            this.valid = valid;
        }

        public static EdgeResult invalid() {
            return new EdgeResult(0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
                    0, Double.POSITIVE_INFINITY, null, Double.POSITIVE_INFINITY, false);
        }
    }

    @FunctionalInterface
    public interface RiskScorer {
        /** Return absolute risk scores in [0,1], in a fixed variant order, for this subsegment. */
        double[] score(double windU, double windV,
                       double currentU, double currentV,
                       double waveHeight, double waveDir,
                       double bearing, double wavePeriod,
                       ShipParameters shipParameters);
    }

    public static final class NeedComponents {
        public final boolean needTime;
        public final boolean needFuel;
        public final boolean needSafety;

        public NeedComponents(boolean needTime, boolean needFuel, boolean needSafety) {
            this.needTime = needTime;
            this.needFuel = needFuel;
            this.needSafety = needSafety;
        }

        public static NeedComponents all() {
            return new NeedComponents(true, true, true);
        }

        public static NeedComponents timeOnly() {
            return new NeedComponents(true, false, false);
        }
    }

    /**
     * 对一条边 from → cand 做子段细化。返回累计时间、油耗、safety 加权。
     *
     * @param ctx                 共享上下文
     * @param fromI               起点 i
     * @param fromJ               起点 j
     * @param candI               终点 i
     * @param candJ               终点 j
     * @param startElapsedHours   到达起点时的累计小时数（从出发算起）
     * @param need                按目标按需计算，节省时间
     * @param useShipClassCorr    船型分类开关（与 NavigationObjective3D.useCompositeShipClassCorrection 对齐）
     */
    public static EdgeResult sample(FsaContext ctx,
                                    int fromI, int fromJ, int candI, int candJ,
                                    double startElapsedHours,
                                    NeedComponents need,
                                    boolean useShipClassCorr) {
        return sample(ctx, fromI, fromJ, candI, candJ, startElapsedHours,
                need, useShipClassCorr, null);
    }

    /**
     * Sample an edge while optionally computing several alternative absolute-risk scores
     * from the same per-subsegment ship-response simulation. A scorer is used only by
     * explicit offline analysis callers; optimizer call sites continue to use the original
     * overload and its canonical score.
     */
    public static EdgeResult sample(FsaContext ctx,
                                    int fromI, int fromJ, int candI, int candJ,
                                    double startElapsedHours,
                                    NeedComponents need,
                                    boolean useShipClassCorr,
                                    RiskScorer riskScorer) {
        if (ResearchResponsePort.installed()) {
            if (riskScorer != null) throw new IllegalArgumentException("Research response policy cannot mix legacy risk variants");
            return ResearchResponsePort.sample(ctx,fromI,fromJ,candI,candJ,startElapsedHours,need,useShipClassCorr);
        }
        if (riskScorer != null && !need.needSafety) {
            throw new IllegalArgumentException("risk variants require needSafety=true");
        }
        if (ctx.searchWorkLedger != null) ctx.searchWorkLedger.recordEdgeSampleInvocation();
        if (!ctx.inBounds(candI, candJ) || ctx.blocked(candI, candJ)) {
            return EdgeResult.invalid();
        }
        if (!ctx.inBounds(fromI, fromJ) || ctx.blocked(fromI, fromJ)) {
            return EdgeResult.invalid();
        }

        double totalDistanceNm = ctx.gridDistanceNm(fromI, fromJ, candI, candJ);
        if (!Double.isFinite(totalDistanceNm) || totalDistanceNm <= FsaContext.EPSILON) {
            return EdgeResult.invalid();
        }

        double bearing = ctx.bearingOf(fromI, fromJ, candI, candJ);

        // 子段数：视野越大采样越密，但物理步长固定 0.5 格。
        int cellsSpan = Math.max(Math.abs(candI - fromI), Math.abs(candJ - fromJ));
        int numSubs = Math.max(1, (int) Math.ceil(cellsSpan / FsaContext.SAFETY_SAMPLE_STEP_CELLS));

        double elapsed = startElapsedHours;
        double totalTimeHours = 0.0;
        double totalFuelJ = 0.0;
        double totalSafetyWeighted = 0.0;
        double totalRiskWeighted = 0.0;
        double[] totalRiskVariants = null;

        ShipParameters ship = ctx.ship;

        for (int k = 0; k < numSubs; k++) {
            if (ctx.searchWorkLedger != null
                    && !ctx.searchWorkLedger.tryRecordEdgeSubsegmentAttempt()) {
                return EdgeResult.invalid();
            }
            // Never silently reuse the final daily frame beyond the supplied observation window.
            if (!ctx.hasWeatherAt(elapsed)) {
                return EdgeResult.invalid();
            }
            double tA = (double) k / numSubs;
            double tB = (double) (k + 1) / numSubs;
            double iMid = fromI + (candI - fromI) * (tA + tB) * 0.5;
            double jMid = fromJ + (candJ - fromJ) * (tA + tB) * 0.5;
            int sampleI = clampInt((int) Math.round(iMid), 0, ctx.ny - 1);
            int sampleJ = clampInt((int) Math.round(jMid), 0, ctx.nx - 1);

            // 2026-09-24: 中途落在障碍格上的长跳必须判为非法。原先只检查两端点，
            // 于是"两端在海上、中间穿陆"的跳跃被判合法；而陆上风险值偏低，
            // SAFETY 目标反而会奖励穿陆（实测 Exp-3 出现一次横穿印度次大陆的跳跃）。
            if (ctx.blocked(sampleI, sampleJ)) {
                return EdgeResult.invalid();
            }

            double subDistanceNm = totalDistanceNm / numSubs;
            int day = ctx.clampDay(elapsed);
            EnvironmentFrame frame = ctx.frames.get(day);

            // Missing required fields are not calm weather. Reject the edge so all
            // route-level objectives share fail-closed environmental semantics.
            if (!Double.isFinite(frame.windUData[sampleI][sampleJ])
                    || !Double.isFinite(frame.windVData[sampleI][sampleJ])
                    || !Double.isFinite(frame.currentUData[sampleI][sampleJ])
                    || !Double.isFinite(frame.currentVData[sampleI][sampleJ])
                    || (need.needSafety && (!Double.isFinite(frame.waveHeightData[sampleI][sampleJ])
                    || !Double.isFinite(frame.waveDirData[sampleI][sampleJ])
                    || !Double.isFinite(frame.wavePeriodData[sampleI][sampleJ])))) {
                return EdgeResult.invalid();
            }

            double windU = EnvironmentFrame.safeValue(frame.windUData[sampleI][sampleJ]);
            double windV = EnvironmentFrame.safeValue(frame.windVData[sampleI][sampleJ]);
            double curU = EnvironmentFrame.safeValue(frame.currentUData[sampleI][sampleJ]);
            double curV = EnvironmentFrame.safeValue(frame.currentVData[sampleI][sampleJ]);
            double windSpeed = Math.sqrt(windU * windU + windV * windV);
            double currentSpeed = Math.sqrt(curU * curU + curV * curV);
            double windDir = angleDeg(windU, windV);
            double currentDir = angleDeg(curU, curV);

            // === 时间 ===
            double subHours;
            double speedMs = ShipUtils.calculateActualSpeedMs(
                    bearing, windSpeed, windDir,
                    currentSpeed, currentDir,
                    ship.shipSpeed, ship.shipLength, ship.shipWidth, ship.shipDraft, ship.shipDisplacement,
                    useShipClassCorr
            );
            if (!Double.isFinite(speedMs) || speedMs <= 0) {
                subHours = subDistanceNm / Math.max(ship.shipSpeed, 0.1);
            } else {
                double speedKn = speedMs / FsaContext.KNOT_TO_MS;
                subHours = subDistanceNm / Math.max(speedKn, 0.01);
            }
            if (!Double.isFinite(subHours) || subHours <= 0) {
                return EdgeResult.invalid();
            }
            if (!ctx.weatherCoversThrough(elapsed + subHours)) {
                return EdgeResult.invalid();
            }

            totalTimeHours += subHours;
            elapsed += subHours;

            // 跨 day 直接继续采样（下一个子段会按新 elapsed 查新 day）

            // === 油耗 ===
            if (need.needFuel) {
                double energy = FuelCostCalculator.calculateFuelEnergyJ(
                        ship, windSpeed, windDir,
                        currentSpeed, currentDir, curU, curV,
                        bearing, subHours, useShipClassCorr
                );
                if (Double.isFinite(energy) && energy >= 0) {
                    totalFuelJ += energy;
                } else {
                    totalFuelJ += subDistanceNm * 1_000_000.0;
                }
            }

            // === 安全 ===
            if (need.needSafety) {
                double waveH = Math.max(0.0, EnvironmentFrame.safeValue(frame.waveHeightData[sampleI][sampleJ]));
                double waveDir = EnvironmentFrame.safeValue(frame.waveDirData[sampleI][sampleJ]);
                double wavePd = Math.max(0.1, EnvironmentFrame.safeValue(frame.wavePeriodData[sampleI][sampleJ]));
                if (riskScorer == null) {
                    double safetyIndex = SafetyCostCalculator.calculateSafetyIndex(
                            windU, windV, curU, curV,
                            waveH, waveDir, bearing, wavePd, ship
                    );
                    if (!Double.isFinite(safetyIndex)) safetyIndex = 0.0;
                    safetyIndex = clampDouble(safetyIndex, 0.0, 1.0);
                    totalSafetyWeighted += safetyIndex * subDistanceNm;
                    totalRiskWeighted += (1.0 - safetyIndex) * subDistanceNm;
                } else {
                    double[] subsegmentRisks = riskScorer.score(
                            windU, windV, curU, curV, waveH, waveDir, bearing, wavePd, ship
                    );
                    if (subsegmentRisks == null || subsegmentRisks.length == 0) {
                        throw new IllegalStateException("risk scorer returned no variants");
                    }
                    if (totalRiskVariants == null) totalRiskVariants = new double[subsegmentRisks.length];
                    if (subsegmentRisks.length != totalRiskVariants.length) {
                        throw new IllegalStateException("risk scorer changed variant count within an edge");
                    }
                    for (int variant = 0; variant < subsegmentRisks.length; variant++) {
                        double risk = Double.isFinite(subsegmentRisks[variant])
                                ? clampDouble(subsegmentRisks[variant], 0.0, 1.0) : 1.0;
                        totalRiskVariants[variant] += risk * subDistanceNm;
                    }
                    double baseRisk = Double.isFinite(subsegmentRisks[0])
                            ? clampDouble(subsegmentRisks[0], 0.0, 1.0) : 1.0;
                    totalSafetyWeighted += (1.0 - baseRisk) * subDistanceNm;
                }
            }
        }

        return new EdgeResult(totalDistanceNm, totalTimeHours, totalFuelJ,
                totalSafetyWeighted, riskScorer == null ? totalRiskWeighted : totalRiskVariants[0],
                totalRiskVariants, elapsed, true);
    }

    private static int clampInt(int v, int lo, int hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    private static double clampDouble(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    private static double angleDeg(double u, double v) {
        if (Math.abs(u) < FsaContext.EPSILON && Math.abs(v) < FsaContext.EPSILON) return 0.0;
        double d = Math.toDegrees(Math.atan2(v, u));
        return d < 0 ? d + 360.0 : d;
    }
}
