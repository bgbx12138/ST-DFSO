package Navigation.Navigation3D;

import Navigation.Navigation3D.Normalization.SafetyNormalizer;
import Navigation.object.ShipParameters;

public final class SafetyCostCalculator {

    private static final double KNOT_TO_MS = 0.5144;
    private static final double G = 9.80665;
    private static final double RHO_WATER = 1025.0;
    private static final double RHO_AIR = 1.225;
    private static final double MAX_SPEED = 30.0;
    private static final double MAX_ROLL_ANGLE_RAD = Math.toRadians(45.0);
    private static final double MAX_ANGULAR_VELOCITY = Math.toRadians(60.0);

    private static final SafetyNormalizer SAFETY_NORMALIZER = new SafetyNormalizer();

    private SafetyCostCalculator() {
    }

    public static double calculateSafetyIndex(double windU,
                                              double windV,
                                              double currentU,
                                              double currentV,
                                              double waveHeight,
                                              double waveDir,
                                              double bearing,
                                              double wavePeriod,
                                              ShipParameters shipParams) {
        Ship ship = convertShip(shipParams);
        ShipDynamicsResult dynamics = simulateFromEnvironment(
                ship, windU, windV, currentU, currentV, waveHeight,
                waveDir, bearing, wavePeriod, false
        );
        double risk = SAFETY_NORMALIZER.normalizeTap4dRisk(
                dynamics.maxRollAngle,
                Math.abs(dynamics.avgYawRate),
                Math.abs(dynamics.avgSway),
                Math.abs(dynamics.avgSurge),
                shipParams
        );
        return 1.0 - risk;
    }

    /**
     * Calculate the response statistics used by the implemented bounded risk proxy.
     * The additional magnitude and RMS summaries support post-hoc sensitivity analysis;
     * the canonical score remains based on maximum absolute roll and absolute signed
     * means for yaw, sway, and surge.
     */
    public static RiskResponseMetrics calculateRiskResponseMetrics(double windU,
                                                                    double windV,
                                                                    double currentU,
                                                                    double currentV,
                                                                    double waveHeight,
                                                                    double waveDir,
                                                                    double bearing,
                                                                    double wavePeriod,
                                                                    ShipParameters shipParams) {
        Ship ship = convertShip(shipParams);
        ShipDynamicsResult dynamics = simulateFromEnvironment(
                ship, windU, windV, currentU, currentV, waveHeight,
                waveDir, bearing, wavePeriod, true
        );

        double tap4dRisk = SAFETY_NORMALIZER.normalizeTap4dRisk(
                dynamics.maxRollAngle,
                Math.abs(dynamics.avgYawRate),
                Math.abs(dynamics.avgSway),
                Math.abs(dynamics.avgSurge),
                shipParams
            );
        return new RiskResponseMetrics(
                tap4dRisk,
                dynamics.maxRollAngle,
                Math.abs(dynamics.avgYawRate),
                dynamics.meanAbsYawRate,
                dynamics.rmsYawRate,
                Math.abs(dynamics.avgSway),
                dynamics.meanAbsSway,
                dynamics.rmsSway,
                Math.abs(dynamics.avgSurge),
                dynamics.meanAbsSurge,
                dynamics.rmsSurge,
                dynamics.meanAbsSurgeDeviation,
                dynamics.rmsSurgeDeviation,
                ship.shipSpeed * KNOT_TO_MS
        );
    }

    private static ShipDynamicsResult simulateFromEnvironment(Ship ship,
                                                              double windU,
                                                              double windV,
                                                              double currentU,
                                                              double currentV,
                                                              double waveHeight,
                                                              double waveDir,
                                                              double bearing,
                                                              double wavePeriod,
                                                              boolean collectSensitivityMetrics) {
        double windSpeed = Math.sqrt(windU * windU + windV * windV);
        double windDir = mod360(Math.toDegrees(Math.atan2(windV, windU)));
        double currentSpeed = Math.sqrt(currentU * currentU + currentV * currentV);
        double currentDir = mod360(Math.toDegrees(Math.atan2(currentV, currentU)));
        return simulateShipDynamics(
                ship, windSpeed, windDir, currentSpeed, currentDir,
                // VMDR is a clockwise-from-North FROM direction. Dynamics
                // uses a Cartesian TO angle, like wind/current components.
                mod360(270.0 - waveDir), wavePeriod, waveHeight,
                ship.shipSpeed * KNOT_TO_MS, Math.toRadians(90.0 - bearing), collectSensitivityMetrics
        );
    }

    public static final class RiskResponseMetrics {
        public final double implementedRisk;
        public final double maxAbsRollDeg;
        public final double absMeanYawRateDegPerSec;
        public final double meanAbsYawRateDegPerSec;
        public final double rmsYawRateDegPerSec;
        public final double absMeanSwayMps;
        public final double meanAbsSwayMps;
        public final double rmsSwayMps;
        public final double absMeanSurgeMps;
        public final double meanAbsSurgeMps;
        public final double rmsSurgeMps;
        public final double meanAbsSurgeDeviationMps;
        public final double rmsSurgeDeviationMps;
        public final double nominalSurgeMps;

        private RiskResponseMetrics(double implementedRisk,
                                    double maxAbsRollDeg,
                                    double absMeanYawRateDegPerSec,
                                    double meanAbsYawRateDegPerSec,
                                    double rmsYawRateDegPerSec,
                                    double absMeanSwayMps,
                                    double meanAbsSwayMps,
                                    double rmsSwayMps,
                                    double absMeanSurgeMps,
                                    double meanAbsSurgeMps,
                                    double rmsSurgeMps,
                                    double meanAbsSurgeDeviationMps,
                                    double rmsSurgeDeviationMps,
                                    double nominalSurgeMps) {
            this.implementedRisk = implementedRisk;
            this.maxAbsRollDeg = maxAbsRollDeg;
            this.absMeanYawRateDegPerSec = absMeanYawRateDegPerSec;
            this.meanAbsYawRateDegPerSec = meanAbsYawRateDegPerSec;
            this.rmsYawRateDegPerSec = rmsYawRateDegPerSec;
            this.absMeanSwayMps = absMeanSwayMps;
            this.meanAbsSwayMps = meanAbsSwayMps;
            this.rmsSwayMps = rmsSwayMps;
            this.absMeanSurgeMps = absMeanSurgeMps;
            this.meanAbsSurgeMps = meanAbsSurgeMps;
            this.rmsSurgeMps = rmsSurgeMps;
            this.meanAbsSurgeDeviationMps = meanAbsSurgeDeviationMps;
            this.rmsSurgeDeviationMps = rmsSurgeDeviationMps;
            this.nominalSurgeMps = nominalSurgeMps;
        }
    }

    private static Ship convertShip(ShipParameters shipParams) {
        Ship ship = new Ship();
        ship.draft = Math.max(shipParams.shipDraft, 0.1);
        ship.Lpp = Math.max(shipParams.shipLength, 0.1);
        ship.B = Math.max(shipParams.shipWidth, 0.1);
        ship.displacementTon = Math.max(shipParams.shipDisplacement, 1.0);

        ship.mass = ship.displacementTon * 1000.0;
        ship.Izz = 0.25 * ship.mass * ship.Lpp * ship.Lpp;
        ship.Ixx = 0.1 * ship.mass * ship.B * ship.B;
        ship.GM = 1.5;
        ship.zG = ship.draft / 2.0;
        ship.mX = 0.05 * ship.mass;
        ship.mY = 0.5 * ship.mass;
        ship.Jxx = 0.2 * ship.mass * ship.B * ship.B;
        ship.rollDamping = 0.1 * Math.sqrt(ship.mass * G * ship.GM * ship.Ixx);
        ship.shipSpeed = Math.max(shipParams.shipSpeed, 0.1);
        return ship;
    }

    private static ShipDynamicsResult simulateShipDynamics(Ship ship,
                                                           double windSpeed,
                                                           double windDir,
                                                           double currentSpeed,
                                                           double currentDir,
                                                           double waveDir,
                                                           double wavePeriod,
                                                           double waveHeight,
                                                           double shipSpeedMs,
                                                           double shipHeadingRad,
                                                           boolean collectSensitivityMetrics) {
        ShipDynamicsResult result = new ShipDynamicsResult();

        if (!isFinite(windSpeed) || !isFinite(windDir) || !isFinite(currentSpeed) || !isFinite(currentDir)
                || !isFinite(waveDir) || !isFinite(wavePeriod) || !isFinite(waveHeight)) {
            result.maxRollAngle = 1.0;
            return result;
        }

        Environment env = new Environment();
        env.U0 = shipSpeedMs;
        env.windSpeed = windSpeed;
        env.windDir = windDir;
        env.currentSpeed = currentSpeed;
        env.currentDir = currentDir;
        env.waveHeight = waveHeight;
        env.waveDir = waveDir;
        env.wavePeriod = Math.max(0.1, wavePeriod);
        env.waveFreq = 2.0 * Math.PI / env.wavePeriod;
        env.heading = Math.toDegrees(shipHeadingRad);

        ShipState state = new ShipState();
        state.u = env.U0;
        state.v = 0.0;
        state.r = 0.0;
        state.p = 0.0;
        state.phi = 0.0;
        state.psi = shipHeadingRad;

        double dt = 0.5;
        double tTotal = Math.max(600.0, 15.0 * env.wavePeriod);
        int steps = Math.max(10, (int) Math.round(tTotal / dt));

        int skipSteps = (int) Math.round(steps * 0.001);
        skipSteps = Math.max(1, Math.min(skipSteps, steps - 10));
        int stableSteps = steps - skipSteps;

        double[] rollAngles = new double[stableSteps];
        double[] yawRates = new double[stableSteps];
        double[] surgeSpeeds = new double[stableSteps];
        double[] swaySpeeds = new double[stableSteps];

        double omega0 = Math.sqrt((ship.mass * G * ship.GM) / ship.Ixx);
        double zeta = ship.rollDamping / (2.0 * Math.sqrt(ship.Ixx * ship.mass * G * ship.GM) + 1e-10);
        zeta = clamp(zeta, 0.01, 0.3);

        for (int step = 0; step < steps; step++) {
            double t = step * dt;

            EnvironmentForces envForces = calcEnvironmentForces(ship, env, state, t, omega0, zeta);
            HullForces hullForces = calcHullForces(ship, env, state);
            RudderForces rudderForces = calcRudderForces(0.0, ship, env, state);
            double xPropeller = calcPropellerForce(ship, env, state);

            double xTotal = hullForces.X + rudderForces.X + xPropeller + envForces.X;
            double yTotal = hullForces.Y + rudderForces.Y + envForces.Y;
            double nTotal = hullForces.N + rudderForces.N + envForces.N;
            double kTotal = hullForces.K + rudderForces.K + envForces.K;

            double uDot = (xTotal + (ship.mass + ship.mY) * state.r * state.v) / (ship.mass + ship.mX);
            double vDot = (yTotal - (ship.mass + ship.mX) * env.U0 * state.r) / (ship.mass + ship.mY);
            double rDot = nTotal / ship.Izz;

            double numerator = kTotal + ship.zG * ship.mY * (vDot + env.U0 * state.r);
            double denominator = ship.Ixx + ship.Jxx;
            double pDot = numerator / denominator;

            state.u = clamp(state.u + uDot * dt, 0.0, MAX_SPEED);
            state.v = clamp(state.v + vDot * dt, -MAX_SPEED / 2.0, MAX_SPEED / 2.0);
            state.r = clamp(state.r + rDot * dt, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
            state.p = clamp(state.p + pDot * dt, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);
            state.phi = clamp(state.phi + state.p * dt, -MAX_ROLL_ANGLE_RAD, MAX_ROLL_ANGLE_RAD);
            state.psi = mod360Rad(state.psi + state.r * dt);

            if (step >= skipSteps) {
                int idx = step - skipSteps;
                rollAngles[idx] = state.phi;
                yawRates[idx] = state.r;
                surgeSpeeds[idx] = state.u;
                swaySpeeds[idx] = state.v;
            }
        }

        double maxRoll = 0.0;
        for (double angle : rollAngles) {
            maxRoll = Math.max(maxRoll, Math.abs(angle));
        }
        result.maxRollAngle = Math.toDegrees(maxRoll);

        if (!isFinite(result.maxRollAngle) || result.maxRollAngle > 1000.0) {
            result.maxRollAngle = 1.0;
        }

        result.avgYawRate = Math.toDegrees(avg(yawRates));
        result.avgSurge = avg(surgeSpeeds);
        result.avgSway = avg(swaySpeeds);
        if (collectSensitivityMetrics) {
            result.meanAbsYawRate = Math.toDegrees(meanAbs(yawRates));
            result.rmsYawRate = Math.toDegrees(rms(yawRates));
            result.meanAbsSway = meanAbs(swaySpeeds);
            result.rmsSway = rms(swaySpeeds);
            result.meanAbsSurge = meanAbs(surgeSpeeds);
            result.rmsSurge = rms(surgeSpeeds);
            result.meanAbsSurgeDeviation = meanAbsDeviation(surgeSpeeds, env.U0);
            result.rmsSurgeDeviation = rmsDeviation(surgeSpeeds, env.U0);
        }
        return result;
    }

    private static EnvironmentForces calcEnvironmentForces(Ship ship,
                                                           Environment env,
                                                           ShipState state,
                                                           double t,
                                                           double omega0,
                                                           double zeta) {
        EnvironmentForces forces = new EnvironmentForces();

        double windDirRel = mod360(env.windDir - Math.toDegrees(state.psi));
        double currentDirRel = mod360(env.currentDir - Math.toDegrees(state.psi));
        double waveDirRel = mod360(env.waveDir - env.heading);

        double windDirRad = Math.toRadians(windDirRel);
        double cosWind = Math.cos(windDirRad);
        double sinWind = Math.sin(windDirRad);

        double currentDirRad = Math.toRadians(currentDirRel);
        double cosCurrent = Math.cos(currentDirRad);
        double sinCurrent = Math.sin(currentDirRad);

        double waveDirRad = Math.toRadians(waveDirRel);
        double cosWave = Math.cos(waveDirRad);
        double sinWave = Math.sin(waveDirRad);

        double aS = ship.Lpp * (ship.draft + ship.B / 2.0);
        double cAx = 1.2 * Math.abs(cosWind);
        double cAy = 1.5 * sinWind;

        double windFactor = 0.5 * RHO_AIR * env.windSpeed * env.windSpeed * aS;
        double xWind = windFactor * cAx;
        double yWind = windFactor * cAy;
        double nWind = 0.25 * RHO_AIR * env.windSpeed * env.windSpeed * aS * ship.Lpp * Math.sin(2.0 * windDirRad);

        double uC = env.currentSpeed * cosCurrent;
        double vC = env.currentSpeed * sinCurrent;
        double uRel = state.u - uC;
        double vRel = state.v - vC;
        double uRelNorm = Math.sqrt(uRel * uRel + vRel * vRel);

        double xCurrent;
        double yCurrent;
        double nCurrent;
        if (uRelNorm < 0.1) {
            xCurrent = -500.0 * uRel;
            yCurrent = -5000.0 * vRel;
            nCurrent = -10000.0 * vRel;
        } else {
            double hullFactor = 0.5 * RHO_WATER * ship.Lpp * ship.draft * uRelNorm;
            xCurrent = -0.1 * hullFactor * uRel;
            yCurrent = -0.5 * hullFactor * vRel;
            nCurrent = -0.5 * RHO_WATER * ship.Lpp * ship.Lpp * ship.draft * 0.05 * vRel * uRelNorm;
        }

        double omegaEn = env.waveFreq - (env.waveFreq * env.waveFreq / G) * env.U0 * cosWave;

        double xWave = 0.0;
        double yWave = 0.0;
        double nWave = 0.0;
        double kWave = 0.0;

        if (env.waveHeight > 0.1) {
            double waveFactor = 0.5 * RHO_WATER * G * env.waveHeight * env.waveHeight * ship.Lpp;
            xWave = 0.05 * waveFactor * cosWave;
            yWave = 0.15 * waveFactor * sinWave;
            nWave = 0.25 * RHO_WATER * G * env.waveHeight * env.waveHeight * ship.Lpp * ship.Lpp * 0.03 * Math.sin(2.0 * waveDirRad);

            double kRestoreMag = ship.mass * G * ship.GM;
            double kCap = 0.6 * kRestoreMag;
            double kWaveMag = RHO_WATER * G * env.waveHeight * ship.B * ship.B * 0.3;
            if (Math.abs(kWaveMag) > kCap) {
                kWaveMag = Math.signum(kWaveMag) * kCap;
            }

            double w = Math.abs(omegaEn) + 1e-10;
            double wo = Math.max(omega0, 1e-10);
            double ratio = w / wo;
            double raoRoll = 1.0 / Math.sqrt(Math.pow(1.0 - ratio * ratio, 2) + Math.pow(2.0 * zeta * ratio, 2));
            raoRoll = Math.min(raoRoll, 5.0);

            kWave = kWaveMag * raoRoll * Math.sin(omegaEn * t);
        }

        forces.X = xWind + xCurrent + xWave;
        forces.Y = yWind + yCurrent + yWave;
        forces.N = nWind + nCurrent + nWave;
        forces.K = kWave;
        return forces;
    }

    private static HullForces calcHullForces(Ship ship, Environment env, ShipState state) {
        HullForces forces = new HullForces();
        double uNorm = Math.sqrt(state.u * state.u + state.v * state.v);
        if (uNorm < 0.1) {
            uNorm = 0.1;
        }

        double xUu = -0.1;
        double yV = -1.2;
        double yR = 0.5;
        double nV = -0.5;
        double nR = -0.2;

        forces.X = 0.5 * RHO_WATER * ship.Lpp * ship.draft * uNorm * uNorm
                * (xUu + 0.1 * Math.pow(state.u / uNorm, 3.0));

        double vNormalized = state.v / uNorm;
        double rNormalized = state.r * ship.Lpp / uNorm;
        forces.Y = 0.5 * RHO_WATER * ship.Lpp * ship.draft * uNorm * uNorm
                * (yV * vNormalized + yR * rNormalized);

        forces.N = 0.5 * RHO_WATER * ship.Lpp * ship.Lpp * ship.draft * uNorm * uNorm
                * (nV * vNormalized + nR * rNormalized);

        double kRestore = -ship.mass * G * ship.GM * Math.sin(state.phi);
        double kDamping = -ship.rollDamping * state.p;
        forces.K = kRestore + kDamping;
        return forces;
    }

    private static RudderForces calcRudderForces(double delta, Ship ship, Environment env, ShipState state) {
        RudderForces forces = new RudderForces();

        double aR = 0.01 * ship.Lpp * ship.draft;
        double tR = 0.25;
        double aH = 0.35;
        double xR = -0.45 * ship.Lpp;

        double uR = Math.max(0.1, 0.8 * env.U0);
        double deltaRad = Math.toRadians(delta);
        double sinDelta = Math.sin(deltaRad);
        double cosDelta = Math.cos(deltaRad);
        double sin2Delta = Math.sin(2.0 * deltaRad);

        double fN = 0.5 * RHO_WATER * aR * uR * uR * sin2Delta;

        forces.X = -(1.0 - tR) * fN * sinDelta;
        forces.Y = -(1.0 + aH) * fN * cosDelta;
        forces.N = -xR * fN * cosDelta;
        forces.K = 0.0;
        return forces;
    }

    private static double calcPropellerForce(Ship ship, Environment env, ShipState state) {
        double dP = 0.25 * ship.draft;
        double nP = 1.5;
        double kT = 0.15;
        double t = RHO_WATER * nP * nP * Math.pow(dP, 4.0) * kT;
        double tP = 0.15;
        return (1.0 - tP) * t;
    }

    private static double avg(double[] values) {
        if (values.length == 0) {
            return 0.0;
        }
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return sum / values.length;
    }

    private static double meanAbs(double[] values) {
        if (values.length == 0) return 0.0;
        double sum = 0.0;
        for (double value : values) sum += Math.abs(value);
        return sum / values.length;
    }

    private static double rms(double[] values) {
        if (values.length == 0) return 0.0;
        double sumSquares = 0.0;
        for (double value : values) sumSquares += value * value;
        return Math.sqrt(sumSquares / values.length);
    }

    private static double meanAbsDeviation(double[] values, double reference) {
        if (values.length == 0) return 0.0;
        double sum = 0.0;
        for (double value : values) sum += Math.abs(value - reference);
        return sum / values.length;
    }

    private static double rmsDeviation(double[] values, double reference) {
        if (values.length == 0) return 0.0;
        double sumSquares = 0.0;
        for (double value : values) {
            double deviation = value - reference;
            sumSquares += deviation * deviation;
        }
        return Math.sqrt(sumSquares / values.length);
    }

    private static boolean isFinite(double value) {
        return !Double.isNaN(value) && !Double.isInfinite(value);
    }

    private static double mod360(double angle) {
        double result = angle % 360.0;
        return result < 0 ? result + 360.0 : result;
    }

    private static double mod360Rad(double angle) {
        double cycle = 2.0 * Math.PI;
        angle %= cycle;
        return angle < 0 ? angle + cycle : angle;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static final class Ship {
        double draft;
        double Lpp;
        double B;
        double displacementTon;
        double mass;
        double Izz;
        double Ixx;
        double GM;
        double zG;
        double mX;
        double mY;
        double Jxx;
        double rollDamping;
        double shipSpeed;
    }

    private static final class ShipState {
        double u;
        double v;
        double r;
        double p;
        double phi;
        double psi;
    }

    private static final class Environment {
        double U0;
        double windSpeed;
        double windDir;
        double currentSpeed;
        double currentDir;
        double waveHeight;
        double waveDir;
        double wavePeriod;
        double waveFreq;
        double heading;
    }

    private static final class ShipDynamicsResult {
        double maxRollAngle;
        double avgYawRate;
        double avgSurge;
        double avgSway;
        double meanAbsYawRate;
        double rmsYawRate;
        double meanAbsSurge;
        double rmsSurge;
        double meanAbsSurgeDeviation;
        double rmsSurgeDeviation;
        double meanAbsSway;
        double rmsSway;
    }

    private static final class EnvironmentForces {
        double X;
        double Y;
        double N;
        double K;
    }

    private static final class HullForces {
        double X;
        double Y;
        double N;
        double K;
    }

    private static final class RudderForces {
        double X;
        double Y;
        double N;
        double K;
    }
}
