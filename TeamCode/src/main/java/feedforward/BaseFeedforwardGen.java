package feedforward;

import java.util.ArrayList;
import java.util.List;

import geometry.PathPoint;

public abstract class BaseFeedforwardGen {

    protected PathPoint[] points = null;
    protected final double v_max;
    protected final double a_max;

    private DebugReport lastReport;

    public BaseFeedforwardGen(double v_max, double a_max) {
        this.v_max = v_max;
        this.a_max = a_max;
    }

    public void setPoints(PathPoint[] points) {
        this.points = points;
    }

    public DebugReport getLastDebugReport() {
        return lastReport;
    }

    // region abstract methods

    /**
     * Calculates the absolute maximum steady-state velocity for a given point,
     * bounded by the motor's 1.0 voltage limit and any specific kinematic limits (e.g. w_max).
     */
    protected abstract double calculateSteadyStateVelocity(PathPoint point);

    /**
     * Evaluates the dynamic power requirements at a specific point and fills the outResult.
     * * @param prev     The previous PathPoint
     * @param current  The current PathPoint
     * @param v_prev   The target velocity at the previous point
     * @param v        The target velocity at the current point
     * @param a_t      The calculated translational acceleration to reach v from v_prev
     * @param outResult The object the subclass must populate with power and utilization data
     */
    protected abstract void evaluatePoint(
            PathPoint prev, PathPoint current,
            double v_prev, double v, double a_t,
            EvaluationResult outResult
    );

    // region master Loop

    public FeedforwardParams[] generate() {
        if (points == null || points.length == 0) {
            throw new IllegalStateException("Points must be set before generating.");
        }

        lastReport = new DebugReport();

        // Phase 1: Base Pass
        FeedforwardParams[] lut = generateBasePass();

        // Phase 2 & 3: Initial Sweeps
        runBackwardPass(lut);
        runForwardPass(lut);

        int maxIterations = 25;
        EvaluationResult currentEval = new EvaluationResult(); // Reused to save memory

        // Phase 4: Iterative Solver
        for (int iter = 0; iter < maxIterations; iter++) {
            lastReport.iterationsRun = iter + 1;

            double worstUtilization = 0.0;
            int worstIndex = -1;

            // To store the state of the worst offender for the log
            EvaluationResult worstEvalState = new EvaluationResult();

            for (int i = 1; i < points.length; i++) {
                double ds = Math.abs(points[i].getDistanceToEnd_in() - points[i - 1].getDistanceToEnd_in());
                if (ds < 1e-6) continue;

                double v = lut[i].getTangentialVel();
                double v_prev = lut[i - 1].getTangentialVel();

                // Base kinematic acceleration
                double a_t = ((v * v) - (v_prev * v_prev)) / (2.0 * ds);

                // Ask the subclass to run the drivetrain-specific physics
                evaluatePoint(points[i - 1], points[i], v_prev, v, a_t, currentEval);

                if (currentEval.maxUtilization > worstUtilization) {
                    worstUtilization = currentEval.maxUtilization;
                    worstIndex = i;
                    worstEvalState.copyFrom(currentEval);
                }
            }

            lastReport.finalMaxUtilization = worstUtilization;

            if (worstUtilization <= 1.0) {
                lastReport.converged = true;
                break;
            }

            // Log the pin
            IterationLog log = new IterationLog();
            log.iteration = iter;
            log.pinnedIndex = worstIndex;
            log.maxUtilization = worstUtilization;
            log.totalPower = worstEvalState.totalPower;
            log.pForward = worstEvalState.pForward;
            log.pLateral = worstEvalState.pLateral;
            log.pHeading = worstEvalState.pHeading;
            log.previousVelocity = lut[worstIndex].getTangentialVel();

            // Pin and Propagate
            lut[worstIndex].setTangentialVel(log.previousVelocity * 0.9);
            log.newVelocity = lut[worstIndex].getTangentialVel();

            lastReport.logs.add(log);

            runBackwardPass(lut);
            runForwardPass(lut);
        }

        return lut;
    }

    // region fwd/bkwd passes

    private FeedforwardParams[] generateBasePass() {
        FeedforwardParams[] lut = new FeedforwardParams[points.length];
        for (int i = 0; i < points.length; i++) {
            lut[i] = new FeedforwardParams();
            double subclassSteadyState = calculateSteadyStateVelocity(points[i]);
            lut[i].setTangentialVel(Math.min(subclassSteadyState, v_max));
        }
        return lut;
    }

    private void runBackwardPass(FeedforwardParams[] lut) {
        lut[points.length - 1].setTangentialVel(0.0);
        for (int i = points.length - 2; i >= 0; i--) {
            double ds = Math.abs(points[i + 1].getDistanceToEnd_in() - points[i].getDistanceToEnd_in());
            double nextVel = lut[i + 1].getTangentialVel();
            double maxReachableVel = Math.sqrt((nextVel * nextVel) + (2.0 * a_max * ds));
            lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), maxReachableVel));
        }
    }

    private void runForwardPass(FeedforwardParams[] lut) {
        lut[0].setTangentialVel(0.0);
        for (int i = 1; i < points.length; i++) {
            double ds = Math.abs(points[i].getDistanceToEnd_in() - points[i - 1].getDistanceToEnd_in());
            double prevVel = lut[i - 1].getTangentialVel();
            double maxReachableVel = Math.sqrt((prevVel * prevVel) + (2.0 * a_max * ds));
            lut[i].setTangentialVel(Math.min(lut[i].getTangentialVel(), maxReachableVel));
        }
    }

    // region logging and csv export

    public static class EvaluationResult {
        public double totalPower = 0.0;
        public double maxUtilization = 0.0;
        public double pForward = 0.0;
        public double pLateral = 0.0;
        public double pHeading = 0.0;

        public void copyFrom(EvaluationResult other) {
            this.totalPower = other.totalPower;
            this.maxUtilization = other.maxUtilization;
            this.pForward = other.pForward;
            this.pLateral = other.pLateral;
            this.pHeading = other.pHeading;
        }
    }

    public static class DebugReport {
        public int iterationsRun = 0;
        public boolean converged = false;
        public double finalMaxUtilization = 0.0;
        public List<IterationLog> logs = new ArrayList<>();

        public String getSummary() {
            return String.format("Converged: %b | Iterations: %d | Final Max Util: %.3f",
                    converged, iterationsRun, finalMaxUtilization);
        }
    }

    public static class IterationLog {
        public int iteration;
        public int pinnedIndex;
        public double previousVelocity, newVelocity;
        public double maxUtilization, totalPower;
        public double pForward, pLateral, pHeading;

        @Override
        public String toString() {
            return String.format("Iter %d | Idx %d | Vel %.1f->%.1f | Util %.2f (Pwr:%.2f -> Fwd:%.2f, Lat:%.2f, Hdg:%.2f)",
                    iteration, pinnedIndex, previousVelocity, newVelocity,
                    maxUtilization, totalPower, pForward, pLateral, pHeading);
        }
    }

    public String exportToCSV(FeedforwardParams[] profile) {
        StringBuilder csv = new StringBuilder();
        csv.append("Index,DistanceToEnd_in,Curvature,CurvatureDerivative,TargetVelocity_ins\n");
        for (int i = 0; i < profile.length; i++) {
            csv.append(i).append(",")
                    .append(String.format("%.4f", points[i].getDistanceToEnd_in())).append(",")
                    .append(String.format("%.6f", points[i].getSignedCurvature())).append(",")
                    .append(String.format("%.6f", points[i].getCurvatureDerivative())).append(",")
                    .append(String.format("%.4f", profile[i].getTangentialVel())).append("\n");
        }
        return csv.toString();
    }
}