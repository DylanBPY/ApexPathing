package tuning;

/**
 * Base class for an Apex Pathing tuning phase.
 *
 * @author Sohum Arora - 22985 Paraducks
 * @author Dylan B. - 18597 RoboClovers - Delta
 */
public abstract class TuningPhase {
    private State state = State.INIT;
    protected final TunerContext context;
    private boolean manualMode;

    protected TuningPhase(TunerContext context) { this.context = context; }

    enum State { INIT, RUNNING, COMPLETE }

    /**
     * Updates the tuning phase. This method should be called repeatedly until it returns true.
     */
    public final boolean update(boolean aWasPressed, boolean bPressed) {
        switch (state) {
            case INIT:
                boolean initialized = initLoop(aWasPressed, bPressed);
                if (initialized) {
                    state = State.RUNNING;
                }
                break;
            case RUNNING:
                boolean complete;
                if (manualMode) {
                    complete = manualUpdate(aWasPressed, bPressed);
                } else {
                    complete = automaticUpdate();
                }

                if (complete) {
                    state = State.COMPLETE;
                }
                break;
            case COMPLETE:
                endLoop();
                if (bPressed) {
                    return true;
                }
                break;
        }

        return false;
    }

    /**
     * @return true when initialization is complete and the tuning phase should begin running.
     */
    private boolean initLoop(boolean aWasPressed, boolean bPressed) {
        context.getTelemetry().addLine(getPhaseName() + " phase initialized");

        if (manualTuneIsPossible() && autoTuneIsPossible()) {
            if (aWasPressed) {
                manualMode = !manualMode;
            }
            context.getTelemetry().addData("Selected Mode", manualMode ? "MANUAL" : "AUTOMATIC");
            context.getTelemetry().addLine("A - Toggle automatic/manual");
        }

        context.getTelemetry().addLine("B - Start tuning");
        context.getTelemetry().update();

        if (bPressed) {
            init();
            return true;
        }

        return false;
    }

    private void endLoop() {
        context.getTelemetry().addLine(getPhaseName() + " phase complete with results:");
        reportResults();
        context.getTelemetry().addLine("B - Exit");
        context.getTelemetry().update();
    }

    /**
     * @return the name of this phase as a string for display purposes.
     */
    protected abstract String getPhaseName();

    /**
     * @return true if manual tuning is possible for this phase, false otherwise.
     */
    protected abstract boolean manualTuneIsPossible();

    /**
     * @return true if automatic tuning is possible for this phase, false otherwise.
     */
    protected abstract boolean autoTuneIsPossible();

    /** Initializes the tuning phase (for automatic or manual) */
    protected abstract void init();

    /**
     * This method should perform manual tuning updates. It will be called repeatedly until it
     * returns true.
     * @return true if the manual tuning is complete, false otherwise
     */
    protected abstract boolean manualUpdate(boolean aWasPressed, boolean bWasPressed);

    protected final boolean isManualMode() { return manualMode; }

    /**
     * This method should perform automatic tuning updates. It will be called repeatedly until it
     * returns true.
     *
     * @return true if the automatic tuning is complete, false otherwise
     */
    protected abstract boolean automaticUpdate();

    /**
     * This method should use the telemetry (context.getTelemetry()) to report the results of
     * the tuning phase. It will be called repeatedly until the user exits the phase.
     */
    protected abstract void reportResults();
}
