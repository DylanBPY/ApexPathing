package org.firstinspires.ftc.teamcode.apexpathing.tuning;

public class TranslationPhase extends PdsTuningPhase {
    public TranslationPhase() {
        super("TRANSLATION", false);
    }

    @Override
    protected double getP(TunerContext context) {
        return context.translationP;
    }

    @Override
    protected void setP(TunerContext context, double value) {
        context.translationP = value;
    }

    @Override
    protected double getD(TunerContext context) {
        return context.translationD;
    }

    @Override
    protected void setD(TunerContext context, double value) {
        context.translationD = value;
    }

    @Override
    protected double getS(TunerContext context) {
        return context.translationS;
    }

    @Override
    protected void setS(TunerContext context, double value) {
        context.translationS = value;
    }

    @Override
    protected TuningPhase nextPhase(TunerContext context) {
        return new VelocityFeedforwardPhase();
    }
}
