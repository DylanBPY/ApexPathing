package feedforward;

public class FeedforwardParams {
    private double tangentialVel;
    private double tangentialAccel;
    private double angularVel;
    private double angularAccel;
    private double distAlongCurve = 0.0;

    /**
     * No-args constructor to initialize a blank, reusable parameters object.
     */
    public FeedforwardParams() {
        this.tangentialVel = 0.0;
        this.tangentialAccel = 0.0;
        this.angularVel = 0.0;
        this.angularAccel = 0.0;
        this.distAlongCurve = 0.0;
    }

    /**
     * All-args constructor for immediate full initialization.
     */
    public FeedforwardParams(double tangentialVel, double tangentialAccel, double angularVel, double angularAccel, double distAlongCurve) {
        this.tangentialVel = tangentialVel;
        this.tangentialAccel = tangentialAccel;
        this.angularVel = angularVel;
        this.angularAccel = angularAccel;
        this.distAlongCurve = distAlongCurve;
    }

    public FeedforwardParams(double tangentialVel, double tangentialAccel, double angularVel, double angularAccel) {
        this.tangentialVel = tangentialVel;
        this.tangentialAccel = tangentialAccel;
        this.angularVel = angularVel;
        this.angularAccel = angularAccel;
    }

    public FeedforwardParams setTangentialVel(double tangentialVel) {
        this.tangentialVel = tangentialVel;
        return this;
    }

    public FeedforwardParams setTangentialAccel(double tangentialAccel) {
        this.tangentialAccel = tangentialAccel;
        return this;
    }

    public FeedforwardParams setAngularVel(double angularVel) {
        this.angularVel = angularVel;
        return this;
    }

    public FeedforwardParams setAngularAccel(double angularAccel) {
        this.angularAccel = angularAccel;
        return this;
    }

    public double getTangentialVel() {
        return tangentialVel;
    }

    public double getTangentialAccel() {
        return tangentialAccel;
    }

    public double getAngularVel() {
        return angularVel;
    }

    public double getAngularAccel() {
        return angularAccel;
    }

    public double getDistAlongCurve() {
        return distAlongCurve;
    }
}