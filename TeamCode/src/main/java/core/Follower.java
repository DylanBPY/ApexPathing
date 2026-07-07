package core;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import controllers.PDSController;
import drivetrains.BaseDrivetrain;
import geometry.Angle;
import geometry.PathSegment;
import geometry.Pose;
import geometry.Vector;
import localizers.BaseLocalizer;
import paths.movements.FollowerMovement;
import paths.movements.Path;
import paths.movements.Turn;

/**
 * Apex Pathing's main Follower class.
 *
 * @author Sohum Arora 22985 Paraducks
 * @author DrPixelCat
 * @author Dylan B. 18597 RoboClovers - Delta
 * @author Xander Haemel 31616 404 Not Found
 */
public class Follower {
    private final FollowerConstants constants;
    private final BaseDrivetrain<?> drivetrain;
    private final BaseLocalizer<?> localizer;

    private final double headingTol; // Radians
    private final double distanceTol; // Inches
    private final double velocityLimit; // Inches per second
    private final double velocityS;

    public final PDSController headingController;
    public final PDSController translationalController;
    public final PDSController driveController;
    public final PDSController velocityController;
    private boolean headingControllerEnabled = true;
    private boolean translationalControllerEnabled = true;
    private boolean driveControllerEnabled = true;

    private FollowerMovement currentMovement = null;
    private boolean paused = false;

    // Temporary values to avoid repeated object creation
    PathSegment segment;
    Angle targetHeading;
    Vector targetTurnPoseVec;

    /** Constructs the drivetrain, localizer, and follower from the given {@link ApexConstants}. */
    public Follower(ApexConstants constants, HardwareMap hardwareMap) {
        this.drivetrain = constants.drivetrainConstants().build(hardwareMap);
        this.localizer = constants.localizerConstants().build(hardwareMap);
        this.constants = new FollowerConstants();

        this.headingTol = this.constants.headingTolerance.getRad();
        this.distanceTol = this.constants.distanceTolerance.getIn();
        this.velocityLimit = this.constants.velocityLimit.getIn();

        this.headingController = new PDSController(this.constants.headingCoeffs);
        this.headingController.setAngularController();
        this.translationalController = new PDSController(this.constants.translationalCoeffs);
        this.driveController = new PDSController(this.constants.driveCoeffs);
        this.velocityController = new PDSController(this.constants.velocityCoeffs);

        this.velocityS = driveController.getCoefficients().kS;
    }

    // region General methods
    public void update() {
        if (currentMovement == null || paused) {
            return;
        }

        // Turn power will always be used
        Pose current = getPose();
        Vector currentPos = current.getPos();
        Angle currentHeading = current.getHeading();

        double headingError = targetHeading.getShortestAngleTo(currentHeading).getRad();
        double turnPower = headingControllerEnabled ?
                headingController.calculateFromError(headingError) : 0;

        if (currentMovement instanceof Turn) {
            if (Math.abs(headingError) < headingTol) {
                this.stop(); return;
            }

            Vector error = targetTurnPoseVec.minus(currentPos);
            double errorMag = error.getMag().getIn();
            if (errorMag > 0) {
                double lateralPower = translationalController.calculateFromError(errorMag);
                Vector feedback = error.normalize().times(lateralPower);
                drivetrain.drive(feedback.getX().getIn(), feedback.getY().getIn(), turnPower);
            } else {
                drivetrain.drive(0, 0, turnPower);
            }
        } else { // TODO: Check heading interpolation here, I have a suspicion that something is messed up
            // Assume it's a Path movement, since that's the only other option.
            // If more movement types are added in the future, this will need to be refactored.
            if (segment == null) {
                this.stop(); return;
            }

            double t = segment.getBestT(currentPos);
            Vector targetPoseVec = segment.getPosition(t);
            Vector velVec = segment.getFirstDerivative(t);
            Vector accelVec = segment.getSecondDerivative(t);
            Vector normal = PathSegment.calculateArcNormal(velVec, accelVec);

            double availableMotorPower = 1.0;
            availableMotorPower -= Math.abs(turnPower); // Heading correction is the highest priority

            Vector robotVelocity = localizer.getVel().getPos();
            double robotVelMag = robotVelocity.getMag().getIn();

            // Project field error onto the normal vector to isolate lateral drift
            Vector fieldError = targetPoseVec.minus(currentPos);
            double lateralFeedbackMag = 0.0;
            if (translationalControllerEnabled) {
                double crossTrackError = fieldError.dot(normal).getIn();
                lateralFeedbackMag = translationalController.calculateFromError(crossTrackError);
            }

            // Required inward acceleration based on ACTUAL current speed
            double radius = PathSegment.calculateRadiusOfCurvature(velVec, accelVec);
            double requiredLateralAccel = (robotVelMag * robotVelMag) / radius;
            double centripetalMag = requiredLateralAccel / constants.maxTranslationalAccel;

            // Combine centripetal feedforward and corrective feedback
            double netLateralMag = centripetalMag + lateralFeedbackMag;
            netLateralMag = Range.clip(netLateralMag, -availableMotorPower, availableMotorPower);

            Vector lateralDriveVec = normal.times(netLateralMag);
            availableMotorPower -= Math.abs(netLateralMag); // Lateral correction is the second priority

            // Cap the desired speed if the upcoming curve is too sharp
            double desiredVelocity = velocityLimit;
            if (radius != Double.POSITIVE_INFINITY) {
                double maxSafeVelocity = Math.sqrt(constants.maxTranslationalAccel * radius);
                if (desiredVelocity > maxSafeVelocity) {
                    desiredVelocity = maxSafeVelocity;
                }
            }

            // Calculate deceleration if current speed exceeds the safe requested speed
            double requiredAccel = 0.0;
            if (desiredVelocity < robotVelMag) {
                requiredAccel = desiredVelocity - robotVelMag;
            }

            // Project field error onto the tangent vector to isolate forward/backward error
            Vector unitTangent = velVec.normalize();

            double feedforward = 0.0;
            double tangentFeedbackMag = 0.0;
            double totalTangentPower = 0.0;
            if (driveControllerEnabled) {
                if (t < 1.0) {
                    feedforward = (constants.translationalKV * desiredVelocity) +
                            (constants.translationalKA * requiredAccel) + velocityS;

                    double alongTrackError = fieldError.dot(unitTangent).getIn();
                    tangentFeedbackMag = driveController.calculateFromError(alongTrackError);
                } else { // Infinite line fallback for end of path
                    Vector endPos = segment.getPosition(1.0);
                    Vector endTangent = segment.getFirstDerivative(1.0).normalize();
                    Vector endToRobot = currentPos.minus(endPos);

                    double distancePastEnd = endToRobot.dot(endTangent).getIn();
                    tangentFeedbackMag = driveController.calculateFromError(-distancePastEnd);
                }

                // Calculate the velocity feedback and safely append the Feedforward and kS
                double velocityFeedback = velocityController.calculateFromError(
                        desiredVelocity - robotVelMag
                ) + feedforward;

                // Cap the spatial position request with the velocity ceiling
                totalTangentPower = Math.abs(velocityFeedback) < Math.abs(tangentFeedbackMag)
                        ? velocityFeedback : tangentFeedbackMag;
                totalTangentPower = Range.clip(totalTangentPower, -availableMotorPower, availableMotorPower);
            }

            // Apply the power scalar to the purely directional unit tangent
            Vector tangentDriveVec = unitTangent.times(totalTangentPower);
            Vector driveOutput = tangentDriveVec.plus(lateralDriveVec);

            double distanceRemaining = segment.getDistanceToEndIn(targetPoseVec, t);
            if (t >= constants.tTolerance && distanceRemaining < distanceTol) {
                this.stop(); return;
            }

            drivetrain.drive(driveOutput.getX().getIn(), driveOutput.getY().getIn(), turnPower);
        }
    }

    /**
     * Checks if the follower is currently performing a movement. This can be used to determine if
     * it's safe to start a new movement or if the current one is still in progress.
     */
    public boolean isBusy() { return currentMovement != null; }

    // region Auto methods
    /**
     * Starts following the given movement.
     *
     * @param movement the {@link FollowerMovement} to be followed
     * @throws IllegalStateException if the follower is already busy executing a movement.
     */
    public void follow(FollowerMovement movement) {
        if (isBusy()) {
            throw new IllegalStateException(
                    "Cannot execute a new movement while another movement is still in progress! Tip: use follower.isBusy() to check if the follower is currently executing a movement before starting a new one."
            );
        }

        this.currentMovement = movement;
        this.currentMovement.setStarted(true);
        this.currentMovement.setEnded(false);
        this.targetHeading = movement.getEndPose().getHeading();

        if (movement instanceof Turn) {
            Turn turn = (Turn) currentMovement;
            this.targetTurnPoseVec = turn.getStartPose().getPos();
        } else if (movement instanceof Path) {
            Path pathSegmentMove = (Path) currentMovement;
            this.segment = pathSegmentMove.getParametricPath();
        }

        this.headingController.reset();
        this.translationalController.reset();
        this.driveController.reset();
        this.velocityController.reset();
    }

    /** Stops the drivetrain and any ongoing movement. The busy state will be set to false. */
    public void stop() {
        if (this.currentMovement != null) {
            this.currentMovement.setEnded(true);
        }

        this.currentMovement = null;
        this.segment = null;
        this.targetHeading = null;
        this.targetTurnPoseVec = null;

        this.drivetrain.stop();
    }

    /** Stops the drivetrain and any ongoing movement. The busy state will remain true. */
    public void pause() {
        this.paused = true;
        this.drivetrain.stop();
    }

    /** Resumes the current movement if it was paused. If no movement is paused, this method does nothing. */
    public void resume() {
        if (this.paused) {
            this.paused = false;
        }
    }
    // endregion

    // region TeleOp methods
    /**
     * Drives the robot using the provided  inputs. The joystick inputs are adjusted for
     * field-centric or robot-centric control based on the constants. This method  will stop the
     * current movement if one is in progress, as manual control takes priority over following a path.
     *
     * @param x the left/right joystick input (positive for right, negative for left)
     * @param y the forward/backward joystick input (positive for forward, negative for backward)
     * @param turn the rotation joystick input (positive for clockwise, negative for counterclockwise)
     */
    public void teleOpDrive(double x, double y, double turn) {
        if (isBusy()) { stop(); }
        drivetrain.drive(x, y, turn, this.getPose().getHeading().getRad());
    }

    /**
     * Drives the robot using standard gamepad inputs. The left stick controls translation (x and y),
     * and the right stick controls rotation (turn). The joystick inputs are adjusted for
     * field-centric or robot-centric control based on the constants. This method will stop the
     * current movement if one is in progress, as manual control takes priority over following a path.
     * @param gamepad the gamepad to read inputs from
     */
    public void teleOpDrive(Gamepad gamepad) {
        teleOpDrive(-gamepad.left_stick_x, -gamepad.left_stick_y, -gamepad.right_stick_x);
    }
    // endregion

    // region Localizer passthrough methods
    /** @return the robot's current pose estimate */
    public Pose getPose() { return localizer.getPose(); }

    /** @param pose the current pose of the robot */
    public void setPose(Pose pose) { localizer.setPose(pose); }

    /**
     * Velocity is expressed in pose form (x and y components in the local robot frame, rotational
     * component in radians per second).
     *
     * @return the robot's current velocity estimate from the localizer
     */
    public Pose getVelocity() { return localizer.getVel(); }

    /**
     * Acceleration is expressed in pose form (x and y components in the local robot frame,
     * rotational component in radians per second squared).
     *
     * @return the robot's current acceleration estimate from the localizer
     */
    public Pose getAcceleration() { return localizer.getAccel(); }

    public void disableHeadingController() { this.headingControllerEnabled = false; }
    public void disableTranslationalController() { this.translationalControllerEnabled = false; }
    public void disableDriveController() { this.driveControllerEnabled = false; }
    public void disableAllControllers() {
        disableHeadingController();
        disableTranslationalController();
        disableDriveController();
    }

    /**
     * DO NOT USE THIS METHOD UNLESS YOU KNOW WHAT YOU ARE DOING.
     * It is intended for internal use only.
     */
    public BaseLocalizer<?> getLocalizer() { return localizer; }

    /**
     * DO NOT USE THIS METHOD UNLESS YOU KNOW WHAT YOU ARE DOING.
     * It is intended for internal use only.
     */
    public BaseDrivetrain<?> getDrivetrain() { return drivetrain; }

    public FollowerConstants getConstants() { return constants; }
    // endregion
}