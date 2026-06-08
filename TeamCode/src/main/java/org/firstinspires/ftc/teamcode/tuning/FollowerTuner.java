package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.TimeUnit;

import core.ApexConfig;
import core.Follower;
import core.FollowerConstants;
import controllers.PDSController.PDSCoefficients;
import drivetrains.BaseDrivetrainConfig;
import localizers.BaseLocalizerConfig;
import geometry.Angle;
import geometry.Dist;
import geometry.Pose;
import geometry.Vector;
import paths.builders.Builder;
import paths.movements.Path;
import util.DistUnit;

import org.firstinspires.ftc.teamcode.Constants;

/**
 * Single unified automatic tuner capable of completely tuning a robot for Apex in minutes in just a single OpMode!
 * All you have to do is follow the telemetry instructions and press a couple buttons here and there
 * Once you have run this tuner, your robot is fully tuned and ready to go Path its way to the Peaks™️😁
 * @author Sohum Arora 22985 Paraducks
 */
@TeleOp(name = "Follower Tuner", group = "Apex Pathing Tuning")
public class FollowerTuner extends LinearOpMode {

    enum TuningState {
        AWAIT_CONFIRM,
        KS_SEARCH,
        STEP_RESPONSE,
        VELOCITY_FF,
        LATERAL_ACCEL,
        LATERAL_ACCEL_TEST,
        CONFIRM,
        SAVE
    }

    enum TuningPhase { HEADING, TRANSLATION, VELOCITY_FF, LATERAL_ACCEL, COMPLETE }

    private TuningPhase phase = TuningPhase.HEADING;
    private TuningState state = TuningState.AWAIT_CONFIRM;

    private double headingP, headingD, headingS;
    private double translationP, translationD, translationS;
    private double velocityFF;
    private double maxLateralAccel = 40.0;
    private double headingToleranceDeg, distanceToleranceIn, tTolerance;

    private double ksMax = 0.2, ksMin = 0.0, ksGuess = 0.0, ksLastGuess = -1.0, ksMaxDeviation;
    private double stepMaxAccel, stepMaxVel, stepLastVel, stepLastTime, stepStartTime, stepTimeStamp, stepVelAtTimeStamp;
    private double accelMaxError;
    private boolean driftDetected;

    private final ElapsedTime timer = new ElapsedTime();
    private final Constants baseConstants = new Constants();
    private final FollowerConstants followerConstants = new FollowerConstants();
    private Follower follower;

    private boolean lastA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        FollowerConstants defaults = baseConstants.followerConfig().loadFromJson();
        headingP = defaults.headingCoeffs.kP;
        headingD = defaults.headingCoeffs.kD;
        headingS = defaults.headingCoeffs.kS;
        translationP = defaults.driveCoeffs.kP;
        translationD = defaults.driveCoeffs.kD;
        translationS = defaults.driveCoeffs.kS;
        velocityFF = defaults.lateralKV;
        headingToleranceDeg = defaults.headingTolerance.getDeg();
        distanceToleranceIn = defaults.distanceTolerance.getIn();
        tTolerance = defaults.tTolerance;
        maxLateralAccel = defaults.maxLateralAccel > 10 ? defaults.maxLateralAccel : 40.0;

        while (opModeInInit()) {
            telemetry.addLine("Robot Initialized");
            telemetry.addLine("Tuning order:\n 1) Heading PDS \n 2) Translation PDS \n 3) Velocity FF \n 4) Max Lateral Accel");
            telemetry.addLine("Run the OpMode to proceed with the Heading Tuner");
            telemetry.addLine("Press 'A' (cross) to directly run the Translation Tuner if you have already run the Heading Tuner");
            telemetry.addLine("Press 'B' (circle) to directly run the Velocity FF Tuner if you have already run the Heading Tuner and Translation Tuner");
            telemetry.addLine("Once all of these 3 tuners are complete, press 'A' (circle) to run the Max Lateral Acceleration Tuner");
            telemetry.addLine("IMPORTANT: Do NOT run the tuners out of order");

            if (gamepad1.a) {
                phase = TuningPhase.TRANSLATION;
            } else if (gamepad1.b) {
                phase = TuningPhase.VELOCITY_FF;
            }

            telemetry.addData("Selected Phase", phase);
            telemetry.update();
        }

        updateFollowerConfig();
        follower = new Follower(customConfig, hardwareMap);

        waitForStart();

        while (opModeIsActive() && phase != TuningPhase.COMPLETE && !isStopRequested()) {
            switch (phase) {
                case HEADING:
                case TRANSLATION: {
                    boolean isAngular = phase == TuningPhase.HEADING;
                    switch (state) {
                        case AWAIT_CONFIRM:
                            telemetry.addLine("Press 'A' to proceed with the " + phase + " tuner");
                            telemetry.update();
                            if (gamepad1.a && !lastA) {
                                resetKsSearch();
                                state = TuningState.KS_SEARCH;
                            }
                            break;

                        case KS_SEARCH:
                            if (Math.abs(ksLastGuess - ksGuess) <= 0.01) {
                                if (isAngular) headingS = ksGuess;
                                else translationS = ksGuess;
                                resetStepResponse();
                                state = TuningState.STEP_RESPONSE;
                                break;
                            }

                            follower.setPose(new Pose(new Vector(Dist.of(0, DistUnit.IN), Dist.of(0, DistUnit.IN)), Angle.fromDeg(0)));
                            follower.update();
                            ksGuess = (ksMax + ksMin) / 2.0;
                            ksMaxDeviation = 0.0;
                            timer.reset();

                            while (opModeIsActive() && timer.time(TimeUnit.MILLISECONDS) < 500) {
                                follower.update();
                                double pos = isAngular
                                        ? follower.getPose().getHeading().getRad()
                                        : follower.getPose().getPos().getX().getIn();
                                ksMaxDeviation = Math.max(Math.abs(pos), ksMaxDeviation);
                                if (isAngular) follower.teleOpDrive(0, 0, ksGuess);
                                else follower.teleOpDrive(ksGuess, 0, 0);
                            }

                            if (ksMaxDeviation > 0.025) ksMax = ksGuess;
                            else ksMin = ksGuess;
                            ksLastGuess = ksGuess;

                            follower.teleOpDrive(0, 0, 0);
                            sleep(500);
                            break;

                        case STEP_RESPONSE:
                            if (timer.time(TimeUnit.MILLISECONDS) >= 2000) {
                                follower.teleOpDrive(0, 0, 0);
                                sleep(500);

                                double L = stepTimeStamp - (stepVelAtTimeStamp / stepMaxAccel);
                                double kP = 1.2 / (L * stepMaxAccel);
                                double kD = 0.6 / stepMaxAccel;

                                if (isAngular) {
                                    headingP = kP > 0 ? kP : 0.01;
                                    headingD = kD > 0 ? kD : 0.001;
                                } else {
                                    translationP = Double.isFinite(kP) && kP > 0 ? kP : 0.01;
                                    translationD = Double.isFinite(kD) && kD > 0 ? kD : 0.001;
                                }

                                updateFollowerConfig();
                                state = TuningState.CONFIRM;
                                break;
                            }

                            follower.update();
                            double curVel = isAngular
                                    ? follower.getVelocity().getHeading().getRad()
                                    : follower.getVelocity().getPos().getX().getIn();

                            double now = System.nanoTime();
                            double deltaT = (now - stepLastTime) / 1e9;
                            double deltaV = curVel - stepLastVel;
                            double accel = deltaT > 1e-6 ? deltaV / deltaT : 0.0;

                            if (accel > stepMaxAccel) {
                                stepMaxAccel = accel;
                                stepTimeStamp = (now - stepStartTime) / 1e9;
                                stepVelAtTimeStamp = curVel;
                            }

                            stepMaxVel = Math.max(curVel, stepMaxVel);
                            stepLastVel = curVel;
                            stepLastTime = now;

                            if (isAngular) follower.teleOpDrive(0, 0, 1.0);
                            else follower.teleOpDrive(1.0, 0, 0);
                            break;

                        case CONFIRM:
                            telemetry.addData("Current Phase", phase);
                            telemetry.addLine("Press 'A' to accept and advance.");
                            if (isAngular) {
                                telemetry.addData("Heading P", headingP);
                                telemetry.addData("Heading D", headingD);
                                telemetry.addData("Heading S", headingS);
                            } else {
                                telemetry.addData("Translation P", translationP);
                                telemetry.addData("Translation D", translationD);
                                telemetry.addData("Translation S", translationS);
                            }
                            telemetry.addData("Robot Pose", follower.getPose().toString());
                            telemetry.update();

                            follower.teleOpDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

                            if (gamepad1.a && !lastA) {
                                phase = isAngular ? TuningPhase.TRANSLATION : TuningPhase.VELOCITY_FF;
                                state = TuningState.AWAIT_CONFIRM;
                                resetKsSearch();
                            }
                            break;
                    }
                    break;
                }

                case VELOCITY_FF: {
                    switch (state) {
                        case AWAIT_CONFIRM:
                            telemetry.addLine("Press 'A' to proceed with the VELOCITY_FF tuner");
                            telemetry.update();
                            if (gamepad1.a && !lastA) {
                                state = TuningState.VELOCITY_FF;
                            }
                            break;

                        case VELOCITY_FF:
                            follower.teleOpDrive(0, 1.0, 0);
                            sleep(1500);
                            double maxVel = Math.abs(follower.getVelocity().getPos().getX().getIn());
                            velocityFF = 1.0 / maxVel;
                            follower.teleOpDrive(0, 0, 0);
                            sleep(500);
                            updateFollowerConfig();
                            state = TuningState.CONFIRM;
                            break;

                        case CONFIRM:
                            telemetry.addData("Current Phase", phase);
                            telemetry.addLine("Press 'A' to accept and advance.");
                            telemetry.addData("Velocity FF (kV)", velocityFF);
                            telemetry.addData("Robot Pose", follower.getPose().toString());
                            telemetry.update();

                            follower.teleOpDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

                            if (gamepad1.a && !lastA) {
                                phase = TuningPhase.LATERAL_ACCEL;
                                state = TuningState.AWAIT_CONFIRM;
                                maxLateralAccel = 50.0;
                                driftDetected = false;
                            }
                            break;
                    }
                    break;
                }

                case LATERAL_ACCEL: {
                    switch (state) {
                        case AWAIT_CONFIRM:
                            telemetry.addLine("Press 'A' to proceed with the LATERAL_ACCEL tuner");
                            telemetry.update();
                            if (gamepad1.a && !lastA) {
                                state = TuningState.LATERAL_ACCEL;
                            }
                            break;

                        case LATERAL_ACCEL:
                            if (driftDetected || maxLateralAccel > 300) {
                                if (!driftDetected) maxLateralAccel -= 20.0;
                                updateFollowerConfig();
                                state = TuningState.CONFIRM;
                                break;
                            }

                            updateFollowerConfig();
                            follower.setPose(new Pose(new Vector(Dist.of(0, DistUnit.IN), Dist.of(0, DistUnit.IN)), Angle.fromDeg(0)));

                            Pose start = follower.getPose();
                            Path testCurve = Builder.path(
                                    start,
                                    new Pose(start.getPos().plus(new Vector(Dist.of(30, DistUnit.IN), Dist.of(0, DistUnit.IN))), start.getHeading()),
                                    new Pose(start.getPos().plus(new Vector(Dist.of(30, DistUnit.IN), Dist.of(30, DistUnit.IN))), start.getHeading().plus(Angle.fromDeg(90))),
                                    new Pose(start.getPos().plus(new Vector(Dist.of(0, DistUnit.IN), Dist.of(30, DistUnit.IN))), start.getHeading().plus(Angle.fromDeg(180)))
                            ).build();

                            follower.follow(testCurve);
                            accelMaxError = 0;
                            state = TuningState.LATERAL_ACCEL_TEST;
                            break;

                        case LATERAL_ACCEL_TEST:
                            follower.update();
                            double err = follower.getPose().getPos().getMag().getIn();
                            if (err > accelMaxError) accelMaxError = err;

                            if (!follower.isBusy()) {
                                if (accelMaxError > 4.0) {
                                    driftDetected = true;
                                    maxLateralAccel -= 20.0;
                                } else {
                                    maxLateralAccel += 20.0;
                                    sleep(1000);
                                }
                                state = TuningState.LATERAL_ACCEL;
                            }
                            break;

                        case CONFIRM:
                            telemetry.addData("Current Phase", phase);
                            telemetry.addLine("Press 'A' to accept and finish.");
                            telemetry.addData("Max Lateral Accel", maxLateralAccel);
                            telemetry.addData("Robot Pose", follower.getPose().toString());
                            telemetry.update();

                            follower.teleOpDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);

                            if (gamepad1.a && !lastA) {
                                state = TuningState.SAVE;
                            }
                            break;

                        case SAVE:
                            saveConstantsToJson();
                            phase = TuningPhase.COMPLETE;
                            break;
                    }
                    break;
                }
            }

            lastA = gamepad1.a;
            telemetry.update();
        }

        while (opModeIsActive()) {
            telemetry.addData("Status", "All Tuning Cycles Complete! Configuration Saved to JSON.");
            telemetry.update();
            follower.teleOpDrive(0, 0, 0);
        }
    }

    private void resetKsSearch() {
        ksMax = 0.2;
        ksMin = 0.0;
        ksGuess = 0.0;
        ksLastGuess = -1.0;
        ksMaxDeviation = 0.0;
    }

    private void resetStepResponse() {
        stepMaxAccel = 0;
        stepMaxVel = 0;
        stepLastVel = 0;
        stepTimeStamp = 0;
        stepVelAtTimeStamp = 0;
        stepLastTime = System.nanoTime();
        stepStartTime = System.nanoTime();
        timer.reset();

        follower.setPose(new Pose(new Vector(Dist.of(0, DistUnit.IN), Dist.of(0, DistUnit.IN)), Angle.fromDeg(0)));
    }

    private void updateFollowerConfig() {
        followerConstants.headingCoeffs = new PDSCoefficients(headingP, headingD, headingS, 0);
        followerConstants.driveCoeffs = new PDSCoefficients(translationP, translationD, translationS, 0);
        followerConstants.lateralCoeffs = new PDSCoefficients(translationP, translationD, translationS, 0);
        followerConstants.lateralKV = velocityFF;
        followerConstants.headingTolerance = Angle.fromDeg(headingToleranceDeg);
        followerConstants.distanceTolerance = Dist.fromIn(distanceToleranceIn);
        followerConstants.tTolerance = tTolerance;
        followerConstants.maxLateralAccel = maxLateralAccel;
    }

    private final ApexConfig customConfig = new ApexConfig() {
        @Override
        public BaseDrivetrainConfig<?> drivetrainConfig() { return baseConstants.drivetrainConfig(); }
        @Override
        public BaseLocalizerConfig<?> localizerConfig() { return baseConstants.localizerConfig(); }
        @Override
        public FollowerConstants followerConfig() { return followerConstants; }
    };

    private void saveConstantsToJson() {
        String jsonPayload = "{\n" +
                "  \"headingP\": " + headingP + ",\n" +
                "  \"headingD\": " + headingD + ",\n" +
                "  \"headingS\": " + headingS + ",\n" +
                "  \"translationP\": " + translationP + ",\n" +
                "  \"translationD\": " + translationD + ",\n" +
                "  \"translationS\": " + translationS + ",\n" +
                "  \"velocityFF\": " + velocityFF + ",\n" +
                "  \"maxLateralAccel\": " + maxLateralAccel + "\n" +
                "}";

        try {
            File outputFolder = new File("/sdcard/FIRST");
            if (!outputFolder.exists()) outputFolder.mkdirs();
            FileWriter fileWriter = new FileWriter(new File(outputFolder, "FollowerConstants.json"));
            fileWriter.write(jsonPayload);
            fileWriter.close();
        } catch (IOException ignored) {}
    }
}