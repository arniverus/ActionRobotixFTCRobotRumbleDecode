package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoCloseBlue Real", group = "Autonomous")
public class AutoCloseBlueReal extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo stopper, hood;

    private final double SHOOT_VELOCITY = 1030;
    double SHOOTER_P = -830.000;
    double SHOOTER_F = 1.00000;

    // Stoppage delay before each shot sequence begins
    private final double SHOOT_DELAY_SECONDS = 0.5;

    // PathChains
    private PathChain shootreload, intake1, shootintake1,
            intake2, shootintake2, intake3, shootintake3,
            intake4, shootintake4, park;

    public void buildPaths() {
        // Preload: drive from start pose to shoot position
        shootreload = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(39.000, 133.000), new Pose(55.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        // Cycle 1
        intake1 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(55.000, 85.000), new Pose(20.000, 82.000)))
                .setTangentHeadingInterpolation()
                .build();

        shootintake1 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(20.000, 82.000), new Pose(55.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .setReversed()
                .build();

        // Cycle 2
        intake2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.000, 85.000),
                        new Pose(34.500, 53.000),
                        new Pose(20.000, 59.000)))
                .setTangentHeadingInterpolation()
                .build();

        shootintake2 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(20.000, 59.000), new Pose(55.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        // Cycle 3
        intake3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.000, 85.000),
                        new Pose(36.000, 50.000),
                        new Pose(11.000, 61.000)))
                .setTangentHeadingInterpolation()
                .build();

        shootintake3 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(11.000, 61.000), new Pose(55.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        // Cycle 4
        intake4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55.000, 85.000),
                        new Pose(36.000, 50.000),
                        new Pose(11.000, 58.400)))
                .setTangentHeadingInterpolation()
                .build();

        shootintake4 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(11.000, 58.400), new Pose(55.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();

        // Placeholder park path — update the end Pose to wherever you want to park.
        park = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(55.000, 85.000), new Pose(55.000, 85.000)))
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(135))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to Shoot Preload
                shooter1.setVelocity(SHOOT_VELOCITY);
                shooter2.setVelocity(SHOOT_VELOCITY);
                follower.followPath(shootreload, 0.8, true);
                setPathState(1);
                break;
            case 1: // Arrived at preload shoot spot — hold 500ms before shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY_SECONDS) {
                    setPathState(2);
                }
                break;
            case 2: // Shoot Preload (3 balls) and transition to Intake 1
                if (!follower.isBusy()) runShootSequence(intake1, 3);
                break;
            case 3: // Driving across to pick up Intake 1, turn on intake
                intake.setPower(-1.0);
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4: // Wait briefly then return to shoot
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(shootintake1, 0.8, true);
                    setPathState(5);
                }
                break;
            case 5: // Arrived at shoot spot — hold 500ms before shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY_SECONDS) {
                    setPathState(6);
                }
                break;
            case 6: // Shoot 1 and transition to Intake 2
                if (!follower.isBusy()) runShootSequence2(intake2, 7);
                break;
            case 7: // Driving across to pick up Intake 2
                intake.setPower(-1.0);
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8: // Wait briefly then return to shoot
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(shootintake2, 0.8, true);
                    setPathState(9);
                }
                break;
            case 9: // Arrived at shoot spot — hold 500ms before shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY_SECONDS) {
                    setPathState(10);
                }
                break;
            case 10: // Shoot 2 and transition to Intake 3
                if (!follower.isBusy()) runShootSequence2(intake3, 11);
                break;
            case 11: // Driving across to pick up Intake 3
                intake.setPower(-1.0);
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12: // Wait briefly then return to shoot
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(shootintake3, 0.8, true);
                    setPathState(13);
                }
                break;
            case 13: // Arrived at shoot spot — hold 500ms before shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY_SECONDS) {
                    setPathState(14);
                }
                break;
            case 14: // Shoot 3 and transition to Intake 4
                if (!follower.isBusy()) runShootSequence2(intake4, 15);
                break;
            case 15: // Driving across to pick up Intake 4
                intake.setPower(-1.0);
                if (!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16: // Wait briefly then return to shoot
                if (pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(shootintake4, 0.8, true);
                    setPathState(17);
                }
                break;
            case 17: // Arrived at shoot spot — hold 500ms before shooting
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > SHOOT_DELAY_SECONDS) {
                    setPathState(18);
                }
                break;
            case 18: // Shoot 4 and transition to Park
                if (!follower.isBusy()) runShootSequence2(park, 19);
                break;
            case 19: // Follow Park path
                if (!follower.isBusy()) {
                    follower.followPath(park, 1.0, true);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) stopRobot();
                break;
        }
    }

    private void runShootSequence(PathChain nextPath, int nextState) {
        hood.setPosition(0.35);
        stopper.setPosition(0.3);
        intake.setPower(-1.0);
        if (pathTimer.getElapsedTimeSeconds() > 3.0) {
            intake.setPower(0);
            stopper.setPosition(0.7);
            if (nextPath != null) follower.followPath(nextPath, 1.0, true);
            setPathState(nextState);
        }
    }

    private void runShootSequence2(PathChain nextPath, int nextState) {
        hood.setPosition(0.35);
        stopper.setPosition(0.3);
        intake.setPower(-1.0);
        if (pathTimer.getElapsedTimeSeconds() > 4.5) {
            intake.setPower(0);
            stopper.setPosition(0.7);
            if (nextPath != null) follower.followPath(nextPath, 1.0, true);
            setPathState(nextState);
        }
    }

    private void stopRobot() {
        intake.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
        stopper.setPosition(0.7);
        setPathState(-1);
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(39.000, 133.000, Math.toRadians(90)));

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hood = hardwareMap.get(Servo.class, "hood");

        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        buildPaths();
        waitForStart();
        if (isStopRequested()) return;
        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();
            telemetry.addData("State", pathState);
            telemetry.update();
        }
    }
}