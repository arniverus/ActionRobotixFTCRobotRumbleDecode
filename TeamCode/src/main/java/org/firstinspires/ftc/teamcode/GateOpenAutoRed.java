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
import com.pedropathing.paths.PathChain;

@Autonomous(name = "Gate Open Red", group = "Autonomous")
public class GateOpenAutoRed extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo stopper, hood;

    private final double SHOOT_VELOCITY = 1250;

    public static double SHOOTER_P = -830.000;
    public static double SHOOTER_F = 1.00000;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13;

    public void buildPaths() {
        // Path 1: shootpreload
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(112.7, 136.737), new Pose(93.9, 87)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(40))
                .build();

        // Path 2: intake ready
        path2 = follower.pathBuilder()
                .addPath(new BezierLine( new Pose(84.90415913200722, 86.37974683544303), new Pose(93.9, 87)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        // Path 3: intake first
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(93.9, 87), new Pose(137.392, 87)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 4: shoot second
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(137.392, 87),  new Pose(93.9, 87)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        // Path 5: intake ready second
        path5 = follower.pathBuilder()
                .addPath(new BezierLine( new Pose(93.9, 87), new Pose(94, 65)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        // Path 6: intake second
        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94, 65), new Pose(142.5, 65)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 7: TANGENTIAL/CURVED Path to avoid gate
        // We add a control point at (135, 95) to push the robot's "belly" away from the gate center
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(142.5, 65),
                        new Pose(120, 65), // Control Point: Adjust this Y up/down if you still clip the gate
                        new Pose(93.9, 87)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        // Path 8: intake third
        path8 = follower.pathBuilder()
                .addPath(new BezierLine( new Pose(84.90415913200722, 86.37974683544303), new Pose(102, 39)))
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();

        // Path 9: intake move
        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(102, 39), new Pose(145, 39)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 10: shoot final
        path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(145, 38.5),  new Pose(93.9, 87)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();
        path11 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.90415913200722, 86.37974683544303), new Pose(121.84865800207264, 69)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();
        path12 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.90415913200722, 86.37974683544303), new Pose(132.15154460886288, 56.98855938488923)))
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(29))
                .build();
        path13 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132.15154460886288, 56.98855938488923), new Pose(84.90415913200722, 86.37974683544303)))
                .setLinearHeadingInterpolation(Math.toRadians(29), Math.toRadians(39))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter1.setVelocity(SHOOT_VELOCITY);
                shooter2.setVelocity(SHOOT_VELOCITY);
                follower.followPath(path1, 0.8, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) runShootSequence(path2, 2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    intake.setPower(-0.6);
                    follower.followPath(path5, 1.0, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(path6, 1.0, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(path7, 1.0, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) runShootSequence(path7, 6);
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(path12, 1.0, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    intake.setPower(-1.0);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(path13, 1.0, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    intake.setPower(-0.6);
                    follower.followPath(path2, 1.0, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(path3, 1.0, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) runShootSequence(path4, 12);
                break;
            case 12:
                follower.followPath(path8, 1.0, true);
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(path9, 1.0, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) runShootSequence(path10, -1);
                stopRobot();
                break;
        }
    }

    private void runShootSequence(PathChain nextPath, int nextState) {
        hood.setPosition(0.5);
        stopper.setPosition(0.3);
        intake.setPower(-1.0);
        if (pathTimer.getElapsedTimeSeconds() > 4.0) {
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
        follower.setStartingPose(new Pose(112.7, 136.737, Math.toRadians(90)));
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        hood = hardwareMap.get(Servo.class, "hood");
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        stopper.setDirection(Servo.Direction.FORWARD);
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