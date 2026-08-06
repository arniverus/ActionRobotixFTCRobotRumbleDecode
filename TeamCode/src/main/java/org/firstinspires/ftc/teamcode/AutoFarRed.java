package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "Far Red Fixed", group = "Autonomous")
public class AutoFarRed extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo stopper, hood;

    public static double SHOOTER_P = -830.000;
    public static double SHOOTER_F = 1.00000;
    private final double SHOOT_VELOCITY = 1530;

    private PathChain initialShoot, goToIntakeReady, doIntake, shootSecond, intakereadyhp, intakehp, shoothp, park;

    public void buildPaths() {
        initialShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(88.348, 7.169), new Pose(90.880, 14.650)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72))
                .build();

        goToIntakeReady = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.880, 14.650), new Pose(89.272, 35)))
                .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(0))
                .build();

        doIntake = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(89.272, 35), new Pose(143, 35)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        shootSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(143, 35), new Pose(90.880, 14.650)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(72))
                .build();

        intakereadyhp = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.880, 14.650), new Pose(110.875, 9.25)))
                .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(0))
                .build();

        intakehp = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(110.875, 9.25), new Pose(143, 9.25)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        shoothp = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(144, 9.25), new Pose(90.880, 14.650)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(72))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.880, 14.650), new Pose(110.875, 11.325)))
                .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(0))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Rev and Move to first shot
                shooter1.setVelocity(SHOOT_VELOCITY);
                shooter2.setVelocity(SHOOT_VELOCITY);
                follower.followPath(initialShoot, 0.7, true);
                setPathState(1);
                break;

            case 1: // Shoot Preload
                if (!follower.isBusy()) {
                    runShootSequence(goToIntakeReady, 2);
                }
                break;

            case 2: // Intake First Sample
                if (!follower.isBusy()) {
                    intake.setPower(-0.8); // High power to grab
                    follower.followPath(doIntake, 0.7, true);
                    setPathState(3);
                }
                break;

            case 3: // Move to second shot
                if (!follower.isBusy()) {
                    intake.setPower(0); // Stop intake once reached
                    follower.followPath(shootSecond, 0.6, true);
                    setPathState(4);
                }
                break;

            case 4: // Shoot Second Sample
                if (!follower.isBusy()) {
                    runShootSequence(intakereadyhp, 5);
                }
                break;

            case 5: // Go to HP Intake Ready
                if (!follower.isBusy()) {
                    intake.setPower(-0.8);
                    follower.followPath(intakehp, 0.7, true);
                    setPathState(6);
                }
                break;

            case 6: // Back to Shoot HP
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(shoothp, 0.6, true);
                    setPathState(7);
                }
                break;

            case 7: // Shoot HP Sample
                if (!follower.isBusy()) {
                    runShootSequence(intakereadyhp, 8);
                }
                break;

                case 8: // Shoot Second Sample
                    if (!follower.isBusy()) {
                        intake.setPower(-0.8);
                        follower.followPath(intakehp, 0.7, true);
                        setPathState(9);
                    }
                    break;

            case 9:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(shoothp, 0.7, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    runShootSequence(park, 11);
                }
                break;

            case 12: // Final Park
                if (!follower.isBusy()) {
                    shooter1.setVelocity(0);
                    shooter2.setVelocity(0);
                    stopRobot();
                }
                break;
        }
    }

    /**
     * Reusable sequence to trigger the stopper and intake to fire a ball.
     */
    private void runShootSequence(PathChain nextPath, int nextState) {
        hood.setPosition(0.7);
        intake.setPower(-1.0);
        stopper.setPosition(0.7);
        if (pathTimer.getElapsedTimeSeconds() > 4.0) {
            intake.setPower(0);
            stopper.setPosition(0.7);
            if (nextPath != null) follower.followPath(nextPath, 0.8, true);
            setPathState(nextState);
        }
    }

    private void stopRobot() {
        intake.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
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
        // Correcting starting pose to match the buildPaths initial Shoot start point
        follower.setStartingPose(new Pose(88.348, 7.169, Math.toRadians(90)));

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");
        hood = hardwareMap.get(Servo.class, "hood");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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