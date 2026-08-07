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

@Autonomous(name = "Far Red Double Intake", group = "Red")
public class AutoFarRedDoubleHP extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake, turret;
    private Servo stopper, hood;

    // Shooter Constants
    public static double SHOOTER_P = -830.000;
    public static double SHOOTER_F = 1.00000;
    private final double SHOOT_VELOCITY = 1550;

    // Turret PID Constants
    public static double Kp = 0.035, Ki = 0.0, Kd = 0.0032;
    public static double TICKS_PER_RADIAN = 559.4190146;
    public static double GOAL_X = 133, GOAL_Y = 133;

    private double lastTurretError = 0, turretIntegral = 0;
    private PathChain initialShoot, goToIntakeReady, doIntake, shootSecond,
            intakereadyhp, intakehp, intakehpReset, shoothp, park;

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
                .addPath(new BezierLine(new Pose(89.272, 35), new Pose(142, 35)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        shootSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(142, 35), new Pose(90.880, 14.650)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(72))
                .build();

        intakereadyhp = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.880, 14.650), new Pose(110.875, 8.25)))
                .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(0))
                .build();

        intakehp = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(110.875, 8.25), new Pose(143, 8.25)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        intakehpReset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(143, 8.25), new Pose(113, 8.25)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        shoothp = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(143, 8.25), new Pose(90.880, 14.650)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(72))
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(90.880, 14.650), new Pose(110.875, 17)))
                .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(72))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Preload
                shooter1.setVelocity(SHOOT_VELOCITY);
                shooter2.setVelocity(SHOOT_VELOCITY);
                follower.followPath(initialShoot, 0.8, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) runShootSequence(goToIntakeReady, 2);
                break;

            case 2: // Spike Mark Intake
                if (!follower.isBusy()) {
                    intake.setPower(-0.8);
                    follower.followPath(doIntake, 1.0, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(shootSecond, 0.8, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) runShootSequence(intakereadyhp, 5);
                break;

            // --- HP CYCLE 1 (With Reset) ---
            case 5: // Grab 1
                if (!follower.isBusy()) {
                    intake.setPower(-1.0);
                    follower.followPath(intakehp, 0.7, true);
                    setPathState(6);
                }
                break;

            case 6: // Back to Ready (Reset)
                if (!follower.isBusy()) {
                    intake.setPower(-0.6);
                    follower.followPath(intakehpReset, 0.6, true);
                    setPathState(7);
                }
                break;

            case 7: // Grab 2
                if (!follower.isBusy()) {
                    follower.followPath(intakehp, 0.7, true);
                    setPathState(8);
                }
                break;

            case 8: // Return to shoot
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(shoothp, 0.8, true);
                    setPathState(9);
                }
                break;

            case 9: // Shoot HP 1
                intake.setPower(-0.6);
                if (!follower.isBusy()) runShootSequence(intakereadyhp, 10);
                break;

            // --- HP CYCLE 2 (Optimized: No Reset) ---
            case 10: // Drive into HP and stay there to grab 3 & 4
                if (!follower.isBusy()) {
                    intake.setPower(-1.0);
                    follower.followPath(intakehp, 0.7, true);
                    setPathState(11);
                }
                break;

            case 11: // Wait briefly at the wall to ensure both pieces are grabbed
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    // We skip intakehpReset and go straight to shooting
                    setPathState(13);
                }
                break;

            // State 12 is skipped in the logic above
            case 13: // Return to shoot
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(shoothp, 0.7, true);
                    setPathState(14);
                }
                break;

            case 14: // Final Shoot
                if (!follower.isBusy()) runShootSequence(park, 15);
                break;

            case 15:
                if (!follower.isBusy()) stopRobot();
                break;
        }
    }

    private void runShootSequence(PathChain nextPath, int nextState) {
        hood.setPosition(0);
        double elapsed = pathTimer.getElapsedTimeSeconds();

        if (elapsed < 3.5) {
            stopper.setPosition(0.3); // Open
            intake.setPower(-1.0);
        } else {
            stopper.setPosition(0.7); // Closed
            intake.setPower(0);
            if (nextPath != null) follower.followPath(nextPath, 0.8, true);
            setPathState(nextState);
        }
    }

    private void updateTurret() {
        double targetRad;
        if (pathState == -1) {
            targetRad = 0;
        } else {
            Pose pose = follower.getPose();
            double fieldAngle = Math.atan2(GOAL_Y - pose.getY(), GOAL_X - pose.getX());
            targetRad = Math.atan2(Math.sin(fieldAngle - pose.getHeading()), Math.cos(fieldAngle - pose.getHeading()));
        }

        double error = (targetRad * TICKS_PER_RADIAN) - turret.getCurrentPosition();
        turretIntegral += error;
        double derivative = error - lastTurretError;
        double power = (Kp * error) + (Ki * turretIntegral) + (Kd * derivative);

        turret.setPower(Math.max(-0.6, Math.min(0.6, power)));
        lastTurretError = error;
    }


    private void stopRobot() {
        intake.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);

        // SAVE POSE FOR TELEOP
        PoseStorage.currentPose = follower.getPose();

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
        follower.setStartingPose(new Pose(88.348, 7.169, Math.toRadians(90)));

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "turret");
        stopper = hardwareMap.get(Servo.class, "stopper");
        hood = hardwareMap.get(Servo.class, "hood");

        stopper.setDirection(Servo.Direction.FORWARD);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        buildPaths();
        waitForStart();

        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();
            updateTurret();

            telemetry.addData("State", pathState);
            telemetry.addData("Turret Pos", turret.getCurrentPosition());
            telemetry.update();
        }
    }
}