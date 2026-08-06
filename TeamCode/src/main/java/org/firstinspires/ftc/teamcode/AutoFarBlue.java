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

@Autonomous(name = "Auto Far Blue", group = "Blue")
public class AutoFarBlue extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake, turret;
    private Servo stopper, hood;

    // PIDF Constants
    public static double SHOOTER_P = -830.000;
    public static double SHOOTER_F = 1.00000;
    private final double SHOOT_VELOCITY = 1540;

    // Turret PID Constants
    public static double Kp = 0.035, Ki = 0.0, Kd = 0.0032;
    public static double TICKS_PER_RADIAN = 559.4190146;

    // Blue Goal Coordinates
    public static double GOAL_X = 6;
    public static double GOAL_Y = 138;

    private double lastTurretError = 0, turretIntegral = 0;
    private final double HEADING_SHOOT = Math.toRadians(125);
    private final double HEADING_INTAKE = Math.toRadians(180);

    private boolean isFinished = false; // Flag to trigger zero-reset

    private PathChain p1_preload, p2_intakeReady1, p3_intake1, p4_shoot2,
            p5_hpReady1, p6_hpIntake1, p6_hpReset, p7_shoot3, p11_park;

    public void buildPaths() {
        p1_preload = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55.25, 7.25), new Pose(59.12, 11.02)))
                .setLinearHeadingInterpolation(Math.toRadians(90), HEADING_SHOOT)
                .build();

        p2_intakeReady1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.12, 10.5), new Pose(57.758, 19)))
                .setLinearHeadingInterpolation(HEADING_SHOOT, HEADING_INTAKE)
                .build();

        p3_intake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(57.758, 22), new Pose(45,15)))
                .setLinearHeadingInterpolation(HEADING_INTAKE, HEADING_INTAKE)
                .build();

        p4_shoot2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(45, 15), new Pose(59.12, 11.02)))
                .setLinearHeadingInterpolation(HEADING_INTAKE, HEADING_SHOOT)
                .build();

        p5_hpReady1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.12, 11.02), new Pose(38.342, 1)))
                .setLinearHeadingInterpolation(HEADING_SHOOT, HEADING_INTAKE)
                .build();

        p6_hpIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(38.342, 1), new Pose(13, 1)))
                .setLinearHeadingInterpolation(HEADING_INTAKE, HEADING_INTAKE)
                .build();

        p6_hpReset = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(13, 1), new Pose(25, 1)))
                .setLinearHeadingInterpolation(HEADING_INTAKE, HEADING_INTAKE)
                .build();

        p7_shoot3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(13, 1), new Pose(59.12, 11.02)))
                .setLinearHeadingInterpolation(HEADING_INTAKE, HEADING_SHOOT)
                .build();

        p11_park = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(59.12, 11.02), new Pose(38.342, 2)))
                .setLinearHeadingInterpolation(HEADING_SHOOT, HEADING_INTAKE)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter1.setVelocity(SHOOT_VELOCITY);
                shooter2.setVelocity(SHOOT_VELOCITY);
                follower.followPath(p1_preload, 0.7, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) runShootSequence(p2_intakeReady1, 2);
                break;
            case 2:
                if (!follower.isBusy()) {
                    intake.setPower(-1.0);
                    follower.followPath(p3_intake1, 0.7, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p4_shoot2, 0.7, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) runShootSequence(p5_hpReady1, 5);
                break;
            case 5: // HP Grab 1
                if (!follower.isBusy()) {
                    intake.setPower(-1.0);
                    follower.followPath(p6_hpIntake1, 0.7, true);
                    setPathState(6);
                }
                break;
            case 6: // HP Reset
                if (!follower.isBusy()) {
                    follower.followPath(p6_hpReset, 0.8, true);
                    setPathState(7);
                }
                break;
            case 7: // HP Grab 2
                if (!follower.isBusy()) {
                    follower.followPath(p6_hpIntake1, 0.7, true);
                    setPathState(8);
                }
                break;
            case 8: // Return to shoot
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p7_shoot3, 1.0, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) runShootSequence(p11_park, 10);
                break;
            case 10:
                if (!follower.isBusy()) stopRobot();
                break;
        }
    }

    private void updateTurret() {
        double targetTicks;
        if (isFinished) {
            targetTicks = 0; // Return to center at end of auto
        } else {
            Pose pose = follower.getPose();
            double fieldAngle = Math.atan2(GOAL_Y - pose.getY(), GOAL_X - pose.getX());
            double targetRad = Math.atan2(Math.sin(fieldAngle - pose.getHeading()), Math.cos(fieldAngle - pose.getHeading()));
            targetTicks = targetRad * TICKS_PER_RADIAN;
        }

        double error = targetTicks - turret.getCurrentPosition();
        turretIntegral += error;
        double derivative = error - lastTurretError;
        double power = (Kp * error) + (Ki * turretIntegral) + (Kd * derivative);

        turret.setPower(Math.max(-0.6, Math.min(0.6, power)));
        lastTurretError = error;
    }

    private void runShootSequence(PathChain nextPath, int nextState) {
        hood.setPosition(0);
        double elapsed = pathTimer.getElapsedTimeSeconds();

        if (elapsed < 4.0) {
            stopper.setPosition(0.3); // Open
            intake.setPower(-1.0);
        } else {
            stopper.setPosition(0.9); // Closed
            intake.setPower(0);
            if (nextPath != null) follower.followPath(nextPath, 0.8, true);
            setPathState(nextState);
        }
    }

    private void stopRobot() {
        intake.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);

        // Save pose for TeleOp
        PoseStorage.currentPose = follower.getPose();

        isFinished = true; // Signal turret to return to 0
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
        follower.setStartingPose(new Pose(55.25, 7.25, Math.toRadians(90)));

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "turret");
        stopper = hardwareMap.get(Servo.class, "stopper");
        hood = hardwareMap.get(Servo.class, "hood");

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