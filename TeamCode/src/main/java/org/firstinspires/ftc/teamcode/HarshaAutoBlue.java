package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.commands.Commands.*;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Ivy-command version of Far Blue auto.
 *
 * Path layout is the original BezierCurve-based route (ScorePreload, Intake1,
 * ScoreRow1, Intake2, Score2, IntakeSecretTunnel1/2, IntakeHP/ScoreHP).
 *
 * Shoot-cycle logic (settle time, then continuous feed for a fixed duration,
 * instead of pulsing the stopper per ball) and the path-following timeout
 * safety are ported from the state-machine version, just expressed as ivy
 * Commands instead of a switch-case loop.
 *
 * NOTE: single physical shooter wheel, driven by TWO motors ("shooter1" +
 * "shooter2") geared into that one wheel for more power/speed, not two
 * independent flywheels.
 */
@Autonomous(name = "HARSHA_AUTO_FAR", group = "Blue")
public class HarshaAutoBlue extends LinearOpMode {

    private static final String BUILD_TAG = "ACTION ROBOTIX PROGRAMMING";

    private Follower follower;
    private Timer pathTimer;

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake, turret;
    private Servo stopper, hood;

    // --- Shooter ---
    public static double SHOOTER_P = -830.000;
    public static double SHOOTER_F = 1.00000;
    private final double SHOOT_VELOCITY = 1460; // ticks/sec (NOT rpm)
    private static final int BALLS_PER_SHOOT_CYCLE = 3;
    private static final double SHOOTER_SETTLE_SECONDS = 0.75;
    private static final double THREE_BALL_FEED_SECONDS = 2.75;
    private static final double PATH_END_TIMEOUT_SECONDS = 4.5;

    // --- Stopper ---
    private static final double STOPPER_CLOSED = 1.0;
    private static final double STOPPER_OPEN = 0.0;

    // --- Turret PID ---
    public static double Kp = 0.035, Ki = 0.0, Kd = 0.0032;
    public static double TICKS_PER_RADIAN = 559.4190146;
    public static double GOAL_X = 6;
    public static double GOAL_Y = 138;
    private double lastTurretError = 0, turretIntegral = 0;

    private int ballsScored = 0;

    /// POINTS

    private final Pose StartPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose ScorePose = new Pose(60, 15, Math.toRadians(115));
    private final Pose ControlPoint1 = new Pose(35, 35, 0);
    private final Pose IntakeRow1 = new Pose(21, 30, Math.toRadians(180));

    private final Pose IntakeBox2 = new Pose(7, 3, Math.toRadians(180));

    private final Pose ControlPoint2 = new Pose(45.5, 35, 0);
    private final Pose Intake3 = new Pose(8, 30, Math.toRadians(180));

    private final Pose Intake4HP = new Pose(6, 13, Math.toRadians(180));

    private final Pose ControlPoint3 = new Pose(36.5, 32, 0);
    private final Pose Intake5 = new Pose(7, 22, Math.toRadians(180));

    private PathChain ScorePreload, Intake1, ScoreRow1, Intake2, Score2,
            IntakeSecretTunnel1, ScoreSecretTunnel1, IntakeHP, ScoreHP, IntakeSecretTunnel2;

    public void buildPaths() {
        ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(StartPose, ScorePose))
                .setLinearHeadingInterpolation(StartPose.getHeading(), ScorePose.getHeading())
                .build();

        Intake1 = follower.pathBuilder()
                .addPath(new BezierCurve(ScorePose, ControlPoint1, IntakeRow1))
                .setLinearHeadingInterpolation(ScorePose.getHeading(), IntakeRow1.getHeading())
                .build();

        ScoreRow1 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeRow1, ScorePose))
                .setLinearHeadingInterpolation(IntakeRow1.getHeading(), ScorePose.getHeading())
                .build();

        Intake2 = follower.pathBuilder()
                .addPath(new BezierLine(ScorePose, IntakeBox2))
                .setLinearHeadingInterpolation(ScorePose.getHeading(), IntakeBox2.getHeading())
                .build();

        Score2 = follower.pathBuilder()
                .addPath(new BezierLine(IntakeBox2, ScorePose))
                .setConstantHeadingInterpolation(IntakeBox2.getHeading())
                .build();

        IntakeSecretTunnel1 = follower.pathBuilder()
                .addPath(new BezierCurve(ScorePose, ControlPoint2, Intake3))
                .setLinearHeadingInterpolation(ScorePose.getHeading(), Intake3.getHeading())
                .build();

        ScoreSecretTunnel1 = follower.pathBuilder()
                .addPath(new BezierLine(Intake3, ScorePose))
                .setLinearHeadingInterpolation(Intake3.getHeading(), ScorePose.getHeading())
                .build();

        IntakeHP = follower.pathBuilder()
                .addPath(new BezierLine(ScorePose, Intake4HP))
                .setConstantHeadingInterpolation(Intake4HP.getHeading())
                .build();

        ScoreHP = follower.pathBuilder()
                .addPath(new BezierLine(Intake4HP, ScorePose))
                .setLinearHeadingInterpolation(Intake4HP.getHeading(), ScorePose.getHeading())
                .build();

        IntakeSecretTunnel2 = follower.pathBuilder()
                .addPath(new BezierCurve(ScorePose, ControlPoint3, Intake5))
                .setLinearHeadingInterpolation(ScorePose.getHeading(), Intake5.getHeading())
                .build();
    }

    // ---------------------------------------------------------------------
    // Intake
    // ---------------------------------------------------------------------
    private Command intakeOn() {
        return instant(() -> intake.setPower(-1.0));
    }

    private Command intakeOff() {
        return instant(() -> intake.setPower(0));
    }

    // ---------------------------------------------------------------------
    // Path following with a timeout safety net.
    // Pedro can stay "busy" if its final correction band is never satisfied
    // (e.g. one bad localization tick). Racing the follow against a timer
    // means a stuck path can never freeze the rest of autonomous.
    // ---------------------------------------------------------------------
    private Command followWithTimeout(PathChain path, boolean holdEnd) {
        return race(
                follow(follower, path, holdEnd),
                waitMs((long) (PATH_END_TIMEOUT_SECONDS * 1000))
        );
    }

    // ---------------------------------------------------------------------
    // Turret: continuous background aiming, ticks-based target (matches
    // turret.getCurrentPosition() directly instead of only chasing heading).
    // ---------------------------------------------------------------------
    private Command aimTurret() {
        return Command.build()
                .setExecute(() -> {
                    Pose pose = follower.getPose();
                    double fieldAngle = Math.atan2(GOAL_Y - pose.getY(), GOAL_X - pose.getX());
                    double targetRad = Math.atan2(
                            Math.sin(fieldAngle - pose.getHeading()),
                            Math.cos(fieldAngle - pose.getHeading())
                    );
                    double targetTicks = targetRad * TICKS_PER_RADIAN;

                    double error = targetTicks - turret.getCurrentPosition();
                    turretIntegral += error;
                    double derivative = error - lastTurretError;
                    double power = (Kp * error) + (Ki * turretIntegral) + (Kd * derivative);
                    lastTurretError = error;

                    turret.setPower(Math.max(-0.6, Math.min(0.6, power)));
                })
                .setEnd(endCondition -> turret.setPower(0))
                .requiring(turret);
    }

    // ---------------------------------------------------------------------
    // Shoot sequence at the current scoring pose.
    // Shooter is expected to already be spinning (started once at the top
    // of auto and left running, same as the settle-time version), so this
    // just handles hood, settle delay, feed window, and ball count.
    // ---------------------------------------------------------------------
    private Command shootSequence() {
        return sequential(
                instant(() -> hood.setPosition(0)),
                instant(() -> {
                    stopper.setPosition(STOPPER_CLOSED);
                    intake.setPower(0);
                }),
                waitMs((long) (SHOOTER_SETTLE_SECONDS * 1000)),
                instant(() -> {
                    stopper.setPosition(STOPPER_OPEN);
                    intake.setPower(-1.0);
                }),
                waitMs((long) (THREE_BALL_FEED_SECONDS * 1000)),
                instant(() -> {
                    stopper.setPosition(STOPPER_CLOSED);
                    intake.setPower(0);
                    ballsScored += BALLS_PER_SHOOT_CYCLE;
                })
        );
    }

    // ---------------------------------------------------------------------
    // Full routine
    // ---------------------------------------------------------------------
    public Command autoRoutine() {
        Command driveAndScore = sequential(
                instant(() -> {
                    shooter1.setVelocity(SHOOT_VELOCITY);
                    shooter2.setVelocity(SHOOT_VELOCITY);
                }), // spin up once, stays on

                followWithTimeout(ScorePreload, true),
                shootSequence(),

                intakeOn(),
                followWithTimeout(Intake1, true),
                intakeOff(),
                followWithTimeout(ScoreRow1, true),
                shootSequence(),

                intakeOn(),
                followWithTimeout(Intake2, true),
                intakeOff(),
                followWithTimeout(Score2, true),
                shootSequence(),

                intakeOn(),
                followWithTimeout(IntakeSecretTunnel1, true),
                intakeOff(),
                followWithTimeout(ScoreSecretTunnel1, true),
                shootSequence(),

                intakeOn(),
                followWithTimeout(IntakeHP, true),
                intakeOff(),
                followWithTimeout(ScoreHP, true),
                shootSequence(),

                intakeOn(),
                followWithTimeout(IntakeSecretTunnel2, true),
                intakeOff(),

                instant(() -> {
                    shooter1.setVelocity(0);
                    shooter2.setVelocity(0);
                    PoseStorage.currentPose = follower.getPose();
                })
        );

        // turret aims continuously in the background; ends when driveAndScore finishes
        return race(
                aimTurret(),
                driveAndScore
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(StartPose);

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        turret = hardwareMap.get(DcMotor.class, "turret");
        stopper = hardwareMap.get(Servo.class, "stopper");
        hood = hardwareMap.get(Servo.class, "hood");

        stopper.setDirection(Servo.Direction.REVERSE);
        stopper.setPosition(STOPPER_CLOSED);
        intake.setPower(0);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        buildPaths();

        telemetry.addData("AUTO BUILD", BUILD_TAG);
        telemetry.addLine("Select 'HARSHA_AUTO_FAR' on Driver Station");
        telemetry.addLine("Stopper is commanded CLOSED during init");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("AUTO BUILD", BUILD_TAG);
            telemetry.addData("Turret Pos", turret.getCurrentPosition());
            telemetry.addData("Shooter Vel", "%.0f / %.0f", shooter1.getVelocity(), shooter2.getVelocity());
            telemetry.addData("Balls Scored", ballsScored);
            telemetry.update();
        }

        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
        intake.setPower(0);
        turret.setPower(0);
    }
}