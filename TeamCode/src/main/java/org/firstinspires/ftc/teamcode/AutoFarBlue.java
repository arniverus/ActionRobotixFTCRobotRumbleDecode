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

@Autonomous(name = "Far Blue Auto", group = "Blue")
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
    private static final int BALLS_PER_SHOOT_CYCLE = 3;
    private static final int PLANNED_SHOT_COUNT = 15;
    private static final double MIN_SHOOTER_SPINUP_SECONDS = 0.45;
    private static final double SHOOTER_READY_TOLERANCE = 75.0;
    private static final double BALL_FEED_SECONDS = 0.45;
    private static final double BALL_SETTLE_SECONDS = 0.20;

    // Turret PID Constants
    public static double Kp = 0.035, Ki = 0.0, Kd = 0.0032;
    public static double TICKS_PER_RADIAN = 559.4190146;

    // Blue Goal Coordinates
    public static double GOAL_X = 6;
    public static double GOAL_Y = 138;

    private double lastTurretError = 0, turretIntegral = 0;
    private final double HEADING_SHOOT = Math.toRadians(115);

    // Stopper positions
    private final double STOPPER_CLOSED = 0.9; // holding balls in during transit
    private final double STOPPER_OPEN = 0.3;   // feeding into shooter

    private boolean isFinished = false; // Flag to trigger zero-reset
    private int ballsScored = 0;
    private boolean shootSequenceStarted = false;
    private double shootFeedStartSeconds = -1.0;
    private PathChain shootNextPath;
    private int shootNextState;

    // Drive legs (tangent heading, no forced turn at the end)
    private PathChain p1_driveToShoot,
            p2_driveToIntake1, p3_driveToShoot1,
            p4_driveToHP, p5_hpAdjust, p6_driveToShoot2,
            p7_driveToIntake2, p8_driveToShoot3,
            p9_driveToIntake3, p10_driveToShoot4;

    // Turn-in-place legs (change heading to face the goal, then shoot)
    private PathChain t1_turnToShoot, t2_turnToShoot, t3_turnToShoot,
            t4_turnToShoot, t5_turnToShoot;

    public void buildPaths() {
        // ---- Preload ----
        p1_driveToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(55.25, 7.25), new Pose(56.000, 34.000)))
                .setTangentHeadingInterpolation()
                .build();

        t1_turnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 34.000), new Pose(56.000, 34.000)))
                .setLinearHeadingInterpolation(Math.toRadians(90), HEADING_SHOOT)
                .build();

        // ---- Cycle 1 ----
        p2_driveToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 34.000), new Pose(11.000, 35.000)))
                .setTangentHeadingInterpolation()
                .build();

        p3_driveToShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(11.000, 35.000), new Pose(56.000, 34.000)))
                .setTangentHeadingInterpolation()
                .build();

        t2_turnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 34.000), new Pose(56.000, 34.000)))
                .setConstantHeadingInterpolation(HEADING_SHOOT)
                .build();

        // ---- Cycle 2 (HP grab with reposition) ----
        p4_driveToHP = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 34.000), new Pose(6.000, 12.000)))
                .setTangentHeadingInterpolation()
                .build();

        p5_hpAdjust = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(6.000, 12.000), new Pose(20.000, 18.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        p6_driveToShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(20.000, 18.000), new Pose(56.000, 34.000)))
                .setTangentHeadingInterpolation()
                .build();

        t3_turnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 34.000), new Pose(56.000, 34.000)))
                .setConstantHeadingInterpolation(HEADING_SHOOT)
                .build();

        // ---- Cycle 3 ----
        p7_driveToIntake2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 34.000), new Pose(4.000, 34.000)))
                .setTangentHeadingInterpolation()
                .build();

        p8_driveToShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(4.000, 34.000), new Pose(56.000, 34.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        t4_turnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 34.000), new Pose(56.000, 34.000)))
                .setConstantHeadingInterpolation(HEADING_SHOOT)
                .build();

        // ---- Cycle 4 ----
        p9_driveToIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 34.000), new Pose(7.000, 18.000)))
                .setTangentHeadingInterpolation()
                .build();

        p10_driveToShoot4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(7.000, 18.000), new Pose(56.000, 34.000)))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        t5_turnToShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(56.000, 34.000), new Pose(56.000, 34.000)))
                .setConstantHeadingInterpolation(HEADING_SHOOT)
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ---- Preload ----
            case 0: // Drive to shoot pose, stopper closed holding balls
                shooter1.setVelocity(SHOOT_VELOCITY);
                shooter2.setVelocity(SHOOT_VELOCITY);
                stopper.setPosition(STOPPER_CLOSED);
                follower.followPath(p1_driveToShoot, 0.7, true);
                setPathState(1);
                break;
            case 1: // Arrived, now turn to face the goal
                if (!follower.isBusy()) {
                    follower.followPath(t1_turnToShoot, 0.7, true);
                    setPathState(2);
                }
                break;
            case 2: // Heading set, shoot preload
                if (!follower.isBusy()) runShootSequence(p2_driveToIntake1, 3);
                break;

            // ---- Cycle 1 ----
            case 3: // Drive to intake, stopper closed, intake running
                stopper.setPosition(STOPPER_CLOSED);
                intake.setPower(-1.0);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p3_driveToShoot1, 0.7, true);
                    setPathState(4);
                }
                break;
            case 4: // Arrived, turn to face the goal
                if (!follower.isBusy()) {
                    follower.followPath(t2_turnToShoot, 0.7, true);
                    setPathState(5);
                }
                break;
            case 5: // Heading set, shoot cycle 1
                if (!follower.isBusy()) runShootSequence(p4_driveToHP, 6);
                break;

            // ---- Cycle 2 (HP grab) ----
            case 6: // Drive to HP zone, stopper closed, intake running
                stopper.setPosition(STOPPER_CLOSED);
                intake.setPower(-1.0);
                if (!follower.isBusy()) {
                    follower.followPath(p5_hpAdjust, 0.7, true);
                    setPathState(7);
                }
                break;
            case 7: // Finish HP grab, drive to shoot
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p6_driveToShoot2, 0.7, true);
                    setPathState(8);
                }
                break;
            case 8: // Arrived, turn to face the goal
                if (!follower.isBusy()) {
                    follower.followPath(t3_turnToShoot, 0.7, true);
                    setPathState(9);
                }
                break;
            case 9: // Heading set, shoot cycle 2
                if (!follower.isBusy()) runShootSequence(p7_driveToIntake2, 10);
                break;

            // ---- Cycle 3 ----
            case 10: // Drive to intake, stopper closed, intake running
                stopper.setPosition(STOPPER_CLOSED);
                intake.setPower(-1.0);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p8_driveToShoot3, 0.7, true);
                    setPathState(11);
                }
                break;
            case 11: // Arrived, turn to face the goal
                if (!follower.isBusy()) {
                    follower.followPath(t4_turnToShoot, 0.7, true);
                    setPathState(12);
                }
                break;
            case 12: // Heading set, shoot cycle 3
                if (!follower.isBusy()) runShootSequence(p9_driveToIntake3, 13);
                break;

            // ---- Cycle 4 ----
            case 13: // Drive to intake, stopper closed, intake running
                stopper.setPosition(STOPPER_CLOSED);
                intake.setPower(-1.0);
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    follower.followPath(p10_driveToShoot4, 0.7, true);
                    setPathState(14);
                }
                break;
            case 14: // Arrived, turn to face the goal
                if (!follower.isBusy()) {
                    follower.followPath(t5_turnToShoot, 0.7, true);
                    setPathState(15);
                }
                break;
            case 15: // Heading set, shoot cycle 4 (final), then stop
                if (!follower.isBusy()) runShootSequence(null, 16);
                break;

            case 16:
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

        // Initialize once per arrival at a shooting pose.  The timer must start here,
        // not while the robot is still driving or turning into the shot.
        if (!shootSequenceStarted) {
            shootSequenceStarted = true;
            shootFeedStartSeconds = -1.0;
            shootNextPath = nextPath;
            shootNextState = nextState;
            pathTimer.resetTimer();
        }

        double elapsed = pathTimer.getElapsedTimeSeconds();
        boolean shooterReady = Math.abs(shooter1.getVelocity()) >= SHOOT_VELOCITY - SHOOTER_READY_TOLERANCE
                && Math.abs(shooter2.getVelocity()) >= SHOOT_VELOCITY - SHOOTER_READY_TOLERANCE;

        // Keep the gate closed until both flywheels have reached speed.  Starting the
        // shot timer only after that point prevents a slow spin-up from skipping balls.
        if (shootFeedStartSeconds < 0.0
                && (elapsed < MIN_SHOOTER_SPINUP_SECONDS || !shooterReady)) {
            stopper.setPosition(STOPPER_CLOSED);
            intake.setPower(0);
            return;
        }

        if (shootFeedStartSeconds < 0.0) shootFeedStartSeconds = elapsed;

        double shotCycleSeconds = BALL_FEED_SECONDS + BALL_SETTLE_SECONDS;
        double feedElapsed = elapsed - shootFeedStartSeconds;
        int shotIndex = (int) (feedElapsed / shotCycleSeconds);

        if (shotIndex < BALLS_PER_SHOOT_CYCLE) {
            double shotElapsed = feedElapsed % shotCycleSeconds;
            boolean feedingBall = shotElapsed < BALL_FEED_SECONDS;
            stopper.setPosition(feedingBall ? STOPPER_OPEN : STOPPER_CLOSED);
            // Only run the intake while the stopper is open, so exactly one timed
            // feed window is used for each planned ball.
            intake.setPower(feedingBall ? -1.0 : 0);
        } else {
            ballsScored += BALLS_PER_SHOOT_CYCLE;
            stopper.setPosition(STOPPER_CLOSED);
            intake.setPower(0);
            shootSequenceStarted = false;
            shootFeedStartSeconds = -1.0;

            if (shootNextPath != null) follower.followPath(shootNextPath, 0.8, true);
            setPathState(shootNextState);
        }
    }

    private void stopRobot() {
        intake.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
        stopper.setPosition(STOPPER_CLOSED);

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

        // Set the safe state before Start is pressed.  State 0 keeps it closed for
        // the entire first drive, so the robot always moves before it begins feeding.
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
        waitForStart();

        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();
            updateTurret();
            telemetry.addData("State", pathState);
            telemetry.addData("Turret Pos", turret.getCurrentPosition());
            telemetry.addData("Shots Planned/Timed", "%d / %d", PLANNED_SHOT_COUNT, ballsScored);
            telemetry.addData("Stopper", shootSequenceStarted ? "shoot sequence" : "closed/transport");
            telemetry.update();
        }
    }
}
