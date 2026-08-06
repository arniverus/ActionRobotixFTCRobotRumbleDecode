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

@Autonomous(name = "AutoCloseBlue", group = "Autonomous")
public class AutoCloseBlue extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo stopper;

    private final double SHOOT_VELOCITY = 1055;
    double SHOOTER_P = -830.000;
    double SHOOTER_F = 1.00000;

    // PathChains from your custom list
    private PathChain shootreload, intakeready1, intake1, shootintake1,
            intakeready2, intake2, shootintake2, intakeready3,
            intake3, shootintake3, park;

    public void buildPaths() {
        shootreload = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(34.323349922048166, 133.0045179701613), new Pose(36.733, 106.611)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(139))
                .build();

        intakeready1 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(36.733, 106.611), new Pose(44.886, 75.5)))
                .setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))
                .build();

        intake1 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(43, 75.5), new Pose(14, 75.5)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        shootintake1 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(14, 75.5), new Pose(36.739, 106.528)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))
                .build();

        intakeready2 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(36.739, 106.528), new Pose(51.461, 51.5)))
                .setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))
                .build();

        intake2 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(51.461, 51.5), new Pose(6.543, 51.5)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        shootintake2 = follower.pathBuilder()
                .addPath(new BezierCurve(

                        new Pose(6.543, 51.5),
                        new Pose(40.180, 51.5), // Control Point: Adjust this Y up/down if you still clip the gate
                        new Pose(36.770, 106.368)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))
                .build();

        intakeready3 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(36.770, 106.368), new Pose(45.372, 28)))
                .setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))
                .build();

        intake3 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(45.372, 28), new Pose(7.8, 28)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();


        shootintake3 = follower.pathBuilder()
                .addPath(new BezierCurve(

                        new Pose(7.8, 28),
                        new Pose(15, 35), // Control Point: Adjust this Y up/down if you still clip the gate
                        new Pose(36.967, 106.082)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(139))
                .build();


        park = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(36.967, 106.082), new Pose(26.004, 69.550)))
                .setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(180))
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
            case 1: // Shoot Preload and transition to Intake Ready 1
                if (!follower.isBusy()) runShootSequence(intakeready1, 2);
                break;
            case 2: // Move to Intake 1
                if (!follower.isBusy()) {
                    intake.setPower(-0.6);
                    follower.followPath(intake1, 0.8, true);
                    setPathState(3);
                }
                break;
            case 3: // Wait for intake completion then move to Shoot 1
                if ( pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(shootintake1, 0.8, true);
                    setPathState(4);
                }
                break;
            case 4: // Shoot 1 and transition to Intake Ready 2
                if (!follower.isBusy()) runShootSequence(intakeready2, 5);
                break;
            case 5: // Move to Intake 2
                if (!follower.isBusy()) {
                    intake.setPower(-0.6);
                    follower.followPath(intake2, 0.8, true);
                    setPathState(6);
                }
                break;
            case 6: // Wait for intake then move to Shoot 2 (Curve Path)
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(shootintake2, 0.8, true);
                    setPathState(7);
                }
                break;
            case 7: // Shoot 2 and transition to Intake Ready 3
                if (!follower.isBusy()) runShootSequence(intakeready3, 8);
                break;
            case 8: // Move to Intake 3
                if (!follower.isBusy()) {
                    intake.setPower(-0.6);
                    follower.followPath(intake3, 0.8, true);
                    setPathState(9);
                }
                break;
            case 9: // Wait for intake then move to Shoot 3
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 1.0) {
                    follower.followPath(shootintake3, 0.8, true);
                    setPathState(10);
                }
                break;
            case 10: // Shoot 3 and transition to Park
                if (!follower.isBusy()) runShootSequence(park, 11);
                break;
            case 11: // Follow Park path
                if (!follower.isBusy()) {
                    follower.followPath(park, 1.0, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) stopRobot();
                break;
        }
    }

    private void runShootSequence(PathChain nextPath, int nextState) {
        stopper.setPosition(0.3);
        intake.setPower(-0.6);
        // Using your 4-second wait logic for shooting
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

        // Start pose matched to the beginning of shootreload
        follower.setStartingPose(new Pose(34.323349922048166, 133.0045179701613, Math.toRadians(90)));

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stopper = hardwareMap.get(Servo.class, "stopper");

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
            telemetry.addData("T-Seconds", pathTimer.getElapsedTimeSeconds());
            telemetry.update();
        }
    }
}