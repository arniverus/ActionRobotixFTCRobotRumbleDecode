package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Robot Centric Blue Close")
public class RobotCentricBlueClose extends OpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    Servo stopper, hood;
    DcMotor intake, turret;
    DcMotorEx shooter1, shooter2;

    double driveMultiplier = 1.0;
    BHI260IMU imu;

    boolean liftOn = false;
    double intakePower = 0.6;

    public static double SHOOTER_P = 133.0;
    public static double SHOOTER_F = 12.0;
    double shooterTargetVelocity = 0;

    public static double Kp  = 0.015;
    public static double Ki = 0.0;
    public static double Kd = 0.03;

    public static double targetAngleDeg = 0;
    public static double TICKS_PER_RADIAN = 559.4190146;

    double integral = 0;
    double lastError = 0;

    Follower follower;

    public static double START_X = 26.004;
    public static double START_Y = 69.550;
    public static double START_HEADING = Math.toRadians(180);

    public static double GOAL_X = 6;
    public static double GOAL_Y = 138;

    boolean autoAim = false;

    @Override
    public void init() {

        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        turret = hardwareMap.get(DcMotor.class, "turret");

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        stopper = hardwareMap.get(Servo.class, "stopper");
        hood = hardwareMap.get(Servo.class, "hood");

        intake = hardwareMap.get(DcMotor.class, "intake");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        stopper.setDirection(Servo.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf =
                new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(START_X, START_Y, START_HEADING)
        );

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize();
    }

    @Override
    public void loop() {

        follower.update();

        // Speed Modes
        if (gamepad1.a) driveMultiplier = 0.5;
        else if (gamepad1.x) driveMultiplier = 0.1;
        else if (gamepad1.b) driveMultiplier = 1.0;

        // =========================
        // DRIVETRAIN (ONLY STRAFE CHANGED)
        // =========================

        double y = -gamepad1.left_stick_y;

        // ✅ STRAFE FIXED (INVERTED)
        double x = -gamepad1.left_stick_x * 1.1;

        double turn = -gamepad1.right_stick_x;

        double fl = (y + x + turn) * driveMultiplier;
        double fr = (y - x - turn) * driveMultiplier;
        double bl = (y - x + turn) * driveMultiplier;
        double br = (y + x - turn) * driveMultiplier;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);

        // AUTO AIM
        if (gamepad2.right_trigger > 0.5) autoAim = true;
        if (gamepad2.left_trigger > 0.5) autoAim = false;

        if (autoAim) {
            Pose pose = follower.getPose();

            double angleToGoal =
                    Math.atan2(GOAL_Y - pose.getY(),
                            GOAL_X - pose.getX());

            double turretTargetRad =
                    angleToGoal - pose.getHeading();

            targetAngleDeg = Math.toDegrees(turretTargetRad);
        }

        // TURRET PID
        int currentTicks = turret.getCurrentPosition();
        double targetTicks =
                targetAngleDeg * Math.PI / 180.0 * TICKS_PER_RADIAN;

        double error = targetTicks - currentTicks;

        integral += error;
        double derivative = error - lastError;

        double power =
                Kp * error +
                        Ki * integral +
                        Kd * derivative;

        power = Math.max(-0.6, Math.min(0.6, power));
        turret.setPower(power);
        lastError = error;

        // INTAKE
        if (gamepad2.right_bumper) liftOn = true;
        if (gamepad2.left_bumper) liftOn = false;

        if (gamepad2.left_bumper)
            intake.setPower(0.3);
        else if (liftOn)
            intake.setPower(-intakePower);
        else
            intake.setPower(0);

        // STOPPER
        if (gamepad2.dpad_right) {
            stopper.setPosition(0.3);
            autoAim = false;
        }
        if (gamepad2.dpad_up) stopper.setPosition(0.7);

        // SHOOTER PRESETS
        if (gamepad2.x) {
            hood.setPosition(0);
            shooterTargetVelocity = 1520;
        }
        else if (gamepad2.b) shooterTargetVelocity = 1800;
        else if (gamepad2.y) {
            hood.setPosition(0.2);
            shooterTargetVelocity = 1100;
        }
        else if (gamepad2.a) {
            hood.setPosition(0.1);
            shooterTargetVelocity = 1400;
        }

        shooter1.setVelocity(shooterTargetVelocity);
        shooter2.setVelocity(shooterTargetVelocity);

        Pose pose = follower.getPose();

        telemetry.addData("Robot X", pose.getX());
        telemetry.addData("Robot Y", pose.getY());
        telemetry.addData("Robot Heading", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}