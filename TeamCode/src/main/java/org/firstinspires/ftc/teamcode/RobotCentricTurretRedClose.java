package org.firstinspires.ftc.teamcode;

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

@TeleOp(name = "Red Close", group = "TeleOp")
public class RobotCentricTurretRedClose extends OpMode {

    // Hardware
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo stopper, hood;
    private DcMotor intake, turret;
    private DcMotorEx shooter1, shooter2;

    // Settings
    private Follower follower;
    private double driveMultiplier = 1.0;
    private boolean liftOn = false;
    private boolean autoAim = false;

    // Shooter PIDF
    public static double SHOOTER_P = -830.000;
    public static double SHOOTER_F = 1.00000;
    private double shooterTargetVelocity = 0;

    // Turret PID
    public static double Kp = 0.035;
    public static double Ki = 0.0;
    public static double Kd = 0.0032;
    public static double TICKS_PER_RADIAN = 559.4190146;

    // Goal (Red Close)
    public static double GOAL_X = 133;
    public static double GOAL_Y = 133;

    private double targetAngleDeg = 0;
    private double autoAimOffset = 0; // The persistent "nudge"
    private double integral = 0;
    private double lastError = 0;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        turret   = hardwareMap.get(DcMotor.class, "turret");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        stopper  = hardwareMap.get(Servo.class, "stopper");
        hood     = hardwareMap.get(Servo.class, "hood");
        intake   = hardwareMap.get(DcMotor.class, "intake");

        stopper.setDirection(Servo.Direction.FORWARD);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);
    }

    @Override
    public void loop() {
        follower.update();

        // 1. DRIVETRAIN
        if (gamepad1.a) driveMultiplier = 0.5;
        else if (gamepad1.x) driveMultiplier = 0.1;
        else if (gamepad1.b) driveMultiplier = 1.0;

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double turn = -gamepad1.right_stick_x;

        double fl = (y + x + turn) * driveMultiplier;
        double fr = (y - x - turn) * driveMultiplier;
        double bl = (y - x + turn) * driveMultiplier;
        double br = (y + x - turn) * driveMultiplier;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);

        // 2. AUTO AIM & DPAD NUDGE
        if (gamepad2.right_trigger > 0.5) autoAim = true;
        if (gamepad2.left_trigger > 0.5) autoAim = false;

        // DPAD MANUAL NUDGE (Adjusts the target angle slightly)
        if (gamepad2.dpad_left) {
            autoAimOffset += 0.5; // Turn left slightly
        } else if (gamepad2.dpad_right) {
            autoAimOffset -= 0.5; // Turn right slightly
        }

        // Click Right Stick to reset the nudge
        if (gamepad2.right_stick_button) autoAimOffset = 0;

        if (autoAim) {
            Pose pose = follower.getPose();
            double fieldAngle = Math.atan2(GOAL_Y - pose.getY(), GOAL_X - pose.getX());
            double targetRad = fieldAngle - pose.getHeading();
            targetRad = Math.atan2(Math.sin(targetRad), Math.cos(targetRad)); // Normalize

            targetAngleDeg = Math.toDegrees(targetRad) + autoAimOffset;
        } else {
            // Manual stick control if AutoAim is off
            if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                autoAimOffset -= gamepad2.right_stick_x * 3.0;
            }
            targetAngleDeg = autoAimOffset;
        }

        // 3. TURRET PID EXECUTION
        int currentTicks = turret.getCurrentPosition();
        double targetTicks = Math.toRadians(targetAngleDeg) * TICKS_PER_RADIAN;
        double error = targetTicks - currentTicks;

        integral += error;
        double derivative = error - lastError;
        double turretPower = (Kp * error) + (Ki * integral) + (Kd * derivative);

        turretPower = Math.max(-0.6, Math.min(0.6, turretPower));
        turret.setPower(turretPower);
        lastError = error;

        // 4. INTAKE
        if (gamepad2.right_bumper) liftOn = true;
        if (gamepad2.left_bumper) {
            liftOn = false;
            intake.setPower(0.3); // Outtake
        } else if (liftOn) {
            intake.setPower(-1.0); // Intake
        } else {
            intake.setPower(0);
        }

        if (gamepad2.x) {
            hood.setPosition(0);
            shooterTargetVelocity = 1520;
        } else if (gamepad2.y) {
            hood.setPosition(0.2);
            shooterTargetVelocity = 1100;
        } else if (gamepad2.a) {
            hood.setPosition(0.1);
            shooterTargetVelocity = 1350;
        } else if (gamepad2.b) {
            shooterTargetVelocity = 1800;
        }

        if (gamepad2.dpad_up) {
            stopper.setPosition(0.7); // Open
        }
        if (gamepad2.dpad_down) {
            stopper.setPosition(0.3); // Closed
        }

        shooter1.setVelocity(shooterTargetVelocity);
        shooter2.setVelocity(shooterTargetVelocity);

        // TELEMETRY
        telemetry.addData("AutoAim", autoAim ? "ON" : "OFF");
        telemetry.addData("Nudge Offset", autoAimOffset);
        telemetry.addData("Turret Angle Deg", targetAngleDeg);
        telemetry.update();
    }
}