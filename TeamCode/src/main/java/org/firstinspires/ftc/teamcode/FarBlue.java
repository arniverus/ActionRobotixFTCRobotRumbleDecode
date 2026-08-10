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

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Blue Far", group = "Blue")
public class FarBlue extends OpMode {

    // --- Hardware ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Servo stopper, hood;
    private DcMotor intake, turret;
    private DcMotorEx shooter1, shooter2;

    // --- Settings & Follower ---
    private Follower follower;
    private double driveMultiplier = 1.0;
    private boolean liftOn = false;
    private boolean autoAim = false;

    // --- Shooter PIDF (Blue Match) ---
    public static double SHOOTER_P = -830.000;
    public static double SHOOTER_F = 1.00000;
    private double shooterTargetVelocity = 0;

    // --- Turret PID Constants ---
    public static double Kp = 0.035;
    public static double Ki = 0.0;
    public static double Kd = 0.0032;
    public static double TICKS_PER_RADIAN = 559.4190146;

    // --- BLUE GOAL COORDINATES ---
    public static double GOAL_X = 6;
    public static double GOAL_Y = 138;

    // --- Logic Variables ---
    private double targetAngleDeg = 0;
    private double autoAimOffset = 0;
    private double integral = 0;
    private double lastError = 0;

    @Override
    public void init() {
        // Drivetrain
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Mechanisms
        turret   = hardwareMap.get(DcMotor.class, "turret");
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        stopper  = hardwareMap.get(Servo.class, "stopper");
        hood     = hardwareMap.get(Servo.class, "hood");
        intake   = hardwareMap.get(DcMotor.class, "intake");

        // Set directions to match Blue Auto config
        stopper.setDirection(Servo.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Shooter PIDF Setup
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Turret Setup
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Pull localization from AutoFarBlue's saved pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.currentPose);
    }

    @Override
    public void loop() {
        follower.update();

        // 1. DRIVETRAIN (Multiplier controls on Gamepad 1)
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

        // 2. AUTO AIM & DPAD NUDGE (Gamepad 2)
        if (gamepad2.right_trigger > 0.5) autoAim = true;
        if (gamepad2.left_trigger > 0.5) autoAim = false;

        // Manual Dpad Nudge (adjusts offset while auto-aiming or manual)
        if (gamepad2.dpad_left)  autoAimOffset += 0.4;
        if (gamepad2.dpad_right) autoAimOffset -= 0.4;
        if (gamepad2.right_stick_button) autoAimOffset = 0;

        if (autoAim) {
            Pose pose = follower.getPose();
            // Calculate field-centric angle to Blue Goal (6, 138)
            double fieldAngle = Math.atan2(GOAL_Y - pose.getY(), GOAL_X - pose.getX());
            double targetRad = fieldAngle - pose.getHeading();
            targetRad = Math.atan2(Math.sin(targetRad), Math.cos(targetRad)); // Normalize

            targetAngleDeg = Math.toDegrees(targetRad) + autoAimOffset;
        } else {
            // Manual stick control
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

        turret.setPower(Math.max(-0.6, Math.min(0.6, turretPower)));
        lastError = error;

        // 4. INTAKE (Gamepad 2 Bumpers)
        if (gamepad2.right_bumper) liftOn = true;
        if (gamepad2.left_bumper)  liftOn = false;

        if (gamepad2.left_trigger > 0.1) intake.setPower(0.5); // Manual outtake
        else if (liftOn) intake.setPower(-1.0);
        else intake.setPower(0);

        // 5. SHOOTER PRESETS (Gamepad 2 Buttons)
        if (gamepad2.x) { // High Shot
            hood.setPosition(0);
            shooterTargetVelocity = 1540;
        } else if (gamepad2.y) { // Low/Mid Shot
            hood.setPosition(0.2);
            shooterTargetVelocity = 1100;
        } else if (gamepad2.a) { // Short Shot
            hood.setPosition(0.2);
            shooterTargetVelocity = 1400;
        } else if(gamepad2.b) {
            hood.setPosition(0.2);
            shooterTargetVelocity = 0;
        }

        // 6. STOPPER (Dpad Up Open, Dpad Down Close)
        if (gamepad2.dpad_down)   stopper.setPosition(1); // Open (Shoot)
        if (gamepad2.dpad_up) stopper.setPosition(0); // Closed (Hold)

        shooter1.setVelocity(shooterTargetVelocity);
        shooter2.setVelocity(shooterTargetVelocity);

        // Telemetry
        telemetry.addData("Mode", autoAim ? "AUTO-AIM" : "MANUAL");
        telemetry.addData("Goal", "BLUE (6, 138)");
        telemetry.addData("Turret Angle Deg", targetAngleDeg);
        telemetry.addData("Nudge Offset", autoAimOffset);
        telemetry.addData("Follower Pose", follower.getPose().toString());
        telemetry.update();
    }
}