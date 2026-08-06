package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Turret PID Tuner", group = "Tuning")
public class TurretPIDTuner extends OpMode {

    // ===== PID VARIABLES (editable in Panels) =====
    public static double Kp = 0.04;
    public static double Ki = 0.0;
    public static double Kd = 0.03;

    // Setpoint in degrees (editable in Panels)
    public static double targetAngleDeg = 0;

    // Your measured ticks per radian
    public static double TICKS_PER_RADIAN = 559.4190146;

    // Power clamp for safety
    public static double maxPower = 0.5;

    // ===== HARDWARE =====
    DcMotor turretMotor;

    // ===== PID STATE =====
    double integral = 0;
    double lastError = 0;

    @Override
    public void init() {

        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        // Reverse if needed
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoder
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Turret PID Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===== GET CURRENT POSITION =====
        int currentTicks = turretMotor.getCurrentPosition();

        // Convert target angle → ticks
        double targetTicks =
                targetAngleDeg * Math.PI / 180.0 * TICKS_PER_RADIAN;

        // Convert ticks → angle
        double currentAngleDeg =
                currentTicks / TICKS_PER_RADIAN * 180.0 / Math.PI;

        // Calculate error
        double errorTicks = targetTicks - currentTicks;
        double errorDeg = targetAngleDeg - currentAngleDeg;

        // ===== PID =====
        integral += errorTicks;
        double derivative = errorTicks - lastError;

        double power =
                Kp * errorTicks +
                        Ki * integral +
                        Kd * derivative;

        // Clamp power
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // Apply power
        turretMotor.setPower(power);

        lastError = errorTicks;

        // ===== TELEMETRY =====
        telemetry.addLine("===== TURRET PID =====");

        telemetry.addData("Target Angle (deg)", targetAngleDeg);
        telemetry.addData("Current Angle (deg)", currentAngleDeg);

        telemetry.addData("Target Ticks", targetTicks);
        telemetry.addData("Current Ticks", currentTicks);

        telemetry.addData("Error (deg)", errorDeg);
        telemetry.addData("Error (ticks)", errorTicks);

        telemetry.addData("Motor Power", power);

        telemetry.addLine("======================");

        telemetry.update();
    }
}
