package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
@TeleOp(name = "Shooter PIDF Tuner")
public class ShooterPIDFTuner extends OpMode {

    // =========================
    // SHOOTER MOTORS
    // =========================
    DcMotorEx shooter1, shooter2;

    // target velocities
    double highVelocity = -1500;
    double lowVelocity = -900;
    double curTargetVelocity = highVelocity;

    // PIDF values
    double P = -830.000;
    double F = 1.00000;


    // step sizes
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};
    int stepIndex = 1;

    @Override
    public void init() {
        // map hardware using RobotCentric names
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        // reverse second shooter to match your robot setup
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // initial PIDF
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        telemetry.addLine("Shooter PIDF Tuner Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // =========================
        // Switch target velocity
        // =========================
        if (gamepad1.y) {
            curTargetVelocity = (curTargetVelocity == highVelocity) ? lowVelocity : highVelocity;
        }

        // =========================
        // Change step size
        // =========================
        if (gamepad1.b) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // =========================
        // Tune F
        // =========================
        if (gamepad1.dpad_left) F -= stepSizes[stepIndex];
        if (gamepad1.dpad_right) F += stepSizes[stepIndex];

        // =========================
        // Tune P
        // =========================
        if (gamepad1.dpad_up) P += stepSizes[stepIndex];
        if (gamepad1.dpad_down) P -= stepSizes[stepIndex];

        // =========================
        // Apply PIDF
        // =========================
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        // set velocity
        shooter1.setVelocity(curTargetVelocity);
        shooter2.setVelocity(curTargetVelocity);

        // =========================
        // Telemetry
        // =========================
        double currentVel1 = shooter1.getVelocity();
        double currentVel2 = shooter2.getVelocity();
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Shooter1 Velocity", "%.2f", currentVel1);
        telemetry.addData("Shooter2 Velocity", "%.2f", currentVel2);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);
        telemetry.update();
    }
}