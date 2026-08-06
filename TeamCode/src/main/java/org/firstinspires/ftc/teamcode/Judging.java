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


@TeleOp(name = "Judging", group = "Red")
public class Judging extends OpMode {
    private Servo stopper, hood, rgbIndicator, rgbIndicator2; // Added rgbIndicator


    @Override
    public void init() {
        rgbIndicator = hardwareMap.get(Servo.class, "rgb1");
        rgbIndicator2 = hardwareMap.get(Servo.class, "rgb2");
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_up) {
            rgbIndicator.setPosition(0.611);
            rgbIndicator2.setPosition(0.611);

        }
        if (gamepad2.dpad_down) {
            rgbIndicator.setPosition(0);
            rgbIndicator2.setPosition(0);

        }
        }

    }


