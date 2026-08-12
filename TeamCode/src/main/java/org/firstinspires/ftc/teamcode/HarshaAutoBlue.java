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


@Autonomous(name = "Auto Harsha Blue", group = "Blue");
public class HarshaAutoBlue extends LinearOpMode {

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

    private PathChain p1_preload, p2_thirdSpike, p3_shoot2, p4_hpIntake1, p5_shoot_3, p6_tunnelIntake1,
            p7_shoot4, p8_hpIntake2, p9_shoot5, p10_tunnelIntake2;


}
