package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

/**
 * Static storage to hold the robot's position between Autonomous and TeleOp.
 */
public class PoseStorage {
    // Default to your Red Close starting position in case Auto isn't run
    public static Pose currentPose = new Pose(55.25, 7.25, Math.toRadians(90));
}