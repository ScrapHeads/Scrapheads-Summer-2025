package org.firstinspires.ftc.teamcode.state;

import com.acmerobotics.roadrunner.Pose2d;

/**
 * Serializable snapshot of robot state to transfer from Auto to TeleOp.
 * Keep units consistent with your Road Runner configuration:
 * - Pose: meters and radians if using default RR units.
 * - Subsystems: use your natural representation (ticks, booleans, etc.).
 */
public class RobotState {
    public int version = 1;

    // Pose on the field
    public Pose2d pose;

    // If true on blue alliance if false on red alliance
    public boolean isBlue;



    public RobotState() {

    }

    public RobotState(Pose2d pose, boolean isBlue) {
        this.pose = pose;
        this.isBlue = isBlue;
    }

    @Override
    public String toString() {
        return "RobotState{" +
                "version=" + version +
                "pose=" +pose +
                "isBlue=" + isBlue +
                "}";
    }
}
