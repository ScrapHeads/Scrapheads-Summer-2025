package org.firstinspires.ftc.teamcode.vision;

public class AutoAlignConfig {
    public final double targetX;       // desired pixel X position
    public final double targetY;       // desired pixel Y position
    public final double minWidth;      // minimum tag size before alignment starts
    public final double kP_strafe;     // proportional gain for left/right correction
    public final double kP_forward;    // proportional gain for forward/back correction
    public final double kP_heading;    // proportional gain for heading correction
    public final double toleranceX;    // pixel tolerance for X axis (strafe)
    public final double toleranceY;    // pixel tolerance for Y axis (forward)

    public AutoAlignConfig(double targetX, double targetY, double minWidth,
                           double kP_strafe, double kP_forward, double kP_heading,
                           double toleranceX, double toleranceY) {
        this.targetX = targetX;
        this.targetY = targetY;
        this.minWidth = minWidth;
        this.kP_strafe = kP_strafe;
        this.kP_forward = kP_forward;
        this.kP_heading = kP_heading;
        this.toleranceX = toleranceX;
        this.toleranceY = toleranceY;
    }
}
