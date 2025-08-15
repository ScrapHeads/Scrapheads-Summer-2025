package org.firstinspires.ftc.teamcode.vision;

public class CameraParams {
    public final double forwardOffset; // inches from robot center forward
    public final double lateralOffset; // inches from robot center right (+)
    public final double verticalOffset; // inches from floor
    public final double yawOffset; // radians from robot forward

    public CameraParams(double forwardOffset, double lateralOffset, double verticalOffset, double yawOffset) {
        this.forwardOffset = forwardOffset;
        this.lateralOffset = lateralOffset;
        this.verticalOffset = verticalOffset;
        this.yawOffset = yawOffset;
    }
}

