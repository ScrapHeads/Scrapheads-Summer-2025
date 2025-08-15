package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Rotation2d;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class VisionProcessor {
    private double cameraForwardOffset;
    private double cameraLeftOffset;
    private double cameraHeight;
    private double cameraHeadingOffset;

    // Calibration factors (tune on the robot!)
    private double distanceConstant = DISTANCE_CONSTANT;
    private double leftScaling = LEFT_SCALING;

    private static final int IMAGE_WIDTH = Constants.IMAGE_WIDTH;
    private static final double CAMERA_HORIZONTAL_FOV = Constants.CAMERA_HORIZONTAL_FOV;

    public VisionProcessor(double forward, double left, double height, double heading) {
        this.cameraForwardOffset = forward;
        this.cameraLeftOffset = left;
        this.cameraHeight = height;
        this.cameraHeadingOffset = heading;
    }

    public VisionProcessor(CameraParams params) {
        this(params.forwardOffset, params.lateralOffset, params.verticalOffset, params.yawOffset);
    }

    /** Converts a HuskyLens block into a robot-relative Pose2d (approx). */
    public Pose2d blockToRobotPose(HuskyLens.Block block) {
        if (block == null) return null;

        // 1) Estimate forward distance (+X) from tag size
        double avgSize = (block.width + block.height) / 2.0;
        if (avgSize <= 0) return null; // avoid div by zero
        double tagForward = distanceConstant / avgSize;

        // 2) Estimate left offset (+Y) from horizontal pixel position
        double normalizedX = (block.x - (IMAGE_WIDTH / 2.0)) / (IMAGE_WIDTH / 2.0);
        double tagLeft = normalizedX * leftScaling;

        Vector2d visionPos = new Vector2d(tagForward, tagLeft);

        // 3) Estimate heading (yaw) from horizontal offset
        double yawRadians = normalizedX * (CAMERA_HORIZONTAL_FOV / 2.0);
        Rotation2d visionHeading = new Rotation2d(Math.cos(yawRadians), Math.sin(yawRadians));

        // 4) Apply camera mount offset
        Vector2d offset = new Vector2d(cameraForwardOffset, cameraLeftOffset);
        Rotation2d finalHeading = visionHeading.plus(cameraHeadingOffset);

        return new Pose2d(visionPos.plus(offset), finalHeading);
    }

    /** Returns corrected robot pose using HuskyLens vision, compensating for motion latency. */
    public Pose2d getCorrectedRobotPoseFromTag(Drivetrain drivetrain, HuskyLens.Block block) {
        if (block == null) return null;

        drivetrain.updatePoseEstimate();
        Pose2d poseBefore = drivetrain.localizer.getPose();

        Pose2d visionPose = blockToRobotPose(block);
        if (visionPose == null) return null;

        drivetrain.updatePoseEstimate();
        Pose2d poseNow = drivetrain.localizer.getPose();

        Pose2d motionOffset = poseNow.minusExp(poseBefore);

        return new Pose2d(
                visionPose.position.plus(motionOffset.position),
                visionPose.heading.plus(motionOffset.heading.toDouble())
        );
    }
}
