package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.teamcode.vision.CameraParams;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;
import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class Vision implements Subsystem {
    private final HuskyLens huskyLens;
    private final VisionProcessor processor;
    private Pose2d latestPose = null;

    // Rate limiting
    private final int READ_PERIOD = 1; // seconds
    private final Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

    private final Drivetrain drivetrain;

    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        huskyLens = hm.get(HuskyLens.class, "huskylens");

        processor = new VisionProcessor(new CameraParams(
                CAMERA_FORWARD_OFFSET,
                CAMERA_LATERAL_OFFSET,
                CAMERA_VERTICAL_OFFSET,
                CAMERA_YAW_OFFSET
        ));

        rateLimit.expire();

        TelemetryPacket packet = new TelemetryPacket();
        if (!huskyLens.knock()) {
            packet.put("Vision", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            packet.put("Vision", "Connected, waiting for start...");
        }
        dashboard.sendTelemetryPacket(packet);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    @Override
    public void periodic() {
        if (!rateLimit.hasExpired()) return;
        rateLimit.reset();

        HuskyLens.Block[] blocks = huskyLens.blocks();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Tag Count", blocks.length);

        for (HuskyLens.Block block : blocks) {
            boolean trackThis = false;
            for (int id : TRACKED_TAG_IDS) {
                if (block.id == id) {
                    trackThis = true;
                    break;
                }
            }

            if (!trackThis) continue;
            if (Math.min(block.width, block.height) < MIN_TAG_PIXEL_SIZE) continue;

            Pose2d correctedPose = processor.getCorrectedRobotPoseFromTag(drivetrain, block);
            if (correctedPose != null) {
                latestPose = correctedPose;
                packet.put("Corrected Pose", correctedPose);
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

    public void setLatestPose(Pose2d pose) {
        this.latestPose = pose;
    }

    /** Returns the last corrected pose, or null if none available */
    public Pose2d getLatestPose() {
        return latestPose;
    }

    /** Returns raw blocks from HuskyLens */
    public HuskyLens.Block[] detectObject() {
        return huskyLens.blocks();
    }
}
