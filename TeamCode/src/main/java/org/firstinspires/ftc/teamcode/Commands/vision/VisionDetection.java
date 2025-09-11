package org.firstinspires.ftc.teamcode.Commands.vision;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;

import static org.firstinspires.ftc.teamcode.Constants.*;

/**
 * Command that continuously processes HuskyLens detections and produces
 * vision-based pose estimates. Optionally, the vision pose can override
 * the drivetrain's localizer estimate.
 * <p>
 * This command runs continuously (never finishes) and should typically
 * be scheduled during TeleOp or autonomous for ongoing tag tracking.
 */
public class VisionDetection extends CommandBase {

    private final Vision vision;
    private final VisionProcessor processor;
    private final Drivetrain drivetrain;

    /** If true, the vision pose will replace the localizer's estimate. */
    private final boolean updateLocalizer;

    /**
     * Creates a new vision detection command.
     *
     * @param vision          the Vision subsystem for handling HuskyLens I/O
     * @param processor       the VisionProcessor that converts blocks to poses
     * @param drivetrain      the drivetrain (used if localizer should be updated)
     * @param updateLocalizer if true, overwrite the localizer's pose with vision data
     */
    public VisionDetection(Vision vision, VisionProcessor processor, Drivetrain drivetrain, boolean updateLocalizer) {
        this.vision = vision;
        this.processor = processor;
        this.drivetrain = drivetrain;
        this.updateLocalizer = updateLocalizer;

        addRequirements(vision);
    }

    /**
     * Reads HuskyLens blocks, filters for valid tracked tags, and converts
     * the first valid detection into a robot-relative pose. Updates the
     * drivetrain localizer and/or vision subsystem as configured.
     */
    @Override
    public void execute() {
        HuskyLens.Block[] blocks = vision.detectObject();

        Pose2d visionPose = null;

        // Look through all detected blocks
        for (HuskyLens.Block block : blocks) {
            boolean trackThis = false;

            // Only process blocks with IDs in TRACKED_TAG_IDS
            for (int id : TRACKED_TAG_IDS) {
                if (block.id == id) {
                    trackThis = true;
                    break;
                }
            }

            if (!trackThis) continue;
            if (Math.min(block.width, block.height) < MIN_TAG_PIXEL_SIZE) continue;

            // Convert the first valid block into a pose estimate
            visionPose = processor.blockToRobotPose(block);
            break;
        }

        if (visionPose != null) {
            if (updateLocalizer) {
                drivetrain.localizer.setPose(visionPose);
            }
            // Always update Vision subsystem with latest pose
            vision.setLatestPose(visionPose);
        }
    }

    /**
     * Runs continuously until explicitly canceled.
     *
     * @return false always
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}