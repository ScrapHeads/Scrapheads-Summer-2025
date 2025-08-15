package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;

import static org.firstinspires.ftc.teamcode.Constants.*;

public class VisionDetection extends CommandBase {
    private final Vision vision;
    private final VisionProcessor processor;
    private final Drivetrain drivetrain;

    // If true, vision pose will override the localizer’s pose estimate
    private final boolean updateLocalizer;

    public VisionDetection(Vision vision, VisionProcessor processor, Drivetrain drivetrain, boolean updateLocalizer) {
        this.vision = vision;
        this.processor = processor;
        this.drivetrain = drivetrain;
        this.updateLocalizer = updateLocalizer;

        addRequirements(vision);
    }

    @Override
    public void execute() {
        HuskyLens.Block[] blocks = vision.detectObject();

        Pose2d visionPose = null;

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

            visionPose = processor.blockToRobotPose(block);
            break; // use the first valid tag
        }

        if (visionPose != null) {
            if (updateLocalizer) {
                drivetrain.localizer.setPose(visionPose);
            }
            // Always keep the Vision subsystem’s latestPose updated
            vision.setLatestPose(visionPose);
        }
    }

    @Override
    public boolean isFinished() {
        return false; // runs continuously
    }
}
