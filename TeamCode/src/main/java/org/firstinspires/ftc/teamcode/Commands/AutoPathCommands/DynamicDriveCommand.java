package org.firstinspires.ftc.teamcode.Commands.AutoPathCommands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DynamicDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final double posTol, headingTol, velTol;

    private Action trajectoryAction;

    /**
     * @param drivetrain Drivetrain subsystem
     * @param targetPoseSupplier Supplies the target pose (this is called at runtime so it always uses the latest robot position)
     * @param posTol Position tolerance in inches
     * @param headingTol Heading tolerance in radians
     * @param velTol Velocity tolerance (path completion velocity threshold)
     */

    public DynamicDriveCommand(Drivetrain drivetrain,
                               Supplier<Pose2d> targetPoseSupplier,
                               double posTol,
                               double headingTol,
                               double velTol) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.updatePoseEstimate();
        Pose2d currentPose = drivetrain.localizer.getPose();
        Pose2d targetPose = targetPoseSupplier.get();

        trajectoryAction = drivetrain.actionBuilder(
                        currentPose,
                        posTol, headingTol, velTol
                )
                .strafeToLinearHeading(targetPose.position, targetPose.heading)
                .build();
    }

    @Override
    public void execute() {
        if (trajectoryAction != null) {
            trajectoryAction.run(new TelemetryPacket());
        }
    }

    @Override
    public boolean isFinished() {
        return trajectoryAction == null || !trajectoryAction.run(new TelemetryPacket());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
