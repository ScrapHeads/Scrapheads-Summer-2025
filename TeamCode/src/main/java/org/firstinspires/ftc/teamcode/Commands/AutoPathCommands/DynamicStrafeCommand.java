package org.firstinspires.ftc.teamcode.Commands.AutoPathCommands;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.AccelConstraint;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DynamicStrafeCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final double posTol, headingTol, velTol;
    private final TurnConstraints turnConstraints;
    private final VelConstraint velConstraints;
    private final AccelConstraint accelConstraint;

    private Action trajectoryAction;

    public DynamicStrafeCommand(Drivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;     // Use drivetrain default
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance; // Use drivetrain default
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;      // Use drivetrain default

        addRequirements(drivetrain);
    }

    public DynamicStrafeCommand(
            Drivetrain drivetrain,
            Supplier<Pose2d> targetPoseSupplier,
            TurnConstraints turnConstraints,
            VelConstraint velConstraint,
            AccelConstraint accelConstraint
    ) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;     // Use drivetrain default
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance; // Use drivetrain default
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;      // Use drivetrain default
    }

    public DynamicStrafeCommand(
            Drivetrain drivetrain,
            Supplier<Pose2d> targetPoseSupplier,
            double posTol,
            double headingTol,
            double velTol ) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    public DynamicStrafeCommand(
            Drivetrain drivetrain,
            Supplier<Pose2d> targetPoseSupplier,
            TurnConstraints turnConstraints,
            VelConstraint velConstraint,
            AccelConstraint accelConstraint,
            double posTol,
            double headingTol,
            double velTol
    ) {
        this.drivetrain = drivetrain;
        this.targetPoseSupplier = targetPoseSupplier;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;
    }

    @Override
    public void initialize() {
        drivetrain.updatePoseEstimate();
        Pose2d currentPose = drivetrain.localizer.getPose();
        Pose2d targetPose = targetPoseSupplier.get();

        trajectoryAction = drivetrain.actionBuilder(
                        currentPose,
                        turnConstraints, velConstraints, accelConstraint,
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

