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

import java.util.function.Supplier;

public class DynamicSplineCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Supplier<Vector2d> targetPositionSupplier;
    private final double targetHeading;
    private final double posTol, headingTol, velTol;
    private final TurnConstraints turnConstraints;
    private final VelConstraint velConstraints;
    private final AccelConstraint accelConstraint;

    private Action trajectoryAction;

    public DynamicSplineCommand(Drivetrain drivetrain,
                                Supplier<Vector2d> targetPositionSupplier,
                                double targetHeading) {
        this.drivetrain = drivetrain;
        this.targetPositionSupplier = targetPositionSupplier;
        this.targetHeading = targetHeading;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance;
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;

        addRequirements(drivetrain);
    }

    public DynamicSplineCommand(Drivetrain drivetrain,
                                Supplier<Vector2d> targetPositionSupplier,
                                double targetHeading,
                                TurnConstraints turnConstraints,
                                VelConstraint velConstraint,
                                AccelConstraint accelConstraint) {
        this.drivetrain = drivetrain;
        this.targetPositionSupplier = targetPositionSupplier;
        this.targetHeading = targetHeading;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance;
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;

        addRequirements(drivetrain);
    }

    public DynamicSplineCommand(Drivetrain drivetrain,
                                Supplier<Vector2d> targetPositionSupplier,
                                double targetHeading,
                                double posTol,
                                double headingTol,
                                double velTol) {
        this.drivetrain = drivetrain;
        this.targetPositionSupplier = targetPositionSupplier;
        this.targetHeading = targetHeading;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }


    public DynamicSplineCommand(Drivetrain drivetrain,
                                Supplier<Vector2d> targetPositionSupplier,
                                double targetHeading,
                                TurnConstraints turnConstraints,
                                VelConstraint velConstraint,
                                AccelConstraint accelConstraint,
                                double posTol,
                                double headingTol,
                                double velTol) {
        this.drivetrain = drivetrain;
        this.targetPositionSupplier = targetPositionSupplier;
        this.targetHeading = targetHeading;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.updatePoseEstimate();
        Pose2d currentPose = drivetrain.localizer.getPose();
        Vector2d targetPosition = targetPositionSupplier.get();

        trajectoryAction = drivetrain.actionBuilder(
                        currentPose,
                        turnConstraints, velConstraints, accelConstraint,
                        posTol, headingTol, velTol
                )
                .splineTo(targetPosition, targetHeading)
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
