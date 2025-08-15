package org.firstinspires.ftc.teamcode.Commands.AutoPathCommands;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DynamicTurnCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final DoubleSupplier targetHeadingSupplier;
    private final double posTol, headingTol, velTol;
    private final TurnConstraints turnConstraints;
    private final VelConstraint velConstraints;
    private final AccelConstraint accelConstraint;

    private Action trajectoryAction;

    public DynamicTurnCommand(Drivetrain drivetrain, DoubleSupplier targetHeadingSupplier) {
                this.drivetrain = drivetrain;
                this.targetHeadingSupplier = targetHeadingSupplier;
                this.turnConstraints = drivetrain.defaultTurnConstraints;
                this.velConstraints = drivetrain.defaultVelConstraint;
                this.accelConstraint = drivetrain.defaultAccelConstraint;
                this.posTol = Drivetrain.PARAMS.defaultPosTolerance;     // Use drivetrain default
                this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance; // Use drivetrain default
                this.velTol = Drivetrain.PARAMS.defaultVelTolerance;      // Use drivetrain default

        addRequirements(drivetrain);
    }

    public DynamicTurnCommand(
            Drivetrain drivetrain,
            DoubleSupplier targetHeadingSupplier,
            TurnConstraints turnConstraints,
            VelConstraint velConstraint,
            AccelConstraint accelConstraint
    ) {
        this.drivetrain = drivetrain;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.turnConstraints = turnConstraints;
        this.velConstraints = velConstraint;
        this.accelConstraint = accelConstraint;
        this.posTol = Drivetrain.PARAMS.defaultPosTolerance;     // Use drivetrain default
        this.headingTol = Drivetrain.PARAMS.defaultHeadingTolerance; // Use drivetrain default
        this.velTol = Drivetrain.PARAMS.defaultVelTolerance;      // Use drivetrain default
    }

    public DynamicTurnCommand(
            Drivetrain drivetrain,
            DoubleSupplier targetHeadingSupplier,
            double posTol,
            double headingTol,
            double velTol ) {
        this.drivetrain = drivetrain;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.turnConstraints = drivetrain.defaultTurnConstraints;
        this.velConstraints = drivetrain.defaultVelConstraint;
        this.accelConstraint = drivetrain.defaultAccelConstraint;
        this.posTol = posTol;
        this.headingTol = headingTol;
        this.velTol = velTol;

        addRequirements(drivetrain);
    }

    public DynamicTurnCommand(
            Drivetrain drivetrain,
            DoubleSupplier targetHeadingSupplier,
            TurnConstraints turnConstraints,
            VelConstraint velConstraint,
            AccelConstraint accelConstraint,
            double posTol,
            double headingTol,
            double velTol
    ) {
        this.drivetrain = drivetrain;
        this.targetHeadingSupplier = targetHeadingSupplier;
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

        // Absolute heading target
        Pose2d targetPose = new Pose2d(
                currentPose.position,
                Rotation2d.exp(targetHeadingSupplier.getAsDouble())
        );

        trajectoryAction = drivetrain.actionBuilder(
                        currentPose,
                        turnConstraints, velConstraints, accelConstraint,
                        posTol, headingTol, velTol
                )
                .turnTo(targetPose.heading)
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
