package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;
import static org.firstinspires.ftc.teamcode.Constants.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.commands.core.LynxGetServoConfigurationCommand;
import org.firstinspires.ftc.teamcode.Commands.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "MoveForward", group = "ScrapHeads")
public class MoveForward extends CommandOpMode {
    // Create all subsystems references
    GamepadEx driver = null;

    Drivetrain drivetrain = null;

    // Create a list of the target positions that we want the robot to go to: call in order
    List<Pose2d> positions = Arrays.asList(
            new Pose2d(0, 0, 0),
            new Pose2d(10, 0, Math.toRadians(0)),
            new Pose2d(10, 10, Math.toRadians(0))
    );

    // Create a list of completed paths: call in order
    List<TrajectoryActionBuilder> path = null;

    @Override
    public void initialize() {
        // Initializing the hardware map for motors, telemetry, and dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        // Initialize the subsystems declared at the top of the code
        // Update startingPose to set the correct starting position on the filed
        drivetrain = new Drivetrain(hm, positions.get(0));
        drivetrain.register();

        // Values used to tune the speed and accuracy of path ovoid when possible
        TurnConstraints turnConstraintsFast = new TurnConstraints(Math.PI, -Math.PI, Math.PI);
        VelConstraint velConstraintFast = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(80),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintFast = new ProfileAccelConstraint(-40, 80);

        TrajectoryActionBuilder driveForward = drivetrain.actionBuilder(positions.get(0))
                .strafeToLinearHeading(positions.get(1).position, positions.get(1).heading);
        path.add(driveForward);

        TrajectoryActionBuilder driveSideWays = drivetrain.actionBuilder(positions.get(1))
                .strafeToLinearHeading(positions.get(2).position, positions.get(2).heading);
        path.add(driveSideWays);

        followPath();
    }

    public void followPath() {
        schedule(new SequentialCommandGroup(
                new FollowDrivePath(drivetrain, path.get(0).build()),

                new FollowDrivePath(drivetrain, path.get(1).build())
        ));
    }


}
