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
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicSplineCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicStrafeCommand;
import org.firstinspires.ftc.teamcode.Commands.AutoPathCommands.DynamicTurnCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "MoveForward", group = "ScrapHeads")
public class MoveForward extends CommandOpMode {

    Drivetrain drivetrain;

    // Field positions (X, Y in inches, heading in radians)
    List<Pose2d> positions = Arrays.asList(
            new Pose2d(0, 0, 0),
            new Pose2d(10, 10, Math.toRadians(0)),
            new Pose2d(20, 5, Math.toRadians(90)),
            new Pose2d(0, 0, Math.toRadians(180))
    );

    @Override
    public void initialize() {
        // Init hardware + dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        drivetrain = new Drivetrain(hm, positions.get(0));
        drivetrain.register();

        // Custom constraints for some moves
        TurnConstraints turnConstraintsFast = new TurnConstraints(4, -4, 4);
        VelConstraint velConstraintFast = new MinVelConstraint(Arrays.asList(
                drivetrain.kinematics.new WheelVelConstraint(80),
                new AngularVelConstraint(Math.PI)));
        AccelConstraint accelConstraintFast = new ProfileAccelConstraint(-40, 80);

        waitForStart();

        // Run a sequence of movements dynamically using live pose grabbing
        schedule(new SequentialCommandGroup(
                // Move to position 1 using defaults
                new DynamicStrafeCommand(drivetrain, () -> positions.get(1)),

                // Move to position 2 with custom constraints & tolerances
                new DynamicSplineCommand(
                        drivetrain,
                        () -> positions.get(2).position,       // dynamic pose grabbing
                        positions.get(2).heading.toDouble(),
                        turnConstraintsFast,          // custom turn constraints
                        velConstraintFast,             // custom velocity constraints
                        accelConstraintFast,           // custom acceleration constraints
                        0.5,                           // position tolerance in inches
                        Math.toRadians(3),             // heading tolerance in radians
                        0.05                           // velocity tolerance in inches/sec
                ),

                // Turn to absolute 90° with custom constraints + tight heading tolerance
                new DynamicTurnCommand(
                        drivetrain,
                        () -> Math.toRadians(90),      // target absolute heading
                        turnConstraintsFast,           // custom turn constraints
                        velConstraintFast,             // custom velocity constraints
                        accelConstraintFast,           // custom acceleration constraints
                        Drivetrain.PARAMS.defaultPosTolerance, // keep default position tolerance
                        Math.toRadians(1),              // heading tolerance in radians
                        Drivetrain.PARAMS.defaultVelTolerance // keep default velocity tolerance
                ),

                // Move back to start with defaults
                new DynamicStrafeCommand(
                        drivetrain,
                        () -> positions.get(3) // dynamic pose grabbing
                )

        ));
    }
}
