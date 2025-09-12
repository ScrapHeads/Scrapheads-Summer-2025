package org.firstinspires.ftc.teamcode.teleops;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.LiftArm;
import org.firstinspires.ftc.teamcode.Commands.ToggleLauncher;
import org.firstinspires.ftc.teamcode.Commands.drive.DriveContinous;
import org.firstinspires.ftc.teamcode.subsystems.ArmLift;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.LauncherToggle;

@TeleOp(name = "FairTele", group = "ScrapHeads")
public class FairTele extends CommandOpMode {
    // Create all subsystems references
    GamepadEx driver = null;

    Drivetrain drivetrain = null;
    ArmLift armLift = null;
    LauncherToggle launcherToggle = null;


    @Override
    public void initialize() {
        // Initializing the hardware map for motors, telemetry, and dashboard
        hm = hardwareMap;
        tele = telemetry;
        dashboard = FtcDashboard.getInstance();

        // Initialize the first controller
        // Can only have one active DRIVETRAIN controller at once
        driver = new GamepadEx(gamepad1);

        // Initialize the subsystems declared at the top of the code
        drivetrain = new Drivetrain(hm, new Pose2d(0,0,0));
        drivetrain.register();

        armLift = new ArmLift(hm);
        armLift.register();

        launcherToggle = new LauncherToggle(hm);
        launcherToggle.register();

        // Calling assignControls to set input commands
        assignControls();
    }

    public void assignControls() {
        drivetrain.setDefaultCommand(new DriveContinous(drivetrain, driver, 1));

//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1)
//                .whenActive(new LiftArm(armLift, 1))
//                .whenInactive(new LiftArm(armLift, 0));
//
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1)
//                .whenActive(new LiftArm(armLift, -1))
//                .whenInactive(new LiftArm(armLift, 0));

        driver.getGamepadButton(RIGHT_BUMPER)
                .whenActive(new LiftArm(armLift, 1))
                .whenInactive(new LiftArm(armLift, 0));

        driver.getGamepadButton(LEFT_BUMPER)
                .whenActive(new LiftArm(armLift, -1))
                .whenInactive(new LiftArm(armLift, 0));

        driver.getGamepadButton(A)
                .whenPressed(new ToggleLauncher(launcherToggle, 1));

        driver.getGamepadButton(B)
                .whenPressed(new ToggleLauncher(launcherToggle, 0));
    }

}
