package org.firstinspires.ftc.teamcode.Commands.drive;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * A continuous drive command for TeleOp.
 * <p>
 * This command reads joystick input from a {@link GamepadEx} and converts it
 * into a {@link PoseVelocity2d}, which is then applied to the drivetrain. It
 * remains scheduled for as long as TeleOp runs, providing field- or robot-centric
 * driving depending on how the drivetrain interprets the velocity.
 * <p>
 * When the command ends (either because the OpMode stops or it is interrupted),
 * the drivetrain is commanded to stop.
 */
public class DriveContinous extends CommandBase {

    private final Drivetrain drivetrain;
    private final GamepadEx driver;
    private final double speed;

    /**
     * Creates a new continuous drive command.
     *
     * @param drivetrain the drivetrain subsystem to control
     * @param driver     the gamepad used for driver input
     * @param speed      scaling factor for input values (0.0–1.0 typical)
     */
    public DriveContinous(Drivetrain drivetrain, GamepadEx driver, double speed) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.speed = speed;

        addRequirements(drivetrain);
    }

    /**
     * Reads joystick input every cycle and applies it as a drive power.
     */
    @Override
    public void execute() {
        PoseVelocity2d pose = new PoseVelocity2d(
                new Vector2d(
                        driver.getLeftY() * speed,   // forward/back
                        -driver.getLeftX() * speed   // strafe (note the inversion)
                ),
                -driver.getRightX() * speed         // rotation (inverted to match driver feel)
        );
        drivetrain.setDrivePowers(pose);
    }

    /**
     * Stops the drivetrain when the command ends, whether completed or interrupted.
     *
     * @param isInterrupted true if the command was canceled or OpMode ended
     */
    @Override
    public void end(boolean isInterrupted) {
        PoseVelocity2d pose = new PoseVelocity2d(
                new Vector2d(0, 0),
                0
        );
        drivetrain.setDrivePowers(pose);
    }
}
