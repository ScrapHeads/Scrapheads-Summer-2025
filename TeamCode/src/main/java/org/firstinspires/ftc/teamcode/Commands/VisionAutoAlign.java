package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.vision.AutoAlignConfig;

public class VisionAutoAlign extends CommandBase {
    private final Vision vision;
    private final Drivetrain drivetrain;
    private final GamepadEx driver;

    public VisionAutoAlign(Vision vision, Drivetrain drivetrain, GamepadEx driver) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.driver = driver;

        addRequirements(vision, drivetrain);
    }

    @Override
    public void execute() {
        HuskyLens.Block[] blocks = vision.detectObject();

        // Driver input (sticks)
        double driverForward = -driver.getLeftY();  // forward/back
        double driverStrafe  = driver.getLeftX();   // strafe
        double driverTurn    = driver.getRightX();  // turn

        // If driver is actively moving, skip auto-align
        boolean driverActive = Math.abs(driverForward) > 0.05 ||
                Math.abs(driverStrafe) > 0.05 ||
                Math.abs(driverTurn) > 0.05;

        if (driverActive) {
            drivetrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(driverForward, driverStrafe),
                    driverTurn
            ));
            sendTelemetry(false, false, false);
            return;
        }

        // No tag? Stop robot
        if (blocks.length == 0) {
            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sendTelemetry(false, false, false);
            return;
        }

        HuskyLens.Block block = blocks[0];
        AutoAlignConfig config = Constants.TAG_CONFIGS.get(block.id);

        if (config == null || Math.min(block.width, block.height) < config.minWidth) {
            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sendTelemetry(true, false, false);
            return;
        }

        // Pixel error from desired target position
        double errorX = block.x - config.targetX;
        double errorY = block.y - config.targetY;

        // Check if we're within tolerance (per axis)
        boolean aligned = Math.abs(errorX) < config.toleranceX &&
                Math.abs(errorY) < config.toleranceY;

        if (aligned) {
            drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            sendTelemetry(true, false, true);
            return;
        }

        // Proportional control
        double strafeCmd  = errorX * config.kP_strafe;
        double forwardCmd = (config.targetY - block.y) * config.kP_forward;
        double turnCmd    = errorX * config.kP_heading;

        // Apply correction
        drivetrain.setDrivePowers(new PoseVelocity2d(
                new Vector2d(forwardCmd, strafeCmd),
                turnCmd
        ));

        sendTelemetry(true, true, false);
    }

    private void sendTelemetry(boolean tagSeen, boolean aligning, boolean aligned) {
        // Driver Hub telemetry
        Constants.tele.addData("AutoAlign", Constants.AUTO_ALIGN_ENABLED ? "ON" : "OFF");
        Constants.tele.addData("Tag Detected", tagSeen ? "YES" : "NO");
        Constants.tele.addData("Aligning", aligning ? "YES" : "NO");
        Constants.tele.addData("Aligned", aligned ? "YES" : "NO");
        Constants.tele.update();

        // Dashboard telemetry
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("AutoAlign", Constants.AUTO_ALIGN_ENABLED ? "ON" : "OFF");
        packet.put("Tag Detected", tagSeen ? "YES" : "NO");
        packet.put("Aligning", aligning ? "YES" : "NO");
        packet.put("Aligned", aligned ? "YES" : "NO");
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return false; // stays active until canceled/toggled off
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}
