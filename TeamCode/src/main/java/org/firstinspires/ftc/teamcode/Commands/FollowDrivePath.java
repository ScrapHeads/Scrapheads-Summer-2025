package org.firstinspires.ftc.teamcode.Commands;

import static org.firstinspires.ftc.teamcode.Constants.dashboard;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class FollowDrivePath extends CommandBase {
    private Action path = null;
    private Drivetrain drivetrain = null;

    private boolean isFinished = false;

    public FollowDrivePath(Drivetrain drivetrain, Action path) {
        this.drivetrain = drivetrain;
        this.path = path;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        isFinished = path.run(packet);
        packet.put("test", true);
        packet.put("path", path);
        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    @Override
    public boolean isFinished() {
        return !isFinished;
    }
}