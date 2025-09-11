package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.subsystems.LauncherToggle;

public class ToggleLauncher extends CommandBase {
    public final LauncherToggle launcherToggle;

    private final double pos;

    public ToggleLauncher(LauncherToggle launcherToggle, double pos) {
        this.launcherToggle = launcherToggle;
        this.pos = pos;

        addRequirements(launcherToggle);
    }

    @Override
    public void initialize() {
        launcherToggle.setPos(pos);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
