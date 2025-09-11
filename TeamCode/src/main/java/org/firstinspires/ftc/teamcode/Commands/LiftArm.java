package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmLift;

public class LiftArm extends CommandBase {
    private final ArmLift armLift;
    private final double power;

    public LiftArm(ArmLift armLift, double power) {
        this.armLift = armLift;
        this.power = power;

        addRequirements(armLift);
    }

    @Override
    public void initialize() {
        armLift.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
