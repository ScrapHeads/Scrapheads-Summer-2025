package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SorterRotate;

public class RotateSorter extends CommandBase {

    private final SorterRotate sorterRotate;

    private final int pos;

    public RotateSorter(SorterRotate sorterRotate, int pos) {
        this.sorterRotate = sorterRotate;
        this.pos = pos;

        addRequirements(sorterRotate);
    }

    @Override
    public void initialize() {
        sorterRotate.setPower(pos);
    }

    @Override
    public boolean isFinished() {
        return sorterRotate.getMagneticSensor();
    }
}
