package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class VisionDetection extends CommandBase {
    private final Vision vision;

    public VisionDetection(Vision vision) {
        this.vision = vision;

        addRequirements(vision);
    }

//    public void initialize() {
//
//    }
}
