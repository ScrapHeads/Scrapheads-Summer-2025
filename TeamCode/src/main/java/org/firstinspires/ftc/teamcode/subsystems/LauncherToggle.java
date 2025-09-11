package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LauncherToggle implements Subsystem {
    public final ServoEx launcher;

    public LauncherToggle(HardwareMap hm) {
        launcher = new SimpleServo(hm, "launcher", -100, 100, AngleUnit.DEGREES);
    }

    public void setPos(double pos) {
        launcher.setPosition(pos);
    }

}
