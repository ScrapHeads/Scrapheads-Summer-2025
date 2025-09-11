package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmLift implements Subsystem {
    private final MotorEx armLift;

    public ArmLift(HardwareMap hm) {
        armLift = new MotorEx(hm, "armLift", Motor.GoBILDA.RPM_312);

        armLift.setInverted(false);

        armLift.resetEncoder();

        armLift.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        armLift.set(power);
    }


}
