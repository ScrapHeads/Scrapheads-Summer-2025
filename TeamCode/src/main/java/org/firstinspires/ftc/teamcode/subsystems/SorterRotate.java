package org.firstinspires.ftc.teamcode.subsystems;

import static com.qualcomm.robotcore.hardware.configuration.ConfigurationType.DeviceFlavor.I2C;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class SorterRotate implements Subsystem {

    private final ServoEx sorterRotate;

    private final DigitalChannel magneticSensor;

//    private final RevColorSensorV3 colorSensorV3;

    public enum SorterStates {
        STATE_ONE,
        STATE_TWO,
        STATE_THREE;
    }

    private SorterStates currentSorterStates = SorterStates.STATE_ONE;

    private boolean ledEnabled;


    public SorterRotate(HardwareMap hm, SorterStates state) {
        sorterRotate = new SimpleServo(hm, "sorterRotate", -1, 1);

        sorterRotate.turnToAngle(0);

        magneticSensor = hm.get(DigitalChannel.class, "sorterSensor");

//        colorSensorV3 = hm.get(RevColorSensorV3.class, "colorSensor");
//
//        colorSensorV3.setGain(1);

//        setLed(true);

        currentSorterStates = state;
    }

//    public void setLed(boolean enable) {
//        if (colorSensorV3 instanceof SwitchableLight) {
//            ((SwitchableLight) colorSensorV3).enableLight(enable);
//        }
//        ledEnabled = enable;
//    }

    public boolean isLedEnabled() { return ledEnabled; }

    public boolean getMagneticSensor() {
        return !magneticSensor.getState();
    }

    public void setPower(int pos) {
        sorterRotate.turnToAngle(pos);
        switch (pos) {
            case 1:
                switch (currentSorterStates) {
                    case STATE_ONE:
                        currentSorterStates = SorterStates.STATE_TWO;
                        break;
                    case STATE_TWO:
                        currentSorterStates = SorterStates.STATE_THREE;
                        break;
                    case STATE_THREE:
                        currentSorterStates = SorterStates.STATE_ONE;
                        break;
                }
                break;
            case -1:
                switch (currentSorterStates) {
                    case STATE_ONE:
                        currentSorterStates = SorterStates.STATE_THREE;
                        break;
                    case STATE_TWO:
                        currentSorterStates = SorterStates.STATE_ONE;
                        break;
                    case STATE_THREE:
                        currentSorterStates = SorterStates.STATE_TWO;
                        break;
                }

        }
    }

    public SorterStates getCurrentSorterState() {
        return currentSorterStates;
    }
}
