package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherBall implements Subsystem {
    private final MotorEx leftShooter;
    private final MotorEx rightShooter;


    // Encoder resolution calculations
    public static final double MOTOR_TPR = 28;  // ticks per rev, 1:1 gearbox
    public static final double GEAR_RATIO = 1.5; // Upgearing 1.5:1
    public static final double TICKS_PER_REV = MOTOR_TPR / GEAR_RATIO;

    // Target and readiness tunables
    private double targetRpm = 6000;
    public double readyToleranceRpm  = 50;
    private double readyHoldTimeSeconds = 0.10;   // how long we must stay in the tolerance

    // Ready to launch state (maintained in periodic)
    private boolean isReadyToLaunch = false;
    private long inTolStartNanos = 0L;


    public LauncherBall(HardwareMap hm) {
        leftShooter = new MotorEx(hm, "leftShooter");
        rightShooter = new MotorEx(hm, "rightShooter");

        leftShooter.setInverted(false);
        rightShooter.setInverted(false);

        leftShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        leftShooter.set(power);
        rightShooter.set(power);
    }

    public void stop() {
        leftShooter.stopMotor();
        rightShooter.stopMotor();
        // Reset readiness when we stop
        inTolStartNanos = 0L;
        isReadyToLaunch = false;
    }

    // ----------------- Target & tuning -----------------
    public void setTargetRpm(double rpm) { targetRpm = rpm; }
    public double getTargetRpm() { return targetRpm; }

    public void setReadyToleranceRpm(double tol) { readyToleranceRpm = Math.max(0, tol); }
    public double getReadyToleranceRpm() { return readyToleranceRpm; }

    public void setReadyHoldTimeSeconds(double sec) { readyHoldTimeSeconds = Math.max(0, sec); }
    public double getReadyHoldTimeSeconds() { return readyHoldTimeSeconds; }

    // ----------------- Telemetry helpers -----------------
    public double getLeftTicksPerSec()  { return leftShooter.getVelocity(); }
    public double getRightTicksPerSec() { return rightShooter.getVelocity(); }

    public double getLeftRPM()  { return (getLeftTicksPerSec()  * 60.0) / TICKS_PER_REV; }
    public double getRightRPM() { return (getRightTicksPerSec() * 60.0) / TICKS_PER_REV; }
    public double getAverageRPM() { return (getLeftRPM() + getRightRPM()) / 2.0; }

    // In case you move to REV velocity control later:
    public double rpmToTicksPerSec(double rpm) { return (rpm * TICKS_PER_REV) / 60.0; }

    // ----------------- Readiness interface -----------------
    public boolean isReadyToLaunch() { return isReadyToLaunch; }

    @Override
    public void periodic() {
        double leftErr = Math.abs(getLeftRPM() - targetRpm);
        double rightErr = Math.abs(getRightRPM() - targetRpm);
        boolean isInTolerance = leftErr <= readyToleranceRpm && rightErr <= readyToleranceRpm;

        long now = System.nanoTime();
        if(isInTolerance) {
            if (inTolStartNanos == 0L) inTolStartNanos = now;
            double heldSec = (now - inTolStartNanos) / 1e9;
            isReadyToLaunch = heldSec >= readyHoldTimeSeconds;
        } else {
            inTolStartNanos = 0L;
            isReadyToLaunch = false;
        }

        telemetry.addData("Target", getTargetRpm());
        telemetry.addData("LeftRPM", getLeftRPM());
        telemetry.addData("RightRPM", getRightRPM());
        telemetry.addData("Ready", isReadyToLaunch);

    }
}
