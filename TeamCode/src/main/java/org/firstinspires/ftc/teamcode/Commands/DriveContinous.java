package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveContinous extends CommandBase {
    private Drivetrain drivetrain;
    private GamepadEx driver;
    private double speed;

    public DriveContinous(Drivetrain drivetrain, GamepadEx driver, double speed) {
        this.drivetrain = drivetrain;
        this.driver = driver;
        this.speed = speed;

        addRequirements(drivetrain);
    }

//    @Override
//    public void initialize(){
//
//    }

    @Override
    public void execute(){
        PoseVelocity2d pose = new PoseVelocity2d(
                new Vector2d(
                        driver.getLeftY() * speed,
                        -driver.getLeftX() * speed
                        ),
                -driver.getRightX() * speed); // Normally we wouldn't need to invert
        drivetrain.setDrivePowers(pose);
    }

    @Override
    public void end(boolean isInterrupted){
        PoseVelocity2d pose = new PoseVelocity2d(
                new Vector2d(
                        0,
                        0
                ),
                0);
        drivetrain.setDrivePowers(pose);
    }
}
