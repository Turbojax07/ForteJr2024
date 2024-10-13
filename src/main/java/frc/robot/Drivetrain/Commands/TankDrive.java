// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;

/** A command that drives a {@link Drivetrain} using open loop tank control. */
public class TankDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Drivetrain drivetrain;
    private Supplier<Double> lSpeedSupplier;
    private Supplier<Double> rSpeedSupplier;

    /**
     * Creates a new TankDrive command.
     * This command drives a {@link Drivetrain} using open loop controls.
     * It drives with one joystick controlling the speed of the left side of the robot, and another joystick controlling the right side of the robot.
     *
     * @param lSpeedSupplier The supplier for the left speed.
     * @param rSpeedSupplier The supplier for the right speed.
     */
    public TankDrive(Supplier<Double> lSpeedSupplier, Supplier<Double> rSpeedSupplier) {
        drivetrain = Drivetrain.getInstance();
        this.lSpeedSupplier = lSpeedSupplier;
        this.rSpeedSupplier = rSpeedSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called every time the scheduler runs while this command is scheduled. */
    @Override
    public void execute() {
        // Executing the suppliers
        double lSpeed = lSpeedSupplier.get();
        double rSpeed = rSpeedSupplier.get();

        // Applying a deadband
        lSpeed = MathUtil.applyDeadband(lSpeed, ControllerConstants.deadband);
        rSpeed = MathUtil.applyDeadband(rSpeed, ControllerConstants.deadband);

        // Smoothing out the deadband (prevents jumping from 0% to 10%)
        if (lSpeed != 0 && lSpeed > 0) lSpeed -= ControllerConstants.deadband;
        if (lSpeed != 0 && lSpeed < 0) lSpeed += ControllerConstants.deadband;
        if (rSpeed != 0 && rSpeed > 0) rSpeed -= ControllerConstants.deadband;
        if (rSpeed != 0 && rSpeed < 0) rSpeed += ControllerConstants.deadband;

        // Scaling for max speeds
        lSpeed *= DriveConstants.maxOpenDriveSpeed;
        rSpeed *= DriveConstants.maxOpenTurnSpeed;

        // Driving the robot
        drivetrain.tankDrive(lSpeed, rSpeed);
    }

    /**
     * Called every time the scheduler runs while this command is scheduled.
     * 
     * @return Whether or not the command should be canceled.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called when the command is cancelled, either by the scheduler or when {@link TankDrive#isFinished} returns true. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }
}