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

/** An example command that uses an example subsystem. */
public class TankDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Drivetrain drivetrain;
    private Supplier<Double> lSpeedSupplier;
    private Supplier<Double> rSpeedSupplier;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public TankDrive(Supplier<Double> lSpeedSupplier, Supplier<Double> rSpeedSupplier) {
        drivetrain = Drivetrain.getInstance();
        this.lSpeedSupplier = lSpeedSupplier;
        this.rSpeedSupplier = rSpeedSupplier;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
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

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.tankDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}