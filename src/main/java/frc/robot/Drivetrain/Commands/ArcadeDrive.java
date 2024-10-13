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
public class ArcadeDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain drivetrain;
    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> zRotatSupplier;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArcadeDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> zRotateSupplier) {
        this.drivetrain = Drivetrain.getInstance();
        this.xSpeedSupplier = xSpeedSupplier;
        this.zRotatSupplier = zRotateSupplier;

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
        double xSpeed = xSpeedSupplier.get();
        double zRotat = zRotatSupplier.get();

        // Applying a deadband
        MathUtil.applyDeadband(xSpeed, ControllerConstants.deadband);
        MathUtil.applyDeadband(zRotat, ControllerConstants.deadband);

        // Smoothing out the deadband (prevents jumping from 0% to 10%)
        if (xSpeed > 0) xSpeed -= ControllerConstants.deadband;
        if (xSpeed < 0) xSpeed += ControllerConstants.deadband;
        if (zRotat > 0) zRotat -= ControllerConstants.deadband;
        if (zRotat < 0) zRotat += ControllerConstants.deadband;

        // Scaling for max speeds
        xSpeed *= DriveConstants.maxOpenDriveSpeed;
        zRotat *= DriveConstants.maxOpenTurnSpeed;

        // Driving the robot
        drivetrain.arcadeDrive(xSpeedSupplier.get(), zRotatSupplier.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}