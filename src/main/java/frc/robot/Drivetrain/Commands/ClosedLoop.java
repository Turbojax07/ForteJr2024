package frc.robot.Drivetrain.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;
import java.util.function.Supplier;

public class ClosedLoop extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> zRotatSupplier;

    public ClosedLoop(Supplier<Double> xSpeedSupplier, Supplier<Double> zRotateSupplier) {
        this.drivetrain = Drivetrain.getInstance();
        this.xSpeedSupplier = xSpeedSupplier;
        this.zRotatSupplier = zRotateSupplier;
    }

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
        xSpeed *= DriveConstants.maxClosedDriveSpeed;
        zRotat *= DriveConstants.maxClosedTurnSpeed;
        
        // Driving the robot
        drivetrain.closedLoop(new ChassisSpeeds(xSpeed, 0, zRotat));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.closedLoop(new ChassisSpeeds());
    }
}