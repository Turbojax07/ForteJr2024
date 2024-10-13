package frc.robot.Drivetrain.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.Drivetrain;
import java.util.function.Supplier;

/** A command that drives a {@link Drivetrain} using closed loop control. */
public class ClosedLoopDrive extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Drivetrain drivetrain;
    private Supplier<Double> xSpeedSupplier;
    private Supplier<Double> zRotatSupplier;

    /**
     * Creates a new ClosedLoopDrive command.
     * This command drives a {@link Drivetrain} using a closed-loop PID controller.
     * 
     * @param xSpeedSupplier The supplier for the x translational speed.
     * @param zRotateSupplier The supplier for the z rotational speed.
     */
    public ClosedLoopDrive(Supplier<Double> xSpeedSupplier, Supplier<Double> zRotateSupplier) {
        this.drivetrain = Drivetrain.getInstance();
        this.xSpeedSupplier = xSpeedSupplier;
        this.zRotatSupplier = zRotateSupplier;

        // Preventing the robot from being driven by multiple commands.
        addRequirements(drivetrain);
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called every time the scheduler runs while this command is scheduled. */
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

    /**
     * Called every time the scheduler runs while this command is scheduled.
     * 
     * @return Whether or not the command should be canceled.
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /** Called when the command is cancelled, either by the scheduler or when {@link ClosedLoopDrive#isFinished} returns true. */
    @Override
    public void end(boolean interrupted) {
        drivetrain.closedLoop(new ChassisSpeeds());
    }
}