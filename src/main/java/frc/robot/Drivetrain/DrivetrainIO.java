package frc.robot.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DrivetrainIO {
    @AutoLog
    // Inputs that will be saved to the logfile.
    public class DrivetrainIOInputs {
        public double leftVelocity;
        public double rightVelocity;
        public Rotation2d heading;
    }

    public void updateInputs(DrivetrainIOInputs inputs);
    public double getLeftSpeed();
    public void setLeftSpeed(double speed);
    public double getRightSpeed();
    public void setRightSpeed(double speed);
    public Rotation2d getHeading();
    public void resetHeading(Rotation2d heading);
}