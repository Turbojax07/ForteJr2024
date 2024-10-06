package frc.robot.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
    @AutoLog
    // Inputs that will be saved to the logfile.
    public class DrivetrainIOInputs {
        public double leftVelocity;
        public double rightVelocity;
    }

    public void updateInputs(DrivetrainIOInputs inputs);
    public double getLeftSpeed();
    public void setLeftSpeed(double speed);
    public double getRightSpeed();
    public void setRightSpeed(double speed);
}