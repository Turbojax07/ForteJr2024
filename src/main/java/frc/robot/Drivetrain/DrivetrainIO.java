package frc.robot.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
    @AutoLog
    public static class DrivetrainIOInputs {
        public static double leftVelocity;
        public static double rightVelocity;
    }

    public void updateInputs();
    public double getLeftSpeed();
    public void setLeftSpeed(double speed);
    public double getRightSpeed();
    public void setRightSpeed(double speed);
    
}