package frc.robot.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
    @AutoLog
    public static class DrivetrainIOInputs {
        public double leftCurrent;
        public double leftDistance;
        public double leftPosition;
        public double leftTemperature;
        public double leftVelocity;
        public double leftVoltage;

        public double rightCurrent;
        public double rightDistance;
        public double rightPosition;
        public double rightTemperature;
        public double rightVelocity;
        public double rightVoltage;
    }

    /** 
     * Updates the logged inputs for the drivetrian.
     * 
     * @param inputs The inputs object to update.
     */
    public void updateInputs();

    /**
     * Returns the current draw of the left side of the robot.
     * 
     * @return The current draw of the left side of the robot in Amps.
     */
    public double getLeftCurrent();

    /**
     * Returns the distance that the left side of the robot has traveled.
     * 
     * @return The distance that the left side of the robot has traveled in meters.
     */
    public double getLeftDistance();
    
    public double getLeftPosition();
    
    public double getLeftTemperature();
    
    public double getLeftVelocity();

    public double getLeftVoltage();
    public void setLeftVoltage(double volts);

    public double getRightCurrent();
    
    public double getRightDistance();
    
    public double getRightPosition();
    
    public double getRightTemperature();

    public double getRightVelocity();
    
    public double getRightVoltage();
    public void setRightVoltage(double volts);
}