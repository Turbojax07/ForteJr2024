package frc.robot.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface DrivetrainIO {
    @AutoLog
    public static class DrivetrainIOInputs {
        public static double leftVoltage; // Volts
        public static double rightVoltage; // Volts

        public static double leftPercent; // Percent
        public static double rightPercent; // Percent

        public static double leftPosition; // Meters
        public static double rightPosition; // Meters

        public static double flCurrent; // Amps
        public static double frCurrent; // Amps
        public static double blCurrent; // Amps
        public static double brCurrent; // Amps

        public static double flTemperature; // Celsius
        public static double frTemperature; // Celsius
        public static double blTemperature; // Celsius
        public static double brTemperature; // Celsius
    }

    public void updateInputs();

    public double getLeftPercent();
    public void setLeftPercent(double percent);

    public double getRightPercent();
    public void setRightPercent(double percent);

    public double getLeftVoltage();
    public void setLeftVoltage(double volts);

    public double getRightVoltage();
    public void setRightVoltage(double volts);
    
}