package frc.robot.Drivetrain;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class DrivetrainIOSim implements DrivetrainIO {
    private DifferentialDrivetrainSim driveSim;

    private DrivetrainIOInputsAutoLogged inputs;

    private double leftVolts;
    private double rightVolts;
    
    public DrivetrainIOSim() {
        inputs = new DrivetrainIOInputsAutoLogged();

        driveSim = new DifferentialDrivetrainSim(DCMotor.getNEO(2), DriveConstants.gearRatio, DriveConstants.momentOfInertia, DriveConstants.mass, DriveConstants.wheelRadius, DriveConstants.robotWidth, null);
    }

    /**
     * This function updates the logged inputs.
     * It should be placed in a periodic function somewhere.
     * 
     * @param inputs An instance of the class that contains the inputs that need to be logged.
     */
    @Override
    public void updateInputs() {
        driveSim.setInputs(leftVolts, rightVolts);

        driveSim.update(0.02);

        // Voltages
        inputs.leftVoltage = getLeftVoltage();
        inputs.rightVoltage = getRightVoltage();

        // Distance
        inputs.leftDistance = getLeftDistance();
        inputs.rightDistance = getRightDistance();

        // Positions
        inputs.leftPosition = getLeftPosition();
        inputs.rightPosition = getRightPosition();

        // Current
        inputs.leftCurrent = getLeftCurrent();
        inputs.rightCurrent = getRightCurrent();

        // Temperature
        inputs.leftTemperature = getLeftTemperature();
        inputs.rightTemperature = getRightTemperature();

        // Velocity
        inputs.leftVelocity = getLeftVelocity();
        inputs.rightVelocity = getRightVelocity();

        Logger.processInputs("DrivetrainSimInputs", inputs);
    }

    @Override
    public double getLeftCurrent() {
        return driveSim.getLeftCurrentDrawAmps();
    }

    @Override
    public double getLeftDistance() {
        return driveSim.getLeftPositionMeters();
    }

    @Override
    public double getLeftPosition() {
        return driveSim.getLeftPositionMeters() * 2 * Math.PI / DriveConstants.rotToMeters;
    }

    @Override
    public double getLeftTemperature() {
        return 0;
    }

    @Override
    public double getLeftVoltage() {
        return leftVolts;
    }

    @Override
    public void setLeftVoltage(double volts) {
        leftVolts = volts;
    }

    @Override
    public double getLeftVelocity() {
        return driveSim.getLeftVelocityMetersPerSecond();
    }

    @Override
    public double getRightCurrent() {
        return driveSim.getRightCurrentDrawAmps();
    }

    @Override
    public double getRightDistance() {
        return driveSim.getRightPositionMeters();
    }

    @Override
    public double getRightPosition() {
        return driveSim.getRightPositionMeters() * 2 * Math.PI / DriveConstants.rotToMeters;
    }

    @Override
    public double getRightTemperature() {
        return 0;
    }

    @Override
    public double getRightVoltage() {
        return rightVolts;
    }

    @Override
    public void setRightVoltage(double volts) {
        rightVolts = volts;
    }

    @Override
    public double getRightVelocity() {
        return driveSim.getRightVelocityMetersPerSecond();
    }
}
