package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;

public class DrivetrainIOSparkMax implements DrivetrainIO {
    private CANSparkMax flMotor;
    private CANSparkMax frMotor;
    private CANSparkMax blMotor;
    private CANSparkMax brMotor;

    private RelativeEncoder flEncoder;
    private RelativeEncoder frEncoder;

    public DrivetrainIOSparkMax() {
        // Initializing the motors
        flMotor = new CANSparkMax(DriveConstants.frontLeftID,  MotorType.kBrushless);
        frMotor = new CANSparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
        blMotor = new CANSparkMax(DriveConstants.backLeftID,   MotorType.kBrushless);
        brMotor = new CANSparkMax(DriveConstants.backRightID,  MotorType.kBrushless);

        // Resetting the motor configs
        // The loops make sure that the configs are completed.
        while (flMotor.restoreFactoryDefaults() != REVLibError.kOk) {}
        while (frMotor.restoreFactoryDefaults() != REVLibError.kOk) {}
        while (blMotor.restoreFactoryDefaults() != REVLibError.kOk) {}
        while (brMotor.restoreFactoryDefaults() != REVLibError.kOk) {}

        // Inverting the motors
        flMotor.setInverted(true);
        frMotor.setInverted(false);
        blMotor.setInverted(true);
        brMotor.setInverted(false);

        // Setting the idle mode of the motors
        while (flMotor.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {}
        while (frMotor.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {}
        while (blMotor.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {}
        while (brMotor.setIdleMode(IdleMode.kBrake) != REVLibError.kOk) {}

        // Making the back motors follow the front motors
        while (blMotor.follow(flMotor) != REVLibError.kOk) {}
        while (brMotor.follow(frMotor) != REVLibError.kOk) {}

        // Getting the encoder for each motor
        flEncoder = flMotor.getEncoder();
        frEncoder = frMotor.getEncoder();

        // Saving the configs
        while (flMotor.burnFlash() != REVLibError.kOk) {}
        while (frMotor.burnFlash() != REVLibError.kOk) {}
        while (blMotor.burnFlash() != REVLibError.kOk) {}
        while (brMotor.burnFlash() != REVLibError.kOk) {}
    }

    /**
     * This function updates the logged inputs.
     * It should be placed in a periodic function somewhere.
     * 
     * @param inputs An instance of the class that contains the inputs that need to be logged.
     */
    @Override
    public void updateInputs() {
        // Voltages
        DrivetrainIOInputsAutoLogged.leftVoltage = getLeftVoltage();
        DrivetrainIOInputsAutoLogged.rightVoltage = getRightVoltage();

        // Percentages
        DrivetrainIOInputsAutoLogged.leftPercent = getLeftPercent();
        DrivetrainIOInputsAutoLogged.rightPercent = getRightPercent();

        // Positions
        DrivetrainIOInputsAutoLogged.leftPosition = flEncoder.getPosition();
        DrivetrainIOInputsAutoLogged.rightPosition = frEncoder.getPosition();

        // Current
        DrivetrainIOInputsAutoLogged.flCurrent = flMotor.getOutputCurrent();
        DrivetrainIOInputsAutoLogged.frCurrent = frMotor.getOutputCurrent();
        DrivetrainIOInputsAutoLogged.blCurrent = blMotor.getOutputCurrent();
        DrivetrainIOInputsAutoLogged.brCurrent = brMotor.getOutputCurrent();

        // Temperature
        DrivetrainIOInputsAutoLogged.flTemperature = flMotor.getMotorTemperature();
        DrivetrainIOInputsAutoLogged.frTemperature = frMotor.getMotorTemperature();
        DrivetrainIOInputsAutoLogged.blTemperature = blMotor.getMotorTemperature();
        DrivetrainIOInputsAutoLogged.brTemperature = brMotor.getMotorTemperature();
    }

    /**
     * Gets the speed of the left side of the robot.
     * 
     * @return The speed of the left side of the robot in percent output.
     */
    @Override
    public double getLeftPercent() {
        return flMotor.getAppliedOutput();
    }

    /**
     * Sets the speed of the left side of the robot.
     * 
     * @param percent The speed of the left side of the robot in percent output.
     */
    @Override
    public void setLeftPercent(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        flMotor.set(percent);
    }

    /**
     * Gets the speed of the right side of the robot.
     * 
     * @return The speed of the right side of the robot in percent output.
     */
    @Override
    public double getRightPercent() {
        return frMotor.getAppliedOutput();
    }

    /**
     * Sets the speed of the right side of the robot.
     * 
     * @param percent The speed of the right side of the robot in percent output.
     */
    @Override
    public void setRightPercent(double percent) {
        percent = MathUtil.clamp(percent, -1, 1);
        frMotor.set(percent);
    }

    /**
     * Gets the speed of the right side of the robot.
     * 
     * @return The speed of the right side of the robot in volts.
     */
    @Override
    public double getLeftVoltage() {
        return flMotor.getAppliedOutput() * flMotor.getBusVoltage();
    }

    /**
     * Sets the speed of the right side of the robot.
     * 
     * @param volts The speed of the right side of the robot in volts.
     */
    @Override
    public void setLeftVoltage(double volts) {
        volts = MathUtil.clamp(volts, -1, 1);
        flMotor.set(volts);
    }

    /**
     * Gets the speed of the right side of the robot.
     * 
     * @return The speed of the right side of the robot in volts.
     */
    @Override
    public double getRightVoltage() {
        return frMotor.getAppliedOutput() * frMotor.getBusVoltage();
    }

    /**
     * Sets the speed of the right side of the robot.
     * 
     * @param volts The speed of the right side of the robot in volts.
     */
    @Override
    public void setRightVoltage(double volts) {
        volts = MathUtil.clamp(volts, -1, 1);
        frMotor.set(volts);
    }
}