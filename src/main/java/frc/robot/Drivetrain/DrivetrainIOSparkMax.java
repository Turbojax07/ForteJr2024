package frc.robot.Drivetrain;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.Units;
import frc.robot.Constants.DriveConstants;

public class DrivetrainIOSparkMax implements DrivetrainIO {
    private CANSparkMax flMotor;
    private CANSparkMax frMotor;
    private CANSparkMax blMotor;
    private CANSparkMax brMotor;

    private RelativeEncoder flEncoder;
    private RelativeEncoder frEncoder;

    private DrivetrainIOInputsAutoLogged inputs;

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

        inputs = new DrivetrainIOInputsAutoLogged();
    }

    @Override
    public void updateInputs() {
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

        Logger.processInputs("DrivetrainSparkMaxInputs", inputs);
    }

    @Override
    public double getLeftCurrent() {
        return (flMotor.getOutputCurrent() + blMotor.getOutputCurrent()) / 2.0;
    }

    @Override
    public double getLeftDistance() {
        return flEncoder.getPosition() * DriveConstants.rotToMeters;
    }

    @Override
    public double getLeftPosition() {
        return (flEncoder.getPosition() % 1) * 2 * Math.PI;
    }

    @Override
    public double getLeftTemperature() {
        return Units.Celsius.of(flMotor.getMotorTemperature()).in(Units.Fahrenheit);
    }

    @Override
    public double getLeftVelocity() {
        return flEncoder.getVelocity() * DriveConstants.rotToMeters / 60;
    }

    @Override
    public double getLeftVoltage() {
        return flMotor.getBusVoltage();
    }

    @Override
    public void setLeftVoltage(double volts) {
        flMotor.setVoltage(volts);
    }

    @Override
    public double getRightCurrent() {
        return (frMotor.getOutputCurrent() + brMotor.getOutputCurrent()) / 2.0;
    }

    @Override
    public double getRightDistance() {
        return frEncoder.getPosition() * DriveConstants.rotToMeters;
    }

    @Override
    public double getRightPosition() {
        return (frEncoder.getPosition() % 1) * 2 * Math.PI;
    }

    @Override
    public double getRightTemperature() {
        return Units.Celsius.of(frMotor.getMotorTemperature()).in(Units.Fahrenheit);
    }

    @Override
    public double getRightVelocity() {
        return frEncoder.getVelocity() * DriveConstants.rotToMeters / 60;
    }

    @Override
    public double getRightVoltage() {
        return frMotor.getBusVoltage();
    }

    @Override
    public void setRightVoltage(double volts) {
        frMotor.setVoltage(volts);
    }
}