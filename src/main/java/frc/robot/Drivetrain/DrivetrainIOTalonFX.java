package frc.robot.Drivetrain;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class DrivetrainIOTalonFX implements DrivetrainIO {
    private TalonFX flMotor;
    private TalonFX frMotor;
    private TalonFX blMotor;
    private TalonFX brMotor;

    private DrivetrainIOInputsAutoLogged inputs;

    public DrivetrainIOTalonFX() {
        // Initializing the motors
        flMotor = new TalonFX(DriveConstants.frontLeftID);
        frMotor = new TalonFX(DriveConstants.frontRightID);
        blMotor = new TalonFX(DriveConstants.backLeftID);
        brMotor = new TalonFX(DriveConstants.backRightID);

        // Resetting the motors
        // The loops either force the configuration to finish, or prevent the program from continuing.
        while (flMotor.getConfigurator().apply(new TalonFXConfiguration()) != StatusCode.OK) {}
        while (frMotor.getConfigurator().apply(new TalonFXConfiguration()) != StatusCode.OK) {}
        while (blMotor.getConfigurator().apply(new TalonFXConfiguration()) != StatusCode.OK) {}
        while (brMotor.getConfigurator().apply(new TalonFXConfiguration()) != StatusCode.OK) {}

        // Inverting the motors
        flMotor.setInverted(true);
        frMotor.setInverted(false);
        blMotor.setInverted(true);
        brMotor.setInverted(false);

        // Setting the neutral mode of the motors.
        flMotor.setNeutralMode(NeutralModeValue.Brake);
        frMotor.setNeutralMode(NeutralModeValue.Brake);
        blMotor.setNeutralMode(NeutralModeValue.Brake);
        brMotor.setNeutralMode(NeutralModeValue.Brake);

        // Making the back motors follow the front motors.
        blMotor.setControl(new Follower(flMotor.getDeviceID(), false));
        brMotor.setControl(new Follower(frMotor.getDeviceID(), false));

        inputs = new DrivetrainIOInputsAutoLogged();
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

        Logger.processInputs("DrivetrainTalonFXInputs", inputs);
    }

    @Override
    public double getLeftCurrent() {
        return (flMotor.getStatorCurrent().getValue() + blMotor.getStatorCurrent().getValue()) / 2.0;
    }

    @Override
    public double getLeftDistance() {
        return flMotor.getPosition().getValue() * DriveConstants.rotToMeters;
    }

    @Override
    public double getLeftPosition() {
        return (flMotor.getPosition().getValue() % 1) * 2 * Math.PI;
    }

    @Override
    public double getLeftTemperature() {
        return Units.Celsius.of(flMotor.getDeviceTemp().getValue()).in(Units.Fahrenheit);
    }

    @Override
    public double getLeftVelocity() {
        return flMotor.getVelocity().getValue() * DriveConstants.rotToMeters;
    }

    @Override
    public double getLeftVoltage() {
        return flMotor.getMotorVoltage().getValue();
    }

    @Override
    public void setLeftVoltage(double volts) {
        flMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public double getRightCurrent() {
        return (frMotor.getStatorCurrent().getValue() + brMotor.getStatorCurrent().getValue()) / 2.0;
    }

    @Override
    public double getRightDistance() {
        return frMotor.getPosition().getValue() * DriveConstants.rotToMeters;
    }

    @Override
    public double getRightPosition() {
        return (frMotor.getPosition().getValue() % 1) * 2 * Math.PI;
    }

    @Override
    public double getRightTemperature() {
        return Units.Celsius.of(frMotor.getDeviceTemp().getValue()).in(Units.Fahrenheit);
    }

    @Override
    public double getRightVelocity() {
        return frMotor.getVelocity().getValue() * DriveConstants.rotToMeters;
    }

    @Override
    public double getRightVoltage() {
        return frMotor.getMotorVoltage().getValue();
    }

    @Override
    public void setRightVoltage(double volts) {
        frMotor.setControl(new VoltageOut(volts));
    }
}