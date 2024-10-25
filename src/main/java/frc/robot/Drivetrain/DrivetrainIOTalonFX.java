package frc.robot.Drivetrain;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;

public class DrivetrainIOTalonFX implements DrivetrainIO {
    private TalonFX flMotor;
    private TalonFX frMotor;
    private TalonFX blMotor;
    private TalonFX brMotor;

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
    }

    /**
     * This function updates the logged inputs.
     * It should be placed in a periodic function somewhere.
     * 
     * @param inputs An instance of the class that contains the inputs that need to be logged.
     */
    @Override
    public void updateInputs() {
        DrivetrainIOInputs.leftVelocity = getLeftSpeed();
        DrivetrainIOInputs.rightVelocity = getRightSpeed();
    }

    /**
     * Gets the speed of the left side of the robot.
     * 
     * @return The speed of the left side of the robot in percent output.
     */
    @Override
    public double getLeftSpeed() {
        return flMotor.getDutyCycle().getValueAsDouble();
    }

    /**
     * Gets the speed of the left side of the robot.
     * 
     * @return The speed of the left side of the robot in percent output.
     */
    @Override
    public void setLeftSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        flMotor.setControl(new DutyCycleOut(speed));
    }

    /**
     * Gets the speed of the left side of the robot.
     * 
     * @return The speed of the left side of the robot in percent output.
     */
    @Override
    public double getRightSpeed() {
        return frMotor.getDutyCycle().getValueAsDouble();
    }
    /**
     * Gets the speed of the right side of the robot.
     * 
     * @return The speed of the right side of the robot in percent output.
     */
    @Override
    public void setRightSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        frMotor.setControl(new DutyCycleOut(speed));
    }
}