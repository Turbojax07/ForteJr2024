package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.DriveConstants;

public class DrivetrainIOSparkMax implements DrivetrainIO {
    private CANSparkMax flMotor;
    private CANSparkMax frMotor;
    private CANSparkMax blMotor;
    private CANSparkMax brMotor;

    public DrivetrainIOSparkMax() {
        // Initializing the motors
        flMotor = new CANSparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);
        frMotor = new CANSparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
        blMotor = new CANSparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
        brMotor = new CANSparkMax(DriveConstants.backRightID, MotorType.kBrushless);

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

        // Saving the configs
        while (flMotor.burnFlash() != REVLibError.kOk) {}
        while (frMotor.burnFlash() != REVLibError.kOk) {}
        while (blMotor.burnFlash() != REVLibError.kOk) {}
        while (brMotor.burnFlash() != REVLibError.kOk) {}
    }

    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        inputs.leftVelocity = getLeftSpeed();
        inputs.rightVelocity = getRightSpeed();
    }

    @Override
    public double getLeftSpeed() {
        return flMotor.get();
    }

    @Override
    public void setLeftSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        flMotor.set(speed);
    }

    @Override
    public double getRightSpeed() {
        return frMotor.get();
    }

    @Override
    public void setRightSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        frMotor.set(speed);
    }
}