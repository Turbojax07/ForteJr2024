package frc.robot.Drivetrain;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.DriveConstants;

public class DrivetrainIOTalonSRX implements DrivetrainIO {
    private TalonSRX flMotor;
    private TalonSRX frMotor;
    private TalonSRX blMotor;
    private TalonSRX brMotor;

    private Pigeon2 gyro;

    public DrivetrainIOTalonSRX() {
        // Initializing the motors
        flMotor = new TalonSRX(DriveConstants.frontLeftID);
        frMotor = new TalonSRX(DriveConstants.frontRightID);
        blMotor = new TalonSRX(DriveConstants.backLeftID);
        brMotor = new TalonSRX(DriveConstants.backRightID);

        // Resetting each TalonSRX's settings.
        // The loops make sure that the reset is completed.
        ErrorCode errorCode = ErrorCode.GENERAL_ERROR;
        while (errorCode.value != 0) {
            errorCode = flMotor.configFactoryDefault();
        }
        errorCode = ErrorCode.GENERAL_ERROR;

        while (errorCode.value != 0) {
            errorCode = frMotor.configFactoryDefault();
        }
        errorCode = ErrorCode.GENERAL_ERROR;

        while (errorCode.value != 0) {
            errorCode = blMotor.configFactoryDefault();
        }
        errorCode = ErrorCode.GENERAL_ERROR;

        while (errorCode.value != 0) {
            errorCode = brMotor.configFactoryDefault();
        }
        errorCode = ErrorCode.GENERAL_ERROR;


        // Inverting the motors
        flMotor.setInverted(true);
        frMotor.setInverted(false);
        blMotor.setInverted(true);
        brMotor.setInverted(false);

        // Putting the motors into brake mode.
        flMotor.setNeutralMode(NeutralMode.Brake);
        frMotor.setNeutralMode(NeutralMode.Brake);
        blMotor.setNeutralMode(NeutralMode.Brake);
        brMotor.setNeutralMode(NeutralMode.Brake);

        // Having the back motors follow the front motors.
        blMotor.follow(flMotor);
        brMotor.follow(frMotor);
    }

    /**
     * This function updates the logged inputs.
     * It should be placed in a periodic function somewhere.
     * 
     * @param inputs An instance of the class that contains the inputs that need to be logged.
     */
    @Override
    public void updateInputs(DrivetrainIOInputs inputs) {
        inputs.leftVelocity = getLeftSpeed();
        inputs.rightVelocity = getRightSpeed();
        inputs.heading = getHeading();
    }

    /**
     * Gets the speed of the left side of the robot.
     * 
     * @return The speed of the left side of the robot in percent output.
     */
    @Override
    public double getLeftSpeed() {
        return flMotor.getMotorOutputPercent();
    }

    /**
     * Sets the speed of the left side of the robot.
     * 
     * @param speed The speed of the left side of the robot in percent output.
     */
    @Override
    public void setLeftSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        flMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Gets the speed of the right side of the robot.
     * 
     * @return The speed of the right side of the robot in percent output.
     */
    @Override
    public double getRightSpeed() {
        return frMotor.getMotorOutputPercent();
    }

    /**
     * Sets the speed of the right side of the robot.
     * 
     * @param speed The speed of the right side of the robot in percent output.
     */
    @Override
    public void setRightSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        frMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Gets the heading of the robot.
     * 
     * @return The heading of the robot as a {@link Rotation2d}.
     */
    @Override
    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    /**
     * Resets the heading of the robot.
     * This does not actually drive the robot so that it faces the specified angle.
     * 
     * @param heading The heading of the robot as a {@link Rotation2d}
     */
    @Override
    public void resetHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }
}