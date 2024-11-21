package frc.robot.Drivetrain;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.DriveConstants;

public class DrivetrainIOSim implements DrivetrainIO {
    private TalonFX flMotor;
    private TalonFX frMotor;
    private TalonFX blMotor;
    private TalonFX brMotor;
    

    public DrivetrainIOSim() {
        // Initializing Motors
        flMotor = new TalonFX(DriveConstants.frontLeftID);
        frMotor = new TalonFX(DriveConstants.frontRightID);
        blMotor = new TalonFX(DriveConstants.backLeftID);
        brMotor = new TalonFX(DriveConstants.backRightID);

        // resetting
        flMotor.getConfigurator().apply(new TalonFXConfiguration());
        frMotor.getConfigurator().apply(new TalonFXConfiguration());
        blMotor.getConfigurator().apply(new TalonFXConfiguration());
        brMotor.getConfigurator().apply(new TalonFXConfiguration());

        // neutral modes
        flMotor.setNeutralMode(NeutralModeValue.Brake);
        frMotor.setNeutralMode(NeutralModeValue.Brake);
        blMotor.setNeutralMode(NeutralModeValue.Brake);
        brMotor.setNeutralMode(NeutralModeValue.Brake);

        // inversions
        flMotor.setInverted(true);
        frMotor.setInverted(false);
        blMotor.setInverted(true);
        brMotor.setInverted(false);

        // followers
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

        // Voltages
        DrivetrainIOInputsAutoLogged.leftVoltage = getLeftVoltage();
        DrivetrainIOInputsAutoLogged.rightVoltage = getRightVoltage();

        // Percentages
        DrivetrainIOInputsAutoLogged.leftPercent = getLeftPercent();
        DrivetrainIOInputsAutoLogged.rightPercent = getRightPercent();

        // Positions
        DrivetrainIOInputsAutoLogged.leftPosition = flMotor.getPosition().getValue() * DriveConstants.rotToMeters;
        DrivetrainIOInputsAutoLogged.rightPosition = frMotor.getPosition().getValue() * DriveConstants.rotToMeters;

        // Current
        DrivetrainIOInputsAutoLogged.flCurrent = flMotor.getStatorCurrent().getValue();
        DrivetrainIOInputsAutoLogged.frCurrent = frMotor.getStatorCurrent().getValue();
        DrivetrainIOInputsAutoLogged.blCurrent = blMotor.getStatorCurrent().getValue();
        DrivetrainIOInputsAutoLogged.brCurrent = brMotor.getStatorCurrent().getValue();

        // Temperature
        DrivetrainIOInputsAutoLogged.flTemperature = 0;
        DrivetrainIOInputsAutoLogged.frTemperature = 0;
        DrivetrainIOInputsAutoLogged.blTemperature = 0;
        DrivetrainIOInputsAutoLogged.brTemperature = 0;
    }

    /**
     * Gets the speed of the left side of the robot.
     * 
     * @return The speed of the left side of the robot in percent output.
     */
    @Override
    public double getLeftPercent() {
        return 0;
    }

    /**
     * Sets the speed of the left side of the robot.
     * 
     * @param percent The speed of the left side of the robot in percent output.
     */
    @Override
    public void setLeftPercent(double percent) {
        // The left motors aren't already reversed, so they're being reversed again.
        percent = -MathUtil.clamp(percent, -1, 1);

        flMotor.set(percent);
    }

    /**
     * Gets the speed of the right side of the robot.
     * 
     * @return The speed of the right side of the robot in percent output.
     */
    @Override
    public double getRightPercent() {
        return 0;
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
        return flMotor.getMotorVoltage().getValue();
    }

    /**
     * Sets the speed of the right side of the robot.
     * 
     * @param volts The speed of the right side of the robot in volts.
     */
    @Override
    public void setLeftVoltage(double volts) {
        volts = MathUtil.clamp(volts, -1, 1);
        frMotor.setControl(new VoltageOut(volts));
    }

    /**
     * Gets the speed of the right side of the robot.
     * 
     * @return The speed of the right side of the robot in volts.
     */
    @Override
    public double getRightVoltage() {
        return frMotor.getMotorVoltage().getValue();
    }

    /**
     * Sets the speed of the right side of the robot.
     * 
     * @param volts The speed of the right side of the robot in volts.
     */
    @Override
    public void setRightVoltage(double volts) {
        volts = MathUtil.clamp(volts, -1, 1);
        frMotor.setControl(new VoltageOut(volts));
    }    
}
