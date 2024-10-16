// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class Drivetrain extends SubsystemBase {
    // Motors
    private CANSparkMax flMotor;
    private CANSparkMax frMotor;
    private CANSparkMax blMotor;
    private CANSparkMax brMotor;
    private DCMotor lMotors = DCMotor.getNEO(2);

    // Encoders
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    // PID Controllers
    private SparkPIDController leftPID;
    private SparkPIDController rightPID;

    // Gyro
    private PigeonIMU gyro;

    // Kinematics
    private DifferentialDriveKinematics kinematics;

    // Odometry
    private Pose2d initPose = new Pose2d();
    private Field2d field = new Field2d();
    private DifferentialDrivePoseEstimator poseEstimator;

    // A common instance of the shooter class so that I don't have to outright initialize it anywhere.
    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) instance = new Drivetrain();

        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    public Drivetrain() {
        // Getting motors
        flMotor = new CANSparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);
        frMotor = new CANSparkMax(DriveConstants.frontRightID, MotorType.kBrushless);
        blMotor = new CANSparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
        brMotor = new CANSparkMax(DriveConstants.backRightID, MotorType.kBrushless);

        // Resetting motors
        flMotor.restoreFactoryDefaults();
        frMotor.restoreFactoryDefaults();
        blMotor.restoreFactoryDefaults();
        brMotor.restoreFactoryDefaults();

        // Creating followers
        blMotor.follow(flMotor);
        brMotor.follow(frMotor);

        // Inverting motors
        blMotor.setInverted(true);
        brMotor.setInverted(false);
        flMotor.setInverted(true);
        frMotor.setInverted(false);

        // Setting idle modes
        flMotor.setIdleMode(IdleMode.kBrake);
        blMotor.setIdleMode(IdleMode.kBrake);
        frMotor.setIdleMode(IdleMode.kBrake);
        brMotor.setIdleMode(IdleMode.kBrake);

        // Getting encoders
        leftEncoder = flMotor.getEncoder();
        rightEncoder = frMotor.getEncoder();

        // Getting gyro
        gyro = new PigeonIMU(10);
        gyro.setYaw(0);

        // Setting conversion factors
        leftEncoder.setPositionConversionFactor(DriveConstants.rotToMeters);
        leftEncoder.setVelocityConversionFactor(DriveConstants.rotToMeters / 60);
        rightEncoder.setPositionConversionFactor(DriveConstants.rotToMeters);
        rightEncoder.setVelocityConversionFactor(DriveConstants.rotToMeters / 60);

        // Getting PID controllers
        leftPID = flMotor.getPIDController();
        rightPID = frMotor.getPIDController();

        // Setting PIDFF values
        leftPID.setP(DriveConstants.leftP);
        leftPID.setI(DriveConstants.leftI);
        leftPID.setD(DriveConstants.leftD);
        leftPID.setFF(DriveConstants.leftFF);

        rightPID.setP(DriveConstants.rightP);
        rightPID.setI(DriveConstants.rightI);
        rightPID.setD(DriveConstants.rightD);
        rightPID.setFF(DriveConstants.rightFF);

        // Saving configs
        brMotor.burnFlash();
        blMotor.burnFlash();
        frMotor.burnFlash();
        flMotor.burnFlash();

        // Initializing the kinematics
        kinematics = new DifferentialDriveKinematics(DriveConstants.robotWidth);

        // Initializing the pose estimator
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, getAngle(), getLeftPosition(), getRightPosition(), initPose);

        // Initializing the autobuilder
        AutoBuilder.configureRamsete(
            this::getPose,
            this::setPose,
            this::getSpeeds,
            this::closedLoop,
            new ReplanningConfig(),
            () -> (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red),
            this
        );
    }

    @Override
    public void simulationPeriodic() {
        REVPhysicsSim.getInstance().addSparkMax(blMotor, lMotors);
    }
    @Override
    public void periodic() {

        // Updating the robot pose.
        poseEstimator.update(getAngle(), getPositions());

        // Updating the robot pose on the field object.
        field.setRobotPose(getPose());

        // Pushing values to NetworkTables
        SmartDashboard.putNumber("/Drivetrain/Left_Actual_MPS", getLeftVelocity());
        SmartDashboard.putNumber("/Drivetrain/Right_Actual_MPS", getRightVelocity());
        SmartDashboard.putNumber("/Drivetrain/Angle", getAngle().getRadians());
        SmartDashboard.putData("/Drivetrain/Field", field);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d newPose) {
        poseEstimator.resetPosition(getAngle(), getPositions(), newPose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()));
    }

    public DifferentialDriveWheelPositions getPositions() {
        return new DifferentialDriveWheelPositions(getLeftPosition(), getRightPosition());
    }

    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public double getLeftVelocity() {
        return flMotor.get();
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public double getRightVelocity() {
        frMotor.getBusVoltage();
        frMotor.getOutputCurrent();

        return frMotor.get();
    }
    
    public void arcadeDrive(double xSpeed, double zRotate) {
        SmartDashboard.putNumber("/Drivetrain/Left_Expected_Speed", xSpeed - zRotate);
        SmartDashboard.putNumber("/Drivetrain/Right_Expected_Speed", xSpeed + zRotate);

        flMotor.set(xSpeed - zRotate);
        frMotor.set(xSpeed + zRotate);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        flMotor.set(leftSpeed);
        frMotor.set(rightSpeed);
    }

    public void closedLoop(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds newSpeeds = kinematics.toWheelSpeeds(speeds);

        SmartDashboard.putNumber("/Drivetrain/Left_Expected_MPS", newSpeeds.leftMetersPerSecond);
        SmartDashboard.putNumber("/Drivetrain/Right_Expected_MPS", newSpeeds.rightMetersPerSecond);

        leftPID.setReference(newSpeeds.leftMetersPerSecond, ControlType.kVelocity);
        rightPID.setReference(newSpeeds.rightMetersPerSecond, ControlType.kVelocity);
    }
}