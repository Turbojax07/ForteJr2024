// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
    private DrivetrainIO drivetrainIO;

    // Gyro
    private PigeonIMU gyro;

    // Kinematics
    private DifferentialDriveKinematics kinematics;

    // Odometry
    private Pose2d initPose = new Pose2d();
    private DifferentialDrivePoseEstimator poseEstimator;

    // A common instance of the shooter class so that I don't have to outright initialize it anywhere.
    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) instance = new Drivetrain();

        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    public Drivetrain() {
        // Determining the IO interface to use
        drivetrainIO = Constants.currentMode.io;

        // Pushing a warning to SmartDashboard if TalonSRXs are in use
        // They don't have built-in encoders and cannot run closed loop
        if (drivetrainIO instanceof DrivetrainIOTalonSRX) {
            SmartDashboard.putString("/Warnings/OpenLoop", "Running open loop!  Logged values will be zero.");
        }

        // Getting gyro
        gyro = new PigeonIMU(10);
        gyro.setYaw(0);

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
    public void periodic() {
        SmartDashboard.putNumber("/Drivetrain/Left_Actual_MPS", getLeftVelocity());
        SmartDashboard.putNumber("/Drivetrain/Right_Actual_MPS", getRightVelocity());
        SmartDashboard.putNumber("/Drivetrain/Angle", getAngle().getRadians());

        drivetrainIO.updateInputs();

        poseEstimator.update(getAngle(), getPosition());
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d newPose) {
        poseEstimator.resetPosition(getAngle(), getPosition(), newPose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity()));
    }

    public DifferentialDriveWheelPositions getPosition() {
        return new DifferentialDriveWheelPositions(getLeftPosition(), getRightPosition());
    }

    public double getLeftPosition() {
        return DrivetrainIOInputsAutoLogged.leftPosition;
    }

    public double getLeftVelocity() {
        return DrivetrainIOInputsAutoLogged.leftPercent;
    }

    public double getRightPosition() {
        return DrivetrainIOInputsAutoLogged.rightPosition;
    }

    public double getRightVelocity() {
        return DrivetrainIOInputsAutoLogged.rightPercent;
    }

    public void setSpeeds(double leftSpeed, double rightSpeed) {
        drivetrainIO.setLeftPercent(leftSpeed);
        drivetrainIO.setRightPercent(rightSpeed);
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds newSpeeds = kinematics.toWheelSpeeds(speeds);

        setSpeeds(newSpeeds.leftMetersPerSecond, newSpeeds.rightMetersPerSecond);
    }
    
    public void arcadeDrive(double xSpeed, double zRotate) {
        drivetrainIO.setLeftPercent(xSpeed - zRotate);
        drivetrainIO.setRightPercent(xSpeed + zRotate);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftSpeed *= DriveConstants.maxOpenDriveSpeed;
        rightSpeed *= DriveConstants.maxOpenTurnSpeed;

        if (leftSpeed < 0.1 && leftSpeed > -0.1) leftSpeed = 0;
        if (rightSpeed < 0.1 && rightSpeed > -0.1) rightSpeed = 0;

        if (leftSpeed  != 0 && leftSpeed  > 0) leftSpeed  -= 0.1;
        if (leftSpeed  != 0 && leftSpeed  < 0) leftSpeed  += 0.1;
        if (rightSpeed != 0 && rightSpeed > 0) rightSpeed -= 0.1;
        if (rightSpeed != 0 && rightSpeed < 0) rightSpeed += 0.1;

        drivetrainIO.setLeftPercent(leftSpeed);
        drivetrainIO.setRightPercent(rightSpeed);
    }

    public void closedLoop(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds newSpeeds = kinematics.toWheelSpeeds(speeds);

    }
}