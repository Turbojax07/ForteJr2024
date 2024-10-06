// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private Command teleopCommand;

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code
     */
    @Override
    public void robotInit() {
        // Saving the project name
        Logger.recordMetadata("ProjectName", "ForteJr");

        // Configuring the Logger based on the robot's mode.
        switch (Constants.currentMode) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
                Logger.addDataReceiver(new NT4Publisher());
                new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
                break;
            case SIM:
                Logger.addDataReceiver(new WPILOGWriter("sim_logs"));
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                Logger.setReplaySource(new WPILOGReader(LogFileUtil.findReplayLog()));
                Logger.addDataReceiver(new WPILOGWriter("replay_logs"));
                Logger.addDataReceiver(new NT4Publisher());
                break;
        }

        // Starting the Logger
        Logger.start();

        // Instantiate the RobotContainer.  This will assign all our button bindings.
        RobotContainer robotContainer = RobotContainer.getInstance();

        autonomousCommand = robotContainer.getRealAutoCommand();
        teleopCommand = robotContainer.getRealTeleopCommand();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled. */
    @Override
    public void disabledInit() {}

    /** This function is called periodically during Disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function is called once each time the robot exits Disabled */
    @Override
    public void disabledExit() {}

    /**
     * This function is called once each time the robot enters Autonomous.
     * It starts the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand.schedule();
    }

    /** This function is called periodically during Autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /**
     * This function is called once each time the robot exits Autonomous.
     * It stops the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousExit() {
        autonomousCommand.cancel();
    }

    /**
     * This function is called once each time the robot enters Teleop.
     * It starts the teleop command selected by your {@link RobotContainer} class.
     */
    @Override
    public void teleopInit() {
        teleopCommand.schedule();
    }

    /** This function is called periodically during Teleop. */
    @Override
    public void teleopPeriodic() {}

    /**
     * This function is called once each time the robot exits Teleop.
     * It stops the teleop command selected by your {@link RobotContainer} class.
     */
    @Override
    public void teleopExit() {
        teleopCommand.cancel();
    }

    /** This function is called once each time the robot enters Test. */
    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once each time the robot exits Test. */
    @Override
    public void testExit() {}

    /** This function is called once each time the robot enters Simulation. */
    @Override
    public void simulationInit() {
        RobotContainer robotContainer = RobotContainer.getInstance();

        autonomousCommand = robotContainer.getSimAutoCommand();
        teleopCommand = robotContainer.getSimTeleopCommand();
    }

    /** This function is called periodically during Simulation. */
    @Override
    public void simulationPeriodic() {}
}