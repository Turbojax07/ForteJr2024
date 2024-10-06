// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Drivetrain.Commands.ArcadeDrive;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Commands.RunFeed;
import frc.robot.Shooter.Commands.RunShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private static RobotContainer instance;

    public static RobotContainer getInstance() {
        if (instance == null) instance = new RobotContainer();

        return instance;
    }

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Initializing subsystems
        Drivetrain.getInstance();
        Shooter.getInstance();

        // Configuring button/trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        operator.a().whileTrue(new RunFeed(1));
        operator.b().whileTrue(new RunShooter(-1)).whileTrue(new RunFeed(-1));
        operator.y().toggleOnTrue(new RunShooter(1));
    }

    /**
     * Use this to pass an autonomous command to the {@link Robot} class.
     * It gets the command meant to run on a real robot, not a simulated one.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getRealAutoCommand() {
        return new PrintCommand("No auto lol");
    }

    /**
     * Use this to pass a teleop command to the {@link Robot} class.
     * It gets the command meant to run on a real robot, not a simulated one.
     * 
     * @return The command to run in Teleop mode.
     */
    public Command getRealTeleopCommand() {
        return new ArcadeDrive(()-> driver.getLeftY(), ()-> driver.getRightX());
    }

    /**
     * Use this to pass an autonomous command to the {@link Robot} class.
     * It gets the command meant to run on a simulated robot, not a real one.
     *
     * @return The command to run in Autonomous mode.
     */
    public Command getSimAutoCommand() {
        return new PrintCommand("No auto lol");
    }

    /**
     * Use this to pass a teleop command to the {@link Robot} class.
     * It gets the command meant to run on a simulated robot, not a real one.
     * 
     * @return The command to run in Teleop mode.
     */
    public Command getSimTeleopCommand() {
        return new ArcadeDrive(()-> driver.getLeftY(), ()-> driver.getRightX());
    }
}