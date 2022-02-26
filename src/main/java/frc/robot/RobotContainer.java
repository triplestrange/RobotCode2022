// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake;
  private final Shooter shooter;
  private final SwerveDrive swerve;
  private final Turret turret;
  private final Conveyor conveyor;

  public static Joystick m_driverController;
  public static Joystick m_operatorController;

  private final RunShooter shoot;
  private final IntakeBall ballIn;
  private final IntakeBall ballOut;
  private final RunConveyor conveyorIn;
  private final RunConveyor conveyorOut;
  private final RunTurret turretLeft;
  private final RunTurret turretRight;
  private final DefaultDrive drive;


  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    intake = new Intake();
    shooter = new Shooter();
    swerve = new SwerveDrive();

    m_driverController = new Joystick(0);
    m_operatorController = new Joystick(1);

    shoot = new RunShooter(shooter);
    ballIn = new IntakeBall(intake, 1);
    ballOut = new IntakeBall(intake, -1);
    conveyorIn = new RunConveyor(conveyor, 1);
    conveyorOut = new RunConveyor(conveyor, -1);
    turretLeft = new RunTurret(turret, -1);
    turretRight = new RunTurret(turret, 1);
    
    drive = new DefaultDrive(swerve, m_driverController, 1);

    swerve.setDefaultCommand(drive);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton opA = new JoystickButton(m_operatorController, 1);
    JoystickButton opB = new JoystickButton(m_operatorController, 2);
    JoystickButton opX = new JoystickButton(m_operatorController, 3);
    JoystickButton opY = new JoystickButton(m_operatorController, 4);
    JoystickButton oplBump = new JoystickButton(m_operatorController, 5);
    JoystickButton oprBump = new JoystickButton(m_operatorController, 6);
    JoystickButton oplWing = new JoystickButton(m_operatorController, 7);
    JoystickButton oprWing = new JoystickButton(m_operatorController, 8);
    JoystickButton oplTrig = new JoystickButton(m_operatorController, 9);
    JoystickButton oprTrig = new JoystickButton(m_operatorController, 10);

    JoystickButton dlBump = new JoystickButton(m_driverController, 5);
    JoystickButton drBump = new JoystickButton(m_driverController, 6);
    JoystickButton dX = new JoystickButton(m_driverController, 3);
    JoystickButton dA = new JoystickButton(m_driverController, 1);
    JoystickButton dB = new JoystickButton(m_driverController, 2);
    JoystickButton dlAnal = new JoystickButton(m_driverController, 9);
    JoystickButton drAnal = new JoystickButton(m_driverController, 10);

    opY.whileHeld(shoot);
    oplTrig.whileHeld(ballOut);
    oprTrig.whileHeld(ballIn);
    oplBump.whileHeld(conveyorOut);
    oprBump.whileHeld(conveyorIn);

    dlBump.whileHeld(turretLeft);
    drBump.whileHeld(turretRight);


  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return shootCommand;
  }
}
