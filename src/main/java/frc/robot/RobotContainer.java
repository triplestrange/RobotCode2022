// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.RunClimb;
import frc.robot.commands.autoRoutines.Auto1;
import frc.robot.commands.autoSubsystems.LoadBall;
import frc.robot.commands.autoSubsystems.ShootBall;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake intake;
  private final Shooter shooter;
  private final SwerveDrive swerve;
  private final Turret turret;
  private final Conveyor conveyor;
  private final Climber climb;

  public static Joystick m_driverController;
  public static Joystick m_operatorController;
  private static ProfiledPIDController theta;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    theta = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);

    intake = new Intake();
    shooter = new Shooter();
    swerve = new SwerveDrive();
    conveyor = new Conveyor();
    turret = new Turret();
    climb = new Climber();

    SmartDashboard.putData(intake);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(swerve);
    SmartDashboard.putData(conveyor);
    SmartDashboard.putData(turret);
    SmartDashboard.putData(climb);

    m_driverController = new Joystick(0);
    m_operatorController = new Joystick(1);

    // Configure the button bindings
    configCommands();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configCommands() {
    JoystickButton opA = new JoystickButton(m_operatorController, 1);
    JoystickButton opB = new JoystickButton(m_operatorController, 2);
    JoystickButton opX = new JoystickButton(m_operatorController, 3);
    JoystickButton opY = new JoystickButton(m_operatorController, 4);
    JoystickButton oplBump = new JoystickButton(m_operatorController, 5);
    JoystickButton oprBump = new JoystickButton(m_operatorController, 6);
    JoystickButton oplWing = new JoystickButton(m_operatorController, 7);
    JoystickButton oprWing = new JoystickButton(m_operatorController, 8);
    JoystickButton oplJoy = new JoystickButton(m_operatorController, 9);
    JoystickButton oprJoy = new JoystickButton(m_operatorController, 10);

    JoystickButton dA = new JoystickButton(m_driverController, 1);
    JoystickButton dB = new JoystickButton(m_driverController, 2);
    JoystickButton dX = new JoystickButton(m_driverController, 3);
    JoystickButton dY = new JoystickButton(m_driverController, 4);
    JoystickButton dlBump = new JoystickButton(m_driverController, 5);
    JoystickButton drBump = new JoystickButton(m_driverController, 6);
    JoystickButton dlWing = new JoystickButton(m_driverController, 7);
    JoystickButton drWing = new JoystickButton(m_driverController, 8);
    JoystickButton dlJoy = new JoystickButton(m_driverController, 9);
    JoystickButton drJoy = new JoystickButton(m_driverController, 10);

    // DRIVER
    // regular default driving
    // reset gyro is left wing
    // shooter stuff (except everything is automated)
    drBump.whileHeld(new ShootBall(shooter, conveyor));
    
    // OPERATOR
    // intaking ball
    // climb stuff
    opA.whenPressed(new InstantCommand(
      () -> intake.toggleIntake(), intake));
    oprBump.whileHeld(new LoadBall(intake, conveyor));
    // outtake balls
    oplBump.whileHeld(new InstantCommand(
      () -> {
        intake.setIntake(1);
        intake.wheelsOut();
        conveyor.runConveyor(-0.75);
      }, intake, conveyor));
    oplWing.whenPressed(new InstantCommand(climb::toggle, climb));

    // couldn't get ConditionalCommand to work
    turret.setDefaultCommand(
      new InstantCommand(
        () -> {
          if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getBoolean(false)) {
            turret.turretVision();
          } else {
            turret.faceGoal();
          }
        }
    ));

    swerve.setDefaultCommand(new DefaultDrive(swerve, m_driverController, 1));
    climb.setDefaultCommand(new RunClimb(climb, m_operatorController));
    swerve.resetEncoders();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Auto1(intake, conveyor, shooter, swerve, theta);
  }
}
