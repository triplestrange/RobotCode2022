// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoIndexBall;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.RunClimb;
import frc.robot.commands.RunConveyor;
import frc.robot.commands.RunShooter;
import frc.robot.commands.RunTurretManual;
import frc.robot.commands.ShootSpeed;
import frc.robot.commands.FaceGoal;
import frc.robot.commands.ToggleHood;
import frc.robot.commands.TurretVision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
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
  private final Climber climb;

  public static Joystick m_driverController;
  public static Joystick m_operatorController;

  private final RunShooter shoot, toggleHood, shootAuto;
  private final IntakeBall ballIn, ballOut;
  private final RunConveyor conveyorIn, conveyorOut;
  private final AutoIndexBall autoIndex;
  private final RunTurretManual turretLeft, turretRight;
  private final DefaultDrive drive;
  private final RunClimb runClimb;
  private final FaceGoal facegoal;
  private final TurretVision turretVision;

  private final boolean autoTurret = true, autoConveyor = false;


  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    intake = new Intake();
    shooter = new Shooter();
    swerve = new SwerveDrive();
    conveyor = new Conveyor();
    turret = new Turret();
    climb = new Climber();

    m_driverController = new Joystick(0);
    m_operatorController = new Joystick(1);

    shoot = new RunShooter(shooter, false, false);
    shootAuto = new RunShooter(shooter, true, false);
    autoIndex = new AutoIndexBall(conveyor, shooter);
    toggleHood = new RunShooter(shooter, false, true);
    ballIn = new IntakeBall(intake, conveyor, 1);
    ballOut = new IntakeBall(intake, conveyor, -1);
    // intakeJoy = new IntakeJoy(intake, conveyor, m_operatorController);
    conveyorIn = new RunConveyor(conveyor, 1);
    conveyorOut = new RunConveyor(conveyor, -1);
    turretLeft = new RunTurretManual(turret, -1);
    turretRight = new RunTurretManual(turret, 1);
    runClimb = new RunClimb(climb, m_operatorController);
    drive = new DefaultDrive(swerve, m_driverController, 1);
    facegoal = new FaceGoal(turret);
    turretVision = new TurretVision(turret);

    swerve.setDefaultCommand(drive);
    climb.setDefaultCommand(runClimb);
    swerve.resetEncoders();
    if (autoTurret) {
      turret.setDefaultCommand(facegoal);
    }
    turret.setDefaultCommand(turretVision);
    
    if (autoConveyor) {
      conveyor.setDefaultCommand(autoIndex);
    }

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putBoolean("autoTurret", autoTurret);
    SmartDashboard.putBoolean("autoIndex", autoConveyor);
    SmartDashboard.putBoolean("Joy1", m_driverController.isConnected());
    SmartDashboard.putBoolean("Joy2", m_operatorController.isConnected());
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
    JoystickButton oplJoy = new JoystickButton(m_operatorController, 9);
    JoystickButton oprJoy = new JoystickButton(m_operatorController, 10);

    JoystickButton dA = new JoystickButton(m_operatorController, 1);
    JoystickButton dB = new JoystickButton(m_operatorController, 2);
    JoystickButton dX = new JoystickButton(m_operatorController, 3);
    JoystickButton dY = new JoystickButton(m_operatorController, 4);
    JoystickButton dlBump = new JoystickButton(m_operatorController, 5);
    JoystickButton drBump = new JoystickButton(m_operatorController, 6);
    JoystickButton dlWing = new JoystickButton(m_operatorController, 7);
    JoystickButton drWing = new JoystickButton(m_operatorController, 8);
    JoystickButton dlJoy = new JoystickButton(m_operatorController, 9);
    JoystickButton drJoy = new JoystickButton(m_operatorController, 10);

    // OPERATOR
    // a, b, x, y - shooter preset
    // right bumper - intake
    // right trigger - shoot auto
    // pov pad - climb stuff
    // left joy - tick speeds
    // right joy - climb stuff
    // right wing - hood
    opA.whileHeld(shootAuto);
    opB.whileHeld(new ShootSpeed(shooter, 1500.0)); // slow shooter
    opY.whileHeld(new ShootSpeed(shooter, 4550.0)); // fast shooter
    opX.whileHeld(new ShootSpeed(shooter, 3000.0)); // another preset
    SmartDashboard.putNumber("ShooterSetpoint", 
      m_operatorController.getRawAxis(2) * 50.0 + SmartDashboard.getNumber("ShooterSetpoint", 3000.0));
    if (Math.abs(m_operatorController.getRawAxis(3)) > 0.05) {
      shooter.setDefaultCommand(new InstantCommand(shooter::setShooter, shooter));
    }
    oprWing.whileHeld(toggleHood);
    oprBump.whileHeld(ballIn);
    // OPERATOR:
    // x - shoot manual, y - shoot automatic, bumpers - conveyor,
    // triggers - intake, joystick - climb, right wing - toggle autonomous turret, b - hood toggle
    // left wing - toggle autonomous conveyor
    // opX.whileHeld(shoot);
    // opY.whileHeld(shootAuto);
    // opB.whenPressed(toggleHood);
    // // opY.whileHeld(shoot);
    // opY.whileHeld(new AutoIndexBall(conveyor));
    // // opA.whenPressed(toggleHood);
    // oplBump.whileHeld(conveyorOut);
    // oprBump.whileHeld(conveyorIn);

    // dlBump.whileHeld(turretLeft);
    // drBump.whileHeld(turretRight);

    // dlBump.whileHeld(turretLeft);
    // drBump.whileHeld(turretRight);

    // ToggleHood test = new ToggleHood(shooter);
    // opA.whenPressed(test);

    // button to dump in low goal


  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return shoot;
  }
}
