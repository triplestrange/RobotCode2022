// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autoRoutines.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.autoSubsystems.*;
import frc.robot.Constants.JoystickButtons;
import frc.robot.Constants.Shooting;

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
  public static SwerveDrive swerve;
  private final Turret turret;
  private final Conveyor conveyor;
  private final Climber climb;
  private final Hood hood;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(SendableChooser<Command> choose) {
    intake = new Intake();
    shooter = new Shooter();
    swerve = new SwerveDrive();
    conveyor = new Conveyor();
    turret = new Turret();
    climb = new Climber();
    hood = new Hood();

    SmartDashboard.putData(intake);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(swerve);
    SmartDashboard.putData(conveyor);
    SmartDashboard.putData(turret);
    SmartDashboard.putData(climb);

    // Configure the button bindings
    configButtons();
    configDefaults();

    // autos
    choose.addOption("TwoBall", new TwoBall(swerve, intake, conveyor, hood, turret, shooter));
    choose.addOption("ThreeBall - shoot first", new ThreeBallA(swerve, intake, conveyor, hood, turret, shooter));
    choose.addOption("ThreeBall - regular", new ThreeBallB(swerve, intake, conveyor, hood, turret, shooter));
    choose.addOption("FiveBall - regular", new FiveBall(swerve, intake, conveyor, hood, turret, shooter));
    choose.addOption("Surprise", new SixBall(swerve, intake, conveyor, hood, turret, shooter));
    SmartDashboard.putBoolean("Blind me", true);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configButtons() {

    // DRIVER
    // regular default driving
    // reset gyro is left wing
    // shoot ball
    JoystickButtons.drBump.whileHeld(new ShootBall(shooter, conveyor));
    JoystickButtons.drBump.whenReleased(new InstantCommand(conveyor::stopConveyor, conveyor));
    JoystickButtons.dlBump.whileHeld(new ShootBallSlow(shooter, conveyor));
    JoystickButtons.dlBump.whenReleased(new InstantCommand(conveyor::stopConveyor, conveyor));
    // manual turret
    JoystickButtons.dlWing.whileHeld(new InstantCommand(() -> turret.runLeft(), turret));
    JoystickButtons.dlWing.whenReleased(new InstantCommand(() -> turret.stop(), turret));
    JoystickButtons.drWing.whileHeld(new InstantCommand(() -> turret.runRight(), turret));
    JoystickButtons.drWing.whileHeld(new InstantCommand(() -> turret.stop(), turret));

    JoystickButtons.dA.whenPressed(new InstantCommand(shooter::stopShooter, shooter));

    // OPERATOR
    // toggle intake
    JoystickButtons.oprWing.whenPressed(new InstantCommand(
        () -> intake.setIntake(1), intake));
    JoystickButtons.oprWing.whenReleased(new InstantCommand(
        () -> intake.setIntake(0), intake));

    // toggle hood
    JoystickButtons.oplWing.whenPressed(new InstantCommand(
        () -> hood.toggleHood()));

    JoystickButtons.opA.whenPressed(new AutoClimb(swerve, climb, 0));

    // toggle climb
    JoystickButtons.oprBump.whenPressed(new InstantCommand(climb::toggle, climb));
  }

  public void configDefaults() {
    swerve.setDefaultCommand(new DefaultDrive(swerve, JoystickButtons.m_driverController, 1));
    turret.setDefaultCommand(new AimBot(turret, hood));
    intake.setDefaultCommand(new RunCommand(() -> {
      if (JoystickButtons.m_operatorController.getRawAxis(3) > 0.05) {
        intake.intake();
      } else if (JoystickButtons.m_operatorController.getRawAxis(2) > 0.05) {
        intake.outtake();
      } else {
        intake.retract();
      }
    }, intake));
    conveyor.setDefaultCommand(new RunCommand(() -> {
      if (JoystickButtons.m_operatorController.getRawAxis(3) > 0.05) {
        conveyor.autoConveyor();
      } else if (JoystickButtons.m_operatorController.getRawAxis(2) > 0.05) {
        conveyor.runConveyor(0.75);
      } else {
        conveyor.stopConveyor();
      }
    }, conveyor));
    climb.setDefaultCommand(new RunCommand(() -> {
      if (Math.abs(JoystickButtons.m_operatorController.getRawAxis(5)) > 0.3) {
        SmartDashboard.putNumber("ClimberSpeedL", JoystickButtons.m_operatorController.getRawAxis(5));
        climb.moveLeft();
      } else {
        climb.stopLeft();
      }
  
      if (Math.abs(JoystickButtons.m_operatorController.getRawAxis(1)) > 0.3) {
        SmartDashboard.putNumber("ClimberSpeedR", JoystickButtons.m_operatorController.getRawAxis(1));
        climb.moveRight();
       } else {
         climb.stopRight();
       }
    }, climb));

    swerve.resetEncoders();
  }

  public void setupTele() {
    intake.wheelsIn(0);
    intake.setIntake(0);
    conveyor.stopConveyor();
    shooter.setShooter(Shooting.idleSpeed);
    climb.extend();
    // TODO: test this
    swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(-Math.PI / 2)));
  }

}
