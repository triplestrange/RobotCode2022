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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DefaultDrive;
import frc.robot.commands.RunClimb;
import frc.robot.subsystems.*;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autoRoutines.*;
import frc.robot.commands.autoSubsystems.*;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;

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
  public final SwerveDrive swerve;
  private final Turret turret;
  private final Conveyor conveyor;
  public final Climber climb;
  private final Hood hood;

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

        
    m_driverController = new Joystick(0);
    m_operatorController = new Joystick(1);

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

    boolean turretAuto = true;

    // DRIVER
    // regular default driving
    // reset gyro is left wing
    //shoot ball
    drBump.whileHeld(new ShootBall(shooter, conveyor));
    dlBump.whileHeld(new ShootBallSlow(shooter, conveyor));

    //manual turret
    dlWing.whileHeld(new InstantCommand(()->turret.runLeft(), turret));
    drWing.whileHeld(new InstantCommand(()->turret.runRight(), turret));



    // OPERATOR
    // toggle intake
    oprWing.whenPressed(new InstantCommand(
      () -> intake.setIntake(1), intake));
    oprWing.whenReleased(new InstantCommand(
      () -> intake.setIntake(0), intake));

    // toggle hood
    oplWing.whenPressed(new InstantCommand(
      () -> hood.toggleHood()));

    //run shooter at preset speeds
    opA.whileHeld(new InstantCommand(
      () -> shooter.setShooter(1500), shooter));
    opA.whenReleased(new InstantCommand(shooter::stopShooter, shooter));
    opY.whileHeld(new InstantCommand(
      () -> shooter.setShooter(4550), shooter));
    opY.whenReleased(new InstantCommand(shooter::stopShooter, shooter));
    opB.whileHeld(new InstantCommand(
      () -> shooter.setShooter(3000), shooter));
    opB.whenReleased(new InstantCommand(shooter::stopShooter, shooter));

    //toggle climb
    opX.whenPressed(new InstantCommand(climb::toggle, climb));

    //intake
    oprBump.whileHeld(new LoadBall(intake, conveyor, 1));

    //outtake
    oplBump.whileHeld(new LoadBall(intake, conveyor, -1));
  
    swerve.setDefaultCommand(new DefaultDrive(swerve, m_driverController, 1));
    climb.setDefaultCommand(new RunClimb(climb, m_operatorController));
    turret.setDefaultCommand(new AimBot(turret, hood));
    swerve.resetEncoders();
    // intake.setDefaultCommand(new ManualFeeder(intake, conveyor, m_operatorController));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig config = new TrajectoryConfig(1.5,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(SwerveDriveConstants.kDriveKinematics)
        .setStartVelocity(0)
        .setEndVelocity(0);

    /********************************************************* */
    // An ExampleCommand will run in autonomous
    //return new Auto1(intake, conveyor, shooter, swerve, theta);
    Trajectory trajectory = TrajectoryGenerator
        .generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(

        ),
            // direction robot moves
            new Pose2d(1, 0, new Rotation2d(0)), config);

    Trajectory ball1ToBall2 = TrajectoryGenerator
            .generateTrajectory(new Pose2d(1, 0, new Rotation2d(-Math.PI / 2)), List.of(
    
            ),
                // direction robot moves
                new Pose2d(1 - 1.19, -2.72, new Rotation2d(-Math.PI / 2)), config);

    Trajectory ball2ToHumanPlayer = TrajectoryGenerator
                .generateTrajectory(new Pose2d(1 - 1.19, -2.72, new Rotation2d(-Math.PI / 2)), List.of(
        
            ),
                    // direction robot moves
                new Pose2d(1, -6.105, new Rotation2d(-Math.PI / 2)), config);
                
                
    Field2d m_field = new Field2d();
    SmartDashboard.putData(m_field);
    m_field.getObject("traj").setTrajectory(trajectory);
    m_field.getObject("traj").setTrajectory(ball1ToBall2);

    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(trajectory,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {return new Rotation2d(0);}, 

        swerve::setModuleStates,

        swerve

    );

    SwerveControllerCommand ball1ToBall2Com = new SwerveControllerCommand(ball1ToBall2,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {return new Rotation2d(-Math.PI/2);},

        swerve::setModuleStates,

        swerve

    );

    SwerveControllerCommand ball2ToHumanPlayerCom = new SwerveControllerCommand(ball2ToHumanPlayer,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {return new Rotation2d(-Math.PI/2);},

        swerve::setModuleStates,

        swerve

    );
    // Pembroke 2-ball auto
    return new InstantCommand(() -> {
      intake.setIntake(1);
      intake.wheelsIn(0.8);
    }).andThen(swerveControllerCommand1).raceWith(new RunCommand(conveyor::autoConveyor, conveyor))
    .andThen(new InstantCommand(() -> {
      intake.setIntake(0);
      intake.wheelsIn(0);
      conveyor.stopConveyor();
      swerve.drive(0, 0, 0, true);
    }))
    .andThen((new RunCommand(() -> {
      shooter.visionShootLong();
      hood.setHood(1);
      if (shooter.atSpeed()) {
        conveyor.runConveyor();
      }
    }, shooter, conveyor)).withTimeout(5)).andThen(new InstantCommand(() -> {
      shooter.stopShooter();
      conveyor.stopConveyor();
    }))
    .andThen(new InstantCommand(() -> {
      intake.setIntake(1);
      intake.wheelsIn(0.8);
      conveyor.autoConveyor();
    }))
    .andThen(ball1ToBall2Com)
    .andThen(ball2ToHumanPlayerCom)
    .andThen((new RunCommand(() -> {
      shooter.visionShootLong();
      hood.setHood(1);
      if (shooter.atSpeed()) {
        conveyor.runConveyor();
      }
    }, shooter, conveyor)).withTimeout(5));

    // 3 ball auto path
    // return new InstantCommand(() -> {
    //   intake.setIntake(1);
    //   intake.wheelsIn(0.8);
    //   conveyor.autoConveyor();
    // }).andThen(swerveControllerCommand1).andThen(ball1ToBall2Com)
    // .andThen(ball2ToHumanPlayerCom)
    // .andThen(new InstantCommand(() -> {
    //   intake.setIntake(0);
    //   intake.wheelsIn(0);
    //   conveyor.stopConveyor();
    // }));
    
  }
}
