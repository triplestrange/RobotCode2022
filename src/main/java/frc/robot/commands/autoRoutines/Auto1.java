// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoRoutines;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.autoSubsystems.LoadBall;
import frc.robot.commands.autoSubsystems.ShootBall;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1 extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public Auto1(Intake intake, Conveyor conveyor, Shooter shooter, SwerveDrive swerve, ProfiledPIDController theta) {
    TrajectoryConfig config = new TrajectoryConfig(2.1,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            // .setKinematics(SwerveDriveConstants.kDriveKinematics)
            .setEndVelocity(1.5);

    Trajectory trajectory = TrajectoryGenerator
        .generateTrajectory(new Pose2d(0, 0, new Rotation2d(-Math.PI / 2)), List.of(

        ),
            // direction robot moves
            new Pose2d(0, -3, new Rotation2d(Math.PI / 2)), config);

    SwerveControllerCommand swerveControllerCommand1 = new SwerveControllerCommand(trajectory,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,

        swerve::setModuleStates,

        swerve

    );
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LoadBall(intake, conveyor), swerveControllerCommand1, new ShootBall(shooter, conveyor));
  }
}
