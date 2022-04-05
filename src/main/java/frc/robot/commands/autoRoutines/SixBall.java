// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoRoutines;

import java.util.List;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixBall extends SequentialCommandGroup {
  /** Creates a new SixBall. */
  public SixBall(SwerveDrive swerve, Intake intake, Conveyor conveyor, Hood hood, Turret turret, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ProfiledPIDController theta = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    TrajectoryConfig config = new TrajectoryConfig(4.5,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(SwerveDriveConstants.kDriveKinematics)
        .setStartVelocity(0)
        .setEndVelocity(0);

    Trajectory toBall1 = TrajectoryGenerator
        .generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(

        ),
            // direction robot moves
            new Pose2d(1, 0, new Rotation2d(0)), config);

    SwerveControllerCommand toBall1Command = new SwerveControllerCommand(toBall1,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {
          return new Rotation2d(0);
        },

        swerve::setModuleStates,

        swerve

    );

    Trajectory ball1ToBall2 = TrajectoryGenerator
        .generateTrajectory(new Pose2d(0.75, 0, new Rotation2d(-Math.PI / 2)), List.of(

        ),
            // direction robot moves
            new Pose2d(0.2, -3.2, new Rotation2d(-Math.PI / 2)), config);

    SwerveControllerCommand ball1ToBall2Command = new SwerveControllerCommand(ball1ToBall2,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {
          return new Rotation2d(-Math.PI / 2);
        },

        swerve::setModuleStates,

        swerve

    );

    Trajectory ball2ToHumanPlayer = TrajectoryGenerator
        .generateTrajectory(new Pose2d(0.2, -3.2, new Rotation2d(-Math.PI / 2)), List.of(

        ),
            // direction robot moves
            new Pose2d(0.8, -7.0, new Rotation2d(-Math.PI / 4.0)), config);
    SwerveControllerCommand ball2ToHumanPlayerCommand = new SwerveControllerCommand(ball2ToHumanPlayer,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {
          return new Rotation2d(-Math.PI / 4.0);
        },

        swerve::setModuleStates,

        swerve

    );

    Trajectory humanPlayerToBall3 = TrajectoryGenerator
        .generateTrajectory(new Pose2d(0.6, -7.2, new Rotation2d(-Math.PI / 2)), List.of(

        ),
            // direction robot moves
            new Pose2d(0.3, -4, new Rotation2d(Math.PI / 4.0)), config);
    SwerveControllerCommand humanPlayerToBall3Command = new SwerveControllerCommand(humanPlayerToBall3,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {
          return new Rotation2d(0);
        },

        swerve::setModuleStates,

        swerve

    );

    // TODO: figure out if this is a runnable
    BooleanSupplier emptyBot = new BooleanSupplier() {
      public boolean getAsBoolean() {
        return conveyor.empty();
      }
    };

    // shoot ball 1, intake ball 2 and 3
    // shoot 2 + 3
    Command sixBall = new RunCommand(() -> {
      shooter.visionShootShort();
      if (shooter.atSpeed()) {
        conveyor.runConveyor();
      }
    }, shooter, conveyor).until(emptyBot).withTimeout(1)
    .andThen(new InstantCommand(() -> {
      shooter.setShooter(Constants.Shooting.idleSpeed);
      conveyor.stopConveyor();
      intake.setIntake(1);
    }, shooter, conveyor, intake))
        .andThen(toBall1Command.raceWith(
            new RunCommand(conveyor::autoConveyor, conveyor)))
        .andThen(ball1ToBall2Command.raceWith(
            new RunCommand(conveyor::autoConveyor, conveyor)))
        .andThen(new RunCommand(() -> {
          hood.setHood(1);
          shooter.visionShootLong();
          if (shooter.atSpeed()) {
            conveyor.runConveyor();
          } else {
            conveyor.autoConveyor();
          }
          swerve.drive(0, 0, 0, true);
        }, shooter, conveyor).until(emptyBot).withTimeout(2))
        .andThen(new InstantCommand(() -> {
          shooter.setShooter(Constants.Shooting.idleSpeed);
          conveyor.stopConveyor();
          intake.wheelsIn(1);
        }, shooter))
        .andThen(ball2ToHumanPlayerCommand.raceWith(
            new RunCommand(conveyor::autoConveyor, conveyor)))
        .andThen(new InstantCommand(() -> {
          swerve.drive(0, 0, 0, true);
          conveyor.stopConveyor();
          intake.setIntake(1);
          intake.wheelsIn(1);
        }))
        .andThen(humanPlayerToBall3Command.raceWith(
            new RunCommand(conveyor::autoConveyor, conveyor)))
        .andThen(new RunCommand(() -> {
          shooter.visionShootLong();
          if (shooter.atSpeed()) {
            conveyor.runConveyor();
          } else {
            conveyor.autoConveyor();
          }
        }).withTimeout(2))
        .andThen(new InstantCommand(() -> {
          shooter.setShooter(Constants.Shooting.idleSpeed);
          conveyor.stopConveyor();
          intake.wheelsIn(1);
          intake.setIntake(1);
        }, shooter, conveyor));

    addCommands(sixBall);
  }
}
