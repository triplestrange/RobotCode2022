// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoRoutines;

import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.*;

public class Autos extends SubsystemBase {
  public SwerveDrive swerve;
  public Intake intake;
  public Conveyor conveyor;
  public Hood hood;
  public Shooter shooter;
  Trajectory trajectory, ball1ToBall2, ball2ToHumanPlayer;
  SwerveControllerCommand swerveControllerCommand1, ball1ToBall2Com, 
    ball2ToHumanPlayerCom, ball1ToHumanPlayerCom, ball1ToHumanPlayer;
  public ProfiledPIDController theta = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
  AutoConstants.kThetaControllerConstraints);
  TrajectoryConfig config = new TrajectoryConfig(4.5,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        // .setKinematics(SwerveDriveConstants.kDriveKinematics)
        .setStartVelocity(0)
        .setEndVelocity(0);


  /** Creates a new Autos. */
  public Autos(SwerveDrive swerve, Intake intake, Conveyor conveyor, Hood hood, Shooter shooter) {
    this.swerve = swerve;
    this.intake = intake;
    this.conveyor = conveyor;
    this.hood = hood;
    this.shooter = shooter;


    /********************************************************* */
    // An ExampleCommand will run in autonomous
    //return new Auto1(intake, conveyor, shooter, swerve, theta);

    Trajectory ball1ToHumanPlayer = TrajectoryGenerator
                .generateTrajectory(new Pose2d(1, 0, new Rotation2d(-Math.PI / 2)), List.of(
        
            ),
                // direction robot moves
                new Pose2d(0.65, -6.205, new Rotation2d(-Math.PI / 2)), config);
                
  }

  public SwerveControllerCommand bob() {
    Trajectory trajectory = TrajectoryGenerator
    .generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(

    ),
        // direction robot moves
        new Pose2d(1, 0, new Rotation2d(0)), config);

    return new SwerveControllerCommand(trajectory,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {return new Rotation2d(0);}, 

        swerve::setModuleStates,

        swerve

    );
  }

  public SwerveControllerCommand bob1() {
    Trajectory ball1ToBall2 = TrajectoryGenerator
            .generateTrajectory(new Pose2d(0.75, 0, new Rotation2d(-Math.PI / 2)), List.of(
    
            ),
                // direction robot moves
                new Pose2d(0.2, -3.2, new Rotation2d(-Math.PI / 2)), config);

    return new SwerveControllerCommand(ball1ToBall2,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {return new Rotation2d(-Math.PI/2);},

        swerve::setModuleStates,

        swerve

    );
  }

  public SwerveControllerCommand bob2() {
    Trajectory ball2ToHumanPlayer = TrajectoryGenerator
                .generateTrajectory(new Pose2d(0.2, -3.2, new Rotation2d(-Math.PI / 2)), List.of(
        
            ),
                // direction robot moves
                new Pose2d(0.8, -7.0, new Rotation2d(-Math.PI / 4.0)), config);
    return new SwerveControllerCommand(ball2ToHumanPlayer,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {return new Rotation2d(-Math.PI/4.0);},

        swerve::setModuleStates,

        swerve

    );
  }

  public SwerveControllerCommand bob3() {
    Trajectory humanPlayerToBall2 = TrajectoryGenerator
                .generateTrajectory(new Pose2d(0.6, -7.2, new Rotation2d(-Math.PI / 2)), List.of(
        
            ),
                // direction robot moves
                new Pose2d(0.3, -4, new Rotation2d(Math.PI / 4.0)), config);
    return new SwerveControllerCommand(humanPlayerToBall2,
        swerve::getPose, // Functional interface to feed supplier
        SwerveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 1, AutoConstants.kDXController),
        new PIDController(AutoConstants.kPYController, 1, AutoConstants.kDYController), theta,
        () -> {return new Rotation2d(0);},

        swerve::setModuleStates,

        swerve

    );
  }

  public Command getTwoBall() {
    Command twoBall = 
     bob().raceWith(new RunCommand(() -> {
       shooter.setShooter(3000);
       intake.setIntake(1);
       intake.wheelsIn(1);
       conveyor.autoConveyor();
      }, conveyor))
    .andThen(new InstantCommand(() -> {
      conveyor.stopConveyor();
      swerve.drive(0, 0, 0, true);
    }, intake, conveyor, swerve))
    .andThen(new RunCommand(() -> {
      hood.setHood(1);
      shooter.visionShootLong();
      if (shooter.atSpeed()) {
        conveyor.runConveyor();
      } else {
        conveyor.autoConveyor();
      }
    }).withTimeout(2))
    .andThen(new InstantCommand(() -> {
      intake.stop();
      intake.setIntake(0);
      conveyor.stopConveyor();
      shooter.stopShooter();
    }, conveyor, shooter));

    return twoBall;
  }

  public Command getThreeBallA() {
    return new RunCommand(() -> {
      shooter.visionShootLong();
      if (shooter.atSpeed()) {
        conveyor.runConveyor();
      } 
    }, shooter, conveyor).withName("SHOOT").withTimeout(2)
    .andThen(new InstantCommand(() -> {
      shooter.setShooter(3000);
      conveyor.stopConveyor();
      intake.setIntake(1);
      intake.wheelsIn();
    }, shooter, conveyor, intake))
    .andThen(bob().raceWith(
      new RunCommand(conveyor::autoConveyor, conveyor)))
    .andThen(bob1().raceWith(
      new RunCommand(conveyor::autoConveyor, conveyor)))
    .andThen(new InstantCommand(() -> {
      swerve.drive(0, 0, 0, true);
      conveyor.stopConveyor();
    }))
    .andThen(new InstantCommand(() -> {
      intake.setIntake(0);
      intake.wheelsIn(0);
    }))
    .andThen(new RunCommand(() -> {
      shooter.visionShootLong();
      if (shooter.atSpeed()) {
        conveyor.runConveyor();
      }
    }).withTimeout(3))
    .andThen(new InstantCommand(() -> {
      shooter.setShooter(3000);
      conveyor.stopConveyor();
    }, shooter, conveyor));
  }

  public Command getThreeBallB() {

    return new InstantCommand(() -> {
      shooter.setShooter(3000);
      intake.setIntake(1);
      intake.wheelsIn();
    }, shooter, conveyor, intake)
    .andThen(bob().raceWith(
      new RunCommand(conveyor::autoConveyor, conveyor)))
    .andThen(new RunCommand(() -> {
        hood.setHood(1);
        shooter.visionShootLong();
        if (shooter.atSpeed()) {
          conveyor.runConveyor();
        } else {
          conveyor.stopConveyor();
        }
        swerve.drive(0, 0, 0, true);
      }, shooter, conveyor).withName("SHOOT").withTimeout(2))
    .andThen(bob1().raceWith(
      new RunCommand(conveyor::autoConveyor, conveyor)))
    .andThen(new InstantCommand(() -> {
      swerve.drive(0, 0, 0, true);
      conveyor.stopConveyor();
    }))
    .andThen(new InstantCommand(() -> {
      intake.setIntake(0);
      intake.wheelsIn(0);
    }))
    .andThen(new RunCommand(() -> {
      shooter.visionShootLong();
      if (shooter.atSpeed()) {
        conveyor.runConveyor();
      }
    }).withTimeout(3))
    .andThen(new InstantCommand(() -> {
      shooter.stopShooter();
      conveyor.stopConveyor();
    }, shooter, conveyor));
  }

  public Command getFiveBallStable() {
    return new InstantCommand(() -> {
      shooter.setShooter(3000);
      intake.setIntake(1);
       intake.wheelsIn(1);
     }, shooter, conveyor, intake)
     .andThen(bob().raceWith(
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
       }, shooter, conveyor).withName("SHOOT").withTimeout(2))
     .andThen(bob1().raceWith(
       new RunCommand(conveyor::autoConveyor, conveyor)))
     .andThen(new InstantCommand(() -> {
       swerve.drive(0, 0, 0, true);
       conveyor.stopConveyor();
     }))
     .andThen(new InstantCommand(() -> {
       intake.setIntake(0);
       intake.wheelsIn(0);
     }))
     .andThen(new RunCommand(() -> {
       shooter.visionShootLong();
       if (shooter.atSpeed()) {
         conveyor.runConveyor();
       } else {
         conveyor.autoConveyor();
       }
     }).withTimeout(1.25))
     .andThen(new InstantCommand(() -> {
       shooter.setShooter(3000);
       conveyor.stopConveyor();
       intake.wheelsIn(1);
       intake.setIntake(1);
     }, shooter, conveyor))
     .andThen(bob2().raceWith(new RunCommand(conveyor::autoConveyor, conveyor)))
     .andThen(bob3().raceWith(new RunCommand(conveyor::autoConveyor, conveyor)))
     .andThen(new InstantCommand(conveyor::stopConveyor, conveyor))
     .andThen(new RunCommand(() -> {
       swerve.drive(0, 0, 0, true);
       shooter.visionShootLong();
       if (shooter.atSpeed()) {
         conveyor.runConveyor();
       } else {
         conveyor.autoConveyor();
       }
     }, shooter, conveyor).withTimeout(2)
     .andThen(new InstantCommand(() -> {
        conveyor.stopConveyor();
        shooter.setShooter(3000);
     }, conveyor, shooter)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
