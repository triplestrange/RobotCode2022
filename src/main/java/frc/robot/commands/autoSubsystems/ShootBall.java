// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoSubsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootBall extends CommandBase {
  private Shooter shooter;
  private Conveyor conveyor;

  /** Creates a new ShootBall. */
  public ShootBall(Shooter shooter, Conveyor conveyor) {
    addRequirements(shooter, conveyor);
    this.shooter = shooter;
    this.conveyor = conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerve.resetOdometryTur();
    SmartDashboard.putBoolean("Shooting", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean extended = SmartDashboard.getBoolean("HoodExtended", false);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    // for shooting while stopped
    if (shooter.atSpeed() && ty != 0) {
      if (SmartDashboard.getBoolean("Turret Good?", false)) {
        conveyor.runConveyor();
      } else {
        conveyor.autoConveyor();
      }
    } else {
      conveyor.autoConveyor();
    }

    if (!extended) {
      shooter.visionShootShort();
    } else {
      shooter.visionShootLong();
    }

    // for shooting while moving
    // shooter.setShooter(3000);
    // if (Math.abs(SmartDashboard.getNumber("ShooterSpeed", 0.0) - 3000)/3000.0 < 0.05) {
    //   if (SmartDashboard.getBoolean("RejectingBall", false)) {
    //     if (Math.abs(SmartDashboard.getNumber("tx", 0.0)) > 16
    //       && Math.abs(SmartDashboard.getNumber("TurretVelocity", 0.0)) < 1000 ) {
    //       conveyor.runConveyor();
    //     } else {
    //       conveyor.autoConveyor();
    //     }
    //   } else {
    //     if (Math.abs(SmartDashboard.getNumber("tx", 0.0)) < 1.5
    //     && Math.abs(SmartDashboard.getNumber("TurretVelocity", 0.0)) < 1000) {
    //       conveyor.runConveyor();
    //     } else {
    //       conveyor.autoConveyor();
    //     }
    //   }
    // } else {
    //   conveyor.autoConveyor();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooter(Constants.Shooting.idleSpeed);
    SmartDashboard.putBoolean("Shooting", false);
    conveyor.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
