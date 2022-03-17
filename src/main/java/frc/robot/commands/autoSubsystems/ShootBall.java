// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoSubsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.atSpeed()) {
      conveyor.runConveyor();
    }

    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    if (shooter.getExtended()) {
      if (ty > 0)
        shooter.visionShootShort();
      else
        shooter.visionShootLong();
    } else {
      if (ty < -19) {
        shooter.visionShootLong();
      } else {
        shooter.visionShootShort();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
    conveyor.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
