// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretVision extends CommandBase {
  private Turret turret;
  private NetworkTable limelight, turretTbl;
  /** Creates a new TurretVision. */
  public TurretVision(Turret turret) {
    this.turret = turret;
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    turretTbl = NetworkTableInstance.getDefault().getTable("turret");
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if there are targets: tv = true
    if (limelight.getEntry("tv").getBoolean(false)) {
      double xOffset = limelight.getEntry("tx").getDouble(0.0);
      turretTbl.getEntry("TurretSetpointP").setDouble(-xOffset);
      
      turret.setVelocity();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
