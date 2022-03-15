// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoSubsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.*;

public class AimBot extends CommandBase {
  private Turret turret;
  /** Creates a new TurretAim. */
  public AimBot(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) !=0) {
    //   turret.turretVision();
    // } else {
    //   turret.faceGoal();
    // }
    if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) !=0) {
       turret.turretVision();
    } else {
      turret.stop();
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
