// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoSubsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.*;

public class AimBot extends CommandBase {
  private Turret turret;
  private Hood hood;
  /** Creates a new TurretAim. */
  public AimBot(Turret turret, Hood hood) {
    this.turret = turret;
    this.hood = hood;
    addRequirements(turret, hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);

    // if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) !=0) {
    //   turret.turretVision();
    // } else {
    //   turret.faceGoal();
    // }
    //if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) !=0) {
       turret.turretVision();
       if (hood.getExtended()) {
        if (ty > 0)
          hood.setHood(0);
        else
          hood.setHood(1);
      } else {
        if (ty < -19) {
          hood.setHood(1);
        } else {
          hood.setHood(0);
        }
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
