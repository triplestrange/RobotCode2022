// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooter extends CommandBase {
  private final Shooter shooter;
  private final boolean toggleHood;
  private boolean auto;
  /** Creates a new Command. */
  public RunShooter(Shooter shooter, boolean auto, boolean toggleHood) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.auto = auto;
    this.shooter = shooter;
    this.toggleHood = toggleHood;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (toggleHood) {
      shooter.test();
    } else {
      if (auto) {
        shooter.visionShoot();
      } else {
        shooter.setShooter();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
