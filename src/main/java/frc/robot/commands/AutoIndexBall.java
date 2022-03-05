// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class AutoIndexBall extends CommandBase {
  private Conveyor conveyor;
  /** Creates a new AutoIndexBall. */
  public AutoIndexBall(Conveyor conveyor) {
    this.conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (conveyor.getBotSensor()) {
      conveyor.runConveyor();
      if (conveyor.getTopSensor()) {
        conveyor.stopConveyor();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}