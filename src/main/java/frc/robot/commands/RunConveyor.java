// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class RunConveyor extends CommandBase {
  private Conveyor conveyor;
  private int dir;
  /** Creates a new RunConveyor. */
  public RunConveyor(Conveyor conveyor, int dir) {
    this.conveyor = conveyor;
    this.dir = dir;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (dir == -1) {
      conveyor.runConveyor(-0.5);
    } else if (dir == 1) {
      conveyor.runConveyor(0.5);
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
