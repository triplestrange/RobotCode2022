// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeBall extends CommandBase {
  private Intake intake;
  private Conveyor conveyor;
  private int wheels;
  /** Creates a new IntakeBall. */
  public IntakeBall(Intake intake, Conveyor conveyor, int wheels) {
    this.intake = intake;
    this.wheels = wheels;
    this.conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntake(1);
    if (wheels == -1) {
      intake.wheelsOut();
      conveyor.runConveyor(-0.5);
    } else if (wheels == 1) {
      intake.wheelsIn();
      conveyor.runConveyor(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntake(0);
    intake.stop();
    conveyor.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
