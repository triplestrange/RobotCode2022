// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoSubsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class LoadBall extends CommandBase {
  private Conveyor conveyor;
  private Intake intake;
  /** Creates a new LoadBall. */
  public LoadBall(Intake intake, Conveyor conveyor) {
    addRequirements(intake, conveyor);
    this.intake = intake;
    this.conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intake until top sensor detects
    if (!conveyor.getTopSensor()) {
      intake.setIntake(1);
      intake.wheelsIn();
    }
    conveyor.autoConveyor();
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
