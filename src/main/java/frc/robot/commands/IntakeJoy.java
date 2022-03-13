// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

public class IntakeJoy extends CommandBase {
  private Intake intake;
  private Conveyor conveyor;
  private Joystick joystick;
  /** Creates a new IntakeJoy. */
  public IntakeJoy(Intake intake, Conveyor conveyor, Joystick joystick) {
    this.intake = intake;
    this.conveyor = conveyor;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // left trigger
    if (Math.abs(joystick.getRawAxis(2)) > 0.05) {
      intake.wheelsIn();
      if (SmartDashboard.getBoolean("autoIndex", false)) {
        conveyor.autoConveyor();
      }
    }
    
    if (Math.abs(joystick.getRawAxis(3)) > 0.05) {
      intake.wheelsOut();
    }

    // if A is pressed, extend
    if (joystick.getRawButton(1)) {
      intake.setIntake(1);
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
