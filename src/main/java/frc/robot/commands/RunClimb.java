// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RunClimb extends CommandBase {
  private final Climber climb;
  private final Joystick joystick;

  /** Creates a new RunClimb. */
  public RunClimb(Climber climb, Joystick joystick) {
    this.climb = climb;
    this.joystick = joystick;
    addRequirements(climb);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(joystick.getRawAxis(5)) > 0.3) {
      SmartDashboard.putNumber("ClimberSpeedL", joystick.getRawAxis(5));
      climb.moveLeft();
    } else {
      climb.stopLeft();
    }

    if (Math.abs(joystick.getRawAxis(1)) > 0.3) {
      SmartDashboard.putNumber("ClimberSpeedR", joystick.getRawAxis(1));
      climb.moveRight();
     } else {
       climb.stopRight();
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
