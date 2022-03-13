// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(joystick.getRawAxis(5)) > 0.05) {
      climb.moveRight(joystick.getRawAxis(5) * 0.5);
    }

    POVButton up = new POVButton(joystick, 0);
    POVButton right = new POVButton(joystick, 90);
    POVButton down = new POVButton(joystick, 180);
    POVButton left = new POVButton(joystick, 270);
    if (up.get()) {
      climb.moveLeft(0.5);
    }
    
    if (down.get()) {
      climb.moveLeft(-0.5);
    }

    if (left.get()) {
      climb.setClimber(1);
    }

    if (right.get()) {
      climb.setClimber(0);
    }
    
    // if (Math.abs(joystick.getRawAxis(1)) > 0.3) {
    //   SmartDashboard.putNumber("ClimberSpeedL", joystick.getRawAxis(1));
    //   climb.moveLeft();
    // } else {
    //   climb.stopLeft();
    // }

    // if (Math.abs(joystick.getRawAxis(5)) > 0.3) {
    //   SmartDashboard.putNumber("ClimberSpeedR", joystick.getRawAxis(5));
    //   climb.moveRight();
    // } else {
    //    climb.stopRight();
    // }

    // if (extend) {
    //   climb.toggleClimb();
    // }

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
