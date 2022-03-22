// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoSubsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Intake;

public class ManualFeeder extends CommandBase {
  private Conveyor conveyor;
  private Intake intake;
  private Joystick joystick;
  /** Creates a new LoadBall. */
  public ManualFeeder(Intake intake, Conveyor conveyor, Joystick joystick) {
    addRequirements(intake, conveyor);
    this.intake = intake;
    this.conveyor = conveyor;
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0;
    if (joystick.getRawAxis(2) > 0.1) {
      speed = joystick.getRawAxis(2);
    } else if (joystick.getRawAxis(3) > 0.1) {
      speed = -joystick.getRawAxis(3);
    }

    intake.wheelsIn(-speed);

    if (speed < 0) {
      conveyor.autoConveyor();
    } else {
      conveyor.runConveyor(-speed);
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
