// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoRoutines;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerve.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimb extends SequentialCommandGroup {
  /** Creates a new AutoClimb. */
  public AutoClimb(SwerveDrive swerve, Climber climb, int mode) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    BooleanSupplier finishedLUp = new BooleanSupplier() {
      public boolean getAsBoolean() {
        return SmartDashboard.getNumber("ClimbL", 0.0) == -194;
      }
    };

    BooleanSupplier finishedLDown = new BooleanSupplier() {
      public boolean getAsBoolean() {
        return SmartDashboard.getNumber("ClimbL", 0.0) == -1;
      }
    };

    BooleanSupplier finishedRUp = new BooleanSupplier() {
      public boolean getAsBoolean() {
        return SmartDashboard.getNumber("ClimbL", 0.0) == 194;
      }
    };

    BooleanSupplier finishedRDown = new BooleanSupplier() {
      public boolean getAsBoolean() {
        return SmartDashboard.getNumber("ClimbL", 0.0) == 1;
      }
    };

    Command leftUp = new RunCommand(() -> {
      climb.setLeft(-195.0);
      if (SmartDashboard.getNumber("ClimbL", 0.0) == -194) {
        clearGroupedCommands();
      }
    }, climb).withInterrupt(finishedLUp);

    Command leftDown = new RunCommand(() -> {
      climb.setLeft(-30);
    }, climb).withInterrupt(finishedLDown);

    Command rightUp = new RunCommand(() -> {
      climb.setLeft(195.0);
    }, climb).withInterrupt(finishedRUp);

    Command rightDown = new RunCommand(() -> {
      climb.setLeft(30);
    }, climb).withInterrupt(finishedRDown);

    Command bothUp = new RunCommand(() -> {
      climb.setRight(-195.0);
      climb.setLeft(195.0);
    }, climb).withInterrupt(finishedLDown);

    Command bothDown = new RunCommand(() -> {
      climb.setRight(0);
      climb.setLeft(0);
    }, climb).withInterrupt(finishedLDown);
    
    
    addCommands(bothUp.withTimeout(1)
      .andThen(new InstantCommand(climb::extend, climb))
      .andThen(new WaitCommand(0.5))
      .andThen(bothDown.withTimeout(1)));
  }
}
