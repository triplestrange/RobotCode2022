// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

/** Add your docs here. */
public class RobotContainer {
    private Climber m_climber = new Climber();
    private Conveyor m_conveyor = new Conveyor();
    private Intake m_intake = new Intake();
    private Shooter m_shooter = new Shooter();
    private SwerveDrive m_swerve = new SwerveDrive();
    private Turret m_turret = new Turret();


    public void configureButtonBindings() {
        Joystick m_driverController = new Joystick(0);
        Joystick m_operatorController = new Joystick(1);
    }
    
} 
