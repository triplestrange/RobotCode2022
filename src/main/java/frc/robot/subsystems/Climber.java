// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Climber extends SubsystemBase {
  private CANSparkMax motor1;
  private CANSparkMax motor2;
  private RelativeEncoder encoder1;
  private RelativeEncoder encoder2;
  private NetworkTable table;
  private double speedL, speedR;

  /** Creates a new Climber. */
  public Climber() {  
    motor1 = new CANSparkMax(Electrical.climbL, MotorType.kBrushless);
    motor2 = new CANSparkMax(Electrical.climbR, MotorType.kBrushless);
    encoder1 = motor1.getEncoder();
    encoder2 = motor2.getEncoder();

    table = NetworkTableInstance.getDefault().getTable("intake");

    periodic();
  }

  public void moveRight() {
    motor1.set(speedR);
  }

  public void moveLeft() {
    motor2.set(speedL);
  }

  public void stop() {
    motor1.set(0);
    motor2.set(0);
  }

  public void stopLeft() {
    motor2.set(0);
  }

  public void stopRight() {
    motor1.set(0);
  }

  public void initDefaultCommand() {
    
  }
   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    speedL = SmartDashboard.getNumber("ClimberSpeedL", 0.25);
    speedR = SmartDashboard.getNumber("ClimberSpeedR", 0.25);
    SmartDashboard.putNumber("ClimbR", encoder1.getPosition());
    SmartDashboard.putNumber("ClimbL", encoder2.getPosition());
  }
}
