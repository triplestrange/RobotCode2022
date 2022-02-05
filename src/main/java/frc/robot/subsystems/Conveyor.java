// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private double speed;
  private NetworkTable table;
  /** Creates a new Conveyor. */
  public Conveyor() {
    motor =  new CANSparkMax(0, MotorType.kBrushless);
    encoder = motor.getEncoder();

    motor.setIdleMode(IdleMode.kBrake);

    table = NetworkTableInstance.getDefault().getTable("conveyor");
  }

  public void runConveyor() {
    motor.set(speed);
  }

  public void stopConveyor() {
    motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    speed = table.getEntry("ConveyorSetpoint").getDouble(0.7);
    table.getEntry("ConveyorSpeed").setDouble(encoder.getVelocity());
  }
}
