// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  private CANSparkMax turretMotor;
  private RelativeEncoder turretEncoder;
  private SparkMaxPIDController m_turretPIDController;
  private double kP, kFF, kI, kD, kIz, kMaxOutput, kMinOutput, setpoint;
  private NetworkTable table;
  
  /** Creates a new Turret. */
  public Turret() {
    table = NetworkTableInstance.getDefault().getTable("turret");
    
    turretMotor = new CANSparkMax(18, MotorType.kBrushless);
    turretMotor.restoreFactoryDefaults();
    turretMotor.setSmartCurrentLimit(30);
    turretMotor.setIdleMode(IdleMode.kBrake);
    turretMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kForward, 0);

    turretMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    turretMotor.setSoftLimit(SoftLimitDirection.kReverse, -130);

    turretMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

    turretMotor.burnFlash();

    turretEncoder = turretMotor.getEncoder();
    turretEncoder.setPosition(0);

    m_turretPIDController = turretMotor.getPIDController();

    // PID coefficients
    kP = 0.1;
    kFF = 1./11000.;
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kMaxOutput = 1;
    kMinOutput = -1;

    // // set PID coefficients
    m_turretPIDController.setP(kP);
    m_turretPIDController.setI(kI);
    m_turretPIDController.setD(kD);
    m_turretPIDController.setIZone(kIz);
    m_turretPIDController.setFF(kFF);
    m_turretPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void setPosition() {
    m_turretPIDController.setReference(setpoint, ControlType.kPosition);
  }

  public void stop() {
    turretMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setpoint = table.getEntry("TurretSetpoint").getDouble(0.0);
    table.getEntry("TurretPos").setDouble(turretEncoder.getPosition());
  }
}
