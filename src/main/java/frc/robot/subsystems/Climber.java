// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Climber extends SubsystemBase {
  private CANSparkMax motor1;
  private CANSparkMax motor2;
  private RelativeEncoder encoder1;
  private RelativeEncoder encoder2;
  private DoubleSolenoid solenoid;
  private NetworkTable table;
  private double speedL, speedR;
  private boolean extended;

  /** Creates a new Climber. */
  public Climber() {  

    motor2 = new CANSparkMax(Electrical.climbR, MotorType.kBrushless);
    motor1 = new CANSparkMax(Electrical.climbL, MotorType.kBrushless);
    motor1.restoreFactoryDefaults();
    motor2.restoreFactoryDefaults();
    motor1.enableSoftLimit(SoftLimitDirection.kForward , false);
    // motor1.setSoftLimit(SoftLimitDirection.kForward, 9.93f);
    motor1.enableSoftLimit(SoftLimitDirection.kReverse, false);
    // motor1.setSoftLimit(SoftLimitDirection.kReverse, 9.93f);
    motor2.enableSoftLimit(SoftLimitDirection.kForward , false);
    // motor2.setSoftLimit(SoftLimitDirection.kForward, -5.80f);
    motor2.enableSoftLimit(SoftLimitDirection.kReverse, false);
    // motor2.setSoftLimit(SoftLimitDirection.kReverse, -5.80f);
    solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 9); 
    solenoid.set(Value.kReverse);
    extended = solenoid.get() == Value.kForward;

    motor1.setSmartCurrentLimit(60);
    motor2.setSmartCurrentLimit(60);

    motor1.burnFlash();
    motor2.burnFlash();

    encoder1 = motor1.getEncoder();
    encoder2 = motor2.getEncoder();

    table = NetworkTableInstance.getDefault().getTable("intake");

    periodic();

    // LiveWindow
    addChild("Right", solenoid);
  }

  public void moveRight() {
    motor1.set(-speedR);
  }

  public void toggle() {
    // if (solenoid.get() == Value.kReverse) {
    //   solenoid.set(Value.kForward);
    // } else {
    //   solenoid.set(Value.kReverse);
    // }
    solenoid.toggle();
  }

  public void extend() {
    solenoid.set(Value.kReverse);
  }

  public void retract() {
    solenoid.set(Value.kForward);
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
    SmartDashboard.putBoolean("ClimbExtended", solenoid.get() == Value.kForward);
  }
}
