// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Electrical;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final DoubleSolenoid intakeSolenoid;
  private boolean extended = false;
  private NetworkTable table;
  private double speed, setpoint;

  public Intake() {
    super();    
    intakeMotor = new CANSparkMax(Electrical.intake, MotorType.kBrushless);
    hopperMotor = new CANSparkMax(Electrical.hopper, MotorType.kBrushless);
    intakeEncoder = intakeMotor.getEncoder();
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,1); 

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.enableVoltageCompensation(11);
    intakeMotor.setSmartCurrentLimit(20);
    intakeMotor.burnFlash();

    setpoint = 0.75;

    table = NetworkTableInstance.getDefault().getTable("intake");
  }

  public void setIntake(int pos) {
      if(pos==0){ //retract
        intakeSolenoid.set(Value.kReverse);
        setExtended(false);
      }
      else { //extend
        intakeSolenoid.set(Value.kForward);
        setExtended(true);
      }
  }

  public void wheelsIn() {
    intakeMotor.set(speed);
    hopperMotor.set(speed);
  }

  public void wheelsOut() {
    intakeMotor.set(-speed);
    hopperMotor.set(-speed);
  }
  
  public void stop() {
    intakeMotor.set(0);
    hopperMotor.set(0);
  }

  public boolean getExtended() {
    return extended;
  }

  public void setExtended(boolean Extended) {
    this.extended = Extended;
  }

  public void initDefaultCommand() {
    
  }

  @Override
  public void periodic() {
    speed = table.getEntry("IntakeSpeedSetpoint").getDouble(setpoint);
    table.getEntry("IntakeExtended").setBoolean(getExtended());
    table.getEntry("IntakeSpeed").setDouble(intakeEncoder.getVelocity());
  }
}
