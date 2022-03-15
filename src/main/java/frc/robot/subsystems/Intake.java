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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final RelativeEncoder intakeEncoder;
  private final DoubleSolenoid solenoid1;
  private boolean extended = false;
  private NetworkTable table;
  private double speedI;
  
  public Intake() {
    super();    
    intakeMotor = new CANSparkMax(Electrical.intake, MotorType.kBrushless);
    intakeMotor.setSmartCurrentLimit(30);
    intakeEncoder = intakeMotor.getEncoder();
    solenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 5, 10); 

    // intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    // intakeMotor.enableVoltageCompensation(11);
    // intakeMotor.setSmartCurrentLimit(80);
    intakeMotor.burnFlash();

    table = NetworkTableInstance.getDefault().getTable("intake");
    speedI = 0.1;

    periodic();

    // LiveWindow
    addChild("Solenoid", solenoid1);
  }

  public void setIntake(int pos) {
      if(pos==0){ //retract
        solenoid1.set(Value.kReverse);
        setExtended(false);
      }
      else { //extend
        solenoid1.set(Value.kForward);
        setExtended(true);
      }
  }

  public void toggleIntake() {
    if (getExtended()) {
      solenoid1.set(Value.kReverse);
      setExtended(false);
    } else {
      solenoid1.set(Value.kForward);
      setExtended(true);
    }
  }

  public void wheelsIn() {
    intakeMotor.set(speedI);
  }

  public void wheelsIn(double speed) {
    intakeMotor.set(speed);
  }

  public void wheelsOut() {
    intakeMotor.set(-speedI);
  }
  
  public void stop() {
    intakeMotor.set(0);
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
    speedI = SmartDashboard.getNumber("IntakeSetpoint", 0.5);
    SmartDashboard.putBoolean("IntakeExtended", getExtended());
    SmartDashboard.putNumber("IntakeSpeed", intakeEncoder.getVelocity());
  }
}
