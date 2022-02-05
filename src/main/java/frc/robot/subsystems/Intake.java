// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
  2/5/22
  Austin Irwin
  Code "adapted" (copied with minor changes)
  from 2021OffseasonCode for intake

  This class controls the intake of our robot and should:
    extend intake arms using solenoids
    retract intake arms using solenoids
    run wheels on the intake bars to grab and pull in
    cargo.
 
  methods contain booleans  input to indicate wether the robot
  is running in autonomous or tele-op.

  What is done:
all necesary methods are created with good reference from 2021
libraries imported


  What still needs done:
initialization of constants.
verification that methods are compatible wtih new robot.
debugging.
possible more, i dont know what i dont know.
replace smartDashboard with glass


*/
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 
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
    intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
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

  /*
  * pos 0 is not extended
  * pos 1 is extended
  *
  */
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

  /*
  * wheels will run at the same speed 
  *
  *
  */
  public void runWheels() {
    intakeMotor.set(speed);
  }


  /***********************getters and setters****************/
  
  //Getter for the boolean variable extended
  public boolean getExtended() {
    return extended;
  }

  //setter for the boolean variable extended
  public void setExtended(boolean Extended) {
    this.extended = Extended;
  }


  @Override
  public void periodic() {
    speed = table.getEntry("IntakeSpeedSetpoint").getDouble(setpoint);
    table.getEntry("IntakeExtended").setBoolean(getExtended());
    table.getEntry("IntakeSpeed").setDouble(intakeEncoder.getVelocity());
  }
}
