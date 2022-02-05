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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; 
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;






public class Intake extends SubsystemBase {

  //intializes local variables

    /*Constants class has not yet been filled inwith constant values
    so constant.intake.motor gets angry for now*/
  private final CANSparkMax intakeMotor 
    = new CANSparkMax(0/*intake sparkmax# */,MotorType.kBrushless);
  private final DoubleSolenoid intakeSolenoid 
    = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0,1); //change from null
  private boolean extended = false;

  /*
   */
  public Intake() {
    super();
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.enableVoltageCompensation(11);
    intakeMotor.setSmartCurrentLimit(20);
    //Add code to connect to Glass, our new dashboard
    //that is replacing smart dashboard
    intakeMotor.burnFlash();
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
  public void runWheels(double speed) {
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


  /*IDK the point of this but it was in the offseason
  so i am puting it here
  may have been a goal that was never finished
  @Override protected void initDefaultCommand() {
    //  TODO Auto-Generated method stub

  } */
}
