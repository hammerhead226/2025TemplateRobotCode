package frc.robot.subsystems.genericSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface fakeMechanismIO {
  @AutoLog
  public static class fakeMechanismIOInputs {
   // add a double positionRad
   //add a double velocityRadPerSec 
   //add a double appliedVolts
   //add a double currentAmps 
  }

  //ALL PARAMETERS SHOULD BE DOUBLES 

 //update inputs parameters:fakeMechanismIOInputs

  /** Run open loop at the specified voltage. */
  //setVoltage parameters: a double volts

  /** Run closed loop at the specified velocity. */
  //setVelocity parameters: velocityRadPerSec and ffVolts

  /** Stop in open loop. */
  //stop method, no parameters

  /** Set velocity PID constants. */
  //configurePID parameters: kP, kI, kD
}
