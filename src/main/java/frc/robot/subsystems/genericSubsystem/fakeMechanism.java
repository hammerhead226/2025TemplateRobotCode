// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.genericSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.physicalConstants;

public class fakeMechanism extends SubsystemBase {
  /** Creates a new fakeMechanism. */
  private final fakeMechanismIO io;
  private final fakeMechanismIOInputsAutoLogged fmInputs = new fakeMechanismIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  public fakeMechanism(fakeMechanismIO io) {
    this.io = io;
    switch (physicalConstants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.1, 0.05);
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.03);
        io.configurePID(0.5, 0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     io.updateInputs(fmInputs);
    Logger.processInputs("Flywheel", fmInputs);
  }
//provide implementation for runMotor
//set the velocity of the motor to 200 and the ffVolts to the feed forward models calculations
//to get feed forward volts use ffModel.calculate
//record the velocity to advantage scope using Logger.recordOutput("give it a sensible name", 200)

  public void runMotor(){
         
    
    // Log motor setpoint
   
  }
}
