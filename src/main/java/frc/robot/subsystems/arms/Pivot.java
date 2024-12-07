// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO pivot;
  private final PivotIOInputsAutoLogged pInputs = new PivotIOInputsAutoLogged();

  private static double kP;
  private static double kG;
  private static double kV;

  private static double maxVelocityDegPerSec;
  private static double maxAccelerationDegPerSecSquared;

  private TrapezoidProfile pivotProfile;
  private TrapezoidProfile.Constraints pivotConstraints;

  private TrapezoidProfile.State pivotGoalStateDegrees = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotCurrentStateDegrees = new TrapezoidProfile.State();

  double goalDegrees;

  private ArmFeedforward pivotFFModel;

  /** Creates a new Pivot. */
  public Pivot(PivotIO pivot) {
    this.pivot = pivot;
    switch (SimConstants.currentMode) {
      case REAL:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
      case REPLAY:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
      case SIM:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
      default:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
    }

    maxVelocityDegPerSec = 150;
    maxAccelerationDegPerSecSquared = 226;
    // maxAccelerationDegPerSecSquared = 180;

    pivotConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    pivotProfile = new TrapezoidProfile(pivotConstraints);

    // setPivotGoal(90);
    // setPivotCurrent(getPivotPositionDegs());
    pivotCurrentStateDegrees =
        pivotProfile.calculate(0, pivotCurrentStateDegrees, pivotGoalStateDegrees);

    pivot.configurePID(kP, 0, 0);
    pivotFFModel = new ArmFeedforward(0, kG, kV, 0);
  }

  public void setBrakeMode(boolean bool) {
    pivot.setBrakeMode(bool);
  }

  public double getPivotPositionDegs() {
    return pInputs.positionDegs;
  }

  public boolean atGoal(double threshold) {
    return (Math.abs(pInputs.positionDegs - goalDegrees) <= threshold);
  }

  private double getPivotError() {
    return pInputs.positionSetpointDegs - pInputs.positionDegs;
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    positionDegs = MathUtil.clamp(positionDegs, 33, 120);
    pivot.setPositionSetpointDegs(
        positionDegs,
        pivotFFModel
            .calculate(
                Angle.ofBaseUnits(positionDegs, Degrees),
                AngularVelocity.ofBaseUnits(velocityDegsPerSec, DegreesPerSecond))
            .in(Volts));
  }

  public void pivotStop() {
    pivot.stop();
  }

  public void setPivotGoal(double goalDegrees) {
    this.goalDegrees = goalDegrees;
    pivotGoalStateDegrees = new TrapezoidProfile.State(goalDegrees, 0);
  }

  public void setPivotCurrent(double currentDegrees) {
    pivotCurrentStateDegrees = new TrapezoidProfile.State(currentDegrees, 0);
  }

  public Command setPivotTarget(double goal, double threshold){

    
      return new InstantCommand(()-> setPivotGoal(goal), this).until(()-> atGoal(threshold));
    
  }




  @Override
  public void periodic() {
    pivot.updateInputs(pInputs);

    pivotCurrentStateDegrees =
        pivotProfile.calculate(
            SubsystemConstants.LOOP_PERIOD_SECONDS,
            pivotCurrentStateDegrees,
            pivotGoalStateDegrees);

    setPositionDegs(pivotCurrentStateDegrees.position, pivotCurrentStateDegrees.velocity);

    Logger.processInputs("Pivot", pInputs);
    Logger.recordOutput("pivot error", getPivotError());

    Logger.recordOutput("pivot goal", goalDegrees);
    // This method will be called once per scheduler run
  }

}
