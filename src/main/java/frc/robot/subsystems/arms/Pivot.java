// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    switch (Constants.getMode()) {
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
        pivotFFModel.calculate(Math.toRadians(positionDegs), Math.toRadians(velocityDegsPerSec)));
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

  @Override
  public void periodic() {
    pivot.updateInputs(pInputs);

    pivotCurrentStateDegrees =
        pivotProfile.calculate(
            Constants.ROBOT_LOOP_PERIOD_SECS, pivotCurrentStateDegrees, pivotGoalStateDegrees);

    setPositionDegs(pivotCurrentStateDegrees.position, pivotCurrentStateDegrees.velocity);

    Logger.processInputs("Pivot", pInputs);
    Logger.recordOutput("pivot error", getPivotError());

    Logger.recordOutput("pivot goal", goalDegrees);
    // This method will be called once per scheduler run
  }
}
