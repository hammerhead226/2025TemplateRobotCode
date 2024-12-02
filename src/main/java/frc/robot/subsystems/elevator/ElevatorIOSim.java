// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.physicalConstants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  //SIM VARIABLES (CHANGE)
  private int gearBoxMotorCount = 2;
  private int gearing = 1;
  private double carriageMassKg = 1;
  private double drumRadiusMeters = 0.01;
  private double minHeightMeters = 0;
  private double maxHeightMeters = 3;
  private boolean simulateGravity = true;
  private double initialPositionMeters = 0.0;

  private final DCMotor simGearbox = DCMotor.getFalcon500(gearBoxMotorCount);
  private ElevatorSim sim = new ElevatorSim(simGearbox, gearing, carriageMassKg, drumRadiusMeters, minHeightMeters, maxHeightMeters, simulateGravity, initialPositionMeters);
  private PIDController pid = new PIDController(0, 0, 0);

  private double positionInches = 0.0;
  private double velocityInchPerSec = 0.0;
  private double appliedVolts = 0.0;
  private double currentAmps = 0.0;
  private double positionSetpointInches = 0.0;

  private double clampedValueLowVolts = -12.0;
  private double clampedValueHighVolts = 12.0;

  private double metersToInches = 39.37;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    positionSetpointInches = pid.getSetpoint();

    appliedVolts +=
        MathUtil.clamp(
            pid.calculate(sim.getPositionMeters() * metersToInches, positionSetpointInches), clampedValueLowVolts, clampedValueHighVolts);

    sim.setInputVoltage(appliedVolts);

    positionInches = sim.getPositionMeters() * metersToInches;
    velocityInchPerSec = sim.getVelocityMetersPerSecond() * metersToInches;
    currentAmps = sim.getCurrentDrawAmps();

    inputs.positionSetpoint = positionSetpointInches;
    inputs.appliedVolts = appliedVolts;
    inputs.elevatorPosition = positionInches;
    inputs.elevatorVelocity = velocityInchPerSec;
    inputs.currentAmps = currentAmps;

    sim.update(physicalConstants.LOOP_PERIOD_SECS);
  }

  @Override
  public void runCharacterization(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPositionSetpoint(double positionInches, double ffVolts) {
    appliedVolts = ffVolts;
    pid.setSetpoint(positionInches);
  }

  @Override
  public void stop() {
    appliedVolts = 0;
    pid.setSetpoint(sim.getPositionMeters() * metersToInches);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
