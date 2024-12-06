// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.CurrentUnit;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.physicalConstants;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

  private final DCMotor simGearbox = DCMotor.getKrakenX60Foc(2);
  private ElevatorSim sim = new ElevatorSim(simGearbox, 1, 1, 0.01, 0.0, 3, true, 0.0);
  private PIDController pid = new PIDController(0, 0, 0);

  private Distance position = Inches.of(0);
  private LinearVelocity velocityInchPerSec = InchesPerSecond.of(0);
  private Voltage appliedVolts = Volts.of(0.0);
  private Current currentAmps = Amps.of(0.0);
  private Distance positionSetpointInches = Inches.of(0);

  private double clampedValueLowVolts = -12.0;
  private double clampedValueHighVolts = 12.0;

  private double metersToInches = 39.37;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    positionSetpointInches = Inches.of(pid.getSetpoint());

    appliedVolts.plus(
        Volts.of(MathUtil.clamp(
            pid.calculate(Meters.of(sim.getPositionMeters()).in(Inches), positionSetpointInches.in(Inches)), -12.0, 12)));

    sim.setInputVoltage(appliedVolts.in(Volts));

    position = Inches.of(Meters.of(sim.getPositionMeters()).in(Inches));
    position = Meters.of(sim.getPositionMeters());

    velocityInchPerSec = InchesPerSecond.of(MetersPerSecond.of(sim.getVelocityMetersPerSecond()).in(MetersPerSecond));
    currentAmps = Amps.of(sim.getCurrentDrawAmps());

    inputs.positionSetpointInch = position.plus(Meters.of(2)); 
    inputs.appliedVolts = appliedVolts.in(Volts);
    inputs.elevatorPosition = position;
    inputs.elevatorVelocity = velocityInchPerSec;
    inputs.currentAmps = currentAmps.in(Amps);

    sim.update(physicalConstants.LOOP_PERIOD_SECS);
  }

  @Override
  public void runCharacterization(double volts) {
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPositionSetpoint(double position, Voltage ffVolts) {
    appliedVolts = ffVolts;
    pid.setSetpoint(position);
  }

  @Override
  public void stop() {
    appliedVolts = Volts.of(0);
    pid.setSetpoint(sim.getPositionMeters() * 39.37);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
