package frc.robot.subsystems.indexers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.PhysicalConstants;

public class IndexerIOSim implements IndexerIO {

  private double indexerGearing = 0;
  private double indexerMOI = 0;
  private int numberOFIndexerMotors = 0;

  private final DCMotor indexer = DCMotor.getKrakenX60(numberOFIndexerMotors);
  private final DCMotorSim sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(indexer, indexerMOI, indexerGearing), indexer);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private double clampedValueLowVolts = -12.0;
  private double clampedValueHighVolts = 12.0;

  private boolean closedLoop = true;

  private double appliedVolts = 0.0;

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  @Override
  public void setPosition(double positionDegs, double ffVolts) {
    // appliedVolts = ffVolts;
    pid.setSetpoint(positionDegs);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    appliedVolts = 0;
    pid.setSetpoint(sim.getAngularPositionRad());
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(
              pid.calculate(sim.getAngularPositionRad()),
              clampedValueLowVolts,
              clampedValueHighVolts);

      sim.setInputVoltage(appliedVolts);
    }

    sim.update(PhysicalConstants.LOOP_PERIOD_SECS);

    inputs.positionRads = sim.getAngularPositionRad();
    inputs.velocityRadsPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }
}
