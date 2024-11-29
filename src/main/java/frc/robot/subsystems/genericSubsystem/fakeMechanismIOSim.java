package frc.robot.subsystems.genericSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class fakeMechanismIOSim implements fakeMechanismIO {

  private final DCMotorSim sim = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.5);

  private double appliedVolts = 0;
  private final PIDController controller = new PIDController(0, 0, 0);
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);

  @Override
  public void updateInputs(fakeMechanismIOInputs inputs) {
    sim.update(0.02);

    inputs.positionRad = sim.getAngularPositionRotations();
    inputs.velocityRadPerSec = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {

    controller.setPID(kP, kI, kD);
  }

  @Override
  public void setPosition(double goalRevolutions, double volts) {
    double currentHeight = sim.getAngularPositionRotations();
    double velocity = sim.getAngularVelocityRadPerSec();

    double voltage = controller.calculate(currentHeight, goalRevolutions);
    double ffVolts = ff.calculate(velocity);

    appliedVolts = voltage + ffVolts;
    double clampedEffort = MathUtil.clamp(appliedVolts, -12, 12);
    sim.setInputVoltage(clampedEffort);
  }

  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    // TODO Auto-generated method stub
    fakeMechanismIO.super.setVelocity(velocityRadPerSec, ffVolts);
  }

  @Override
  public void setVoltage(double volts) {
    // TODO Auto-generated method stub
    fakeMechanismIO.super.setVoltage(volts);
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    sim.setInputVoltage(0);
  }
}
