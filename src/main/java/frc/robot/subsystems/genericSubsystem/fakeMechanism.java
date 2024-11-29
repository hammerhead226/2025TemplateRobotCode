package frc.robot.subsystems.genericSubsystem;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.physicalConstants;
import org.littletonrobotics.junction.Logger;

public class fakeMechanism extends SubsystemBase {
  /** Creates a new fakeMechanism. */
  private final fakeMechanismIO io;

  private final fakeMechanismIOInputsAutoLogged fmInputs = new fakeMechanismIOInputsAutoLogged();
  private final SimpleMotorFeedforward ffModel;

  private TrapezoidProfile motorProfile;

  private TrapezoidProfile.Constraints motorConstraints = new TrapezoidProfile.Constraints(30, 85);

  private TrapezoidProfile.State motorGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State motorCurrent = new TrapezoidProfile.State();

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

    motorProfile = new TrapezoidProfile(motorConstraints);
    motorCurrent = motorProfile.calculate(0, motorCurrent, motorGoal);
  }

  public double getMotorPosition() {

    return fmInputs.positionRad;
  }

  public void runMotor() {

    io.setVelocity(200, ffModel.calculate(200));

    // Log flywheel setpoint
    Logger.recordOutput("Flywheel/SetpointRPM", 200);
  }

  public void motorStop() {

    io.stop();
  }

  public void setPositionGoal(double goal) {
    motorGoal = new TrapezoidProfile.State(goal, 0);
  }

  public void setPosition(double position, double velocity) {
    io.setPosition(position, ffModel.calculate(velocity));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(fmInputs);
    Logger.processInputs("Flywheel", fmInputs);

    motorCurrent =
        motorProfile.calculate(physicalConstants.LOOP_PERIOD_SECS, motorCurrent, motorGoal);

    setPosition(motorCurrent.position, motorCurrent.velocity);
  }
}
