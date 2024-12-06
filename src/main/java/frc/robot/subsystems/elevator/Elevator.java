package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevator;

  private final ElevatorIOInputsAutoLogged eInputs = new ElevatorIOInputsAutoLogged();

  private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("Elevator/kP");
  private static final LoggedNetworkNumber kI = new LoggedNetworkNumber("Elevator/kI");

  private static final LoggedNetworkNumber kS = new LoggedNetworkNumber("Elevator/kS");
  private static final LoggedNetworkNumber kG = new LoggedNetworkNumber("Elevator/kG");
  private static final LoggedNetworkNumber kV = new LoggedNetworkNumber("Elevator/kV");
  private static final LoggedNetworkNumber kA = new LoggedNetworkNumber("Elevator/kA");

  // amp bar gains

  private static final LoggedNetworkNumber barkP = new LoggedNetworkNumber("Bar/kP");
  private static final LoggedNetworkNumber barkV = new LoggedNetworkNumber("Bar/kV");
  private static final LoggedNetworkNumber barkG = new LoggedNetworkNumber("Bar/kG");

  private TrapezoidProfile extenderProfile;
  private TrapezoidProfile.Constraints extenderConstraints =
      new TrapezoidProfile.Constraints(30, 85);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();

  private static double maxVelocityDegPerSec;
  private static double maxAccelerationDegPerSecSquared;

  private TrapezoidProfile barProfile;
  private TrapezoidProfile.Constraints barConstraints;

  private TrapezoidProfile.State barGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State barCurrent = new TrapezoidProfile.State();

  private double goal;
  // private double barGoalPos;
  private final ElevatorFeedforward elevatorFFModel;

  public Elevator(ElevatorIO elevator) {
    this.elevator = elevator;

    switch (physicalConstants.getMode()) {
      case REAL:
      
        kS.setDefault(0);
        kG.setDefault(0.25);
        kV.setDefault(0);
        kA.setDefault(0);

        kP.setDefault(0.44);
        kI.setDefault(0);

        barkP.setDefault(0.25);
        barkV.setDefault(0.2);
        barkG.setDefault(0);
        break;
      case REPLAY:
        kS.setDefault(0);
        kG.setDefault(0.13);
        kV.setDefault(0);
        kA.setDefault(0);

        kP.setDefault(15);
        kI.setDefault(0);

        barkP.setDefault(0);
        barkV.setDefault(0);
        barkG.setDefault(0);
        break;
      case SIM:
        kS.setDefault(0);
        kG.setDefault(0.04);
        kV.setDefault(0);
        kA.setDefault(0);

        kP.setDefault(1);
        kI.setDefault(0);

        barkP.setDefault(0);
        barkV.setDefault(0);
        barkG.setDefault(0);
        break;
      default:
        kS.setDefault(0);
        kG.setDefault(0.13);
        kV.setDefault(0);
        kA.setDefault(0);

        kP.setDefault(15);
        kI.setDefault(0);

        barkP.setDefault(0);
        barkV.setDefault(0);
        barkG.setDefault(0);
        break;
    }

    setExtenderGoal(0.162);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
    extenderCurrent = extenderProfile.calculate(0, extenderCurrent, extenderGoal);

    maxVelocityDegPerSec = 1200;
    maxAccelerationDegPerSecSquared = 1550;

    barConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    barProfile = new TrapezoidProfile(barConstraints);
    barCurrent = barProfile.calculate(0, barCurrent, barGoal);

    this.elevator.configurePID(kP.get(), 0, 0);
    elevatorFFModel = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());
  }

  public boolean atGoal() {
    return (Math.abs(eInputs.elevatorPosition - goal)
        <= physicalConstants.ElevatorConstants.THRESHOLD);
  }

  public double getElevatorPosition() {
    return eInputs.elevatorPosition;
  }

  private double getElevatorError() {
    return eInputs.positionSetpoint - eInputs.elevatorPosition;
  }

  public boolean elevatorAtSetpoint() {
    return (Math.abs(getElevatorError()) <= physicalConstants.ElevatorConstants.THRESHOLD);
  }

  // public void setbarCurrent(double current) {

  //   barCurrent = new TrapezoidProfile.State(current, 0);
  // }

  public void setExtenderGoal(double setpoint) {
    goal = setpoint;
    extenderGoal = new TrapezoidProfile.State(setpoint, 0);
  }

  public void setPositionExtend(double position, double velocity) {
    elevator.setPositionSetpoint(position, elevatorFFModel.calculate(MetersPerSecond.of(velocity)));
  }

  public void elevatorStop() {
    elevator.stop();
  }

  public double calculateAngle() {
    double angle = 0.0;
    return angle;
  }

  public void setConstraints(
      double maxVelocityMetersPerSec, double maxAccelerationMetersPerSecSquared) {
    extenderConstraints =
        new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSec, maxAccelerationMetersPerSecSquared);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
  }

  public boolean isExtended() {
    return extenderGoal.position == physicalConstants.ElevatorConstants.EXTEND_SETPOINT_INCH;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());

    elevator.updateInputs(eInputs);

    extenderCurrent =
        extenderProfile.calculate(
            physicalConstants.LOOP_PERIOD_SECS, extenderCurrent, extenderGoal);

    barCurrent = barProfile.calculate(physicalConstants.LOOP_PERIOD_SECS, barCurrent, barGoal);

    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    Logger.processInputs("Elevator", eInputs);

    Logger.recordOutput("amp bar currentPos", barCurrent.position);
   // if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
    // elevator.configurePID(kP.get(), kI.get(), 0);
  // }
   // if (barkP.hasChanged(hashCode())
   //     || barkV.hasChanged(hashCode())
  //      || barkG.hasChanged(hashCode())) {}
 // }

 }
}