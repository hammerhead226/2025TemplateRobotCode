// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexers;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PhysicalConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final IndexerIO indexer;

  private final IndexerIOInputsAutoLogged Iinputs = new IndexerIOInputsAutoLogged();

  private static double kP;
  private static double kG;
  private static double kV;

  private static double maxVelocityRotPerSec;
  private static double maxAccelerationRotPerSecSquared;

  private TrapezoidProfile indexerProfile;
  private TrapezoidProfile.Constraints indexerConstraints;

  private TrapezoidProfile.State indexerGoalStateRotations = new TrapezoidProfile.State();
  private TrapezoidProfile.State indexerCurrentStateRotations = new TrapezoidProfile.State();

  public Indexer(IndexerIO indexer) {
    this.indexer = indexer;

    switch (PhysicalConstants.getMode()) {
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

    maxVelocityRotPerSec = 4;
    maxAccelerationRotPerSecSquared = 4;

    indexerConstraints = new TrapezoidProfile.Constraints(maxVelocityRotPerSec, maxAccelerationRotPerSecSquared);
    indexerProfile = new TrapezoidProfile(indexerConstraints);
    
    indexerCurrentStateRotations = indexerProfile.calculate(0, indexerCurrentStateRotations, indexerGoalStateRotations);
    
    indexer.configurePID(kP, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    indexer.updateInputs(Iinputs);
  }

  public void index(double linearDistanceInches) {

    indexer.setPosition(positionRots, ffvolts);
  }

}
