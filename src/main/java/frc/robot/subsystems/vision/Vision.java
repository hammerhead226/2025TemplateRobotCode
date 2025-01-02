// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static frc.robot.constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public Vision(VisionConsumer consumer, VisionIO... io) {
    this.consumer = consumer;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > aprilTagLayout.getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > aprilTagLayout.getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  // ----

  public Translation2d getNotePositionRobotRelative() {

    double staticOffset = 0; // camera degrees

    double distInch = (1 / (40 - ((30) * visionInputs.iTY / 23)) * 1000); // Convert degrees to inch
    double noteYawAngleDegCorrected =
        -visionInputs.iTX - staticOffset; // account for static offset, reverse to be CCW+
    double radiusInchCorrected =
        distInch / Math.cos(Units.degreesToRadians(noteYawAngleDegCorrected));

    double noteYawAngleDegRaw = -visionInputs.iTX; // account for static offset, reverse to be CCW+
    double radiusInchRaw = distInch / Math.cos(Units.degreesToRadians(noteYawAngleDegRaw));

    Logger.recordOutput("NoteTracking/distInch", distInch);
    Logger.recordOutput("NoteTracking/noteYawAngleDegCorrected", noteYawAngleDegCorrected);
    Logger.recordOutput("NoteTracking/noteYawAngleDegRaw", noteYawAngleDegRaw);
    Logger.recordOutput("NoteTracking/radiusCorrected", radiusInchCorrected);

    // camera relative -> bot relative -> field relative
    Translation2d camRelNoteLocT2dCorrected =
        new Translation2d(
            Units.inchesToMeters(radiusInchCorrected),
            Rotation2d.fromDegrees(noteYawAngleDegCorrected));
    Logger.recordOutput("NoteTracking/camRelNoteLocT2dCorrected", camRelNoteLocT2dCorrected);

    Translation2d camRelNoteLocT2dRaw =
        new Translation2d(
            Units.inchesToMeters(radiusInchRaw), Rotation2d.fromDegrees(noteYawAngleDegRaw));

    // Translation2d roboRelNoteLocT2dRaw =
    //     camRelNoteLocT2dRaw
    //         .rotateBy(Rotation2d.fromDegrees(0))
    //         .plus(new Translation2d(Units.inchesToMeters(12), 0));

    Translation2d roboRelNoteLocT2dCorrected =
        camRelNoteLocT2dCorrected
            .rotateBy(Rotation2d.fromDegrees(0))
            .plus(new Translation2d(Units.inchesToMeters(12), 0));

    return roboRelNoteLocT2dCorrected;
  }

  // rejectyiong old posiutions

  public Pose3d calculateNotePositionFieldRelative() {

    Translation2d roboRelNoteLocT2dCorrected = getNotePositionRobotRelative();

    Pose2d pickedRobotPose =
        posePicker(
            Timer.getFPGATimestamp()
                - (visionInputs.iPIPELINELATENCY / 1000.)
                - (visionInputs.iCAPTURELATENCY / 1000.));
    Translation2d fieldRelNoteLocT2dCorrected =
        roboRelNoteLocT2dCorrected
            .rotateBy(pickedRobotPose.getRotation())
            .plus(pickedRobotPose.getTranslation());

    // Translation2d fieldRelNoteLocT2dRaw =
    //     roboRelNoteLocT2dRaw
    //         .rotateBy(pickedRobotPose.getRotation())
    //         .plus(pickedRobotPose.getTranslation());

    // Logger.recordOutput("NoteTracking/fieldRelNoteLocT2dRaw", fieldRelNoteLocT2dRaw);
    Logger.recordOutput("NoteTracking/fieldRelNoteLocT2dCorrected", fieldRelNoteLocT2dCorrected);
    Logger.recordOutput(
        "distance from center of robot",
        Units.metersToInches(fieldRelNoteLocT2dCorrected.getDistance(getPose().getTranslation())));

    // Translation3d fieldRelNoteLocT3d = new Translation3d(fieldRelNoteLocT2dCorrected.getX(),
    // fieldRelNoteLocT2dCorrected.getY(), 10);
    Pose3d fieldRelNoteLocP3d =
        new Pose3d(new Pose2d(fieldRelNoteLocT2dCorrected, new Rotation2d()));

    Logger.recordOutput("NoteTracking/fieldRelNoteLocP3d", fieldRelNoteLocP3d);
    return fieldRelNoteLocP3d;
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
