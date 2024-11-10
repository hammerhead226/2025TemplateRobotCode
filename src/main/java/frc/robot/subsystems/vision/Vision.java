package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.physicalConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private static double multiplier = 1.0;
  private static boolean toggle = false;

  private boolean overridePathplanner = false;

  // private NetworkTable limelightintake =
  //   NetworkTableInstance.getDefault().getTable(physicalConstants.LL_INTAKE);

  private final VisionIO visionIO;
  private final VisionIOInputsAutoLogged visionInputs = new VisionIOInputsAutoLogged();

  public Vision(VisionIO visionIO) {
    this.visionIO = visionIO;
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(visionInputs);
    LimelightHelpers.SetRobotOrientation(
        physicalConstants.LL_ALIGN,
        RobotContainer.drive.poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    Logger.recordOutput(
        "vision something", DriverStation.getAlliance().isPresent() && visionInputs.aTV);
    Logger.recordOutput("isDisabled", DriverStation.isDisabled());

    if (DriverStation.getAlliance().isPresent() && visionInputs.aTV) {
      Logger.recordOutput(
          "tags > 1 or disabled ", visionInputs.tagCount > 1 || DriverStation.isDisabled());

      if (visionInputs.tagCount > 1 || DriverStation.isDisabled()) {
        visionLogic();
      } else {
        // mt2TagFiltering();
        visionLogic();
      }
    }
  }

  public void mt2TagFiltering() {
    boolean doRejectUpdate = false;

    LimelightHelpers.PoseEstimate mt2 =
        new PoseEstimate(
            visionInputs.mt2VisionPose,
            visionInputs.timestampSeconds,
            visionInputs.latency,
            visionInputs.tagCount,
            visionInputs.tagSpan,
            visionInputs.avgTagDist,
            visionInputs.avgTagArea,
            new RawFiducial[] {});
    if (Math.abs(RobotContainer.drive.yawVelocityRadPerSec) > Math.toRadians(360)) {
      doRejectUpdate = true;
    }

    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (mt2.pose.getTranslation().getDistance(new Translation2d(7.9, 4.1)) < 0.4) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      RobotContainer.drive.poseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(0.7, 0.7, Units.degreesToRadians(9999999)));
      RobotContainer.drive.poseEstimator.addVisionMeasurement(
          mt2.pose, mt2.timestampSeconds - (mt2.latency / 1000.));
    }

    Logger.recordOutput("Vision Measurement", mt2.pose);
    Logger.recordOutput("Rejecting Tags", doRejectUpdate);
  }

  public void visionLogic() {
    LimelightHelpers.PoseEstimate limelightMeasurement =
        new PoseEstimate(
            visionInputs.mt1VisionPose,
            visionInputs.timestampSeconds,
            visionInputs.latency,
            visionInputs.tagCount,
            visionInputs.tagSpan,
            visionInputs.avgTagDist,
            visionInputs.avgTagArea,
            new RawFiducial[] {});

    double xMeterStds;
    double yMeterStds;
    double headingDegStds;

    double poseDifference = getVisionPoseDifference(limelightMeasurement.pose);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Logger.recordOutput("avg area", limelightMeasurement.avgTagArea);

    if (limelightMeasurement.tagCount >= 2 && limelightMeasurement.avgTagArea > 0.04) {
      xMeterStds = 0.7;
      yMeterStds = 0.7;
      headingDegStds = 8;
    } else if (limelightMeasurement.tagCount == 1
        && poseDifference < 0.5) { // && poseDifference < 0.5
      xMeterStds = 5;
      yMeterStds = 5;
      headingDegStds = 30;
    } else if (limelightMeasurement.tagCount == 1 && poseDifference < 3) { // && poseDifference < 3
      xMeterStds = 11.43;
      yMeterStds = 11.43;
      headingDegStds = 9999;
    } else return;

    Logger.recordOutput("number of tags", limelightMeasurement.tagCount);

    RobotContainer.drive.poseEstimator.setVisionMeasurementStdDevs(
        VecBuilder.fill(xMeterStds, yMeterStds, Units.degreesToRadians(headingDegStds)));

    Pose2d pose = limelightMeasurement.pose;

    if (isFlipped) {
      pose.getRotation().plus(new Rotation2d(Math.PI));
    }

    Logger.recordOutput("Vision Measurement", limelightMeasurement.pose);

    addVisionMeasurement(
        pose, limelightMeasurement.timestampSeconds - (limelightMeasurement.latency / 1000.));
  }

  public double getVisionPoseDifference(Pose2d visionPose) {
    return RobotContainer.drive.getPose().getTranslation().getDistance(visionPose.getTranslation());
  }

  public boolean acceptableMeasurements(Pose2d visionMeasurement) {
    return Math.abs(visionMeasurement.getX() - RobotContainer.drive.getPose().getX()) < 1
        && Math.abs(visionMeasurement.getY() - RobotContainer.drive.getPose().getY()) < 1;
  }

  public boolean canCorrect(Pose2d visionMeasurement, double timeSinceLastCorrection) {
    if (timeSinceLastCorrection < 5) {
      if (acceptableMeasurements(visionMeasurement)) return true;
    } else {
      return true;
    }
    return false;
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    RobotContainer.drive.poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }
}