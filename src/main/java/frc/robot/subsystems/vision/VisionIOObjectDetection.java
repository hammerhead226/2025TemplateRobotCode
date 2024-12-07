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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

/** IO implementation for real Limelight hardware. */
public class VisionIOObjectDetection implements VisionIO {
    // Second class for the limelight pointing DOWN so it doesn't constantly check for apriltags to update orientation
    private final DoubleArrayPublisher orientationPublisher;
    private final DoubleSubscriber latencySubscriber;
    private final DoubleSubscriber txSubscriber;
    private final DoubleSubscriber tySubscriber;
    private final DoubleArraySubscriber megatag1Subscriber;
    private final DoubleArraySubscriber megatag2Subscriber;

    public VisionIOObjectDetection(String name) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        megatag2Subscriber =
            table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
        }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = (RobotController.getFPGATime() - latencySubscriber.getLastChange()) < 250;

        // Update target observation
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));
        }
        
}