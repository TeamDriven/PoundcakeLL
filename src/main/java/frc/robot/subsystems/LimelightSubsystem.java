// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;
import static frc.robot.RobotContainer.m_gyro;
import static frc.robot.RobotContainer.m_poseEstimator;
import limelight.*;

public class LimelightSubsystem extends SubsystemBase {

    /** Creates a new LimelightSubsystem. */
    public LimelightSubsystem() {
    }

    @Override
    public void periodic() {
        Limelight limelight = new Limelight("test");


        // This method will be called once per scheduler run
        // Basic targeting data
        double tx = LimelightHelpers.getTX(""); // Horizontal offset from crosshair to target in degrees
        double ty = LimelightHelpers.getTY(""); // Vertical offset from crosshair to target in degrees
        double ta = LimelightHelpers.getTA(""); // Target area (0% to 100% of image)
        boolean hasTarget = LimelightHelpers.getTV(""); // Do you have a valid target?

        double txnc = LimelightHelpers.getTXNC(""); // Horizontal offset from principal pixel/point to target in degrees
        double tync = LimelightHelpers.getTYNC(""); // Vertical offset from principal pixel/point to target in degrees

        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex("", 0);

        // Let the current pipeline control the LEDs
        LimelightHelpers.setLEDMode_PipelineControl("");

        // Force LEDs on/off/blink
        LimelightHelpers.setLEDMode_ForceOff("");

        // First, tell Limelight your robot's current orientation
        double robotYaw = m_gyro.getYaw().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation("", robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);

        // Get the pose estimate
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("");

        // Add it to your pose estimator
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        m_poseEstimator.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds);

        // Set a custom crop window for improved performance (-1 to 1 for each value)
        LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

        // Change the camera pose relative to robot center (x forward, y left, z up,
        // degrees)
        LimelightHelpers.setCameraPose_RobotSpace("",
                0.5, // Forward offset (meters)
                0.0, // Side offset (meters)
                0.5, // Height offset (meters)
                0.0, // Roll (degrees)
                30.0, // Pitch (degrees)
                0.0 // Yaw (degrees)
        );

        // Set AprilTag offset tracking point (meters)
        LimelightHelpers.setFiducial3DOffset("",
                0.0, // Forward offset
                0.0, // Side offset
                0.5 // Height offset
        );

        // Configure AprilTag detection
        LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] { 1, 2, 3, 4 }); // Only track these tag IDs
        LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half resolution for improved framerate
                                                                   // and reduced range

    }
}
