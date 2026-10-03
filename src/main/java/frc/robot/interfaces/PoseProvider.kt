/*
 * (C) 2025 Galvaknights
 */
package frc.robot.interfaces

import org.wpilib.math.geometry.Pose3d

interface PoseProvider {
    val tagPose: Pose3d?
}
