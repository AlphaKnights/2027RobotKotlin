/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import frc.robot.Constants
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.kinematics.ChassisVelocities
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

object AutoAlignCalc {
    @Suppress("LongMethod")
    fun getAlignSpeeds(
        goalX: Double,
        goalZ: Double,
        curPose: Pose3d,
    ): ChassisVelocities {
        // Units are in meters and radians
        var x: Double = -curPose.x // Inverted to match the robot's coordinate system
        var z: Double = curPose.z
        val yaw: Double = curPose.rotation.y

        var xNormalized = 1.0
        var zNormalized = 1.0

        // Calculate the offsets for the Apriltag
        x -= (sin(yaw) * goalZ) + (cos(yaw) * goalX)
        z += -(cos(yaw) * goalZ) - (sin(yaw) * goalX)

        // Calculate the distance to the target
        var dist = sqrt(x.pow(2) + z.pow(2))
        dist =
            if (dist >
                Constants.AlignConstants.FINE_ALIGN_DEADZONE
            ) {
                1.0
            } else {
                dist /
                    Constants.AlignConstants.FINE_ALIGN_DEADZONE
            }

        // Normalize the x and z values
        if (abs(x) > abs(z)) {
            zNormalized = abs(z) / abs(x)
            xNormalized = 1.0
        }
        if (abs(x) < abs(z)) {
            zNormalized = 1.0
            xNormalized = abs(x) / abs(z)
        }

        if (z < 0) zNormalized *= -1.0

        if (x > 0) xNormalized *= -1.0

        xNormalized =
            if (abs(x) <
                Constants.AlignConstants.ALIGN_DEADZONE
            ) {
                0.0
            } else {
                xNormalized
            }
        zNormalized =
            if (abs(z) <
                Constants.AlignConstants.ALIGN_DEADZONE
            ) {
                0.0
            } else {
                zNormalized
            }

        // Calculate the rotation speed
        val rotSign: Double =
            if (abs(yaw) <
                Constants.AlignConstants.ALIGN_ROT_DEADZONE
            ) {
                0.0
            } else {
                yaw /
                    abs(yaw)
            }

        var angularDistance =
            if (abs(yaw) >
                Constants.AlignConstants.FINE_ALIGN_ROT_DEADZONE
            ) {
                1.0
            } else {
                max(
                    Constants.AlignConstants.MAX_ANGULAR_SPEED,
                    abs(yaw) /
                        Constants.AlignConstants.FINE_ALIGN_ROT_DEADZONE,
                )
            }

        // Finalize drivetrain controls
        dist = sqrt(dist)
        angularDistance = sqrt(angularDistance)

        return ChassisVelocities(
            zNormalized * dist *
                Constants.AlignConstants.MAX_SPEED,
            -xNormalized * dist *
                Constants.AlignConstants.MAX_SPEED,
            -rotSign * angularDistance *
                Constants.AlignConstants.MAX_ANGULAR_SPEED,
        )
    }
}
