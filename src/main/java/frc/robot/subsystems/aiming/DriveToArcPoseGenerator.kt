/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems.aiming

import frc.robot.Constants.AimingConstants
import frc.robot.subsystems.DriveSubsystem
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.driverstation.Alliance
import org.wpilib.driverstation.MatchState.getAlliance
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

object DriveToArcPoseGenerator {
    fun generatePath(curpose: Pose2d = DriveSubsystem.getPose()): Pose2d {
        val allianceRed = (getAlliance().get() == Alliance.RED)
        val hubPos =
            if (allianceRed) {
                Translation2d(AimingConstants.RED_HUB_X, AimingConstants.RED_HUB_Y)
            } else {
                Translation2d(AimingConstants.BLUE_HUB_X, AimingConstants.BLUE_HUB_Y)
            }
        val distance = (curpose.translation.minus(hubPos))
        val scalar = AimingConstants.DISTANCE / sqrt(distance.x.pow(2) + distance.y.pow(2))
        val target = (distance.times(scalar)) + hubPos

        val angleChange =
            when {
                atan2(distance.y, distance.x) > 0 -> atan2(distance.y, distance.x) - Math.PI
                atan2(distance.y, distance.x) < 0 -> atan2(distance.y, distance.x) + Math.PI
                else -> 0.0
            }

        return Pose2d(Translation2d(target.x, target.y), Rotation2d(angleChange))
    }
}
