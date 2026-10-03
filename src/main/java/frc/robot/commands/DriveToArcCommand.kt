package frc.robot.commands

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem
import org.wpilib.command2.Command
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import java.util.NoSuchElementException
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sqrt

class DriveToArcCommand : Command() {
    private val pose = generatePose()

    fun generatePose(): Pose2d {
        val curpose = DriveSubsystem.getPose()

        val allianceRed = Constants.MatchConstants.allianceRed

        val hubPos =
            if (allianceRed) {
                Translation2d(Constants.AimingConstants.RED_HUB_X, Constants.AimingConstants.RED_HUB_Y)
            } else {
                Translation2d(Constants.AimingConstants.BLUE_HUB_X, Constants.AimingConstants.BLUE_HUB_Y)
            }
        val distance = (curpose.translation.minus(hubPos))
        val scalar = Constants.AimingConstants.DISTANCE / sqrt(distance.x.pow(2) + distance.y.pow(2))
        val target = (distance.times(scalar)) + hubPos

        val angleChange =
            when {
                atan2(distance.y, distance.x) > 0 -> atan2(distance.y, distance.x) - Math.PI
                atan2(distance.y, distance.x) < 0 -> atan2(distance.y, distance.x) + Math.PI
                else -> 0.0
            }

        return Pose2d(Translation2d(target.x, target.y), Rotation2d(angleChange))
    }

    override fun execute() {
        DriveSetPointCommand(pose.x, pose.y, pose.rotation.radians)
    }
}
