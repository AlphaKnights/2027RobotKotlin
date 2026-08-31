/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands.autoalign

import frc.robot.Constants
import frc.robot.subsystems.AutoAlignCalc
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.LimelightSubsystem
import org.wpilib.command2.Command
import org.wpilib.math.geometry.Pose3d

class AutoAlignManualCommand(
    private val direction: Constants.AlignDirection,
) : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        val curPose: Pose3d =
            LimelightSubsystem.tagPose ?: run {
                DriveSubsystem.setX()
                return
            }

        val speeds =
            AutoAlignCalc.getAlignSpeeds(
                goalX =
                    if (direction ==
                        Constants.AlignDirection.LEFT
                    ) {
                        Constants.AlignConstants.LEFT_X_OFFSET
                    } else {
                        Constants.AlignConstants.RIGHT_X_OFFSET
                    },
                goalZ =
                    if (direction ==
                        Constants.AlignDirection.LEFT
                    ) {
                        Constants.AlignConstants.LEFT_Z_OFFSET
                    } else {
                        Constants.AlignConstants.RIGHT_Z_OFFSET
                    },
                curPose = curPose,
            )

        if (
            speeds.vx == 0.0 &&
            speeds.vy == 0.0 &&
            speeds.omega == 0.0
        ) {
            DriveSubsystem.setX()
            return
        }

        DriveSubsystem.drive(
            speeds,
            fieldRelative = false,
        )
    }

    override fun isFinished(): Boolean = false
}
