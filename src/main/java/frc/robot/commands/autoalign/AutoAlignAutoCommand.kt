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
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.system.Timer

class AutoAlignAutoCommand(
    private val direction: Constants.AlignDirection,
) : Command() {
    private val timer = Timer()
    private var latestPose: Pose3d? = null

    init {
        addRequirements(DriveSubsystem)
    }

    override fun initialize() {
        super.initialize()
        timer.start()
        timer.reset()
    }

    override fun execute() {
        val curPose: Pose3d? = LimelightSubsystem.tagPose
        curPose ?: return

        timer.reset()

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

        latestPose = curPose
    }

    override fun isFinished(): Boolean {
        val curPose: Pose3d? = latestPose

        if (curPose == null) {
            val time = timer.get()
            if (time >
                Constants.AlignConstants.ALIGN_TIMEOUT
            ) {
                return true
            }

            if (time >
                Constants.AlignConstants.ALIGN_SEEK_TIMEOUT
            ) {
                DriveSubsystem.drive(
                    speeds =
                        ChassisVelocities(
                            0.0,
                            0.0,
                            Constants.AlignConstants.MAX_SPEED,
                        ),
                    fieldRelative = false,
                )
            }
            return false
        }
        return LimelightSubsystem.isAligned()
    }
}
