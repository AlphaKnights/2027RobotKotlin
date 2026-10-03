/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import frc.robot.subsystems.DriveSubsystem
import org.wpilib.command2.Command
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d

class ResetOdometry : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()
        DriveSubsystem.resetOdometry(Pose2d(0.0, 0.0, Rotation2d(0.0))) // DriveSubsystem.getPose().rotation
    }
}
