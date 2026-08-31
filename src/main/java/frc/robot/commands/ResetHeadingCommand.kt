/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import frc.robot.subsystems.DriveSubsystem
import org.wpilib.command2.Command

class ResetHeadingCommand : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        DriveSubsystem.zeroHeading()
    }

    override fun isFinished(): Boolean = false
}
