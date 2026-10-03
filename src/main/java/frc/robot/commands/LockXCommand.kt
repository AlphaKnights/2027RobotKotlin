/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import frc.robot.subsystems.DriveSubsystem
import org.wpilib.command2.Command

class LockXCommand : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()
        DriveSubsystem.setX()
    }
}
