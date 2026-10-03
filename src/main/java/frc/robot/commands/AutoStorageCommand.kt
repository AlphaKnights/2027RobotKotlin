/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands
import frc.robot.Constants
import frc.robot.subsystems.StorageSubsystem
import org.wpilib.command2.Command
import org.wpilib.system.Timer

class AutoStorageCommand(
    reversed: Boolean,
) : Command() {
    val reverse = reversed
    val timer: Timer = Timer()

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(StorageSubsystem)
    }

    override fun initialize() {
        super.initialize()
        timer.start()
    }

    override fun execute() {
        if (reverse) {
            StorageSubsystem.roll(-Constants.RollerConstants.ROLLER_SPEED)
        } else {
            StorageSubsystem.roll(Constants.RollerConstants.ROLLER_SPEED)
        }
    }

    override fun isFinished(): Boolean = if (timer.get() > 3) true else false

    override fun end(interrupted: Boolean) {
        StorageSubsystem.rollerstop()
        timer.stop()
        timer.reset()
        super.end(interrupted)
    }
}
