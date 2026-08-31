package frc.robot.commands
import frc.robot.Constants
import frc.robot.subsystems.StorageSubsystem
import org.wpilib.command2.Command
import org.wpilib.system.Timer

class SuperStorageCommand(
    reversed: Boolean,
) : Command() {
    val reverse = reversed

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(StorageSubsystem)
    }

    override fun execute() {
        if (reverse) {
            StorageSubsystem.roll(-Constants.RollerConstants.ALT_ROLLER_SPEED)
        } else {
            StorageSubsystem.roll(Constants.RollerConstants.ALT_ROLLER_SPEED)
        }
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        StorageSubsystem.rollerstop()
        super.end(interrupted)
    }
}
