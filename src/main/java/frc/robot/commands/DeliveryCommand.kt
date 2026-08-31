package frc.robot.commands

import frc.robot.Constants
import frc.robot.subsystems.DeliverySubsystem
import org.wpilib.command2.Command
import org.wpilib.system.Timer

class DeliveryCommand(
    deliverySpeed: Double,
) : Command() {
    val speed = deliverySpeed

    init {
        // each subsystem used by the command must be passed into the addRequirements() method
        addRequirements(DeliverySubsystem)
    }

    override fun execute() {
        DeliverySubsystem.forward(speed)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        DeliverySubsystem.stop()
    }
}
