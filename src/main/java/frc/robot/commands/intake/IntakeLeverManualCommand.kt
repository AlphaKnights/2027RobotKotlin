package frc.robot.commands.intake

import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem
import org.wpilib.command2.Command

class IntakeLeverManualCommand(
    private val direction: Constants.IntakeDirection,
) : Command() {
    init {
        addRequirements(IntakeSubsystem)
    }

    override fun execute() {
        val speed =
            when (direction) {
                Constants.IntakeDirection.IN -> {
                    Constants.IntakeConstants.LEVER_SPEED
                }

                Constants.IntakeDirection.OUT -> {
                    -Constants.IntakeConstants.LEVER_SPEED
                }
            }
        IntakeSubsystem.moveLever(speed)
    }

    override fun isFinished(): Boolean = IntakeSubsystem.limitSwitchPressed()

    override fun end(interrupted: Boolean) {
        IntakeSubsystem.stopIntakeLever()
    }
}
