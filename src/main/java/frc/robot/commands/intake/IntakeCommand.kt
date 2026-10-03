/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands.intake

import frc.robot.Constants
import frc.robot.subsystems.IntakeSubsystem
import org.wpilib.command2.Command

class IntakeCommand(
    private val isReversed: Boolean,
) : Command() {
    init {
        addRequirements(IntakeSubsystem)
    }

    override fun execute() {
        IntakeSubsystem.limitOutput()
        val intakeSpeed =
            if (isReversed) {
                -Constants.IntakeConstants.INTAKE_SPEED - 0.2
            } else {
                Constants.IntakeConstants.INTAKE_SPEED
            }

        IntakeSubsystem.runIntake(intakeSpeed)
    }

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        IntakeSubsystem.stopIntake()
    }
}
