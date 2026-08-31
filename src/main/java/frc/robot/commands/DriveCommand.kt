/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.aiming.AimingCalc
import org.wpilib.command2.Command
import org.wpilib.math.kinematics.ChassisVelocities

class DriveCommand(
    private val x: () -> Double,
    private val y: () -> Double,
    private val rot: () -> Double,
    private val autoAngle: () -> Boolean,
    private val fieldRel: Boolean = true,
) : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    override fun execute() {
        super.execute()

        if (!autoAngle()) {
            DriveSubsystem.drive(
                ChassisVelocities(
                    x() *
                        Constants.DriveConstants.MAX_METERS_PER_SECOND,
                    y() *
                        Constants.DriveConstants.MAX_METERS_PER_SECOND,
                    rot() *
                        Constants.DriveConstants.MAX_ANGULAR_SPEED,
                ),
                fieldRelative = fieldRel,
            )
        } else {
            DriveSubsystem.drive(
                ChassisVelocities(
                    x() *
                        Constants.DriveConstants.MAX_METERS_PER_SECOND,
                    y() *
                        Constants.DriveConstants.MAX_METERS_PER_SECOND,
                    AimingCalc.getAimingAngleChange(
                        DriveSubsystem.getPose(),
                        DriveSubsystem.getCurrentSpeeds().vx,
                        DriveSubsystem.getCurrentSpeeds().vy,
                    ),
                ),
                fieldRelative = fieldRel,
            )
        }
//        } else {
//            DriveSubsystem.drive(
//                AimingCalc.getArcDriveSpeeds(
//                    DriveSubsystem.getPose(),
//                    x(), // controller X → tangential movement along arc
//                    DriveSubsystem.getCurrentSpeeds(), // current velocity for rotation feedforward
//                    // y() intentionally omitted — arc system controls radial position
//                ),
//                fieldRelative = true,
//            )
//        }
    }

    override fun end(interrupted: Boolean) {
        DriveSubsystem.drive(
            ChassisVelocities(
                0.0,
                0.0,
                0.0,
            ),
            fieldRelative = false,
        )
    }
}
