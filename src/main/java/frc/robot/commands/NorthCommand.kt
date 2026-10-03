/*
 * (C) 2025 Galvaknights
 */
package frc.robot.commands

import frc.robot.Constants.DriveConstants
import frc.robot.subsystems.DriveSubsystem
import org.wpilib.command2.Command
import java.lang.Math.clamp
import org.wpilib.math.controller.PIDController
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.ChassisVelocities
import kotlin.math.min

class NorthCommand(
    private val x: () -> Double,
    private val y: () -> Double,
) : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

// Do angle optimization (south) and scalable tuning based on max speed
    override fun execute() {
        super.execute()
        // take current rotation in radians and make a new PID Controller
        val curpose = DriveSubsystem.getPose().rotation.radians
        val rotateController =
            PIDController(
                DriveConstants.ROTATE_CONTROLLER_P,
                DriveConstants.ROTATE_CONTROLLER_I,
                DriveConstants.ROTATE_CONTROLLER_D,
            )
        val dir =
            when {
                (curpose > Math.PI / 2) -> Math.PI
                (curpose < -Math.PI / 2) -> -Math.PI
                else -> 0.0
            }

        // set PID deadzones and angle wrapping
        rotateController.setTolerance(DriveConstants.ROTATE_SETPOINT_TOLERANCE)
        rotateController.enableContinuousInput(-Math.PI, Math.PI)
        // calculate rotational speed using PID controller, making sure max speed is respected
        val rotSpeed =
            clamp(
                rotateController.calculate(curpose, dir),
                -1.0,
                1.0,
            ) * DriveConstants.MAX_ANGULAR_SPEED

        DriveSubsystem.drive(
            ChassisVelocities(
                x() *
                    DriveConstants.MAX_METERS_PER_SECOND,
                y() *
                    DriveConstants.MAX_METERS_PER_SECOND,
                rotSpeed,
            ),
            fieldRelative = true,
        )
    }

    override fun isFinished(): Boolean = false
}
