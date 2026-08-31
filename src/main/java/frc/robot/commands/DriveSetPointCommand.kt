package frc.robot.commands

import frc.robot.Constants
import frc.robot.subsystems.DriveSubsystem
import org.wpilib.command2.Command
import org.wpilib.command2.WrapperCommand
import org.wpilib.math.controller.PIDController
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.ChassisVelocities
import java.lang.Math.clamp

class DriveSetPointCommand(
    private val X: () -> Double,
    private val Y: () -> Double,
    private val Angle: () -> Double,
) : Command() {
    init {
        addRequirements(DriveSubsystem)
    }

    val rotateController = PIDController(1.0, 0.0, 0.01)
    val driveController = PIDController(0.5, 0.0, 0.0)

    // Do angle optimization (south) and scalable tuning based on max speed
    override fun execute() {
        super.execute()

        // println("X() = ${X()}, Y() = ${Y()}, Angle() = ${Angle()}")

        // take current rotation in radians and make a new PID Controller
        val curpose = DriveSubsystem.getPose()

//        val dir = when {
//            (curpose > Math.PI/2)  -> Math.PI
//            (curpose < -Math.PI/2) -> -Math.PI
//            else -> 0.0
//        }

        // set PID deadzones and angle wrapping
        rotateController.setTolerance(Rotation2d.fromRadians(1.0).radians)
        rotateController.enableContinuousInput(-Math.PI, Math.PI)

        driveController.setTolerance(0.05) // meters

        // calculate rotational speed using PID controller, making sure max speed is respected
        val rotSpeed =
            clamp(
                rotateController.calculate(curpose.rotation.radians, Angle()),
                -1.0,
                1.0,
            ) * Constants.DriveConstants.MAX_ANGULAR_SPEED

        val driveSpeedX =
            clamp(
                driveController.calculate(curpose.translation.x, X()),
                -1.0,
                1.0,
            ) * Constants.DriveConstants.MAX_METERS_PER_SECOND

        val driveSpeedY =
            clamp(
                driveController.calculate(curpose.translation.y, Y()),
                -1.0,
                1.0,
            ) * Constants.DriveConstants.MAX_METERS_PER_SECOND

        DriveSubsystem.drive(
            ChassisVelocities(
                driveSpeedX,
                driveSpeedY,
                rotSpeed,
            ),
            fieldRelative = true,
        )
    }

    override fun isFinished(): Boolean {
        val curpose = DriveSubsystem.getPose()
        return if (
            driveController.atSetpoint() &&
            rotateController.atSetpoint()
//            curpose.x > X() - 0.05 &&
//            curpose.x < X() + 0.05 &&
//            curpose.y > Y() - 0.05 &&
//            curpose.y < Y() + 0.05 &&
//            curpose.rotation.radians > Angle() + 0.1 &&
//            curpose.rotation.radians < Angle() - 0.1
        ) {
            true
        } else {
            false
        }
    }
}
