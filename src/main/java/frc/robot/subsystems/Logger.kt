/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import frc.robot.Constants
import frc.robot.subsystems.aiming.AimingCalc
import frc.robot.subsystems.aiming.DriveToArcPoseGenerator
import org.wpilib.command2.Command
import org.wpilib.command2.SubsystemBase
import org.wpilib.driverstation.MatchState
import org.wpilib.math.controller.PIDController
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.networktables.StructArrayPublisher
import org.wpilib.smartdashboard.Field2d
import org.wpilib.system.RobotController
import org.wpilib.telemetry.Telemetry
import org.wpilib.tunable.*

/**
Only use Logger for telemetry, not inputs or choosers!
 */
object Logger : SubsystemBase() {
    private val table = Telemetry.getTable()
    private val field = Field2d()

    override fun periodic() {
        table.log("Field", field)
        field.robotPose = Pose2d(Translation2d.ZERO, Rotation2d.ZERO)
        
        table.log("intake", IntakeSubsystem)
        table.log("shooter", DeliverySubsystem)
        table.log("drive", DriveSubsystem)
        
        table.log("Swerve Module Velocities", DriveSubsystem.getStates(), SwerveModuleVelocity.struct)
        
        table.log("limelight pose", LimelightSubsystem.getPose(), Pose3d.struct)

        table.log(
            "Match Time", MatchState.getMatchTime(),
        )

        table.log("Battery Voltage", RobotController.getBatteryVoltage())

        table.log(
            "CAN Utilization",
            Constants.CANBusIDs.DRIVE_CANBUS.status.BusUtilization.toDouble() * 100
        )
        table.log(
            "Tag Detected",
            LimelightSubsystem.tagPose != null,
        )
        table.log(
            "Aligned to Tag",
            LimelightSubsystem.isAligned(),
        )

        table.log(
            "Shooting Distance",
            AimingCalc.canShoot(DriveSubsystem.getPose()),
        )
        table.log("Field", field)

        table.log(
            "Reset Robot Pose",
            object : Command() {
                override fun execute() {
                    DriveSubsystem.resetPose(Pose2d.ZERO)
                }

                override fun isFinished(): Boolean = true
            },
        )
         field.getObject("targetPose").pose = DriveToArcPoseGenerator.generatePath(field.robotPose)
         field.robotPose = DriveSubsystem.getPose()
    }

    fun PIDController.makeTunable(key: String): PIDController {
        val controller = this
        Tunables.publishValue<PIDController>(key, {controller}, null, PIDController::class.java)
        return this
    }
}
