/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import frc.robot.Constants
import frc.robot.subsystems.aiming.AimingCalc
import frc.robot.subsystems.aiming.DriveToArcPoseGenerator
import org.wpilib.command2.Command
import org.wpilib.command2.SubsystemBase
import org.wpilib.driverstation.DriverStation
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
import org.wpilib.smartdashboard.SmartDashboard
import org.wpilib.system.RobotController

/**
Only use Logger for telemetry, not inputs or choosers!
 */
object Telemetry : SubsystemBase() {
    private val table = NetworkTableInstance.getDefault()
    private val field = Field2d()

    private val swervePublisher: StructArrayPublisher<SwerveModuleVelocity?> =
        table
            .getStructArrayTopic("MyStates", SwerveModuleVelocity.struct)
            .publish()
    private val limeLightPublisher: StructArrayPublisher<Pose3d?> =
        table
            .getStructArrayTopic("limeLight pose", Pose3d.struct)
            .publish()

    fun initTelemetry() {
        SmartDashboard.putData("Field", field)
        field.robotPose = Pose2d(Translation2d.kZero, Rotation2d.kZero)
        SmartDashboard.putData("intake", IntakeSubsystem)
        SmartDashboard.putData("shooter", DeliverySubsystem)
        SmartDashboard.putData("drive", DriveSubsystem)

        SmartDashboard.putNumber(
            "Match Time", MatchState.getMatchTime(),
        )

        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage())

        SmartDashboard.putNumber(
            "CAN Utilization",
            Constants.CANBusIDs.DRIVE_CANBUS.status.BusUtilization.toDouble() * 100
        )
        SmartDashboard.putBoolean(
            "Tag Detected",
            LimelightSubsystem.tagPose != null,
        )
        SmartDashboard.putBoolean(
            "Aligned to Tag",
            LimelightSubsystem.isAligned(),
        )
//        SmartDashboard.putNumber(
//            "limelight x",
//            LimelightSubsystem.tagPose?.x ?: -1.0,
//        )
//        SmartDashboard.putNumber(
//            "limelight z",
//            LimelightSubsystem.tagPose?.z ?: -1.0,
//        )
//        SmartDashboard.putNumber(
//            "limelight yaw",
//            LimelightSubsystem.tagPose?.rotation?.y ?: -1.0,
//        )
        limeLightPublisher.set(arrayOf(LimelightSubsystem.tagPose))

        SmartDashboard.putNumber(
            "Shooting Distance",
            AimingCalc.canShoot(DriveSubsystem.getPose()),
        )
        SmartDashboard.putData("Field", field)

        swervePublisher.set(DriveSubsystem.getStates())

        SmartDashboard.putData(
            "Reset Robot Pose",
            object : Command() {
                override fun execute() {
                    DriveSubsystem.resetPose(Pose2d.kZero)
                }

                override fun isFinished(): Boolean = true
            },
        )
    }

    override fun periodic() {
        // field.robotPose = DriveSubsystem.getPose()
        field.getObject("targetPose").pose = DriveToArcPoseGenerator.generatePath(field.robotPose)
    }

    fun PIDController.makeTunable(key: String): PIDController {
        val table = NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable(key)
        // ex: SmartDashboard/DrivePID

        // set this PID controller to the NT persistent values
        this.setPID(
            table.getEntry("p").getDouble(0.0),
            table.getEntry("i").getDouble(0.0),
            table.getEntry("d").getDouble(0.0),
        )

        // if this is a new controller, make the values persistent (ignored otherwise)
        for (name in listOf("p", "i", "d")) {
            val entry = table.getEntry(name)
            if (entry.exists()) {
                entry.setPersistent()
            }
        }
        // Use the Sendable api to make a PIDController appear on the dashboard with our constants
        SmartDashboard.putData(key, this)

        // return the PIDController as tuned from Glass (or elsewhere)
        return SmartDashboard.getData(key) as PIDController
    }
}
