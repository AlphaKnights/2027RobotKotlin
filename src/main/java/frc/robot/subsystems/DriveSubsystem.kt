/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import frc.robot.Constants
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.PathPlannerConstants
import frc.robot.Robot
import frc.robot.interfaces.SwerveModule
import org.wpilib.command2.SubsystemBase
import org.wpilib.driverstation.Alliance
import org.wpilib.driverstation.DriverStation
import org.wpilib.driverstation.MatchState
import org.wpilib.driverstation.RobotState
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveDriveOdometry
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.smartdashboard.SmartDashboard
import org.wpilib.simulation.DCMotorSim

interface IDriveSubsystem {
    fun getPose(): Pose2d

    fun getStates(): Array<SwerveModuleVelocity>?

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    fun getCurrentSpeeds(): ChassisVelocities

    fun resetOdometry(pose: Pose2d)

    fun resetPose(pose: Pose2d)

    fun shouldFlipPath(): Boolean = (MatchState.getAlliance().get()) == Alliance.BLUE

    fun drive(
        speeds: ChassisVelocities,
        fieldRelative: Boolean,
    )

    fun setX()

    fun zeroHeading()
}

class RealDriveSubsystem :
    SubsystemBase(),
    IDriveSubsystem {
    data class Swerve(
        val fl: SwerveModule,
        val fr: SwerveModule,
        val bl: SwerveModule,
        val br: SwerveModule,
    )

    private var gyro: Pigeon2 = Pigeon2(DriveConstants.PIDGEON2_ID, Constants.CANBusIDs.PIDGEON_CANBUS)
//    private var gyro: AHRS = AHRS(AHRS.NavXComType.kMXP_SPI)

    private val driveTrain: Swerve =
        Swerve(
            SwerveModuleIOTalon(
                DriveConstants.FRONT_LEFT_DRIVING_ID,
                DriveConstants.FRONT_LEFT_TURNING_ID,
                DriveConstants.FRONT_LEFT_CANCODER_ID,
                DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
            ),
            SwerveModuleIOTalon(
                DriveConstants.FRONT_RIGHT_DRIVING_ID,
                DriveConstants.FRONT_RIGHT_TURNING_ID,
                DriveConstants.FRONT_RIGHT_CANCODER_ID,
                DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
            ),
            SwerveModuleIOTalon(
                DriveConstants.REAR_LEFT_DRIVING_ID,
                DriveConstants.REAR_LEFT_TURNING_ID,
                DriveConstants.REAR_LEFT_CANCODER_ID,
                DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
            ),
            SwerveModuleIOTalon(
                DriveConstants.REAR_RIGHT_DRIVING_ID,
                DriveConstants.REAR_RIGHT_TURNING_ID,
                DriveConstants.REAR_RIGHT_CANCODER_ID,
                DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
            ),
        )

    // TODO: Unify Units to Radians
    private var odometry =
        SwerveDriveOdometry(
            DriveConstants.DRIVE_KINEMATICS,
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
            // gyro.rotation3d.x
            arrayOf(
                driveTrain.fl.getPosition(),
                driveTrain.fr.getPosition(),
                driveTrain.bl.getPosition(),
                driveTrain.br.getPosition(),
            ),
        )

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    init {
        // gyro.reset()
//        gyro.enableBoardlevelYawReset(false)
        gyro.reset()

        SmartDashboard.putData("fl", driveTrain.fl)
        SmartDashboard.putData("fr", driveTrain.fr)
        SmartDashboard.putData("bl", driveTrain.bl)
        SmartDashboard.putData("br", driveTrain.br)

//        for (type in Constants.SomeConstants.SwerveType.entries) {
//            swerveTypeChooser.addOption(type.name, type)
//        }
//        SmartDashboard.putData("Swerve Type Chooser", swerveTypeChooser)
        AutoBuilder
            .configure(
                this::getPose,
                this::resetPose,
                this::getCurrentSpeeds,
                { speeds: ChassisVelocities, _: DriveFeedforwards ->
                    drive(speeds, fieldRelative = false)
                },
                PPHolonomicDriveController(
                    PIDConstants(
                        PathPlannerConstants.TRANSLATION_P,
                        PathPlannerConstants.TRANSLATION_I,
                        PathPlannerConstants.TRANSLATION_D,
                    ),
                    PIDConstants(
                        PathPlannerConstants.ROTATION_P,
                        PathPlannerConstants.ROTATION_I,
                        PathPlannerConstants.ROTATION_D,
                    ),
                    1.0,
                ),
                config,
                this::shouldFlipPath,
                this,
            )
    }

//    override fun initSendable(builder: SendableBuilder?) {
//        super.initSendable(builder)
//
//
//        fun SendableBuilder.addMotorStats(vararg motors: Pair<String, TalonFX>) {
//            motors.forEach { m ->
//                val name = m.first
//                val motor = m.second
//                builder?.apply {
//                    addDoubleProperty("$name motor voltage", { motor.motorVoltage.valueAsDouble }, null)
//                    addDoubleProperty("$name stator current", { motor.statorCurrent.valueAsDouble }, null)
//                }
//            }
//        }
//
//        builder?.addMotorStats(drive.fl.)
//    }

    override fun periodic() {
        // This method will be called once per scheduler run
//        if (Robot.isAutonomous()) {

        odometry.update(
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
            arrayOf(
                driveTrain.fl.getPosition(),
                driveTrain.fr.getPosition(),
                driveTrain.bl.getPosition(),
                driveTrain.br.getPosition(),
            ),
        )

        resetOdometry(
            LimelightSubsystem.getPose()?.toPose2d()?.relativeTo(
                Pose2d(PathPlannerConstants.FIELD_SIZE, Rotation2d()),
            ) ?: getPose(),
        ) // limelight synchronization
    }

    override fun getStates(): Array<SwerveModuleVelocity> =
        arrayOf(
            driveTrain.fl.getState(),
            driveTrain.fr.getState(),
            driveTrain.bl.getState(),
            driveTrain.br.getState(),
        )

    override fun getPose(): Pose2d = odometry.pose

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    override fun getCurrentSpeeds(): ChassisVelocities =
        DriveConstants.DRIVE_KINEMATICS.toChassisVelocities(
            driveTrain.fl.getState(),
            driveTrain.fr.getState(),
            driveTrain.bl.getState(),
            driveTrain.br.getState(),
        )

    override fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
            // gyro.getRotation2d(),
            arrayOf(
                driveTrain.fl.getPosition(),
                driveTrain.fr.getPosition(),
                driveTrain.bl.getPosition(),
                driveTrain.br.getPosition(),
            ),
            pose,
        )
    }

    override fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
    }

    override fun drive(
        speeds: ChassisVelocities,
        fieldRelative: Boolean,
    ) {
        var SwerveModuleVelocities =  DriveConstants.DRIVE_KINEMATICS.toSwerveModuleVelocities(speeds)

        if (fieldRelative) {
            SwerveModuleVelocities = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleVelocities(speeds.toFieldRelative(Rotation2d.fromDegrees(gyro.yaw.valueAsDouble)))
        }

        driveTrain.fl.setDesiredState(SwerveModuleVelocities[0])
        driveTrain.fr.setDesiredState(SwerveModuleVelocities[1])
        driveTrain.bl.setDesiredState(SwerveModuleVelocities[2])
        driveTrain.br.setDesiredState(SwerveModuleVelocities[3])
    }

    override fun setX() {
        driveTrain.fl.setDesiredState(
            SwerveModuleVelocity(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
        driveTrain.fr.setDesiredState(
            SwerveModuleVelocity(0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        driveTrain.bl.setDesiredState(
            SwerveModuleVelocity(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        driveTrain.br.setDesiredState(
            SwerveModuleVelocity(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
    }

    override fun zeroHeading() {
        gyro.reset()
    }
}

//class PhysicsSimDriveSubsystem :
//    SubsystemBase(),
//    IDriveSubsystem {
//    data class Swerve(
//        val fl: SwerveModule,
//        val fr: SwerveModule,
//        val bl: SwerveModule,
//        val br: SwerveModule,
//    )
//
//    private var gyro = GyroSim.gyro
//
//    private var m_speeds: ChassisVelocities = ChassisVelocities()
//
//    private val driveTrain: Swerve = Swerve(SwerveModuleSim(), SwerveModuleSim(), SwerveModuleSim(), SwerveModuleSim())
//
//    private var odometry =
//        SwerveDriveOdometry(
//            DriveConstants.DRIVE_KINEMATICS,
//            Rotation2d(gyro.angularPosition),
//            arrayOf(
//                driveTrain.fl.getPosition(),
//                driveTrain.fr.getPosition(),
//                driveTrain.bl.getPosition(),
//                driveTrain.br.getPosition(),
//            ),
//        )
//
//    private val config: RobotConfig = RobotConfig.fromGUISettings()
//
//    init {
//        // gyro.reset()
////        gyro.enableBoardlevelYawReset(false)
//
//        AutoBuilder.configure(
//            this::getPose,
//            this::resetPose,
//            this::getCurrentSpeeds,
//            { speeds: ChassisVelocities, _: DriveFeedforwards ->
//                drive(speeds, fieldRelative = false)
//            },
//            PPHolonomicDriveController(
//                PIDConstants(
//                    PathPlannerConstants.TRANSLATION_P,
//                    PathPlannerConstants.TRANSLATION_I,
//                    PathPlannerConstants.TRANSLATION_D,
//                ),
//                PIDConstants(
//                    PathPlannerConstants.ROTATION_P,
//                    PathPlannerConstants.ROTATION_I,
//                    PathPlannerConstants.ROTATION_D,
//                ),
//                1.0,
//            ),
//            config,
//            this::shouldFlipPath,
//            this,
//        )
//    }
//
//    override fun periodic() {
//        // This method will be called once per scheduler run
////        if (Robot.isAutonomous()) {
//
//        GyroSim.update(m_speeds)
//
//        odometry.update(
//            Rotation2d(gyro.angularPosition),
//            arrayOf(
//                driveTrain.fl.getPosition(),
//                driveTrain.fr.getPosition(),
//                driveTrain.bl.getPosition(),
//                driveTrain.br.getPosition(),
//            ),
//        )
//
//        resetOdometry(
//            LimelightSubsystem.getPose()?.toPose2d()?.relativeTo(
//                Pose2d(PathPlannerConstants.FIELD_SIZE, Rotation2d()),
//            ) ?: getPose(),
//        ) // limelight synchronization
//    }
//
//    override fun getStates(): Array<SwerveModuleVelocity> =
//        arrayOf(
//            driveTrain.fl.getState(),
//            driveTrain.fr.getState(),
//            driveTrain.bl.getState(),
//            driveTrain.br.getState(),
//        )
//
//    override fun getPose(): Pose2d = odometry.pose
//
//    // IDE bug, the detected and actual signatures are different
//    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
//    override fun getCurrentSpeeds(): ChassisVelocities =
//        DriveConstants.DRIVE_KINEMATICS.toChassisVelocities(
//            driveTrain.fl.getState(),
//            driveTrain.fr.getState(),
//            driveTrain.bl.getState(),
//            driveTrain.br.getState(),
//        )
//
//    override fun resetOdometry(pose: Pose2d) {
//        odometry.resetPosition(
//            Rotation2d(gyro.angularPosition),
//            // gyro.getRotation2d(),
//            arrayOf(
//                driveTrain.fl.getPosition(),
//                driveTrain.fr.getPosition(),
//                driveTrain.bl.getPosition(),
//                driveTrain.br.getPosition(),
//            ),
//            pose,
//        )
//    }
//
//    override fun resetPose(pose: Pose2d) {
//        resetOdometry(pose)
//    }
//
//    override fun drive(
//        speeds: ChassisVelocities,
//        fieldRelative: Boolean,
//    ) {
//        var SwerveModuleVelocities = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleVelocities(speeds)
//        m_speeds = speeds
//        if (fieldRelative) {
//            SwerveModuleVelocities = speeds.toFieldRelative(Rotation2d.fromDegrees(gyro.angularPosition))
//        }
//
//        driveTrain.fl.setDesiredState(SwerveModuleVelocities[0])
//        driveTrain.fr.setDesiredState(SwerveModuleVelocities[1])
//        driveTrain.bl.setDesiredState(SwerveModuleVelocities[2])
//        driveTrain.br.setDesiredState(SwerveModuleVelocities[3])
//    }
//
//    override fun setX() {
//        driveTrain.fl.setDesiredState(
//            SwerveModuleVelocity(
//                0.0,
//                Rotation2d.fromDegrees(45.0),
//            ),
//        )
//        driveTrain.fr.setDesiredState(
//            SwerveModuleVelocity(
//                0.0,
//                Rotation2d.fromDegrees(-45.0),
//            ),
//        )
//        driveTrain.bl.setDesiredState(
//            SwerveModuleVelocity(
//                0.0,
//                Rotation2d.fromDegrees(-45.0),
//            ),
//        )
//        driveTrain.br.setDesiredState(
//            SwerveModuleVelocity(
//                0.0,
//                Rotation2d.fromDegrees(45.0),
//            ),
//        )
//    }
//
//    override fun zeroHeading() {
//        gyro.setState(0.0, 0.0)
//    }
//
//    private object GyroSim {
//        val gyro = DCMotorSim(LinearSystemId.createDCMotorSystem(1.0, 2.0), DCMotor.getKrakenX60Foc(1))
//
//        // TODO: Tune kV and kA
//
//        fun update(speeds: ChassisVelocities) {
//            gyro.inputVoltage = speeds.toTwist2d(0.20).dtheta
//            gyro.update(0.20)
//        }
//    }
//}

class SimpleSimDriveSubsystem :
    SubsystemBase(),
    IDriveSubsystem {
    var m_pose: Pose2d = Pose2d.kZero
    var m_speeds: ChassisVelocities = ChassisVelocities()

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    init {
        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getCurrentSpeeds,
            { speeds: ChassisVelocities, _: DriveFeedforwards ->
                drive(speeds, fieldRelative = false)
            },
            PPHolonomicDriveController(
                PIDConstants(
                    PathPlannerConstants.TRANSLATION_P,
                    PathPlannerConstants.TRANSLATION_I,
                    PathPlannerConstants.TRANSLATION_D,
                ),
                PIDConstants(
                    PathPlannerConstants.ROTATION_P,
                    PathPlannerConstants.ROTATION_I,
                    PathPlannerConstants.ROTATION_D,
                ),
                1.0,
            ),
            config,
            this::shouldFlipPath,
            this,
        )
    }

    override fun periodic() {
        // println(m_pose.toString())
        // SmartDashboard.putNumber("dTheta", m_speeds.toTwist2d(0.20).dtheta)
    }

    override fun getPose(): Pose2d = m_pose

    override fun getStates(): Array<SwerveModuleVelocity>? = null

    override fun getCurrentSpeeds(): ChassisVelocities = m_speeds

    override fun resetOdometry(pose: Pose2d) {
        m_pose = pose
    }

    override fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
    }

    override fun drive(
        speeds: ChassisVelocities,
        fieldRelative: Boolean,
    ) {
        m_speeds = speeds
        m_pose = m_pose.plus(speeds.toTwist2d(0.20).exp())
    }

    override fun setX() {
        // Does nothing in the sim
    }

    override fun zeroHeading() {
        // Does nothing in the sim
    }
}

private val delegate: IDriveSubsystem by lazy {
    if (Robot.isReallyReal) {
        RealDriveSubsystem()
    } else {
        SimpleSimDriveSubsystem()
    }
}

object DriveSubsystem :
    SubsystemBase(),
    IDriveSubsystem by delegate
