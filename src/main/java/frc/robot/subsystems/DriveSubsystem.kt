package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.Pigeon2
import com.ctre.phoenix6.hardware.TalonFX
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.util.DriveFeedforwards
import frc.robot.Constants
import frc.robot.XboxController
import org.wpilib.command2.SubsystemBase
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.math.kinematics.SwerveDriveOdometry
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.networktables.DoublePublisher
import org.wpilib.networktables.NetworkTableInstance
import org.wpilib.networktables.StructArrayPublisher

object DriveSubsystem : SubsystemBase() {
    private val xBoxController = XboxController()

    private var frontLeft: TalonSwerveModule =
        TalonSwerveModule(
            Constants.DriveConstants.FRONT_LEFT_DRIVING_ID,
            Constants.DriveConstants.FRONT_LEFT_TURNING_ID,
            Constants.DriveConstants.FRONT_LEFT_CANCODER_ID,
            Constants.DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET,
        )
    private var frontRight: TalonSwerveModule =
        TalonSwerveModule(
            Constants.DriveConstants.FRONT_RIGHT_DRIVING_ID,
            Constants.DriveConstants.FRONT_RIGHT_TURNING_ID,
            Constants.DriveConstants.FRONT_RIGHT_CANCODER_ID,
            Constants.DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET,
        )
    private var rearLeft: TalonSwerveModule =
        TalonSwerveModule(
            Constants.DriveConstants.REAR_LEFT_DRIVING_ID,
            Constants.DriveConstants.REAR_LEFT_TURNING_ID,
            Constants.DriveConstants.REAR_LEFT_CANCODER_ID,
            Constants.DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET,
        )
    private var rearRight: TalonSwerveModule =
        TalonSwerveModule(
            Constants.DriveConstants.REAR_RIGHT_DRIVING_ID,
            Constants.DriveConstants.REAR_RIGHT_TURNING_ID,
            Constants.DriveConstants.REAR_RIGHT_CANCODER_ID,
            Constants.DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET,
        )
    private var FrontRightEncoder = CANcoder(Constants.DriveConstants.FRONT_RIGHT_CANCODER_ID, Constants.CANBusIDs.DRIVE_CANBUS)
    private var fL: TalonFX = TalonFX(Constants.DriveConstants.FRONT_LEFT_DRIVING_ID, Constants.CANBusIDs.DRIVE_CANBUS)

    private var gyro: Pigeon2 = Pigeon2(20, Constants.CANBusIDs.PIDGEON_CANBUS)
//    private var gyro: AHRS = AHRS(AHRS.NavXComType.kMXP_SPI)

    private var odometry: SwerveDriveOdometry

    private val config: RobotConfig = RobotConfig.fromGUISettings()

    var swervePublisher: StructArrayPublisher<SwerveModuleVelocity> =
        NetworkTableInstance
            .getDefault()
            .getStructArrayTopic("MyStates", SwerveModuleVelocity.struct)
            .publish()
    var currentPublisher: DoublePublisher =
        NetworkTableInstance
            .getDefault()
            .getDoubleTopic("Drive/StatorCurrent")
            .publish()
    var counter = 0

    init {
        // gyro.reset()
//        gyro.enableBoardlevelYawReset(false)
        gyro.reset()

        odometry =
            SwerveDriveOdometry(
                Constants.DriveConstants.DRIVE_KINEMATICS,
                Rotation2d.fromDegrees(gyro.yaw.valueAsDouble), // gyro.rotation3d.x
                arrayOf(
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    rearLeft.getPosition(),
                    rearRight.getPosition(),
                ),
            )

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getCurrentSpeeds,
            { speeds: ChassisVelocities, _: DriveFeedforwards ->
                drive(speeds, fieldRelative = false)
            },
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.01),
                PIDConstants(2.0, 0.0, 0.01),
                1.0,
            ),
            config,
            this::shouldFlipPath,
            this,
        )
    }

    override fun periodic() {
        counter++
        // This method will be called once per scheduler run
//        if (Robot.isAutonomous()) {

        odometry.update(
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble),
            arrayOf(
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition(),
            ),
        )

        resetOdometry(
            LimelightSubsystem.getPose()?.toPose2d()?.relativeTo(
                Pose2d(Translation2d(-16.54099 / 2, -8.069326 / 2), Rotation2d()),
            )
                ?: getPose(),
        ) // limelight synchronization
        println(getPose())

        // println("ArcPose = ${DriveToArcPoseGenerator.generatePath()}")
        // println("AllianceRed = ${(DriverStation.getAlliance() ?: DriverStation.Alliance.Red) == DriverStation.Alliance.Red}")

        val states: Array<SwerveModuleVelocity> =
            arrayOf(
                frontLeft.getState(),
                frontRight.getState(),
                rearLeft.getState(),
                rearRight.getState(),
            )

        // this will make the swerve diagram much more choppier,
        // delete the if statement if you want it to be cleaner

        if (counter % 5 == 0) {
            swervePublisher.set(states)
            currentPublisher.set(fL.getStatorCurrent().getValueAsDouble())
        }

//        }

//        println("Front Right:"+ FrontRightEncoder.getVelocity())
//        println("Front Right Speed: "+frontRight.getState().velocity)
//        println("Front Left Speed: "+frontLeft.getState().velocity)
//        println("Back Right Speed: "+rearRight.getState().velocity)
//        println("Back Left Speed: "+rearLeft.getState().velocity)
//        println("Odometry:"+getPose())
//
//        println("angle:"+gyro.getYaw())
//        println(xBoxController.getRawAxis(0))
    }

    fun getPose(): Pose2d = odometry.pose

    fun resetPose(pose: Pose2d) {
        resetOdometry(pose)
    }

    // IDE bug, the detected and actual signatures are different
    @Suppress("TYPE_MISMATCH", "TOO_MANY_ARGUMENTS")
    fun getCurrentSpeeds(): ChassisVelocities =
        Constants.DriveConstants.DRIVE_KINEMATICS.toChassisVelocities(
            frontLeft.getState(),
            frontRight.getState(),
            rearLeft.getState(),
            rearRight.getState(),
        )

    fun resetOdometry(pose: Pose2d) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(gyro.yaw.valueAsDouble), // gyro.getRotation2d(),
            arrayOf(
                frontLeft.getPosition(),
                frontRight.getPosition(),
                rearLeft.getPosition(),
                rearRight.getPosition(),
            ),
            pose,
        )
    }

    fun shouldFlipPath(): Boolean {
        return false // (DriverStation.getAlliance() ?: DriverStation.Alliance.Red) == DriverStation.Alliance.Red
    }

    fun drive(
        speeds: ChassisVelocities,
        fieldRelative: Boolean,
    ) {
        var SwerveModuleVelocities = Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleVelocities(speeds)

        if (fieldRelative) {
            SwerveModuleVelocities =
                Constants.DriveConstants.DRIVE_KINEMATICS.toSwerveModuleVelocities(
                    speeds.toFieldRelative(Rotation2d.fromDegrees(gyro.yaw.valueAsDouble)),
                ) // gyro.getRotation2d()
        }

        frontLeft.setDesiredState(SwerveModuleVelocities[0])
        frontRight.setDesiredState(SwerveModuleVelocities[1])
        rearLeft.setDesiredState(SwerveModuleVelocities[2])
        rearRight.setDesiredState(SwerveModuleVelocities[3])
    }

    fun setX() {
        frontLeft.setDesiredState(
            SwerveModuleVelocity(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
        frontRight.setDesiredState(
            SwerveModuleVelocity(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        rearLeft.setDesiredState(
            SwerveModuleVelocity(
                0.0,
                Rotation2d.fromDegrees(-45.0),
            ),
        )
        rearRight.setDesiredState(
            SwerveModuleVelocity(
                0.0,
                Rotation2d.fromDegrees(45.0),
            ),
        )
    }

    fun zeroHeading() {
        gyro.reset()
    }
}
