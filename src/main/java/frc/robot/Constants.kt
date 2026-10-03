/*
 * (C) 2025 Galvaknights
 */
package frc.robot

/*
 * The Constants file provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This file should not be used for any other purpose.
 * All String, Boolean, and numeric (Int, Long, Float, Double) constants should use
 * `const` definitions. Other constant types should use `val` definitions.
 */

import com.ctre.phoenix6.CANBus
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.kinematics.SwerveDriveKinematics
import org.wpilib.math.util.Units
import kotlin.math.PI

object Constants {
    object OperatorConstants {
        const val DRIVER_CONTROLLER_PORT = 0
        const val DRIVE_DEADBAND = 0.2
        const val LERP_VAL = 0.035

        const val RESET_HEADING_BUTTON = 11

        const val ALIGN_LEFT_BUTTON = 9 // joystick
        const val ALIGN_RIGHT_BUTTON = 10 // joystick

        const val BUTTON_BOARD_PORT = 2

        const val ANGLE_BUTTON = 12 // joystick

        const val INTAKE_BUTTON = 10
        const val INTAKE_REVERSE_BUTTON = 2
        const val INTAKE_LEVER_OUT_MANUAL_BUTTON = 5
        const val INTAKE_LEVER_IN_MANUAL_BUTTON = 4
        const val INTAKE_LEVER_OUT_AUTO_BUTTON = 8
        const val INTAKE_LEVER_IN_AUTO_BUTTON = 9
        const val DELIVERY_BUTTON = 12
        const val DELIVERY_REVERSE_BUTTON = 1
        const val INDEXER_REVERSE_BUTTON = 3
    }

    object IntakeConstants {
        const val INTAKE_MOTOR_ID = 34
        const val RIGHT_LEVER_MOTOR_ID = 31
        const val LEFT_LEVER_MOTOR_ID = 30
        const val INTAKE_SPEED = 0.67

        // Limits should be in rotations
        const val LEVER_LIMIT_FORWARD = 0.0
        const val LEVER_LIMIT_REVERSE = 0.25

        const val P = 0.1
        const val I = 0.0
        const val D = 0.01

        // basically a controller deadzone, how close to the setpoint it needs to be to stop moving
        const val GAIN_SCHEDULE_ERROR_THRESHOLD = 0.1

        const val LEVER_OUT_POSITION = 12.0
        const val LEVER_IN_POSITION = 0.0

        const val LEVER_SPEED = 0.2 // in rpm

        const val INTAKE_CURRENT_LIMIT = 20.0
        const val INTAKE_STATOR_LIMIT = 40.0
    }

    enum class IntakeDirection {
        IN,
        OUT,
    }

    object SomeConstants {
        val isReal: Boolean = RobotBase.isReal()

        enum class SwerveType {
            TALON,
            SPARKMAX,
        }
    }

    object DriveConstants {
        const val MAX_METERS_PER_SECOND = 2.0
        const val MAX_ANGULAR_SPEED = 2.0
        const val MAX_SLIDING_SPEED_PERCENTAGE = 0.5

        private val TRACK_WIDTH = Units.inchesToMeters(25.5)
        private val WHEEL_BASE = Units.inchesToMeters(25.5)

        private val MODULE_POSITIONS =
            arrayOf(
                Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
            )
        val DRIVE_KINEMATICS = SwerveDriveKinematics(*MODULE_POSITIONS)

        val FRONT_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.831299)
        val FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.113525 + 0.5)
        val BACK_LEFT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(-0.170166) // + is clockwise
        val BACK_RIGHT_CHASSIS_ANGULAR_OFFSET: Rotation2d = Rotation2d.fromRotations(0.002686 + 0.5) // - counter-clockwise

        //   back right - > front left
        //   back left - >front right
        //   front left -> back right
        // front right -> back left

        const val FRONT_LEFT_DRIVING_ID = 8 // 8->4
        const val REAR_LEFT_DRIVING_ID = 7 // 5->19
        const val FRONT_RIGHT_DRIVING_ID = 19 // 19->5
        const val REAR_RIGHT_DRIVING_ID = 4 // 4->8

        const val FRONT_LEFT_TURNING_ID = 2 // 2->3
        const val REAR_LEFT_TURNING_ID = 5 // 7->6
        const val FRONT_RIGHT_TURNING_ID = 6 // 6->7
        const val REAR_RIGHT_TURNING_ID = 3 // 3->2

        const val FRONT_LEFT_CANCODER_ID = 11 // 11->9
        const val REAR_LEFT_CANCODER_ID = 12 // 12->10
        const val FRONT_RIGHT_CANCODER_ID = 10 // 10->12
        const val REAR_RIGHT_CANCODER_ID = 9 // 9->11

        const val ROTATE_CONTROLLER_P = 2.0
        const val ROTATE_CONTROLLER_I = 0.0
        const val ROTATE_CONTROLLER_D = 0.01
        const val TRANSLATION_CONTROLLER_P = 5.0
        const val TRANSLATION_CONTROLLER_I = 0.0
        const val TRANSLATION_CONTROLLER_D = 0.01

        const val DRIVE_SETPOINT_TOLERANCE = 0.05
        const val ROTATE_SETPOINT_TOLERANCE = 0.08

        const val PIDGEON2_ID = 20
    }

    object ModuleConstants {
        const val DRIVE_RATIO = 5.36
        val WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * PI

        // const val WHEEL_CIRCUMFERENCE = 0.5 // meters

        const val DRIVING_P = 0.8
        const val DRIVING_I = 0.0
        const val DRIVING_D = 0.0
        const val DRIVING_FF = 1.0
        const val DRIVING_V = 0.12 // 0.12*DRIVE_RATIO
        const val DRIVING_A = 1.5
        const val TURNING_P = 40.0
        const val TURNING_I = 0.0
        const val TURNING_D = 0.0
        const val TURNING_FF = 0.0

        const val DRIVING_MOTOR_CURRENT_LIMIT = 60.0
        const val TURNING_MOTOR_CURRENT_LIMIT = 60.0
        const val DRIVING_STATOR_CURRENT_LIMIT = 120.0
        const val TURNING_STATOR_CURRENT_LIMIT = 120.0

        val CANBUS: CANBus = CANBus("didy")
    }

    object LimelightConstants {
        const val POLLING_RATE = 20L
        const val TIMEOUT = 500L // milliseconds
        const val IP_ADDR = "10.66.95.201"
//        const val IP_ADDR = "172.29.0.1"
    }

    object LaunchConstants {
        const val LEFT_LAUNCHMOTOR_ID = 54
        const val RIGHT_LAUNCHMOTOR_ID = 28
        const val LAUNCH_SPEED = 0.65
        const val ALT_LAUNCH_SPEED = 0.85
        const val LAUNCH_P = 0.1
        const val LAUNCH_I = 0.0
        const val LAUNCH_D = 0.0
        const val LAUNCH_FF = 0.0
        const val LAUNCH_V = 0.0071
        const val LAUNCH_A = 0.0
        const val LAUNCH_MOTOR_CURRENT_LIMITS = 40.0
        const val LAUNCH_MOTOR_STATOR_LIMITS = 80.0
    }

    object RollerConstants {
        const val ROLLER_MOTOR_ID = 23

        const val ROLLER_MOTOR_ID_2 = 24
        const val ROLLER_SPEED = 0.65
        const val ALT_ROLLER_SPEED = 0.9
        const val ROLLER_MOTOR_CURRENT_LIMITS = 20.0
        const val ROLLER_MOTOR_STATOR_LIMITS = 40.0
        const val BUTTON = 11
    }

    object AlignConstants {
        const val ALIGN_DEADZONE = 0.03
        val ALIGN_ROT_DEADZONE = Units.degreesToRadians(5.0)

        const val FINE_ALIGN_DEADZONE = 1.0
        val FINE_ALIGN_ROT_DEADZONE = Units.degreesToRadians(5.0)

        const val MAX_SPEED = 1.0
        const val MAX_ANGULAR_SPEED = 1.0

        const val LEFT_X_OFFSET = -0.1625
        const val LEFT_Z_OFFSET = 0.457

        const val RIGHT_X_OFFSET = 0.1625
        const val RIGHT_Z_OFFSET = 0.457

        const val ALIGN_TIMEOUT = 5 // seconds
        const val ALIGN_SEEK_TIMEOUT = 1 // seconds
    }

    enum class AlignDirection {
        LEFT,
        RIGHT,
    }

    object AimingConstants {
        const val DISTANCE = 1.0
        const val GOOD_DISTANCE_TOLERANCE = 0.5
        const val MIDDLING_DISTANCE_TOLERANCE = 1.0

        // Hub field positions (meters). Set to real field measurements before competition.
        // Red hub: robot approaches from y < RED_HUB_Y
        // Blue hub: robot approaches from y > BLUE_HUB_Y
        // TODO: measure using center field origin
        const val RED_HUB_X = 4.625
        const val RED_HUB_Y = 4.0
        const val BLUE_HUB_X = -3.644
        const val BLUE_HUB_Y = 4.0 // placeholder — team must tune

        // Valid shooting-arc sector, in degrees, measured from hub center.
        //   0° = +X on field,  90° = +Y,  180° = -X,  270° = -Y (toward driver station)
        // These values apply for red alliance (robot below hub, y < RED_HUB_Y).
        // For blue alliance the sector is mirrored vertically — team must tune.
        const val MIN_ANGLE_DEGREES = 0.0
        const val MAX_ANGLE_DEGREES = 180.0

        // Fallback when DriverStation hasn't reported an alliance yet (e.g. practice mode).
        // true = red alliance, false = blue alliance.
        const val DEFAULT_TO_RED_ALLIANCE = true

        const val MAX_SPEED = 1.0
        const val MAX_ANGULAR_SPEED = 1.0
        const val SLOW_DISTANCE = 1.0
        const val MIN_SPEED = 0.2

        const val DIST_DEADZONE = 0.1 // m
        const val ANGLE_DEADZONE = 1.0 // degrees
    }

    object PathPlannerConstants {
        const val TRANSLATION_P = 5.0
        const val TRANSLATION_I = 0.0
        const val TRANSLATION_D = 0.0

        const val ROTATION_P = 17.0
        const val ROTATION_I = 0.0
        const val ROTATION_D = 0.0

        val FIELD_SIZE = Translation2d(16.54099, 8.069326) // get from /deploy/pathplanner/navgrid.json
    }

    object CANBusIDs {
        /*
        Possible CAN bus strings are:
            "can_s0" to "can_s24" for the native Systemcore/Motioncore CAN buses
            CANivore name or serial number
            SocketCAN interface (non-FRC Linux only)
            "*" for any CANivore seen by the program
            empty string (default) to select the default for the system:
                "can_s1" on Systemcore
                "can0" on Linux
                "*" on Windows
         */
        val DRIVE_CANBUS: CANBus = CANBus("can_s0")
        val PIDGEON_CANBUS: CANBus = CANBus("can_s1")
    }
}



