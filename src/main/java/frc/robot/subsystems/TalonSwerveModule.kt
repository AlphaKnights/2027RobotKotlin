/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.Constants.ModuleConstants
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.kinematics.SwerveModuleVelocity

class TalonSwerveModule(
    driveMotorId: Int,
    turnMotorId: Int,
    encoderId: Int,
    private val offset: Rotation2d,
) {
    private val driveMotor = TalonFX(driveMotorId)
    private val turnMotor = TalonFX(turnMotorId)
    private val encoder = CANcoder(encoderId)
    private var desiredState =
        SwerveModuleVelocity(
            0.0,
            Rotation2d.fromRotations(
                encoder.position.valueAsDouble,
            ) +
                offset,
        )


    init {
        val driveMotorConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit =
                        ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit =
                        ModuleConstants.DRIVING_STATOR_CURRENT_LIMIT
                }

                Slot0.apply {
                    kP = ModuleConstants.DRIVING_P
                    kI = ModuleConstants.DRIVING_I
                    kD = ModuleConstants.DRIVING_D
                    kS = ModuleConstants.DRIVING_FF
                    kV = ModuleConstants.DRIVING_V
                    kA = ModuleConstants.DRIVING_A
                }

                OpenLoopRamps.apply {
                    DutyCycleOpenLoopRampPeriod = 0.0
                }

                ClosedLoopRamps.apply {
                    DutyCycleClosedLoopRampPeriod = 0.0
                }

                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                }

                Feedback.apply {
                    SensorToMechanismRatio =
                        ModuleConstants.DRIVE_RATIO
                }
            }

        val turnMotorConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit =
                        ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit =
                        ModuleConstants.TURNING_STATOR_CURRENT_LIMIT
                }

                Feedback.apply {
                    SensorToMechanismRatio = 1.0
                    FeedbackRemoteSensorID = encoderId
                    FeedbackSensorSource =
                        FeedbackSensorSourceValue.RemoteCANcoder
                }

                ClosedLoopGeneral.apply {
                    ContinuousWrap = true
                }

                Slot0.apply {
                    kP = ModuleConstants.TURNING_P
                    kI = ModuleConstants.TURNING_I
                    kD = ModuleConstants.TURNING_D
                    kS = ModuleConstants.TURNING_FF
                }

                OpenLoopRamps.apply {
                    DutyCycleOpenLoopRampPeriod = 0.0
                }

                ClosedLoopRamps.apply {
                    DutyCycleClosedLoopRampPeriod = 0.0
                }

                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                    Inverted =
                        InvertedValue.Clockwise_Positive
                }
            }

        driveMotor.getConfigurator().apply(driveMotorConfig)
        turnMotor.getConfigurator().apply(turnMotorConfig)

        driveMotor.setPosition(0.0)
    }

    fun getPosition(): SwerveModulePosition =
        SwerveModulePosition(
            // driveMotor.rotor
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveMotor.position.valueAsDouble,
            Rotation2d.fromRotations(
                turnMotor.position.valueAsDouble,
            ) +
                offset,
        )

    fun getState(): SwerveModuleVelocity =
        SwerveModuleVelocity(
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveMotor.velocity.valueAsDouble,
            Rotation2d.fromRotations(
                turnMotor.position.valueAsDouble,
            ) +
                offset,
        )

    fun setDesiredState(desiredState: SwerveModuleVelocity) {
        val correctedState =
            SwerveModuleVelocity(
                desiredState.velocity,
                desiredState.angle - offset,
            )
        correctedState.optimize(
            Rotation2d.fromRotations(
                encoder.position.valueAsDouble,
            ),
        )

        driveMotor.setControl(
            VelocityVoltage(
                correctedState.velocity / ModuleConstants.WHEEL_CIRCUMFERENCE,
            ),
        )
        turnMotor.setControl(
            PositionVoltage(correctedState.angle.rotations),
        )

        this.desiredState = desiredState
    }
}
