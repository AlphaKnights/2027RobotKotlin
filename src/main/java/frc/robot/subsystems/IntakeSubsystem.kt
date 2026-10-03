/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GainSchedBehaviorValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.config.ClosedLoopConfig
import com.revrobotics.spark.config.SparkBaseConfig
import com.revrobotics.spark.config.SparkMaxConfig
import frc.robot.Constants
import org.wpilib.command2.SubsystemBase
import org.wpilib.hardware.discrete.DigitalInput

object IntakeSubsystem : SubsystemBase() {
    private val CAN = Constants.ModuleConstants.CANBUS
    private val intakeMotor = TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID, CAN)

    val rightleverMotor =
        TalonFX(
            Constants.IntakeConstants.RIGHT_LEVER_MOTOR_ID,
            CAN,
        )

    val leftLeverMotor =
        TalonFX(
            Constants.IntakeConstants.LEFT_LEVER_MOTOR_ID,
            CAN,
        )

    // val limitUp: DigitalInput = DigitalInput(2)
    // val limitDown: DigitalInput = DigitalInput(3)

    val globalConfig =
        TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = Constants.IntakeConstants.INTAKE_CURRENT_LIMIT
                StatorCurrentLimitEnable = true
                StatorCurrentLimit = Constants.IntakeConstants.INTAKE_STATOR_LIMIT
            }

            SoftwareLimitSwitch.apply {
                ForwardSoftLimitEnable
                ReverseSoftLimitEnable

                ForwardSoftLimitThreshold = Constants.IntakeConstants.LEVER_LIMIT_FORWARD
                ReverseSoftLimitThreshold = Constants.IntakeConstants.LEVER_LIMIT_REVERSE
            }

            Slot0.apply {
                kI = Constants.IntakeConstants.I
                kP = Constants.IntakeConstants.P
                kD = Constants.IntakeConstants.D
            }
            Slot0.GainSchedBehavior = GainSchedBehaviorValue.ZeroOutput
            ClosedLoopGeneral.GainSchedErrorThreshold = Constants.IntakeConstants.GAIN_SCHEDULE_ERROR_THRESHOLD

            MotorOutput.apply {
                InvertedValue.CounterClockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }

            ResetMode.kResetSafeParameters
            PersistMode.kPersistParameters

            /*encoder.apply {
                positionConversionFactor(1.0)
                velocityConversionFactor(1.0)
            }

            closedLoop.apply {
                feedbackSensor(
                    FeedbackSensor.kPrimaryEncoder, //ClosedLoopConfig.Feedback.Sensor
                )
                pid(
                    ClimbConstants.P,
                    ClimbConstants.I,
                    ClimbConstants.D,
                )
                outputRange(-1.0, 1.0)
                positionWrappingEnabled(false)
                }
             */
        }

    init {
        val intakeMotorConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = Constants.IntakeConstants.INTAKE_CURRENT_LIMIT
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit = Constants.IntakeConstants.INTAKE_STATOR_LIMIT
                }

                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Brake
                }
            }

        intakeMotor.configurator.apply(intakeMotorConfig)
        leftLeverMotor.configurator.apply(globalConfig)
        rightleverMotor.configurator.apply(globalConfig)
    }

    override fun initSendable(builder: SendableBuilder) {
        builder.apply {
            setSafeState {
                rightleverMotor.disable()
                intakeMotor.disable()
            }
            addDoubleProperty("position", ::getPosition, ::setPosition)
            addDoubleProperty("lever motor voltage", { rightleverMotor.motorVoltage.valueAsDouble }, null)
            addDoubleProperty("lever stator current", { rightleverMotor.statorCurrent.valueAsDouble }, null)
            addDoubleProperty("intake motor voltage", { intakeMotor.motorVoltage.valueAsDouble }, null)
            addDoubleProperty("intake stator current", { intakeMotor.statorCurrent.valueAsDouble }, null)
            addDoubleProperty("setpoint", { rightleverMotor.closedLoopReference.valueAsDouble }, null)
            addDoubleProperty("control error", { rightleverMotor.closedLoopError.valueAsDouble }, null)

            addDoubleProperty(
                "P",
                {
                    globalConfig.Slot0.kP
                },
                { value: Double ->
                    rightleverMotor.configurator.apply(globalConfig.Slot0.withKP(value))
                },
            )

            addDoubleProperty(
                "I",
                {
                    globalConfig.Slot0.kI
                },
                { value: Double ->
                    rightleverMotor.configurator.apply(globalConfig.Slot0.withKI(value))
                },
            )

            addDoubleProperty(
                "D",
                {
                    globalConfig.Slot0.kD
                },
                { value: Double ->
                    rightleverMotor.configurator.apply(globalConfig.Slot0.withKD(value))
                },
            )

            addDoubleProperty(
                "kG",
                {
                    globalConfig.Slot0.kG
                },
                { value: Double ->
                    rightleverMotor.configurator.apply(globalConfig.Slot0.withKG(value))
                },
            )
        }
    }

    fun runIntake(speed: Double) {
        intakeMotor.throttle = -speed
    }

    fun setPosition(position: Double) {
        val mRequest = PositionDutyCycle(0.0).withSlot(0)

        rightleverMotor.setControl(mRequest.withPosition(-position))
        leftLeverMotor.setControl(Follower(rightleverMotor.deviceID, MotorAlignmentValue.Opposed))
    }

    fun isInPosition(deadzone: Double): Boolean = (rightleverMotor.closedLoopError.valueAsDouble < deadzone)

    fun moveLever(speed: Double) {
        rightleverMotor.throttle = speed
        leftLeverMotor.throttle -speed
    }

    fun getPosition(): Double = rightleverMotor.position.valueAsDouble

    fun stopIntake() {
        intakeMotor.stopMotor()
    }

    fun stopIntakeLever() {
        rightleverMotor.stopMotor()
        leftLeverMotor.stopMotor()
    }

    fun limitSwitchPressed(): Boolean {
        // return (limitUp.get() || limitDown.get())
        return false
    }

    fun limitOutput() {
        // print("Limit Up Pressed: "+limitUp.get())
        // print("Limit Down Pressed: "+limitDown.get())
    }
}
