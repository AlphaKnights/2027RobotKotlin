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
import frc.robot.Constants
import org.wpilib.command2.SubsystemBase
import org.wpilib.hardware.discrete.DigitalInput
import org.wpilib.telemetry.Telemetry
import org.wpilib.telemetry.TelemetryLoggable
import org.wpilib.telemetry.TelemetryTable
import org.wpilib.tunable.Tunables


object IntakeSubsystem : SubsystemBase() {
    private val CAN = Constants.ModuleConstants.CANBUS
    private val intakeMotor = TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID, CAN)
    private val telemetryTable = Telemetry.getTable("intake")

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
        
        initTunable()
    }


    fun initTunable() {
    
        val tunableTable = Tunables.getTable("intake")
        
        tunableTable.publishDouble("position", ::getPosition, ::setPosition)
    

        tunableTable.publishDouble(
            "P",
            {
                globalConfig.Slot0.kP
            },
            { value: Double ->
                rightleverMotor.configurator.apply(globalConfig.Slot0.withKP(value))
            },
        )

        tunableTable.publishDouble(
            "I",
            {
                globalConfig.Slot0.kI
            },
            { value: Double ->
                rightleverMotor.configurator.apply(globalConfig.Slot0.withKI(value))
            },
        )

        tunableTable.publishDouble(
            "D",
            {
                globalConfig.Slot0.kD
            },
            { value: Double ->
                rightleverMotor.configurator.apply(globalConfig.Slot0.withKD(value))
            },
        )

        tunableTable.publishDouble(
            "kG",
            {
                globalConfig.Slot0.kG
            },
            { value: Double ->
                rightleverMotor.configurator.apply(globalConfig.Slot0.withKG(value))
            },
        )
    
        
    }

    override fun logTo(table: TelemetryTable?) {
        super.logTo(table)
        
        table?.apply {
            log("lever motor voltage", rightleverMotor.motorVoltage.valueAsDouble)
            log("lever stator current", rightleverMotor.statorCurrent.valueAsDouble)
            log("intake motor voltage", intakeMotor.motorVoltage.valueAsDouble)
            log("intake stator current", intakeMotor.statorCurrent.valueAsDouble)
            log("setpoint", rightleverMotor.closedLoopReference.valueAsDouble)
            log("control error", rightleverMotor.closedLoopError.valueAsDouble)
        }
    }
    
    override fun periodic() {
        logTo(telemetryTable)
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
