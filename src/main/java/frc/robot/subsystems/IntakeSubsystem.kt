package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TalonFXConfigurator
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionDutyCycle
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.Constants
import org.wpilib.command2.SubsystemBase
import org.wpilib.hardware.discrete.DigitalInput

object IntakeSubsystem : SubsystemBase() {
    private val CAN = CANBus("didy")
    private val intakeMotor = TalonFX(Constants.IntakeConstants.INTAKE_MOTOR_ID, CAN)

    private val rightleverMotor =
        TalonFX(
            Constants.IntakeConstants.RIGHT_LEVER_MOTOR_ID,
            CAN,
        )

    private val leftLeverMotor =
        TalonFX(
            Constants.IntakeConstants.LEFT_LEVER_MOTOR_ID,
            CAN,
        )

    // val limitUp: DigitalInput = DigitalInput(2)
    // val limitDown: DigitalInput = DigitalInput(3)

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
                    positionWrappingEnabled(false)*/
            }
        intakeMotor.configurator.apply(intakeMotorConfig)
        leftLeverMotor.configurator.apply(globalConfig)
        rightleverMotor.configurator.apply(globalConfig)
    }

    override fun periodic() {
        super.periodic()
        //println("Lever Position = ${leftLeverMotor.position}")
    }

    fun runIntake(speed: Double) {
        intakeMotor.throttle = -speed
    }

    fun setPosition(position: Double) {
        var m_request = PositionDutyCycle(0.0).withSlot(0)

        rightleverMotor.setControl(m_request.withPosition(-position))
        leftLeverMotor.setControl(Follower(rightleverMotor.deviceID, MotorAlignmentValue.Opposed))
    }

    fun isInPosition(deadzone: Double): Boolean = (rightleverMotor.getClosedLoopError().valueAsDouble < deadzone)

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
