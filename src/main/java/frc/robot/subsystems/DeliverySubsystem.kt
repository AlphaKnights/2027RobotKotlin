/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import frc.robot.Constants
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import frc.robot.Constants.LaunchConstants
import org.wpilib.command2.SubsystemBase
import org.wpilib.util.sendable.SendableBuilder

object DeliverySubsystem : SubsystemBase() {
    /*
    // Ultrasonic if we need one
    private val rangeFinder =
        Ultrasonic(
            Constants.UltrasonicConstants.PING_CHANNEL,
            Constants.UltrasonicConstants.ECHO_CHANNEL,
        )
     */
    private val CAN = Constants.ModuleConstants.CANBUS
    private val leftLaunchMotor =
        TalonFX(
            LaunchConstants.LEFT_LAUNCHMOTOR_ID,
            CAN,
        )
    private val rightLaunchMotor =
        TalonFX(
            LaunchConstants.RIGHT_LAUNCHMOTOR_ID,
            CAN,
        )

    init {
        //Ultrasonic.setAutomaticMode(true)

        val launchMotorConfig1 =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = LaunchConstants.LAUNCH_MOTOR_CURRENT_LIMITS
                }

                Slot0.apply {
                    kP = LaunchConstants.LAUNCH_P
                    kI = LaunchConstants.LAUNCH_I
                    kD = LaunchConstants.LAUNCH_D
                    kS = LaunchConstants.LAUNCH_FF
                    kV = LaunchConstants.LAUNCH_V
                    kA = LaunchConstants.LAUNCH_A
                }

                MotorOutput.apply {
                    InvertedValue.Clockwise_Positive
                }
            }

        leftLaunchMotor.configurator.apply(launchMotorConfig1)

        val launchMotorConfig2 =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = LaunchConstants.LAUNCH_MOTOR_CURRENT_LIMITS
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit = LaunchConstants.LAUNCH_MOTOR_STATOR_LIMITS
                }

                Slot0.apply {
                    kP = LaunchConstants.LAUNCH_P
                    kI = LaunchConstants.LAUNCH_I
                    kD = LaunchConstants.LAUNCH_D
                    kS = LaunchConstants.LAUNCH_FF
                    kV = LaunchConstants.LAUNCH_V
                    kA = LaunchConstants.LAUNCH_A
                }

                MotorOutput.apply {
                    InvertedValue.CounterClockwise_Positive
                }
            }

        rightLaunchMotor.configurator.apply(launchMotorConfig2)
        // rangeFinder.isEnabled = true

//        val launchMotorConfig =
//            SparkMaxConfig().apply {
//                idleMode(SparkBaseConfig.IdleMode.kBrake)
//            }
//
//        launchMotor.configure(
//            launchMotorConfig,
//            SparkBase.ResetMode.kResetSafeParameters,
//            SparkBase.PersistMode.kPersistParameters,
//        )
    }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)

        builder.apply {
            addDoubleProperty("left motor voltage", {
                leftLaunchMotor.motorVoltage.valueAsDouble
            }, null)
            addDoubleProperty("right motor voltage", {
                rightLaunchMotor.motorVoltage.valueAsDouble
            }, null)
            addDoubleProperty("left stator current", {
                leftLaunchMotor.statorCurrent.valueAsDouble
            }, null)
            addDoubleProperty("right stator current", {
                rightLaunchMotor.statorCurrent.valueAsDouble
            }, null)
        }
    }

    fun forward(launchProp: Double) {
        // leftLaunchMotor.setControl(VelocityVoltage(launchProp))
        // rightLaunchMotor.setControl(VelocityVoltage(launchProp))
        leftLaunchMotor.throttle = launchProp
        rightLaunchMotor.throttle = -launchProp
    }

    fun stop() {
        leftLaunchMotor.stopMotor()
        rightLaunchMotor.stopMotor()
    }

    /*
    fun fuelInside(): Boolean =
    rangeFinder.rangeInches <
    Constants.UltrasonicConstants.CORAL_DISTANCE
     */
}
