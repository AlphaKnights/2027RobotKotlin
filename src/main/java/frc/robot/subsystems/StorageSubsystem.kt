package frc.robot.subsystems

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.Constants.RollerConstants
import org.wpilib.command2.SubsystemBase

object StorageSubsystem : SubsystemBase() {
    private val CAN = CANBus("didy")
    private val rollerMotor = TalonFX(RollerConstants.ROLLER_MOTOR_ID, CAN)

    private val rollerMotor2 = TalonFX(RollerConstants.ROLLER_MOTOR_ID_2, CAN)

    init {

        val rollerMotorConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimitEnable = true
                    SupplyCurrentLimit = RollerConstants.ROLLER_MOTOR_CURRENT_LIMITS
                    StatorCurrentLimitEnable = true
                    StatorCurrentLimit = RollerConstants.ROLLER_MOTOR_STATOR_LIMITS
                }
            }

        rollerMotor.configurator.apply(rollerMotorConfig)
        rollerMotor2.configurator.apply(rollerMotorConfig)
    }

    fun roll(rollerSpeed: Double) {
        rollerMotor.throttle = rollerSpeed
        rollerMotor2.throttle = -rollerSpeed
    }

    fun rollerstop() {
        rollerMotor.stopMotor()
        rollerMotor2.stopMotor()
    }
}
