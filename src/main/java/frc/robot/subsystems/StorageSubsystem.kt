/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import frc.robot.Constants.RollerConstants
import org.wpilib.command2.SubsystemBase

object StorageSubsystem : SubsystemBase() {
    private val CAN = Constants.ModuleConstants.CANBUS
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

    override fun initSendable(builder: SendableBuilder?) {
        super.initSendable(builder)
        builder?.apply {
            addDoubleProperty("roller 1 motor voltage", { rollerMotor.motorVoltage.valueAsDouble }, null)
            addDoubleProperty("roller 2 motor voltage", { rollerMotor2.motorVoltage.valueAsDouble }, null)
            addDoubleProperty("roller 1 stator current", { rollerMotor.statorCurrent.valueAsDouble }, null)
            addDoubleProperty("roller 2 stator current", { rollerMotor2.statorCurrent.valueAsDouble }, null)
        }
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
