/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems


import frc.robot.Constants.ModuleConstants
import frc.robot.interfaces.SwerveModule
import frc.robot.subsystems.Telemetry.makeTunable
import org.wpilib.math.controller.PIDController
import org.wpilib.math.geometry.Rotation2d
import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.math.system.DCMotor
import org.wpilib.math.system.Models
import org.wpilib.simulation.DCMotorSim
import org.wpilib.smartdashboard.SmartDashboard

class SwerveModuleSim : SwerveModule {
    private val DRIVE_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)
    private val TURN_GEARBOX: DCMotor = DCMotor.getKrakenX60Foc(1)

    val driveSim = DCMotorSim(
        Models.singleJointedArmFromPhysicalConstants(
            DRIVE_GEARBOX,
            2.0,
            ModuleConstants.DRIVE_RATIO,
        ),
        DRIVE_GEARBOX,
    )
    private val turnSim =
        DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                TURN_GEARBOX,
                2.0,
                ModuleConstants.DRIVE_RATIO,
            ),
            TURN_GEARBOX,
        )

    val driveController = PIDController(0.0, 0.0, 0.0).makeTunable("DrivePID")

    // val turnController = PIDController(0.0, 0.0, 0.0)

    init {
        // Enable wrapping for turn PID
        // turnController.enableContinuousInput(-Math.PI, Math.PI)
    }

    override fun getPosition(): SwerveModulePosition =
        SwerveModulePosition(
            driveSim.angularPosition / (2*Math.PI) * ModuleConstants.WHEEL_CIRCUMFERENCE,
            Rotation2d.fromRotations(
                turnSim.angularPosition / (2*Math.PI),
            ),
        )

    override fun getState(): SwerveModuleVelocity =
        SwerveModuleVelocity(
            ModuleConstants.WHEEL_CIRCUMFERENCE * driveSim.angularVelocity / (2*Math.PI),
            Rotation2d.fromRotations(
                turnSim.angularPosition / (2*Math.PI),
            ),
        )

    override fun setDesiredState(desiredState: SwerveModuleVelocity) {
        desiredState.optimize(
            Rotation2d.fromRotations(
                turnSim.angularPosition,
            ),
        )


        val driveAppliedVolts: Double =
            driveController.calculate(
                driveSim.angularVelocity,
                desiredState.velocity / ModuleConstants.WHEEL_CIRCUMFERENCE,
            )
        SmartDashboard.putNumber("driveAppliedVolts", driveAppliedVolts)
//        val turnAppliedVolts: Double =
//            turnController.calculate(
//                turnSim.angularPosition,
//                desiredState.angle.rotations,
//            ) + (turnSim.angularAccelerationRadPerSecSq * turn_KV * Math.PI)

//        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0))
        turnSim.setAngle(desiredState.angle.radians)
        driveSim.setInputVoltage(driveAppliedVolts.coerceIn(-12.0,12.0))


        turnSim.update(0.02)
        driveSim.update(0.02)
    }
}
