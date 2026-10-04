/*
 * (C) 2025 Galvaknights
 */
package frc.robot.interfaces

import org.wpilib.math.kinematics.SwerveModulePosition
import org.wpilib.math.kinematics.SwerveModuleVelocity
import org.wpilib.telemetry.TelemetryLoggable
import org.wpilib.telemetry.TelemetryTable

interface SwerveModule : TelemetryLoggable {
    override fun logTo(table: TelemetryTable?) {
        null
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    fun getPosition(): SwerveModulePosition

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    fun getState(): SwerveModuleVelocity

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    fun setDesiredState(desiredState: SwerveModuleVelocity)
}
