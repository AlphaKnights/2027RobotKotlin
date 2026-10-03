/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems

import com.fasterxml.jackson.databind.DeserializationFeature
import com.fasterxml.jackson.databind.ObjectMapper
import frc.robot.Constants
import frc.robot.LimelightHelpers.LimelightResults
import frc.robot.interfaces.LimelightService
import frc.robot.interfaces.PoseProvider
import kotlinx.coroutines.*
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Pose3d
import kotlin.math.abs
import kotlin.math.absoluteValue

object LimelightSubsystem : PoseProvider {
    private val mutex = Mutex()
    private var _tagPose: Pose3d? = null
    override val tagPose: Pose3d?
        get() = runBlocking { mutex.withLock { _tagPose } }
    var coroutineScope =
        CoroutineScope(
            Dispatchers.Default + SupervisorJob(),
        )

    var limelightService: LimelightService = HttpLimelightService

    fun startPolling() {
        coroutineScope.launch {
            while (true) {
                setTagPose(updateTagPosition())
                delay(
                    Constants.LimelightConstants.POLLING_RATE,
                )
            }
        }
    }

    suspend fun updateTagPosition(): Pose3d? {
        kotlin
            .runCatching {
                limelightService
                    .fetchResults()
                    ?.let(::parseJson)
                    ?.targets_Fiducials
                    ?.firstOrNull()
                    ?.robotPose_FieldSpace
            }.onSuccess { pose ->
                return pose
            }.onFailure {
                println("Failed to update tag position: $it")
                return null
            }
        return null
    }

    suspend fun setTagPose(pose: Pose3d?) {
        mutex.withLock {
            _tagPose = pose
        }
    }



    fun isAligned(): Boolean {
        val pose = tagPose ?: return false

        return (
            isWithinLeftPosition(pose) ||
                isWithinRightPosition(pose)
        ) &&
            isRotationAligned(pose)
    }

    fun getPose(): Pose3d? = tagPose

    private fun isWithinLeftPosition(pose: Pose3d): Boolean {
        val translation = pose.translation
        return abs(
            translation.x +
                Constants.AlignConstants.LEFT_X_OFFSET,
        ) <=
            Constants.AlignConstants.ALIGN_DEADZONE &&
            abs(
                translation.z -
                    Constants.AlignConstants.LEFT_Z_OFFSET,
            ) <=
            Constants.AlignConstants.ALIGN_DEADZONE
    }

    private fun isWithinRightPosition(pose: Pose3d): Boolean {
        val translation = pose.translation
        return (
            abs(
                translation.x +
                    Constants.AlignConstants.RIGHT_X_OFFSET,
            ) <=
                Constants.AlignConstants.ALIGN_DEADZONE &&
                abs(
                    translation.z -
                        Constants.AlignConstants.RIGHT_Z_OFFSET,
                ) <=
                Constants.AlignConstants.ALIGN_DEADZONE
        )
    }

    private fun isRotationAligned(pose: Pose3d): Boolean =
        pose.rotation.z.absoluteValue <= Constants.AlignConstants.ALIGN_ROT_DEADZONE

    fun parseJson(json: String?): LimelightResults? {
        val checkedJson: String = json ?: return null
        val mapper: ObjectMapper =
            ObjectMapper().configure(
                DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES,
                false,
            )

        return try {
            mapper.readValue(
                checkedJson,
                LimelightResults::class.java,
            )
        } catch (e: Exception) {
            println("Failed to parse JSON: $e")
            null
        }
    }
}
