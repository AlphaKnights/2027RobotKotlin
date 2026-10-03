/*
 * (C) 2025 Galvaknights
 */
package frc.robot.limelight

import frc.robot.Constants.AlignConstants
import frc.robot.subsystems.LimelightSubsystem
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import org.wpilib.math.geometry.Pose3d
import org.wpilib.math.geometry.Rotation3d

internal class IsAlignedTest {
    @Test
    fun `isAligned returns true when april tag on different sides of robot`() =
        runTest {
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET,
                    0.0,
                    AlignConstants.LEFT_Z_OFFSET,
                    Rotation3d(
                        0.0,
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                    ),
                ),
            )
            assertEquals(
                true,
                LimelightSubsystem.isAligned(),
            )

            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET,
                    0.0,
                    AlignConstants.RIGHT_Z_OFFSET,
                    Rotation3d(
                        0.0,
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                    ),
                ),
            )
            assertEquals(
                true,
                LimelightSubsystem.isAligned(),
            )
        }

    @Test
    fun `isAligned returns false when the x position of the tag is beyond the range`() =
        runTest {
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET +
                        AlignConstants.ALIGN_DEADZONE +
                        0.1,
                    AlignConstants.LEFT_Z_OFFSET +
                        AlignConstants.ALIGN_DEADZONE +
                        0.1,
                    0.0,
                    Rotation3d(
                        0.0,
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(),
            )

            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET -
                        AlignConstants.ALIGN_DEADZONE -
                        0.1,
                    AlignConstants.RIGHT_Z_OFFSET -
                        AlignConstants.ALIGN_DEADZONE -
                        0.1,
                    0.0,
                    Rotation3d(
                        0.0,
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(),
            )
        }

    @Test
    fun `isAligned returns false when the z position of the tag is beyond the range`() =
        runTest {
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET +
                        AlignConstants.ALIGN_DEADZONE +
                        0.1,
                    AlignConstants.LEFT_Z_OFFSET +
                        AlignConstants.ALIGN_DEADZONE +
                        0.1,
                    0.0,
                    Rotation3d(
                        0.0,
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(),
            )

            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET -
                        AlignConstants.ALIGN_DEADZONE -
                        0.1,
                    AlignConstants.RIGHT_Z_OFFSET -
                        AlignConstants.ALIGN_DEADZONE -
                        0.1,
                    0.0,
                    Rotation3d(
                        0.0,
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(),
            )
        }

    @Test
    fun `isAligned returns false when the rotation of the tag is beyond the range`() =
        runTest {
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET,
                    AlignConstants.LEFT_Z_OFFSET,
                    0.0,
                    Rotation3d(
                        0.0,
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE +
                            0.1,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(),
            )

            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET,
                    AlignConstants.RIGHT_Z_OFFSET,
                    0.0,
                    Rotation3d(
                        0.0,
                        0.0,
                        -AlignConstants.ALIGN_ROT_DEADZONE -
                            0.1,
                    ),
                ),
            )
            assertEquals(
                false,
                LimelightSubsystem.isAligned(),
            )
        }

    @Test
    fun `isAligned returns true when the rotation of the tag is within the range`() =
        runTest {
            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.LEFT_X_OFFSET,
                    0.0,
                    AlignConstants.LEFT_Z_OFFSET,
                    Rotation3d(
                        0.0,
                        0.0,
                        AlignConstants.ALIGN_ROT_DEADZONE,
                    ),
                ),
            )
            assertEquals(
                true,
                LimelightSubsystem.isAligned(),
            )

            LimelightSubsystem.setTagPose(
                Pose3d(
                    AlignConstants.RIGHT_X_OFFSET,
                    0.0,
                    AlignConstants.RIGHT_Z_OFFSET,
                    Rotation3d(
                        0.0,
                        0.0,
                        -AlignConstants.ALIGN_ROT_DEADZONE,
                    ),
                ),
            )
            assertEquals(
                true,
                LimelightSubsystem.isAligned(),
            )
        }
}
