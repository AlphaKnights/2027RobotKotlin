/*
 * (C) 2025 Galvaknights
 */
package frc.robot.limelight

import frc.robot.interfaces.LimelightService
import frc.robot.subsystems.LimelightSubsystem
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.joinAll
import kotlinx.coroutines.launch
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.TestCoroutineScheduler
import kotlinx.coroutines.test.UnconfinedTestDispatcher
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.runTest
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.mockito.Mockito.mock
import org.mockito.Mockito.reset
import org.mockito.kotlin.whenever
import org.wpilib.math.geometry.Pose3d

@Suppress("LongMethod")
internal class LimelightTest {
    private val mockLimelightService: LimelightService =
        mock()
    private lateinit var originalService: LimelightService
    private val testScheduler = TestCoroutineScheduler()
    private val testDispatcher =
        StandardTestDispatcher(testScheduler)

    @BeforeEach
    fun setup() {
        originalService =
            LimelightSubsystem.limelightService
        LimelightSubsystem.limelightService =
            mockLimelightService // Mock first!
        LimelightSubsystem.coroutineScope =
            CoroutineScope(testDispatcher + SupervisorJob())
        LimelightSubsystem.startPolling()
    }

    @AfterEach
    fun tearDown() {
        LimelightSubsystem.limelightService =
            originalService
        LimelightSubsystem.coroutineScope.cancel()
        reset(mockLimelightService)
    }

    @OptIn(ExperimentalCoroutinesApi::class)
    @Test
    fun `updateTagPosition sets tagPose correctly when Limelight has no target`() =
        runTest {
            val validEmptyJson = """
            {
            "Classifier": [],
            "Detector": [],
            "Fiducial": [],
            "Retro": [],
            "pID": 0,
            "tl": 19.78130340576172,
            "ts": 3284447.910569,
            "v": 1
        }
        """

            whenever(
                mockLimelightService.fetchResults(),
            ).thenReturn(validEmptyJson)

            advanceUntilIdle()
            val result =
                LimelightSubsystem
                    .updateTagPosition()

            assertNull(result)
        }

    @Test
    fun `updateTagPosition sets tagPose correctly when Limelight has target`() =
        runBlocking {
            val validJson = """
            {
            "Classifier": [],
            "Detector": [],
            "Fiducial": [
            {
                "fID": 2,
                "fam": "16H5C",
                "pts": [],
                "skew": [],
                "t6c_ts": [
                0.33247368976801916,
                -0.05672695778305914,
                -2.5042031405987144,
                -4.680849607956358,
                -5.171154989721864,
                4.528697946312339
                ],
                "t6r_fs": [
                4.738896418276903,
                -1.5926603672041666,
                0.5194469577830592,
                4.522658587661256,
                4.258580454853879,
                5.5236539893713275
                ],
                "t6r_ts": [
                0.33247368976801916,
                -0.05672695778305914,
                -2.5042031405987144,
                -4.680849607956358,
                -5.171154989721864,
                4.528697946312339
                ],
                "t6t_cs": [
                -0.09991902572799474,
                -0.1234042720218289,
                2.5218203039582496,
                4.278368708252767,
                5.508508005282244,
                -4.1112864453027775
                ],
                "t6t_rs": [
                -0.09991902572799474,
                -0.1234042720218289,
                2.5218203039582496,
                4.278368708252767,
                5.508508005282244,
                -4.1112864453027775
                ],
                "ta": 0.005711808800697327,
                "tx": -2.0525293350219727,
                "txp": 149.4874725341797,
                "ty": 2.7294836044311523,
                "typ": 107.14710235595703
            }
            ],
            "Retro": [],
            "pID": 0,
            "tl": 19.78130340576172,
            "ts": 3284447.910569,
            "v": 1
        }
        """
            whenever(
                mockLimelightService.fetchResults(),
            ).thenReturn(validJson)
            LimelightSubsystem.limelightService =
                mockLimelightService

            val tagPose =
                LimelightSubsystem
                    .updateTagPosition()
            assert(tagPose != null)
            assertEquals(-0.09991902572799474, tagPose!!.x)
        }

    @OptIn(ExperimentalCoroutinesApi::class)
    @Test
    fun `mutex prevents race conditions in tagPose updates`() =
        runTest {
            val testScope =
                CoroutineScope(
                    UnconfinedTestDispatcher(testScheduler),
                )
            LimelightSubsystem.coroutineScope = testScope

            val numCoroutines = 100
            val jobs =
                List(numCoroutines) { idx ->
                    testScope.launch {
                        repeat(1000) {
                            LimelightSubsystem.setTagPose(
                                Pose3d(
                                    idx.toDouble(),
                                    0.0,
                                    0.0,
                                    null,
                                ),
                            )
                        }
                    }
                }

            testScheduler.advanceUntilIdle() // Execute all coroutines instantly
            jobs.joinAll()

            // Verify final state is consistent
            val finalPose = LimelightSubsystem.tagPose
            assertNotNull(finalPose)
            assertEquals(numCoroutines - 1.0, finalPose?.x) // Last write wins
        }

    @OptIn(ExperimentalCoroutinesApi::class)
    @Test
    fun `mutex ensures atomicity under worst-case scheduling`() =
        runTest {
            val testDispatcher =
                UnconfinedTestDispatcher(testScheduler)
            val testScope = CoroutineScope(testDispatcher)
            LimelightSubsystem.coroutineScope = testScope

            // Launch two coroutines that will interleave execution
            testScope.launch {
                LimelightSubsystem.setTagPose(
                    Pose3d(1.0, 0.0, 0.0, null),
                )
            }

            testScope.launch {
                LimelightSubsystem.setTagPose(
                    Pose3d(2.0, 0.0, 0.0, null),
                )
            }

            testScheduler.advanceUntilIdle() // Force interleaved execution
            assertEquals(2.0, LimelightSubsystem.tagPose?.x) // Last writer wins
        }

//    As of now, this test won't be used since we don't want to cancel the coroutine scope
//    @Test
//    fun `coroutineScope cancels when fetchResults returns null`() = runTest(testScheduler) {
//        mock<LimelightService>().apply {
//            whenever(fetchResults()).thenReturn(null)
//        }
//
//        // Capture the exception
//        val exception = assertThrows<CancellationException> {
//            LimelightSubsystem.updateTagPosition()
//            testScheduler.advanceUntilIdle()
//        }
//
//        // Verify cancellation details
//        assertEquals("Limelight returned null results", exception.message)
//        assertFalse(LimelightSubsystem.coroutineScope.isActive)
//        assertNull(LimelightSubsystem.tagPose)
//    }
}
