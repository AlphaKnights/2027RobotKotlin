/*
 * (C) 2025 Galvaknights
 */
package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import com.pathplanner.lib.commands.PathPlannerAuto
import frc.robot.commands.*
import frc.robot.commands.autoalign.AutoAlignAutoCommand
import frc.robot.commands.intake.*
import frc.robot.subsystems.DriveSubsystem
import frc.robot.subsystems.LimelightSubsystem
import frc.robot.subsystems.Logger
import org.wpilib.command2.Command
import org.wpilib.command2.InstantCommand
import org.wpilib.command2.button.CommandJoystick
import org.wpilib.driverstation.GenericHID
import org.wpilib.tunable.*
import java.io.File


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 *
 * In Kotlin, it is recommended that all your Subsystems are Kotlin objects. As such, there
 * can only ever be a single instance. This eliminates the need to create reference variables
 * to the various subsystems in this container to pass into to commands. The commands can just
 * directly reference the (single instance of the) object.
 */
object RobotContainer {
    // private val joystickController = JoystickController()
    private val XboxController = XboxController()

    private val buttonBoard = CommandJoystick(Constants.OperatorConstants.BUTTON_BOARD_PORT)

    private val autoChooser = Selectable<PathPlannerAuto>()

    init {
        LimelightSubsystem.startPolling()
        NamedCommands.registerCommands(
            mapOf<String, Command>(
                "Left" to AutoAlignAutoCommand(Constants.AlignDirection.LEFT),
                "Right" to AutoAlignAutoCommand(Constants.AlignDirection.RIGHT),
                "Delivery" to AutoDeliveryCommand(Constants.LaunchConstants.LAUNCH_SPEED),
                "Supershoot" to AutoDeliveryCommand(Constants.LaunchConstants.ALT_LAUNCH_SPEED),
                "Intake_Lever_In" to IntakeLeverCommand(Constants.IntakeConstants.LEVER_IN_POSITION),
                "Intake_Lever_Out" to IntakeLeverCommand(Constants.IntakeConstants.LEVER_OUT_POSITION),
                "Intake" to IntakeCommand(false),
                "Indexer" to AutoStorageCommand(false),
                "Intake Manual In" to IntakeLeverManualCommand(Constants.IntakeDirection.IN),
                "Intake Manual Out" to IntakeLeverManualCommand(Constants.IntakeDirection.OUT),
            ),
        )
        configureAuto()
        configureBindings()
        Logger.initTelemetry()
    }

    private fun configureBindings() {
        // Drive control

        // x is forward
        DriveSubsystem.defaultCommand =
            DriveCommand(
                x = { XboxController.x() },
                y = { XboxController.y() },
                rot = { XboxController.rot() },
                autoAngle = {
                    // xBoxController.autoAim().asBoolean
                    false
                },
            )

        // Reset heading
        XboxController.heading().whileTrue(
            ResetHeadingCommand(),
        )
        // Auto Align
//        xBoxController.alignL().whileTrue(
//            AutoAlignManualCommand(
//                Constants.AlignDirection.LEFT,
//            ),
//        )
        //      xBoxController.alignR().whileTrue(
        //      AutoAlignManualCommand(
        //      Constants.AlignDirection.RIGHT,
        // ),
        // )

        XboxController
            .resetOdometry().whileTrue(
            ResetOdometry(),
        )

        XboxController
            .driveToArc()
            .onTrue(DriveToArcCommand())

//        xBoxController
//            .slideLeft()
//            .whileTrue(
//                DriveCommand(
//                    { 0.0 },
//                    {
//                        ArcSlidingCalc.getYChange(
//                            Constants.DriveConstants.MAX_ANGULAR_SPEED *
//                                Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE,
//                        )
//                    },
//                    {
//                        Constants.DriveConstants.MAX_ANGULAR_SPEED *
//                            Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE
//                    },
//                    { false },
//                    false,
//                ),
//            )
//
//        xBoxController
//            .slideRight()
//            .whileTrue(
//                DriveCommand(
//                    { 0.0 },
//                    {
//                        ArcSlidingCalc.getYChange(
//                            Constants.DriveConstants.MAX_ANGULAR_SPEED *
//                                Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE,
//                        )
//                    },
//                    {
//                        -Constants.DriveConstants.MAX_ANGULAR_SPEED *
//                            Constants.DriveConstants.MAX_SLIDING_SPEED_PERCENTAGE
//                    },
//                    { false },
//                    false,
//                ),
//            )

        XboxController.north()
            .whileTrue(
            NorthCommand(
                x = { XboxController.x() },
                y = { XboxController.y() },
            ),
        )

        XboxController.xLock()
            .whileTrue(
            LockXCommand(),
        )

        XboxController.altDelivery()
            .whileTrue(
            DeliveryCommand(Constants.LaunchConstants.ALT_LAUNCH_SPEED),
        )

        XboxController.altIntake().whileTrue(
            IntakeCommand(false),
        )
        buttonBoard.button(6).multiPress(2, 1.0).toggleOnTrue(
            InstantCommand({
                XboxController.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 1.0)
            }),
        )
        buttonBoard.button(6).multiPress(2, 1.0).toggleOnFalse(
            InstantCommand({
                XboxController.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0.0)
            }),
        )

        buttonBoard.button(7).whileTrue(
            SuperStorageCommand(false),
        )

        // Button Board
        buttonBoard.button(Constants.RollerConstants.BUTTON).whileTrue(
            StorageCommand(false),
        )
        buttonBoard.button(Constants.OperatorConstants.INDEXER_REVERSE_BUTTON).whileTrue(
            StorageCommand(true),
        )

        buttonBoard.button(Constants.OperatorConstants.DELIVERY_BUTTON).whileTrue(
            DeliveryCommand(Constants.LaunchConstants.LAUNCH_SPEED),
        )
        if (XboxController.deliveryScale() >= 0.5) { // Yo what fucking dumbass made this
            DeliveryCommand(XboxController.deliveryScale())
        }
        buttonBoard.button(Constants.OperatorConstants.DELIVERY_REVERSE_BUTTON).whileTrue(
            DeliveryCommand(-Constants.LaunchConstants.LAUNCH_SPEED),
        )

        buttonBoard
            .button(
                Constants.OperatorConstants.INTAKE_LEVER_IN_AUTO_BUTTON,
            ).onTrue(
                IntakeLeverCommand(
                    Constants.IntakeConstants.LEVER_IN_POSITION,
                ),
            )

        buttonBoard
            .button(
                Constants.OperatorConstants.INTAKE_LEVER_OUT_AUTO_BUTTON,
            ).onTrue(
                IntakeLeverCommand(
                    Constants.IntakeConstants.LEVER_OUT_POSITION,
                ),
            )

        buttonBoard
            .button(
                Constants.OperatorConstants.INTAKE_LEVER_IN_MANUAL_BUTTON,
            ).whileTrue(
                IntakeLeverManualCommand(
                    Constants.IntakeDirection.IN,
                ),
            )

        buttonBoard.button(6).multiPress(2, 1.0).toggleOnTrue(
            InstantCommand({
                XboxController.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 1.0)
            }),
        )
        buttonBoard.button(6).multiPress(2, 1.0).toggleOnFalse(
            InstantCommand({
                XboxController.setRumble(GenericHID.RumbleType.RIGHT_RUMBLE, 0.0)
            }),
        )

        buttonBoard
            .button(
                Constants.OperatorConstants.INTAKE_LEVER_OUT_MANUAL_BUTTON,
            ).whileTrue(
                IntakeLeverManualCommand(
                    Constants.IntakeDirection.OUT,
                ),
            )

        buttonBoard.button(Constants.OperatorConstants.INTAKE_BUTTON).whileTrue(
            IntakeCommand(
                false,
            ),
        )
        buttonBoard.button(Constants.OperatorConstants.INTAKE_REVERSE_BUTTON).whileTrue(
            IntakeCommand(
                true,
            ),
        )
    }

    private fun configureAuto() {
        autoChooser.addDefault("Default", PathPlannerAuto("Auto Deliver Only"))
        val autoList =
            buildList {
                File("src/main/deploy/pathplanner/autos/").listFiles()?.forEach { auto ->
                    add(PathPlannerAuto(auto.name.toString().removeSuffix(".auto")))
                }
            }
        for (auto in autoList) {
            autoChooser.add(auto.name, auto)
        }
        autoChooser.publishTunable(Tunables.getTable())
    }

    fun getAutonomousCommand(): Command = autoChooser.selected
}
