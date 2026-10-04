package frc.robot

import org.wpilib.math.util.MathUtil.applyDeadband
import org.wpilib.command2.button.CommandGamepad
import org.wpilib.command2.button.Trigger
import org.wpilib.math.util.MathUtil.lerp


class XboxController : CommandGamepad(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT) {
    private var lerpX = 0.0
    private var lerpY = 0.0
    private var lerpRot = 0.0

    fun x(): Double {
        lerpX =
            lerp(
                lerpX,
                -applyDeadband(
                    rightY, // right y
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                Constants.OperatorConstants.LERP_VAL
            )
        return lerpX
    }

    fun y(): Double {
        lerpY =
            lerp(
                lerpY,
                -applyDeadband(
                    rightX, // right x
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                Constants.OperatorConstants.LERP_VAL
            )
        return lerpY
    }

    fun rot(): Double {
        lerpRot =
            lerp(
                lerpRot,
                -applyDeadband(
                    leftX, // left x
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                Constants.OperatorConstants.LERP_VAL
            )
        return lerpRot
    }

    fun speedScale(): Double = ((rightTrigger + 1))

    fun deliveryScale(): Double = leftTrigger

    fun heading(): Trigger = faceUp()

    //   fun alignL() : Trigger {
    //      return Trigger { xButton }
    // }
    // fun alignR() : Trigger {
    //   return Trigger { bButton }
    // }
    fun autoAim(): Trigger = faceDown()



    fun resetOdometry(): Trigger = rightStick()

    fun driveToArc(): Trigger = start()

//    fun slideLeft(): Trigger {
//        return Trigger { leftBumperButton }
//    }
//
//    fun slideRight(): Trigger {
//        return Trigger { rightBumperButton }
//    }

    fun north(): Trigger {
        return back() // Select Button
    }

    fun xLock(): Trigger = leftBumper()

    fun altDelivery(): Trigger = faceDown()

    fun altIntake(): Trigger = rightBumper()

    fun altIndexer(): Trigger = faceRight()

    fun shake(): Trigger = faceRight()
}
