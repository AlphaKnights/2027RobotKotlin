package frc.robot

import org.wpilib.math.util.MathUtil.applyDeadband
import org.wpilib.command2.button.CommandGamepad
import org.wpilib.command2.button.Trigger


/*
    leftstick          # (back)       = (start)                 Y (northFace)
                                                        X (westFace)    B (eastFace)
                  dpad                    rstick                A (southFace)
 */


class XboxController : CommandGamepad(Constants.OperatorConstants.DRIVER_CONTROLLER_PORT) {
    private var lerpX = 0.0
    private var lerpY = 0.0
    private var lerpRot = 0.0

    fun x(): Double {
        lerpX =
            lerp(
                -applyDeadband(
                    getRawAxis(1), // right y
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                lerpX,
            )
        return lerpX
    }

    fun y(): Double {
        lerpY =
            lerp(
                -applyDeadband(
                    getRawAxis(0), // right x
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                lerpY,
            )
        return lerpY
    }

    fun rot(): Double {
        lerpRot =
            lerp(
                -applyDeadband(
                    getRawAxis(4), // left x
                    Constants.OperatorConstants.DRIVE_DEADBAND,
                ) * speedScale(),
                lerpRot,
            )
        return lerpRot
    }

    fun speedScale(): Double = ((-getRightTriggerAxis() + 1))

    fun deliveryScale(): Double = getLeftTriggerAxis()

    fun heading(): Trigger = this.northFace()

    //   fun alignL() : Trigger {
    //      return Trigger { xButton }
    // }
    // fun alignR() : Trigger {
    //   return Trigger { bButton }
    // }
    fun autoAim(): Trigger = this.southFace()

    fun lerp(
        ref: Double,
        start: Double,
    ): Double {
        if (ref > start) {
            return if (start + Constants.OperatorConstants.LERP_VAL > ref) {
                ref
            } else {
                start + Constants.OperatorConstants.LERP_VAL
            }
        } else {
            return if (start - Constants.OperatorConstants.LERP_VAL < ref) {
                ref
            } else {
                start - Constants.OperatorConstants.LERP_VAL
            }
        }
    }

    fun resetOdometry(): Trigger = this.rightStick()

    fun driveToArc(): Trigger = this.start()

//    fun slideLeft(): Trigger {
//        return Trigger { leftBumperButton }
//    }
//
//    fun slideRight(): Trigger {
//        return Trigger { rightBumperButton }
//    }

    fun north(): Trigger {
        return this.back() // Select Button
    }

    fun XLock(): Trigger = this.leftBumper()

    fun altDelivery(): Trigger = this.southFace()

    fun altIntake(): Trigger = this.rightBumper()

    fun altIndexer(): Trigger = this.eastFace()

    fun shake(): Trigger = this.eastFace()
}
