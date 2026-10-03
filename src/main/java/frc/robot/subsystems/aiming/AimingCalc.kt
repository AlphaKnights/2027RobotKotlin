/*
 * (C) 2025 Galvaknights
 */
package frc.robot.subsystems.aiming

import frc.robot.Constants.AimingConstants
import org.wpilib.driverstation.Alliance
import org.wpilib.math.geometry.Pose2d
import org.wpilib.math.geometry.Translation2d
import org.wpilib.math.kinematics.ChassisVelocities
import org.wpilib.driverstation.DriverStation
import org.wpilib.driverstation.MatchState
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.*

object AimingCalc {
    fun canShoot(curPose: Pose2d): Double {
        // in meters
        val x: Double = -curPose.x
        val y: Double = curPose.y

        val hub = getHubPosition()
        val distanceX: Double = abs(x - hub.x)
        val distanceY: Double = abs(y - hub.y)
        val distanceTotal: Double = sqrt(distanceX.pow(2) + distanceY.pow(2))

        var distanceGood: Boolean =
            (
                AimingConstants.DISTANCE - AimingConstants.GOOD_DISTANCE_TOLERANCE <= distanceTotal &&
                    distanceTotal <= AimingConstants.DISTANCE + AimingConstants.GOOD_DISTANCE_TOLERANCE
            )
        var distanceMiddlingLow: Boolean =
            (
                AimingConstants.DISTANCE - AimingConstants.MIDDLING_DISTANCE_TOLERANCE <= distanceTotal &&
                    distanceTotal <= AimingConstants.DISTANCE &&
                    !distanceGood
            )
        var distanceMiddlingHigh: Boolean =
            (
                AimingConstants.DISTANCE <= distanceTotal &&
                    distanceTotal <= AimingConstants.DISTANCE + AimingConstants.MIDDLING_DISTANCE_TOLERANCE &&
                    !distanceGood
            )
        val distanceBadHigh: Boolean =
            (distanceTotal > AimingConstants.DISTANCE + AimingConstants.MIDDLING_DISTANCE_TOLERANCE)

        // to-do: integrate with dashboard

        if (distanceBadHigh) {
            return 1.0
        } else if (distanceMiddlingHigh) {
            return 2.0
        } else if (distanceGood) {
            return 3.0
        } else if (distanceMiddlingLow) {
            return 4.0
        }
        return 5.0
    }

    fun getAimingAngleChange(
        curPose: Pose2d,
        vx: Double,
        vy: Double,
    ): Double {
        // in meters and radians
        val x: Double = -curPose.x
        val y: Double = curPose.y
        val angle: Double = curPose.rotation.radians

        val distanceX: Double = AimingConstants.RED_HUB_X - x
        val distanceY: Double = AimingConstants.RED_HUB_Y - y

        // (-dx/dt(-rx) + dy/dt(-ry)) / (hx-rx)^2
        val termOne: Double = (vx - vy) / (distanceX.pow(2))

        // 1/(1+((hy-ry)/(hx-rx))^2)
        val termTwo: Double = 1 / (1 + (distanceY / distanceX).pow(2))

        // atan2(hy-ry,hx-rx)-angle
        val termThree: Double = atan2(distanceY, distanceX) - angle
        var angularDistance = 1.0
        if (abs(termThree) <= AimingConstants.SLOW_DISTANCE) {
            angularDistance = max(AimingConstants.MIN_SPEED, abs(termThree) / AimingConstants.SLOW_DISTANCE)
        }
        angularDistance = sqrt(angularDistance)

        return angularDistance * AimingConstants.MAX_SPEED + sign(x) * termOne * termTwo
    }

    // claude starts here beware
    // Returns true when targeting red hub, false for blue.
    // Reads DriverStation at runtime; falls back to the constant if DS hasn't set it.
    val allianceRed = (MatchState.getAlliance().get()) == Alliance.BLUE

    // Returns the hub's field position for the current alliance.
    private fun getHubPosition(): Translation2d =
        if (allianceRed) {
            Translation2d(AimingConstants.RED_HUB_X, AimingConstants.RED_HUB_Y)
        } else {
            Translation2d(AimingConstants.BLUE_HUB_X, AimingConstants.BLUE_HUB_Y)
        }

    // Returns (minAngle, maxAngle) in radians for the valid shooting-arc sector.
    // For blue alliance the sector is mirrored vertically (θ → −θ, then swap so min < max).
    //
    // Coordinate diagram (field, top view):
    //
    //        +Y
    //         │   ← Blue hub region  (robot above hub, θ ≈ 20°–160°)
    //  ───────●──────  hub  ── +X
    //         │   ← Red hub region   (robot below hub, θ ≈ 200°–340°)
    //        -Y
    private fun getAngleBounds(): Pair<Double, Double> {
        val minRad = Math.toRadians(AimingConstants.MIN_ANGLE_DEGREES)
        val maxRad = Math.toRadians(AimingConstants.MAX_ANGLE_DEGREES)
        return if (allianceRed) Pair(minRad, maxRad) else Pair(-maxRad, -minRad)
    }

    // Calculates field-relative ChassisVelocities to keep the robot on the shooting arc.
    //
    // What this does, in three parts:
    //   1. Radial correction — drives toward/away from the hub to maintain DISTANCE.
    //      Replaces the controller's Y input entirely.
    //   2. Tangential movement — controller X slides the robot along the arc.
    //      Clamped to zero at the MIN/MAX angle boundaries.
    //   3. Rotation — proportional feedback PLUS feedforward for translation velocity.
    //
    // Why a rotation feedforward?  As the robot moves tangentially, the angle it needs
    // to point to face the hub changes continuously.  If we only use proportional feedback
    // on the heading error, the robot will always lag slightly behind — the hub drifts out
    // of aim while moving.  The feedforward term computes exactly how fast that desired
    // heading is changing and pre-rotates to compensate, keeping the robot precisely on target.
    //
    // Coordinate conventions (looking down at the field):
    //
    //   theta = atan2(robot_y − hub_y,  robot_x − hub_x)
    //   radial unit vector    =  (cos θ,  sin θ)   ← points away from hub
    //   tangential unit vector = (−sin θ, cos θ)   ← 90° CCW from radial
    //
    //   Positive controllerX → CCW movement around the hub.
    //
    // ChassisVelocities axes (WPILib field-relative convention):
    //   vx → +X on field (toward red alliance wall)
    //   vy → +Y on field
    //   omega → positive = CCW
    fun getArcDriveSpeeds(
        curPose: Pose2d,
        controllerX: Double,
        currentSpeeds: ChassisVelocities,
    ): ChassisVelocities {
        val hub = getHubPosition()
        val dx = curPose.x - hub.x // robot_x − hub_x
        val dy = curPose.y - hub.y // robot_y − hub_y
        val distanceSq = dx.pow(2) + dy.pow(2)
        val distance = sqrt(distanceSq)

        // Guard: robot is exactly at hub centre — no defined direction, stop.
        if (distance < 0.1) return ChassisVelocities(0.0, 0.0, 0.0)

        val theta = atan2(dy, dx)
        val cosT = cos(theta)
        val sinT = sin(theta)

        // --- 1. Radial correction ---
        // distanceError > 0: robot too far → move inward (negative radial speed)
        val distanceError = distance - AimingConstants.DISTANCE
        val radialScale =
            if (abs(distanceError) > AimingConstants.SLOW_DISTANCE) {
                1.0
            } else if (abs(distanceError) < AimingConstants.DIST_DEADZONE) {
                0.0
            } else {
                max(AimingConstants.MIN_SPEED, abs(distanceError) / AimingConstants.SLOW_DISTANCE)
            }
        val radialSpeed = -sign(distanceError) * sqrt(radialScale) * AimingConstants.MAX_SPEED

        // --- 2. Tangential movement ---
        val (minAngle, maxAngle) = getAngleBounds()
        // Stop tangential input at angular boundaries so the robot can't leave the valid sector.
        val tangentialSpeed =
            when {
                theta <= minAngle && controllerX < 0.0 -> 0.0
                theta >= maxAngle && controllerX > 0.0 -> 0.0
                else -> controllerX * AimingConstants.MAX_SPEED
            }

        // Decompose radial + tangential into field-relative vx/vy.
        val vx = radialSpeed * cosT + tangentialSpeed * (-sinT)
        val vy = radialSpeed * sinT + tangentialSpeed * cosT

        // --- 3. Rotation: face the hub ---
        //
        // Proportional term: close the gap between current heading and direction-to-hub.
        val angleToHub = atan2(-dy, -dx) // direction from robot toward hub
        val currentAngle = curPose.rotation.radians
        // Wrap to (−π, π] so robot always turns the short way around.
        val angleDiff = (angleToHub - currentAngle) // + PI).mod(2 * PI) - PI
        val rotScale =
            if (abs(angleDiff) > AimingConstants.SLOW_DISTANCE) {
                1.0
            } else if (abs(angleDiff) < AimingConstants.ANGLE_DEADZONE) {
                0.0
            } else {
                max(AimingConstants.MIN_SPEED, abs(angleDiff) / AimingConstants.SLOW_DISTANCE)
            }
        val proportionalOmega = sign(angleDiff) * sqrt(rotScale) * AimingConstants.MAX_ANGULAR_SPEED

        // Feedforward term: d(angleToHub)/dt — how fast the desired heading changes as the
        // robot translates.  Derived from differentiating atan2(hy−ry, hx−rx) w.r.t. time:
        //
        //   d/dt atan2(hy−ry, hx−rx)  =  (vx·(hy−ry) − vy·(hx−rx)) / distance²
        //                              =  (−vx·dy + vy·dx) / distance²
        //
        // where vx/vy are the robot's current field-relative velocities.
        val feedforwardOmega =
            (
                -currentSpeeds.vx * dy +
                    currentSpeeds.vy * dx
            ) / distanceSq
            (-currentSpeeds.vxMetersPerSecond * dy + currentSpeeds.vyMetersPerSecond * dx) / distanceSq

        return ChassisVelocities(vx, vy, proportionalOmega + feedforwardOmega)
    }
}
