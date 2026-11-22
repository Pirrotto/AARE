package net.prototux.aare.car

import kotlin.math.abs
import kotlin.math.asin
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Keeps the position moving in tunnels when the phone gets the car's GPS ([PhoneLocationFeed]).
 * In a tunnel the car has no fix: the position stops (or the car repeats the last one) and the
 * navigation app stops with it. The car still sends its speed, compass and gyroscope, so from the
 * last good fix this estimates where the car is (dead reckoning), once a second, until the GPS is
 * back.
 *
 * Heading: the car's compass while it changes (a compass the car derives from GPS freezes in
 * tunnels), else the last heading turned by the gyroscope's yaw rate. Cars mount the gyroscope
 * differently, so its sign is learned from the GPS bearing while driving.
 *
 * Plain logic with no Android types, so it can be tested on the JVM. Not thread-safe: the caller
 * serializes [onFix] and [tick].
 */
class TunnelEstimator(
    /** No new GPS position for this long while moving: the estimate starts. */
    private val staleMs: Long = 2_500,
    /** At most this long without GPS: past it, an estimate is too far off to help. */
    private val maxMs: Long = 15 * 60_000L,
    private val movingMps: Float = 2f,
) {
    /** A car fix; [atMs] on a monotonic clock (elapsedRealtime). */
    data class Fix(val lat: Double, val lon: Double, val speedMps: Float?, val bearingDeg: Float?, val atMs: Long)

    /** The car's sensors right now, each null when the car hasn't sent it recently. */
    data class Sensors(val speedMps: Float?, val compassDeg: Float?, val yawRadPerS: Float?)

    data class Estimate(val lat: Double, val lon: Double, val speedMps: Float, val bearingDeg: Float, val accuracyM: Float)

    private var lastFix: Fix? = null
    private var lastFixAt = 0L
    private var lastMovedAt = 0L

    private var estimate: Estimate? = null
    private var estimateAt = 0L
    private var bearing = Double.NaN
    private var lastCompass: Float? = null

    // Gyroscope sign, learned from GPS bearings: a running count of agreeing yaw and bearing turns.
    private var gyroCorrelation = 0.0
    private var previousFix: Fix? = null

    /** True while an estimate is running (the GPS is gone). */
    val estimating: Boolean get() = estimate != null

    /**
     * A new car fix. Returns true to drop it: a repeat of the last position while driving, which
     * some cars send in tunnels instead of nothing, so the estimate isn't pulled back.
     */
    fun onFix(fix: Fix, sensors: Sensors): Boolean {
        val last = lastFix
        val sameSpot = last != null && distanceM(fix.lat, fix.lon, last.lat, last.lon) < 1.0
        if (sameSpot && moving(sensors)) return estimate != null

        learnGyroSign(fix, sensors.yawRadPerS)
        lastFix = fix
        lastFixAt = fix.atMs
        if (!sameSpot) lastMovedAt = fix.atMs
        estimate = null
        if (fix.bearingDeg != null && fix.speedMps != null && fix.speedMps > movingMps) bearing = fix.bearingDeg.toDouble()
        return false
    }

    /** Once a second: the estimated position, or null when there's nothing to add to the GPS. */
    fun tick(nowMs: Long, sensors: Sensors): Estimate? {
        val last = lastFix ?: return null
        val sinceFix = nowMs - max(lastFixAt, lastMovedAt)
        if (sinceFix < staleMs || nowMs - lastMovedAt > maxMs) return null
        val speed = sensors.speedMps ?: return null

        var current = estimate
        if (current == null) {
            if (bearing.isNaN() && last.bearingDeg != null) bearing = last.bearingDeg.toDouble()
            if (bearing.isNaN()) return null
            current = Estimate(last.lat, last.lon, speed, bearing.toFloat(), 5f)
            estimate = current
            estimateAt = lastMovedAt
        }

        val dt = (nowMs - estimateAt) / 1000.0
        estimateAt = nowMs
        if (dt <= 0 || dt > 5) return null

        val compass = movingCompass(sensors.compassDeg)
        val yaw = sensors.yawRadPerS
        if (compass != null) {
            bearing = compass.toDouble()
        } else if (yaw != null && abs(gyroCorrelation) > 2) {
            bearing = normalize(bearing + sign(gyroCorrelation) * Math.toDegrees(yaw.toDouble()) * dt)
        }

        val (lat, lon) = destination(current.lat, current.lon, bearing, speed * dt)
        val travelled = speed * (nowMs - lastMovedAt) / 1000.0
        val next = Estimate(lat, lon, speed, bearing.toFloat(), min(200.0, 5.0 + travelled * 0.03).toFloat())
        estimate = next
        return next
    }

    /** Forgets everything: a new drive. */
    fun reset() {
        lastFix = null
        previousFix = null
        estimate = null
        bearing = Double.NaN
        lastCompass = null
        lastFixAt = 0
        lastMovedAt = 0
    }

    private fun moving(s: Sensors) = (s.speedMps ?: 0f) > movingMps

    /** The car's compass, only while it changes or agrees with our heading. */
    private fun movingCompass(deg: Float?): Float? {
        if (deg == null) return null
        val prev = lastCompass
        lastCompass = deg
        if (prev != null && abs(wrap((deg - prev).toDouble())) < 0.01 && abs(wrap(deg - bearing)) > 3) return null
        return deg
    }

    /** Learns which way the gyroscope turns, from the GPS bearing while driving. */
    private fun learnGyroSign(fix: Fix, yaw: Float?) {
        val prev = previousFix
        previousFix = fix
        if (prev == null || fix.bearingDeg == null || prev.bearingDeg == null || (fix.speedMps ?: 0f) < 5f) return
        val dt = (fix.atMs - prev.atMs) / 1000.0
        if (dt <= 0.2 || dt > 3) return
        val rate = Math.toRadians(wrap((fix.bearingDeg - prev.bearingDeg).toDouble())) / dt // clockwise, rad/s
        if (yaw == null || abs(rate) < 0.03) return
        gyroCorrelation = (gyroCorrelation + sign(rate * yaw)).coerceIn(-50.0, 50.0)
    }

    companion object {
        private const val EARTH_M = 6_371_000.0

        /** The point [meters] from ([lat], [lon]) towards [bearingDeg]. */
        fun destination(lat: Double, lon: Double, bearingDeg: Double, meters: Double): Pair<Double, Double> {
            val lat1 = Math.toRadians(lat)
            val lon1 = Math.toRadians(lon)
            val b = Math.toRadians(bearingDeg)
            val d = meters / EARTH_M
            val lat2 = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(b))
            val lon2 = lon1 + atan2(sin(b) * sin(d) * cos(lat1), cos(d) - sin(lat1) * sin(lat2))
            return Math.toDegrees(lat2) to Math.toDegrees(lon2)
        }

        /** Great-circle distance in meters. */
        fun distanceM(lat1: Double, lon1: Double, lat2: Double, lon2: Double): Double {
            val dLat = Math.toRadians(lat2 - lat1)
            val dLon = Math.toRadians(lon2 - lon1)
            val a = sin(dLat / 2) * sin(dLat / 2) + cos(Math.toRadians(lat1)) * cos(Math.toRadians(lat2)) * sin(dLon / 2) * sin(dLon / 2)
            return 2 * EARTH_M * asin(min(1.0, sqrt(a)))
        }

        /** Degrees into (-180, 180]. */
        fun wrap(deg: Double): Double {
            var d = deg % 360
            if (d > 180) d -= 360
            if (d < -180) d += 360
            return d
        }

        fun normalize(deg: Double): Double {
            val d = deg % 360
            return if (d < 0) d + 360 else d
        }
    }
}
