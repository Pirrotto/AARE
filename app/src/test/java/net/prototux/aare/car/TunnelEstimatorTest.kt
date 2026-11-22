package net.prototux.aare.car

import net.prototux.aare.car.TunnelEstimator.Fix
import net.prototux.aare.car.TunnelEstimator.Sensors
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

class TunnelEstimatorTest {
    private val lat = 46.0
    private val lon = 8.95
    private val driving = Sensors(speedMps = 20f, compassDeg = null, yawRadPerS = null)

    /** Drives north at 20 m/s for [seconds], one fix a second, ending at [endMs]. */
    private fun TunnelEstimator.driveNorth(seconds: Int, endMs: Long, sensors: Sensors = driving): Fix {
        var fix: Fix? = null
        for (i in 0 until seconds) {
            val (la, lo) = TunnelEstimator.destination(lat, lon, 0.0, 20.0 * i)
            fix = Fix(la, lo, 20f, 0f, endMs - (seconds - 1 - i) * 1000L)
            onFix(fix, sensors)
        }
        return fix!!
    }

    @Test
    fun nothingWhileTheGpsIsFresh() {
        val t = TunnelEstimator()
        t.driveNorth(5, endMs = 10_000)
        assertNull(t.tick(11_000, driving))
        assertNull(t.tick(12_000, driving))
    }

    @Test
    fun keepsMovingAlongTheLastBearingWhenTheGpsStops() {
        val t = TunnelEstimator()
        val last = t.driveNorth(5, endMs = 10_000)
        val e = t.tick(13_000, driving)
        assertNotNull(e)
        e!!
        // 3 s at 20 m/s, northwards.
        assertEquals(60.0, TunnelEstimator.distanceM(last.lat, last.lon, e.lat, e.lon), 1.0)
        assertTrue(e.lat > last.lat)
        assertEquals(last.lon, e.lon, 1e-6)
        val next = t.tick(14_000, driving)!!
        assertEquals(20.0, TunnelEstimator.distanceM(e.lat, e.lon, next.lat, next.lon), 0.5)
        assertTrue(t.estimating)
    }

    @Test
    fun noEstimateWithoutTheCarsSpeed() {
        val t = TunnelEstimator()
        t.driveNorth(5, endMs = 10_000)
        assertNull(t.tick(13_000, Sensors(null, null, null)))
    }

    @Test
    fun aRepeatedFixWhileDrivingIsDroppedOnceEstimating() {
        val t = TunnelEstimator()
        val last = t.driveNorth(5, endMs = 10_000)
        assertNotNull(t.tick(13_000, driving))
        assertTrue(t.onFix(last.copy(atMs = 13_500), driving))
    }

    @Test
    fun aNewFixEndsTheEstimate() {
        val t = TunnelEstimator()
        val last = t.driveNorth(5, endMs = 10_000)
        assertNotNull(t.tick(13_000, driving))
        val (la, lo) = TunnelEstimator.destination(last.lat, last.lon, 0.0, 80.0)
        assertFalse(t.onFix(Fix(la, lo, 20f, 0f, 14_000), driving))
        assertFalse(t.estimating)
        assertNull(t.tick(15_000, driving))
    }

    @Test
    fun followsAChangingCompass() {
        val t = TunnelEstimator()
        val last = t.driveNorth(5, endMs = 10_000)
        val e = t.tick(13_000, driving.copy(compassDeg = 90f))!!
        assertEquals(90f, e.bearingDeg, 0.01f)
        assertTrue(e.lon > last.lon)
    }

    @Test
    fun turnsWithTheGyroscopeOnceItsSignIsLearned() {
        val t = TunnelEstimator()
        // Turning right (clockwise) at ~5.7°/s while the car reports a negative yaw rate.
        val turnRate = Math.toRadians(5.7)
        val yaw = Sensors(20f, null, -turnRate.toFloat())
        var bearing = 0.0
        var la = lat
        var lo = lon
        for (i in 0 until 6) {
            t.onFix(Fix(la, lo, 20f, bearing.toFloat(), i * 1000L), yaw)
            val next = TunnelEstimator.destination(la, lo, bearing, 20.0)
            la = next.first; lo = next.second
            bearing += Math.toDegrees(turnRate)
        }
        val e1 = t.tick(8_000, yaw)!!
        val e2 = t.tick(9_000, yaw)!!
        // The estimate keeps turning clockwise, despite the gyroscope's negative sign.
        assertTrue(TunnelEstimator.wrap((e2.bearingDeg - e1.bearingDeg).toDouble()) > 4)
    }

    @Test
    fun givesUpAfterTheLongestGap() {
        val t = TunnelEstimator(maxMs = 60_000)
        t.driveNorth(5, endMs = 10_000)
        assertNull(t.tick(80_000, driving))
    }

    @Test
    fun wrapAndNormalize() {
        assertEquals(-170.0, TunnelEstimator.wrap(190.0), 1e-9)
        assertEquals(170.0, TunnelEstimator.wrap(-190.0), 1e-9)
        assertEquals(350.0, TunnelEstimator.normalize(-10.0), 1e-9)
    }
}
