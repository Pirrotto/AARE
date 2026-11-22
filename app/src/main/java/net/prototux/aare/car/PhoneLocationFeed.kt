package net.prototux.aare.car

import android.annotation.SuppressLint
import android.app.AppOpsManager
import android.content.Context
import android.location.Criteria
import android.location.Location
import android.location.LocationManager
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.os.Process
import android.os.SystemClock
import android.util.Log
import net.prototux.aare.Settings

/**
 * Feeds the car's GPS to the phone's own location providers while connected, so every app
 * (CoMaps, Google Maps…) sees one position. Otherwise a navigation app mixes the car's position,
 * which AARE gives car apps, with the phone's own fix, and the map jumps between the two.
 *
 * Android only allows it for the app chosen as "mock location app" in Developer options.
 *
 * In tunnels ([Settings.tunnelNavigation]) the position keeps moving: [TunnelEstimator] works it
 * out from the car's speed, compass and gyroscope, once a second, until the GPS is back.
 */
class PhoneLocationFeed(context: Context) {
    private val settings = Settings(context)
    private val tunnel = TunnelEstimator()
    private var ticker: Handler? = null
    private val lm = context.getSystemService(LocationManager::class.java)
    private val providers = buildList {
        add(LocationManager.GPS_PROVIDER)
        add(LocationManager.NETWORK_PROVIDER)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) add(LocationManager.FUSED_PROVIDER)
    }
    private val active = HashSet<String>()
    @Volatile private var failed = false

    /** A new car fix (any thread). */
    @Synchronized
    fun push(car: Location) {
        if (failed) return
        if (settings.tunnelNavigation) {
            startTicker()
            val fix = TunnelEstimator.Fix(car.latitude, car.longitude, car.speed.takeIf { car.hasSpeed() },
                car.bearing.takeIf { car.hasBearing() }, car.elapsedRealtimeNanos / 1_000_000)
            if (tunnel.onFix(fix, sensors())) return
        }
        publish(car)
    }

    private fun publish(car: Location) {
        if (active.isEmpty()) start()
        for (p in active) {
            val l = Location(car).apply {
                provider = p
                if (!hasAccuracy()) accuracy = 5f
                time = System.currentTimeMillis()
                elapsedRealtimeNanos = SystemClock.elapsedRealtimeNanos()
            }
            runCatching { lm.setTestProviderLocation(p, l) }.onFailure { Log.w(TAG, "setTestProviderLocation $p", it) }
        }
    }

    // The boolean form works on every Android version; Criteria's POWER_LOW and ACCURACY_FINE have
    // the same values as the ProviderProperties constants lint expects.
    @Suppress("DEPRECATION")
    @SuppressLint("WrongConstant")
    private fun start() {
        for (p in providers) {
            runCatching {
                lm.addTestProvider(p, false, false, false, false, true, true, true, Criteria.POWER_LOW, Criteria.ACCURACY_FINE)
                lm.setTestProviderEnabled(p, true)
                active += p
            }.onFailure { Log.w(TAG, "addTestProvider $p", it) }
        }
        // Not the mock location app: don't keep trying for every fix.
        if (active.isEmpty()) failed = true
    }

    @Synchronized
    fun stop() {
        ticker?.looper?.quitSafely()
        ticker = null
        tunnel.reset()
        for (p in active) runCatching { lm.removeTestProvider(p) }
        active.clear()
        failed = false
    }

    private fun startTicker() {
        if (ticker != null) return
        val thread = HandlerThread("aare-tunnel").apply { start() }
        val handler = Handler(thread.looper)
        ticker = handler
        val tick = object : Runnable {
            override fun run() {
                runCatching { tick() }.onFailure { Log.w(TAG, "tunnel", it) }
                if (ticker === handler) handler.postDelayed(this, 1000)
            }
        }
        handler.post(tick)
    }

    @Synchronized
    private fun tick() {
        if (failed || !settings.tunnelNavigation || !CarData.state.value.connected) return
        val wasEstimating = tunnel.estimating
        val e = tunnel.tick(SystemClock.elapsedRealtime(), sensors()) ?: return
        if (!wasEstimating) Log.i(TAG, "no GPS while moving: estimating from the last fix")
        publish(Location(LocationManager.GPS_PROVIDER).apply {
            latitude = e.lat
            longitude = e.lon
            speed = e.speedMps
            bearing = e.bearingDeg
            accuracy = e.accuracyM
        })
    }

    /** The car's speed (else the mean of its wheel speeds), compass and yaw rate, if sent in the last two seconds. */
    private fun sensors(): TunnelEstimator.Sensors {
        val s = CarData.state.value
        val now = System.currentTimeMillis()
        fun <T> CarData.Timed<T>?.fresh(): T? = this?.takeIf { now - it.atMillis <= 2_000 }?.value
        val speed = s.speed.fresh()?.metersPerSecond?.let { kotlin.math.abs(it) }
            ?: s.deadReckoning.fresh()?.wheelSpeedsMps?.takeIf { it.isNotEmpty() }?.let { kotlin.math.abs(it.average().toFloat()) }
        return TunnelEstimator.Sensors(speed, s.compass.fresh()?.bearingDeg, s.gyroscope.fresh()?.z)
    }

    companion object {
        private const val TAG = "AareLocation"

        /** AARE is the phone's mock location app (Developer options > Select mock location app). */
        fun allowed(context: Context): Boolean = runCatching {
            val ops = context.getSystemService(AppOpsManager::class.java)
            val mode = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) ops.unsafeCheckOpNoThrow(AppOpsManager.OPSTR_MOCK_LOCATION, Process.myUid(), context.packageName)
                else @Suppress("DEPRECATION") ops.checkOpNoThrow(AppOpsManager.OPSTR_MOCK_LOCATION, Process.myUid(), context.packageName)
            mode == AppOpsManager.MODE_ALLOWED
        }.getOrDefault(false)
    }
}
