// State and bus work of the field check app. Every bus call runs on one
// background thread, so requests never interleave.
//
// SPDX-License-Identifier: MIT
package com.openrobot.motorcheck

import android.app.Application
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.openrobot.motorcheck.can.MotorBus
import com.openrobot.motorcheck.can.NoReply
import com.openrobot.motorcheck.can.SlcanTransport
import com.openrobot.motorcheck.protocol.Protocol
import kotlinx.coroutines.Job
import kotlinx.coroutines.asCoroutineDispatcher
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.update
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.util.concurrent.Executors

data class Live(
    val status: Protocol.Status? = null,
    val multiTurnDeg: Double? = null,
    val mode: String = "",
    val mitPosDeg: Double? = null,        // MIT hold: output position from the hold point
)

enum class JogMode(val label: String) {
    DPS("DPS"), ERPM("eRPM"), POSITION("Position"), CURRENT("Current"), MIT("MIT hold")
}

data class Details(
    val faults: List<Int> = emptyList(),
    val limits: Protocol.Limits? = null,
    val mit: Protocol.MitParams? = null,
)

data class UiState(
    val adapter: String? = null,          // connected adapter, null = not connected
    val busy: String? = null,             // long operation in progress
    val motors: List<Int> = emptyList(),
    val scanned: Boolean = false,
    val selected: Int? = null,
    val live: Live = Live(),
    val details: Details = Details(),
    val jogArmed: Boolean = false,        // user confirmed the shaft is free
    val jogMode: JogMode = JogMode.DPS,
    val jogDps: Float = 360f,             // DPS mode, motor side
    val jogErpm: Float = 1000f,           // eRPM mode
    val stepDeg: Float = 10f,             // position mode, output side per step
    val posDps: Float = 360f,             // position mode speed limit, motor side
    val currentA: Float = 1f,             // current mode
    val mitKp: Float = 5f,                // MIT hold, Nm/rad
    val mitKd: Float = 0.2f,              // MIT hold, Nm s/rad
    val mitPDeg: Float = 0f,              // MIT targets, output side, from the hold point
    val mitV: Float = 0f,                 // rad/s
    val mitT: Float = 0f,                 // feed-forward, Nm
    val jogging: Int = 0,                 // held button: -1, 0, +1
    val mitActive: Boolean = false,
    val messages: List<String> = emptyList(),
)

class CheckViewModel(app: Application) : AndroidViewModel(app) {
    private val _state = MutableStateFlow(UiState())
    val state: StateFlow<UiState> = _state

    private val busThread = Executors.newSingleThreadExecutor { Thread(it, "can-bus") }.asCoroutineDispatcher()
    private var transport: SlcanTransport? = null
    private var bus: MotorBus? = null
    private var pollJob: Job? = null
    private var jogJob: Job? = null

    private fun say(msg: String) = _state.update { it.copy(messages = (it.messages + msg).takeLast(20)) }
    fun clearMessages() = _state.update { it.copy(messages = emptyList()) }

    // ------------------------------------------------------------ connection --

    fun connect(driver: UsbSerialDriver, usb: UsbManager) {
        viewModelScope.launch(busThread) {
            disconnectNow()
            try {
                val port = driver.ports[0]
                port.open(usb.openDevice(driver.device) ?: throw IllegalStateException("USB permission missing"))
                val t = SlcanTransport(port, Protocol.DEFAULT_BITRATE)
                transport = t
                bus = MotorBus(t).also { b -> b.onFault = { id, code -> say("Motor $id: fault $code, ${Protocol.faultName(code)}") } }
                _state.update { it.copy(adapter = deviceName(driver.device)) }
                say("Connected to ${deviceName(driver.device)}, 1 Mbit/s")
                scanNow()
            } catch (e: Exception) {
                say("Cannot open the adapter: ${e.message}")
                disconnectNow()
            }
        }
    }

    fun disconnect() {
        viewModelScope.launch(busThread) { disconnectNow() }
    }

    private fun disconnectNow() {
        stopJobs()
        releaseSelectedNow()
        transport?.close()
        transport = null
        bus = null
        _state.update { UiState(jogDps = it.jogDps, messages = it.messages) }
    }

    private fun deviceName(d: UsbDevice) =
        (d.productName ?: "USB serial") + " (%04X:%04X)".format(d.vendorId, d.productId)

    // ------------------------------------------------------------ scan, select --

    fun scan() {
        viewModelScope.launch(busThread) { scanNow() }
    }

    private fun scanNow() {
        val b = bus ?: return
        stopJobs()
        if (_state.value.jogArmed) releaseSelectedNow()
        _state.update { it.copy(busy = "Scanning IDs 1..32", jogArmed = false) }
        val ids = try { b.scan() } catch (e: Exception) { say("Scan failed: ${e.message}"); emptyList() }
        _state.update { it.copy(busy = null, motors = ids, scanned = true, selected = null, live = Live(), details = Details()) }
        if (ids.isEmpty()) say("No motor answered. Check power, wiring, termination and the bit rate (1 Mbit/s).")
        else select(ids.first())
    }

    fun select(id: Int) {
        val prev = _state.value.selected
        // A jogged motor may still be turning, holding or in MIT mode: release it first.
        if (prev != null && prev != id && _state.value.jogArmed) viewModelScope.launch(busThread) {
            try { bus?.request(prev, Protocol.request(Protocol.MIT_EXIT)) } catch (_: Exception) {}
            try { bus?.request(prev, Protocol.motorOff()) } catch (_: Exception) {}
        }
        stopJobs()
        _state.update { it.copy(selected = id, live = Live(), details = Details(), jogArmed = false) }
        viewModelScope.launch(busThread) { readDetailsNow(id) }
        startPolling(id)
    }

    fun refreshDetails() {
        val id = _state.value.selected ?: return
        viewModelScope.launch(busThread) { readDetailsNow(id) }
    }

    private fun readDetailsNow(id: Int) {
        val b = bus ?: return
        fun <T> tryRead(f: () -> T): T? = try { f() } catch (_: NoReply) { null }
        val faults = tryRead { Protocol.decodeFaults(b.request(id, Protocol.request(Protocol.READ_FAULTS))) }
        val limits = tryRead { Protocol.decodeLimits(b.request(id, Protocol.request(Protocol.READ_LIMITS))) }
        val mit = tryRead { Protocol.decodeMitParams(b.request(id, Protocol.request(Protocol.READ_MIT_PARAMS))) }
        if (faults == null && limits == null && mit == null) say("Motor $id does not answer")
        _state.update { if (it.selected == id) it.copy(details = Details(faults ?: emptyList(), limits, mit)) else it }
    }

    // Status, mode and multi-turn angle at 10 Hz while nothing else runs.
    private fun startPolling(id: Int) {
        pollJob = viewModelScope.launch(busThread) {
            var missed = 0
            while (isActive) {
                val b = bus ?: break
                try {
                    val s = Protocol.decodeStatus(b.request(id, Protocol.request(Protocol.READ_STATUS)))
                    val ph = Protocol.decodePhases(b.request(id, Protocol.request(Protocol.READ_PHASES)))
                    val mt = Protocol.decodeMultiTurn(b.request(id, Protocol.request(Protocol.READ_MULTI_TURN)))
                    missed = 0
                    _state.update { it.copy(live = Live(s, mt, ph.controlModeName)) }
                } catch (_: NoReply) {
                    if (++missed == 3) say("Motor $id stopped answering")
                } catch (e: Exception) {
                    say("Adapter error: ${e.message}")
                    break
                }
                delay(100)
            }
        }
    }

    private fun stopJobs() {
        jogJob?.cancel(); jogJob = null
        pollJob?.cancel(); pollJob = null
        endJob = null
        _state.update { it.copy(jogging = 0, mitActive = false) }
    }

    // ------------------------------------------------------------ actions --

    fun clearFault() {
        val id = _state.value.selected ?: return
        viewModelScope.launch(busThread) {
            try {
                val r = bus?.request(id, Protocol.request(Protocol.CLEAR_FAULT)) ?: return@launch
                val left = r[1].toInt() and 0xFF
                say(if (left == 0) "Motor $id: fault cleared" else "Motor $id: fault still present, ${Protocol.faultName(left)}")
            } catch (e: NoReply) {
                say(e.message ?: "no reply")
            }
            readDetailsNow(id)
        }
    }

    // ------------------------------------------------------------ jog --
    //
    // One motion at a time runs as jogJob. endJob is what undoes it (stop,
    // release, leave MIT); stopMotion() runs both and resumes polling.

    private var endJob: (suspend (Int) -> Unit)? = null

    private fun stopMotion(after: (suspend (Int) -> Unit)? = null) {
        val id = _state.value.selected ?: return
        val end = endJob
        jogJob?.cancel(); jogJob = null
        endJob = null
        _state.update { it.copy(jogging = 0, mitActive = false) }
        pollJob?.cancel(); pollJob = null
        viewModelScope.launch(busThread) {
            end?.invoke(id)
            after?.invoke(id)
            startPolling(id)
        }
    }

    fun motorOff() = stopMotion { releaseSelectedNow() }

    // Leaves MIT mode too: 0xC2 is harmless when the motor is not in it.
    private fun releaseSelectedNow() {
        val id = _state.value.selected ?: return
        try { bus?.request(id, Protocol.request(Protocol.MIT_EXIT)) } catch (_: Exception) {}
        try { bus?.request(id, Protocol.motorOff()) } catch (_: Exception) { say("Motor $id: Motor off got no reply") }
    }

    fun armJog(armed: Boolean) {
        if (!armed && (jogJob != null || _state.value.mitActive)) motorOff()
        _state.update { it.copy(jogArmed = armed) }
    }

    fun setJogMode(m: JogMode) {
        if (m == _state.value.jogMode) return
        motorOff()                                   // never carry one mode into the next
        _state.update { it.copy(jogMode = m) }
    }

    fun setJogSpeed(v: Float) = _state.update { it.copy(jogDps = v) }
    fun setJogErpm(v: Float) = _state.update { it.copy(jogErpm = v) }
    fun setStep(v: Float) = _state.update { it.copy(stepDeg = v) }
    fun setPosSpeed(v: Float) = _state.update { it.copy(posDps = v) }
    fun setCurrent(v: Float) = _state.update { it.copy(currentA = v) }
    fun setMitKp(v: Float) = _state.update { it.copy(mitKp = v) }
    fun setMitKd(v: Float) = _state.update { it.copy(mitKd = v) }
    fun setMitP(v: Float) = _state.update { it.copy(mitPDeg = v) }
    fun setMitV(v: Float) = _state.update { it.copy(mitV = v) }
    fun setMitT(v: Float) = _state.update { it.copy(mitT = v) }

    /** Feed-forward torque bound: the jog current bound times the output Kt. */
    fun maxJogTorque(): Float = maxJogCurrent() * (_state.value.details.mit?.kt ?: 0.0).toFloat()

    /** The most current the jog may use: 30 % of the motor current limit. */
    fun maxJogCurrent(): Float = ((_state.value.details.limits?.motorCurrentA ?: 10.0) * 0.3).toFloat()

    // Sends cmd() every period ms as jogJob until stopped or the motor stops answering.
    private fun stream(id: Int, label: String, period: Long, cmd: () -> ByteArray, onReply: (ByteArray) -> Unit) {
        pollJob?.cancel(); pollJob = null
        jogJob = viewModelScope.launch(busThread) {
            var missed = 0
            while (isActive) {
                val b = bus ?: break
                try {
                    onReply(b.request(id, cmd()))
                    missed = 0
                } catch (_: NoReply) {
                    if (++missed >= 3) {
                        say("Motor $id stopped answering during $label")
                        break
                    }
                }
                delay(period)
            }
        }
    }

    private fun takeStatus(r: ByteArray, mode: String) {
        val s = Protocol.decodeStatus(r)
        _state.update { it.copy(live = it.live.copy(status = s, mode = mode)) }
    }

    /** DPS, eRPM and current modes: run while a button is held. [dir] -1 or +1; 0 = released. */
    fun hold(dir: Int) {
        val st = _state.value
        val id = st.selected ?: return
        if (dir == 0) {
            if (st.jogging == 0) return
            when (st.jogMode) {
                // Stop, let it settle, then release so the shaft is free again.
                JogMode.DPS -> stopMotion { i ->
                    try { bus?.request(i, Protocol.motorStop()) } catch (_: Exception) {}
                    delay(500)
                    releaseSelectedNow()
                }
                JogMode.ERPM -> stopMotion { i ->
                    try { bus?.request(i, Protocol.speedErpm(0.0)) } catch (_: Exception) {}
                    delay(500)
                    releaseSelectedNow()
                }
                else -> stopMotion { releaseSelectedNow() }
            }
            return
        }
        if (!st.jogArmed || jogJob != null) return
        _state.update { it.copy(jogging = dir) }
        when (st.jogMode) {
            JogMode.DPS -> stream(id, "jog", 20, { Protocol.speedDps(dir * _state.value.jogDps.toDouble()) }) {
                takeStatus(it, "speed (DPS)")
            }
            JogMode.ERPM -> stream(id, "jog", 20, { Protocol.speedErpm(dir * _state.value.jogErpm.toDouble()) }) {
                takeStatus(it, "speed (eRPM)")
            }
            JogMode.CURRENT -> stream(id, "jog", 20, {
                Protocol.current(dir * _state.value.currentA.coerceAtMost(maxJogCurrent()).toDouble())
            }) { takeStatus(it, "current") }
            else -> _state.update { it.copy(jogging = 0) }
        }
    }

    /** Position mode: one step of stepDeg on the output shaft per press. The motor holds the new position. */
    fun step(dir: Int) {
        val st = _state.value
        val id = st.selected ?: return
        if (!st.jogArmed) return
        val gear = st.details.mit?.gearRatio?.takeIf { it > 0.01 }
        if (gear == null) {
            say("Gear ratio unknown: press Read again first")
            return
        }
        viewModelScope.launch(busThread) {
            val b = bus ?: return@launch
            try {
                val now = Protocol.decodeMultiTurn(b.request(id, Protocol.request(Protocol.READ_MULTI_TURN)))
                val target = now + dir * _state.value.stepDeg * gear
                takeStatus(b.request(id, Protocol.positionLimited(target, _state.value.posDps.toDouble())), "position")
            } catch (e: Exception) {
                say("Step failed: ${e.message}")
            }
        }
    }

    /** MIT mode: hold the present output position with a spring (kp) and a damper (kd). */
    fun mitHold(on: Boolean) {
        val st = _state.value
        val id = st.selected ?: return
        if (!on) {
            if (st.mitActive) stopMotion()
            return
        }
        if (!st.jogArmed || st.mitActive || jogJob != null) return
        val par = st.details.mit
        if (par == null) {
            say("MIT parameters unknown: press Read again first")
            return
        }
        pollJob?.cancel(); pollJob = null
        _state.update { it.copy(mitActive = true, mitPDeg = 0f, mitV = 0f, mitT = 0f) }   // start by holding still
        viewModelScope.launch(busThread) {
            val b = bus ?: return@launch
            try {
                b.request(id, Protocol.request(Protocol.MIT_ENTER))
                b.request(id, Protocol.mitSetZero(0.0))          // here = 0 rad (runtime only)
            } catch (e: NoReply) {
                say("Motor $id: cannot enter MIT mode")
                _state.update { it.copy(mitActive = false) }
                startPolling(id)
                return@launch
            }
            endJob = { i ->
                try { bus?.request(i, Protocol.request(Protocol.MIT_EXIT)) } catch (_: Exception) {}
                releaseSelectedNow()
            }
            stream(id, "MIT hold", 10, {
                val s = _state.value
                val tMax = maxJogTorque().toDouble()
                Protocol.mitControl(Math.toRadians(s.mitPDeg.toDouble()), s.mitV.toDouble(), s.mitKp.toDouble(),
                    s.mitKd.toDouble(), s.mitT.toDouble().coerceIn(-tMax, tMax), par)
            }) { r ->
                val m = Protocol.decodeMitState(r)
                _state.update {
                    it.copy(live = it.live.copy(
                        status = Protocol.Status(m.temperatureC, m.currentA, m.speedDps, it.live.status?.angleRaw ?: 0),
                        mode = "MIT", mitPosDeg = Math.toDegrees(m.positionRad)))
                }
            }
        }
    }

    /** App left the foreground: never leave a held motion running. */
    fun onBackground() {
        val st = _state.value
        if (st.jogging != 0) hold(0)
        if (st.mitActive) mitHold(false)
    }

    override fun onCleared() {
        val t = transport
        val id = _state.value.selected
        stopJobs()
        if (t != null) {
            // The view model scope is gone; release synchronously on the bus thread.
            busThread.executor.execute {
                if (id != null) try { bus?.request(id, Protocol.motorOff()) } catch (_: Exception) {}
                t.close()
            }
        }
        busThread.close()
    }
}
