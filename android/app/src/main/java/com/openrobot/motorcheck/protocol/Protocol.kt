// OpenRobot CAN protocol: frame encoding and decoding, no I/O.
// Same layout as docs/can_protocol.md and the Python reference
// (python/openrobot_motor/protocol.py); the unit tests compare bytes with it.
//
// SPDX-License-Identifier: MIT
package com.openrobot.motorcheck.protocol

import kotlin.math.roundToLong

object Protocol {
    const val BASE_ID = 0x140
    const val CURRENT_LSB = 33.0 / 2048.0      // A per LSB
    const val ANGLE_14BIT = 16384
    const val DEFAULT_BITRATE = 1_000_000

    const val MOTOR_OFF = 0x80
    const val MOTOR_STOP = 0x81
    const val CURRENT = 0xA1
    const val SPEED = 0xA2
    const val POSITION_LIMITED = 0xA4
    const val MIT_CONTROL = 0xC0
    const val MIT_ENTER = 0xC1
    const val MIT_EXIT = 0xC2
    const val MIT_SET_ZERO = 0xC3
    const val READ_STATUS = 0x9C
    const val READ_PHASES = 0x9D
    const val READ_MULTI_TURN = 0x92
    const val READ_OUTPUT_ENCODER = 0x93
    const val CLEAR_FAULT = 0x9B
    const val READ_FAULTS = 0xB0
    const val READ_LIMITS = 0xB1
    const val READ_MIT_PARAMS = 0xC4
    const val WRITE_MIT_PARAMS = 0xC5
    const val FAULT_NOTICE = 0xBF

    fun canId(motorId: Int) = BASE_ID + motorId

    /** Code the board puts in data[0] of the reply to [request]. */
    fun replyCode(request: Int) = if (request == WRITE_MIT_PARAMS) READ_MIT_PARAMS else request

    val CONTROL_MODES = listOf(
        "none", "released", "current brake", "duty", "current", "damped current",
        "position", "position (multi-turn)", "speed (timeout)", "speed",
        "position with speed limit", "trajectory", "MIT",
    )

    val FAULTS = listOf(
        "none", "over voltage", "under voltage", "gate driver", "over current",
        "over temperature, power stage", "over temperature, motor", "gate driver over voltage",
        "gate driver under voltage", "MCU under voltage", "watchdog reset at boot", "encoder (SPI)",
        "encoder amplitude low", "encoder amplitude high", "flash corruption",
        "current sensor offset high, phase 1", "current sensor offset high, phase 2",
        "current sensor offset high, phase 3", "unbalanced currents", "brake", "resolver LOT",
        "resolver DOS", "resolver LOS", "flash corruption, app configuration",
        "flash corruption, motor configuration", "encoder no magnet", "position error too large",
    )

    fun faultName(code: Int) = FAULTS.getOrElse(code) { "unknown ($code)" }
    fun controlModeName(mode: Int) = CONTROL_MODES.getOrElse(mode) { "unknown ($mode)" }

    // ------------------------------------------------------------ encode --

    private fun frame(cmd: Int, vararg body: Int): ByteArray {
        val d = ByteArray(8)
        d[0] = cmd.toByte()
        body.forEachIndexed { i, b -> d[1 + i] = b.toByte() }
        return d
    }

    private fun putI32(d: ByteArray, at: Int, v: Int) {
        for (i in 0 until 4) d[at + i] = (v shr (8 * i)).toByte()
    }

    fun request(cmd: Int) = frame(cmd)
    fun motorOff() = frame(MOTOR_OFF)
    fun motorStop() = frame(MOTOR_STOP)

    private fun putI16(d: ByteArray, at: Int, v: Int) {
        d[at] = v.toByte(); d[at + 1] = (v shr 8).toByte()
    }

    private fun i16Of(x: Double) = x.roundToLong().coerceIn(-32768, 32767).toInt()
    private fun i32Of(x: Double) = x.roundToLong().coerceIn(Int.MIN_VALUE.toLong(), Int.MAX_VALUE.toLong()).toInt()

    /** 0xA1. damping 0 = pure current control, otherwise 0.01..100. */
    fun current(amps: Double, damping: Double = 0.0): ByteArray {
        val d = frame(CURRENT)
        putI16(d, 4, i16Of(amps / CURRENT_LSB))
        putI16(d, 6, i16Of(damping.coerceIn(0.0, 100.0) * 100.0))
        return d
    }

    /** 0xA2 mode 0, Speed (DPS, position-tracked): motor speed in deg/s. */
    fun speedDps(dps: Double): ByteArray {
        val d = frame(SPEED)
        putI32(d, 4, i32Of(dps * 100.0))
        return d
    }

    /** 0xA2 mode 1, Speed (eRPM, VESC speed loop): electrical RPM. */
    fun speedErpm(erpm: Double): ByteArray {
        val d = frame(SPEED, 1)
        putI32(d, 4, i32Of(erpm))
        return d
    }

    const val SPEED_LIMIT_MAX = 25000

    /** 0xA4, multi-turn motor angle with a speed limit of 1..25000 deg/s.
     *  The board ignores an out-of-range limit without telling, so it is refused here. */
    fun positionLimited(deg: Double, maxDps: Double): ByteArray {
        val lim = maxDps.roundToLong()
        require(lim in 1..SPEED_LIMIT_MAX) { "speed limit must be 1..$SPEED_LIMIT_MAX deg/s" }
        val d = frame(POSITION_LIMITED)
        putI16(d, 2, lim.toInt())
        putI32(d, 4, i32Of(deg * 100.0))
        return d
    }

    // ------------------------------------------------------------ MIT --

    const val MIT_P_MAX = 12.56
    const val MIT_KP_MAX = 500.0
    const val MIT_KD_MAX = 5.0

    private fun f2u(x: Double, lo: Double, hi: Double, bits: Int): Int {
        val span = (1 shl bits) - 1
        return ((x.coerceIn(lo, hi) - lo) / (hi - lo) * span).roundToLong().toInt()
    }

    /** 0xC0. Output-side targets; v and t ranges come from the board (0xC4). */
    fun mitControl(pRad: Double, vRads: Double, kp: Double, kd: Double, tNm: Double, par: MitParams): ByteArray {
        val kdMax = MIT_KD_MAX * (if (par.kdScale == 0) 1 else par.kdScale)
        val p = f2u(pRad, -MIT_P_MAX, MIT_P_MAX, 16)
        val v = f2u(vRads, -par.vMax, par.vMax, 12)
        val k = f2u(kp, 0.0, MIT_KP_MAX, 12)
        val dd = f2u(kd, 0.0, kdMax, 8)
        val t = f2u(tNm, -par.tMax, par.tMax, 8)
        return frame(MIT_CONTROL, p shr 8, p and 0xFF, v shr 4, ((v and 0x0F) shl 4) or (k shr 8), k and 0xFF, dd, t)
    }

    /** 0xC3. The present output position becomes offsetDeg (runtime only). */
    fun mitSetZero(offsetDeg: Double = 0.0): ByteArray {
        val d = frame(MIT_SET_ZERO)
        putI16(d, 6, i16Of(offsetDeg * 100.0))
        return d
    }

    // ------------------------------------------------------------ decode --

    private fun u8(d: ByteArray, at: Int) = d[at].toInt() and 0xFF
    private fun i8(d: ByteArray, at: Int) = d[at].toInt()
    private fun u16(d: ByteArray, at: Int) = u8(d, at) or (u8(d, at + 1) shl 8)
    private fun i16(d: ByteArray, at: Int) = u16(d, at).toShort().toInt()
    private fun i32(d: ByteArray, at: Int) = u16(d, at) or (u16(d, at + 2) shl 16)

    /** Common reply of motion commands and 0x9C. Motor side. */
    data class Status(val temperatureC: Int, val currentA: Double, val speedDps: Int, val angleRaw: Int) {
        val angleDeg get() = angleRaw * 360.0 / ANGLE_14BIT
    }

    fun decodeStatus(d: ByteArray) = Status(i8(d, 1), i16(d, 2) * CURRENT_LSB, i16(d, 4), u16(d, 6))

    /** 0x92 -> motor multi-turn angle, degrees (signed 56-bit, 0.01 deg). */
    fun decodeMultiTurn(d: ByteArray): Double {
        var raw = 0L
        for (i in 7 downTo 1) raw = (raw shl 8) or u8(d, i).toLong()
        raw = (raw shl 8) shr 8                                 // sign-extend 56 bits
        return raw / 100.0
    }

    data class Phases(val controlMode: Int, val ia: Double, val ib: Double, val ic: Double) {
        val controlModeName get() = controlModeName(controlMode)
    }

    fun decodePhases(d: ByteArray) = Phases(u8(d, 1), i16(d, 2) / 64.0, i16(d, 4) / 64.0, i16(d, 6) / 64.0)

    /** 0xB0 -> the last seven fault codes. */
    fun decodeFaults(d: ByteArray) = (1 until 8).map { u8(d, it) }

    data class Limits(val motorCurrentA: Double, val absCurrentA: Double, val batteryCurrentA: Double, val gateOcMode: Int)

    fun decodeLimits(d: ByteArray) = Limits(u16(d, 2) / 100.0, u16(d, 4) / 100.0, u16(d, 6) / 100.0, u8(d, 1))

    data class MitParams(val vMax: Double, val tMax: Double, val kt: Double, val gearRatio: Double, val kdScale: Int)

    fun decodeMitParams(d: ByteArray) =
        MitParams(u8(d, 1).toDouble(), u8(d, 2).toDouble(), u16(d, 3) / 1000.0, u16(d, 5) / 100.0, u8(d, 7))

    data class OutputEncoder(val angleDeg: Double, val multiTurnDeg: Double, val valid: Boolean,
                             val calibrated: Boolean, val dualCalibrated: Boolean)

    fun decodeOutputEncoder(d: ByteArray): OutputEncoder {
        val flags = u8(d, 7)
        return OutputEncoder(u16(d, 1) * 360.0 / 65536.0, i32(d, 3) / 100.0,
            flags and 1 != 0, flags and 2 != 0, flags and 4 != 0)
    }

    /** 0xC0 reply. Output side, position relative to the MIT zero. */
    data class MitState(val temperatureC: Int, val currentA: Double, val speedDps: Int, val positionRad: Double)

    fun decodeMitState(d: ByteArray) = MitState(i8(d, 1), i16(d, 2) * CURRENT_LSB, i16(d, 4), i16(d, 6) * MIT_P_MAX / 32767.0)
}
