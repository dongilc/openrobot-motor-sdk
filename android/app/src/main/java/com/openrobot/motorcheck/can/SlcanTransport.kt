// slcan (Lawicel ASCII) over a USB serial port: CANable and compatible
// adapters plugged into the phone with an OTG cable.
//
// SPDX-License-Identifier: MIT
package com.openrobot.motorcheck.can

import com.hoho.android.usbserial.driver.UsbSerialPort
import com.hoho.android.usbserial.util.SerialInputOutputManager
import java.io.IOException
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit

class CanFrame(val id: Int, val data: ByteArray)

interface CanTransport {
    fun send(id: Int, data: ByteArray)
    /** One standard frame, or null after [timeoutMs]. */
    fun recv(timeoutMs: Long): CanFrame?
    fun close()
}

class SlcanTransport(private val port: UsbSerialPort, bitrate: Int) : CanTransport, SerialInputOutputManager.Listener {
    private val rx = LinkedBlockingQueue<CanFrame>()
    private val line = StringBuilder()
    private val io = SerialInputOutputManager(port, this)
    @Volatile var error: Exception? = null
        private set

    init {
        val code = BITRATES[bitrate] ?: throw IllegalArgumentException("slcan: unsupported bit rate $bitrate")
        port.setParameters(115200, 8, UsbSerialPort.STOPBITS_1, UsbSerialPort.PARITY_NONE)
        try { port.dtr = true } catch (_: Exception) {}     // some CDC firmwares wait for DTR
        write("C\r")                                       // close, in case it was left open
        write("S$code\r")
        write("O\r")
        io.start()
    }

    private fun write(s: String) = port.write(s.toByteArray(Charsets.US_ASCII), WRITE_TIMEOUT_MS)

    override fun send(id: Int, data: ByteArray) {
        val sb = StringBuilder(22)
        sb.append('t').append(String.format("%03X", id and 0x7FF)).append(data.size)
        for (b in data) sb.append(String.format("%02X", b.toInt() and 0xFF))
        sb.append('\r')
        try {
            write(sb.toString())
        } catch (e: IOException) {
            error = e
            throw e
        }
    }

    override fun recv(timeoutMs: Long): CanFrame? = rx.poll(timeoutMs, TimeUnit.MILLISECONDS)

    override fun close() {
        try { write("C\r") } catch (_: Exception) {}
        io.listener = null
        io.stop()
        try { port.close() } catch (_: Exception) {}
    }

    // Serial bytes arrive in arbitrary chunks; frames end with '\r'.
    override fun onNewData(bytes: ByteArray) {
        for (b in bytes) {
            val c = b.toInt().toChar()
            if (c == '\r') {
                parse(line)
                line.setLength(0)
            } else if (c != '\n' && c != '\u0007') {
                line.append(c)
                if (line.length > 64) line.setLength(0)   // garbage, resync
            }
        }
    }

    override fun onRunError(e: Exception) {
        error = e
    }

    private fun parse(s: CharSequence) {
        // Only standard data frames: t iii l dd..  (extended 'T' and remote 'r' are ignored)
        if (s.length < 5 || s[0] != 't') return
        val id = s.substring(1, 4).toIntOrNull(16) ?: return
        val len = s[4].digitToIntOrNull() ?: return
        if (len != 8 || s.length < 5 + 16) return
        val d = ByteArray(8)
        for (i in 0 until 8) d[i] = (s.substring(5 + 2 * i, 7 + 2 * i).toIntOrNull(16) ?: return).toByte()
        rx.offer(CanFrame(id, d))
    }

    companion object {
        private const val WRITE_TIMEOUT_MS = 100
        private val BITRATES = mapOf(
            10_000 to 0, 20_000 to 1, 50_000 to 2, 100_000 to 3, 125_000 to 4,
            250_000 to 5, 500_000 to 6, 800_000 to 7, 1_000_000 to 8,
        )
    }
}
