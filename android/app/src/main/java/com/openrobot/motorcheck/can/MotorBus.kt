// Request/reply on one CAN bus. Not thread-safe: call from one thread
// (the view model runs every bus call on a single-thread dispatcher).
//
// SPDX-License-Identifier: MIT
package com.openrobot.motorcheck.can

import com.openrobot.motorcheck.protocol.Protocol

class NoReply(motorId: Int, cmd: Int) : Exception("motor $motorId: no reply to 0x%02X".format(cmd))

class MotorBus(private val t: CanTransport) {
    /** Called for every fault notice (0xBF): motor id, fault code. */
    var onFault: ((Int, Int) -> Unit)? = null

    /** Send [req] to [motorId] and return the matching reply payload. */
    fun request(motorId: Int, req: ByteArray, timeoutMs: Long = 30, retries: Int = 1): ByteArray {
        val want = Protocol.replyCode(req[0].toInt() and 0xFF)
        val id = Protocol.canId(motorId)
        repeat(retries + 1) {
            drain()
            t.send(id, req)
            val end = System.nanoTime() + timeoutMs * 1_000_000
            while (true) {
                val left = (end - System.nanoTime()) / 1_000_000
                if (left <= 0) break
                val f = t.recv(left) ?: break
                if (!accept(f)) continue
                if (f.id == id && (f.data[0].toInt() and 0xFF) == want) return f.data
            }
        }
        throw NoReply(motorId, req[0].toInt() and 0xFF)
    }

    /** Motors in [first]..[last] that answer a harmless read (0x92). */
    fun scan(first: Int = 1, last: Int = 32): List<Int> = (first..last).filter {
        try {
            request(it, Protocol.request(Protocol.READ_MULTI_TURN), timeoutMs = 15, retries = 0)
            true
        } catch (_: NoReply) {
            false
        }
    }

    /** Check for fault notices without sending anything. */
    fun poll() = drain()

    private fun drain() {
        while (true) accept(t.recv(0) ?: return)
    }

    // false for frames that are not replies (fault notices are reported).
    private fun accept(f: CanFrame): Boolean {
        if ((f.data[0].toInt() and 0xFF) != Protocol.FAULT_NOTICE) return true
        onFault?.invoke(f.id - Protocol.BASE_ID, f.data[1].toInt() and 0xFF)
        return false
    }
}
