package com.openrobot.motorcheck.protocol

import org.junit.Assert.assertArrayEquals
import org.junit.Assert.assertEquals
import org.junit.Test

// Byte-for-byte against the Python reference (tools/gen_vectors.py).
class ProtocolTest {
    @Test
    fun encodeMatchesPython() {
        for ((name, got, want) in ProtocolVectors.ENCODE) assertArrayEquals(name, want, got)
    }

    @Test
    fun decodersMatchPython() {
        assertEquals(ProtocolVectors.STATUS_WANT, Protocol.decodeStatus(ProtocolVectors.STATUS))
        assertEquals(ProtocolVectors.MULTI_WANT, Protocol.decodeMultiTurn(ProtocolVectors.MULTI), 1e-9)
        assertEquals(ProtocolVectors.PHASES_WANT, Protocol.decodePhases(ProtocolVectors.PHASES))
        assertEquals(ProtocolVectors.FAULTS_WANT, Protocol.decodeFaults(ProtocolVectors.FAULTS))
        assertEquals(ProtocolVectors.LIMITS_WANT, Protocol.decodeLimits(ProtocolVectors.LIMITS))
        assertEquals(ProtocolVectors.MIT_WANT, Protocol.decodeMitParams(ProtocolVectors.MIT))
        assertEquals(ProtocolVectors.MITSTATE_WANT, Protocol.decodeMitState(ProtocolVectors.MITSTATE))
        assertEquals(ProtocolVectors.OUTENC_WANT, Protocol.decodeOutputEncoder(ProtocolVectors.OUTENC))
    }

    @Test
    fun replyCodes() {
        assertEquals(Protocol.READ_MIT_PARAMS, Protocol.replyCode(Protocol.WRITE_MIT_PARAMS))
        assertEquals(Protocol.READ_STATUS, Protocol.replyCode(Protocol.READ_STATUS))
    }
}
