// The one screen of the app: adapter, motors on the bus, live state,
// faults and settings of the selected motor, jog.
//
// SPDX-License-Identifier: MIT
package com.openrobot.motorcheck

import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Card
import androidx.compose.material3.Checkbox
import androidx.compose.material3.FilterChip
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Slider
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.openrobot.motorcheck.protocol.Protocol
import kotlin.math.roundToInt

@Composable
fun CheckScreen(vm: CheckViewModel, adapters: () -> List<UsbSerialDriver>, open: (UsbSerialDriver) -> Unit) {
    val dark = isSystemInDarkTheme()
    MaterialTheme(colorScheme = if (dark) darkColorScheme() else lightColorScheme()) {
        Scaffold { pad ->
            val st by vm.state.collectAsState()
            Column(
                Modifier.fillMaxSize().padding(pad).padding(horizontal = 16.dp).verticalScroll(rememberScrollState()),
                verticalArrangement = Arrangement.spacedBy(12.dp),
            ) {
                Spacer(Modifier.height(4.dp))
                Text("OpenRobot Motor Check", style = MaterialTheme.typography.titleLarge)
                AdapterCard(st, adapters, open, vm)
                st.busy?.let {
                    Text(it)
                    LinearProgressIndicator(Modifier.fillMaxWidth())
                }
                if (st.adapter != null && st.scanned) MotorChips(st, vm)
                st.selected?.let { id ->
                    LiveCard(id, st)
                    FaultCard(st, vm)
                    SettingsCard(st)
                    JogCard(st, vm)
                }
                MessagesCard(st, vm)
                Spacer(Modifier.height(16.dp))
            }
        }
    }
}

@Composable
private fun Section(title: String, content: @Composable () -> Unit) {
    Card(Modifier.fillMaxWidth()) {
        Column(Modifier.padding(16.dp), verticalArrangement = Arrangement.spacedBy(8.dp)) {
            Text(title, style = MaterialTheme.typography.titleMedium)
            content()
        }
    }
}

@Composable
private fun ValueRow(label: String, value: String) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label, color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, fontFamily = FontFamily.Monospace)
    }
}

@Composable
private fun AdapterCard(st: UiState, adapters: () -> List<UsbSerialDriver>, open: (UsbSerialDriver) -> Unit, vm: CheckViewModel) {
    Section("CAN adapter") {
        if (st.adapter != null) {
            Text(st.adapter)
            Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                Button(onClick = vm::scan, enabled = st.busy == null) { Text("Scan") }
                OutlinedButton(onClick = vm::disconnect) { Text("Disconnect") }
            }
        } else {
            var found by remember { mutableStateOf<List<UsbSerialDriver>?>(null) }
            Text("Plug an slcan adapter (CANable or compatible) into the phone with an OTG cable.")
            Button(onClick = { found = adapters() }) { Text("Find adapters") }
            found?.let { list ->
                if (list.isEmpty()) Text("No USB serial adapter found.")
                list.forEach { d ->
                    OutlinedButton(onClick = { open(d) }, Modifier.fillMaxWidth()) {
                        Text((d.device.productName ?: "USB serial") + "  %04X:%04X".format(d.device.vendorId, d.device.productId))
                    }
                }
            }
        }
    }
}

@Composable
private fun MotorChips(st: UiState, vm: CheckViewModel) {
    Section("Motors on the bus") {
        if (st.motors.isEmpty()) Text("None found")
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            st.motors.forEach { id ->
                FilterChip(selected = st.selected == id, onClick = { vm.select(id) }, label = { Text("ID $id") })
            }
        }
    }
}

@Composable
private fun LiveCard(id: Int, st: UiState) {
    val s = st.live.status
    val gear = st.details.mit?.gearRatio?.takeIf { it > 0.01 }
    Section("Motor $id") {
        ValueRow("Mode", st.live.mode.ifEmpty { "-" })
        ValueRow("Temperature", s?.let { "${it.temperatureC} °C" } ?: "-")
        ValueRow("Current", s?.let { "%.2f A".format(it.currentA) } ?: "-")
        ValueRow("Speed, motor", s?.let { "${it.speedDps} deg/s" } ?: "-")
        ValueRow("Angle, motor", st.live.multiTurnDeg?.let { "%.1f deg".format(it) } ?: "-")
        if (gear != null) ValueRow("Angle, output", st.live.multiTurnDeg?.let { "%.2f deg".format(it / gear) } ?: "-")
    }
}

@Composable
private fun FaultCard(st: UiState, vm: CheckViewModel) {
    Section("Faults") {
        val recent = st.details.faults.filter { it != 0 }
        if (recent.isEmpty()) Text("No recent faults")
        else recent.forEach { Text("$it  ${Protocol.faultName(it)}") }
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            OutlinedButton(onClick = vm::clearFault) { Text("Clear fault") }
            TextButton(onClick = vm::refreshDetails) { Text("Read again") }
        }
    }
}

@Composable
private fun SettingsCard(st: UiState) {
    Section("Settings (read only)") {
        val m = st.details.mit
        val l = st.details.limits
        ValueRow("Gear ratio", m?.let { "%.2f".format(it.gearRatio) } ?: "-")
        ValueRow("Kt, output", m?.let { "%.3f Nm/A".format(it.kt) } ?: "-")
        ValueRow("MIT v_max, t_max", m?.let { "${it.vMax.roundToInt()} rad/s, ${it.tMax.roundToInt()} Nm" } ?: "-")
        ValueRow("MIT Kd scale", m?.let { "${if (it.kdScale == 0) 1 else it.kdScale}" } ?: "-")
        ValueRow("Motor current limit", l?.let { "%.1f A".format(it.motorCurrentA) } ?: "-")
    }
}

@Composable
private fun JogCard(st: UiState, vm: CheckViewModel) {
    Section("Jog") {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Checkbox(checked = st.jogArmed, onCheckedChange = vm::armJog)
            Text("The shaft is free to turn and nothing can be hurt")
        }
        Row(Modifier.horizontalScroll(rememberScrollState()), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            JogMode.entries.forEach { m ->
                FilterChip(selected = st.jogMode == m, onClick = { vm.setJogMode(m) }, label = { Text(m.label) })
            }
        }
        val idle = st.jogging == 0 && !st.mitActive
        when (st.jogMode) {
            JogMode.DPS -> {
                SliderRow("Speed, motor side", "${st.jogDps.roundToInt()} deg/s", st.jogDps, 30f..1800f, idle, vm::setJogSpeed)
                Hint("Speed (DPS, position-tracked): a target angle advances at this speed and the position " +
                    "loop follows it. Hold a button to turn; releasing stops the motor, then releases it.")
                HoldPair(st, vm)
            }
            JogMode.ERPM -> {
                SliderRow("Speed, electrical", "${st.jogErpm.roundToInt()} eRPM", st.jogErpm, 100f..10000f, idle, vm::setJogErpm)
                Hint("Speed (eRPM, VESC speed loop): eRPM = mechanical RPM x pole pairs. Compare with the " +
                    "measured deg/s above. Hold a button to turn; releasing stops the motor, then releases it.")
                HoldPair(st, vm)
            }
            JogMode.CURRENT -> {
                val max = vm.maxJogCurrent()
                SliderRow("Current", "%.2f A".format(st.currentA.coerceAtMost(max)), st.currentA.coerceAtMost(max),
                    0.1f..max, idle, vm::setCurrent)
                Hint("Hold a button to apply the current (torque). At most 30 % of the motor current limit. " +
                    "An unloaded motor speeds up quickly: start low.")
                HoldPair(st, vm)
            }
            JogMode.POSITION -> {
                SliderRow("Step, output side", "${st.stepDeg.roundToInt()} deg", st.stepDeg, 1f..90f, true, vm::setStep)
                SliderRow("Speed limit, motor side", "${st.posDps.roundToInt()} deg/s", st.posDps, 30f..1800f, true, vm::setPosSpeed)
                Hint("Each press moves one step from where the shaft is, then the motor holds there. MOTOR OFF releases it.")
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(12.dp)) {
                    Button(onClick = { vm.step(-1) }, enabled = st.jogArmed, modifier = Modifier.weight(1f).height(72.dp)) {
                        Text("◀  − step")
                    }
                    Button(onClick = { vm.step(1) }, enabled = st.jogArmed, modifier = Modifier.weight(1f).height(72.dp)) {
                        Text("+ step  ▶")
                    }
                }
            }
            JogMode.MIT -> {
                TargetRow("kp, stiffness", "%.1f Nm/rad".format(st.mitKp), st.mitKp, 0f..50f, vm::setMitKp)
                TargetRow("kd, damping", "%.2f Nm s/rad".format(st.mitKd), st.mitKd, 0f..2f, vm::setMitKd)
                Hint("Holds the output shaft where it is with a spring and a damper; turn it by hand to feel " +
                    "the stiffness. torque = kp (p - p_act) + kd (v - v_act) + t_ff, positions from the hold point.")
                val tMax = vm.maxJogTorque()
                TargetRow("p, position", "%.1f deg".format(st.mitPDeg), st.mitPDeg, -180f..180f, vm::setMitP)
                TargetRow("v, velocity", "%.2f rad/s".format(st.mitV), st.mitV, -10f..10f, vm::setMitV)
                if (tMax > 0f) TargetRow("t_ff, torque", "%.2f Nm".format(st.mitT.coerceIn(-tMax, tMax)),
                    st.mitT.coerceIn(-tMax, tMax), -tMax..tMax, vm::setMitT)
                if (st.mitActive) {
                    val kt = st.details.mit?.kt ?: 0.0
                    ValueRow("Deflection", st.live.mitPosDeg?.let { "%.2f deg".format(it) } ?: "-")
                    ValueRow("Torque", st.live.status?.let { "%.2f Nm".format(it.currentA * kt) } ?: "-")
                }
                Button(
                    onClick = { vm.mitHold(!st.mitActive) },
                    enabled = st.jogArmed,
                    modifier = Modifier.fillMaxWidth().height(72.dp),
                ) { Text(if (st.mitActive) "Release" else "Hold here") }
            }
        }
        Button(
            onClick = vm::motorOff,
            modifier = Modifier.fillMaxWidth().height(56.dp),
            colors = ButtonDefaults.buttonColors(containerColor = MaterialTheme.colorScheme.error),
        ) { Text("MOTOR OFF", fontWeight = FontWeight.Bold) }
    }
}

@Composable
private fun SliderRow(label: String, value: String, v: Float, range: ClosedFloatingPointRange<Float>, enabled: Boolean,
                      onChange: (Float) -> Unit) {
    ValueRow(label, value)
    Slider(value = v, onValueChange = onChange, valueRange = range, enabled = enabled)
}

// A slider with a button that puts it back to 0.
@Composable
private fun TargetRow(label: String, value: String, v: Float, range: ClosedFloatingPointRange<Float>,
                      onChange: (Float) -> Unit) {
    Row(Modifier.fillMaxWidth(), verticalAlignment = Alignment.CenterVertically) {
        Text(label, Modifier.weight(1f), color = MaterialTheme.colorScheme.onSurfaceVariant)
        Text(value, fontFamily = FontFamily.Monospace)
        Spacer(Modifier.width(12.dp))
        OutlinedButton(onClick = { onChange(0f) }, Modifier.width(72.dp).height(44.dp)) {
            Text("0", style = MaterialTheme.typography.titleMedium, fontWeight = FontWeight.Bold)
        }
    }
    Slider(value = v, onValueChange = onChange, valueRange = range)
}

@Composable
private fun Hint(text: String) = Text(text, style = MaterialTheme.typography.bodySmall)

@Composable
private fun HoldPair(st: UiState, vm: CheckViewModel) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(12.dp)) {
        HoldButton("◀  Reverse", st.jogArmed, st.jogging == -1, Modifier.weight(1f)) { vm.hold(if (it) -1 else 0) }
        HoldButton("Forward  ▶", st.jogArmed, st.jogging == 1, Modifier.weight(1f)) { vm.hold(if (it) 1 else 0) }
    }
}

// A button that reports press (true) and release (false).
@Composable
private fun HoldButton(label: String, enabled: Boolean, active: Boolean, modifier: Modifier, onHold: (Boolean) -> Unit) {
    val bg = when {
        !enabled -> MaterialTheme.colorScheme.surfaceVariant
        active -> MaterialTheme.colorScheme.primary
        else -> MaterialTheme.colorScheme.primaryContainer
    }
    val fg = if (active) MaterialTheme.colorScheme.onPrimary else MaterialTheme.colorScheme.onPrimaryContainer
    Box(
        modifier.height(72.dp).background(bg, RoundedCornerShape(12.dp)).pointerInput(enabled) {
            if (!enabled) return@pointerInput
            detectTapGestures(onPress = {
                onHold(true)
                try { awaitRelease() } finally { onHold(false) }
            })
        },
        contentAlignment = Alignment.Center,
    ) { Text(label, color = if (enabled) fg else Color.Gray) }
}

@Composable
private fun MessagesCard(st: UiState, vm: CheckViewModel) {
    if (st.messages.isEmpty()) return
    Section("Messages") {
        st.messages.reversed().forEach { Text(it, style = MaterialTheme.typography.bodySmall) }
        TextButton(onClick = vm::clearMessages) { Text("Clear") }
        Spacer(Modifier.width(1.dp))
    }
}
