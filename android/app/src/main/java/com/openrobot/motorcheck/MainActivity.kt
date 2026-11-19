// Field check app for OpenRobot motor controllers: find motors on the CAN
// bus, read their state and faults, jog them briefly.
//
// SPDX-License-Identifier: MIT
package com.openrobot.motorcheck

import android.app.PendingIntent
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.hardware.usb.UsbConstants
import android.hardware.usb.UsbDevice
import android.hardware.usb.UsbManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.viewModels
import androidx.core.content.ContextCompat
import com.hoho.android.usbserial.driver.CdcAcmSerialDriver
import com.hoho.android.usbserial.driver.ProbeTable
import com.hoho.android.usbserial.driver.UsbSerialDriver
import com.hoho.android.usbserial.driver.UsbSerialProber

class MainActivity : ComponentActivity() {
    private val vm: CheckViewModel by viewModels()
    private lateinit var usb: UsbManager
    private var pending: UsbSerialDriver? = null

    private val permission = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            if (intent.action != ACTION_USB_PERMISSION) return
            val driver = pending ?: return
            pending = null
            if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) vm.connect(driver, usb)
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        usb = getSystemService(Context.USB_SERVICE) as UsbManager
        ContextCompat.registerReceiver(this, permission, IntentFilter(ACTION_USB_PERMISSION),
            ContextCompat.RECEIVER_NOT_EXPORTED)
        setContent { CheckScreen(vm, ::adapters, ::open) }
    }

    override fun onDestroy() {
        unregisterReceiver(permission)
        super.onDestroy()
    }

    override fun onPause() {
        vm.onBackground()
        super.onPause()
    }

    /** USB serial adapters plugged in now. */
    private fun adapters(): List<UsbSerialDriver> = usb.deviceList.values.mapNotNull { dev ->
        UsbSerialProber.getDefaultProber().probeDevice(dev)
            ?: EXTRA.probeDevice(dev)
            ?: if (hasCdcData(dev)) CdcAcmSerialDriver(dev) else null
    }

    private fun open(driver: UsbSerialDriver) {
        if (usb.hasPermission(driver.device)) {
            vm.connect(driver, usb)
            return
        }
        pending = driver
        val flags = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) PendingIntent.FLAG_MUTABLE else 0
        val intent = Intent(ACTION_USB_PERMISSION).setPackage(packageName)
        usb.requestPermission(driver.device, PendingIntent.getBroadcast(this, 0, intent, flags))
    }

    // Any device with a CDC data interface is worth a try as an slcan port.
    private fun hasCdcData(dev: UsbDevice) =
        (0 until dev.interfaceCount).any { dev.getInterface(it).interfaceClass == UsbConstants.USB_CLASS_CDC_DATA }

    companion object {
        private const val ACTION_USB_PERMISSION = "com.openrobot.motorcheck.USB_PERMISSION"
        private val EXTRA = UsbSerialProber(ProbeTable().apply {
            addProduct(0x16D0, 0x117E, CdcAcmSerialDriver::class.java)   // CANable (slcan firmware)
            addProduct(0x0483, 0x5740, CdcAcmSerialDriver::class.java)   // STM32 virtual COM port
        })
    }
}
