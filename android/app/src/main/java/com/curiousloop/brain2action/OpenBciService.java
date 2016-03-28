package com.curiousloop.brain2action;

import android.app.PendingIntent;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.os.Binder;
import android.os.Handler;
import android.os.IBinder;
import android.support.annotation.Nullable;

import com.felhr.usbserial.CDCSerialDevice;
import com.felhr.usbserial.UsbSerialDevice;
import com.felhr.usbserial.UsbSerialInterface;

import java.io.UnsupportedEncodingException;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Created by pedro on 28/03/16.
 */
public class OpenBciService extends Service {

    public static final int SERIAL_PORT_CONNECTED = 0;
    public static final int SERIAL_PORT_DISCONNECTED = 1;
    public static final int MESSAGE_FROM_SERIAL_PORT = 2;
    public static final int CTS_CHANGE = 3;
    public static final int DSR_CHANGE = 4;

    static final int OPENBCI_VENDOR_ID = 1027;

    public static boolean SERVICE_CONNECTED = false;

    final byte START_BYTE = (byte)0xA0;
    final byte END_BYTE = (byte)0xC0;

    private static final int BAUD_RATE = 115200;

    public class UsbBinder extends Binder {
        public OpenBciService getService() {
            return OpenBciService.this;
        }
    }

    private Handler handler;
    private IBinder binder = new UsbBinder();

    private Context context;
    private UsbManager usbManager;
    private UsbDevice device;
    private UsbDeviceConnection connection;
    private UsbSerialDevice serialPort;

    private boolean serialPortConnected;

    private ArrayList<Integer[]> samples = new ArrayList<>();
    byte[] currentSample = new byte[33];
    int currentSampleByteCount = 0;

    private boolean isReceivedHeader = false;
    private boolean isStreaming = false;

    private UsbSerialInterface.UsbReadCallback mCallback = new UsbSerialInterface.UsbReadCallback() {
        @Override
        public void onReceivedData(byte[] arg0) {
            try {
                if(!isReceivedHeader) {
                    String data = new String(arg0, "UTF-8");
                    if (data.contains("$$$")) {
                        isReceivedHeader = true;
                    }
                }
                else {
                    isStreaming = true;
                    for(int i=0; i<arg0.length; i++) {
                        if(currentSampleByteCount >= 33) {
                            currentSample = new byte[33];
                            currentSampleByteCount = 0;
                        }
                        currentSample[currentSampleByteCount++] = arg0[i];
                        if(currentSampleByteCount == 33) {
                            Integer[] sample = readSample(currentSample);
                            if(sample != null) {
                                samples.add(sample);
                            }
                        }
                    }
                    if (handler != null)
                        handler.obtainMessage(OpenBciService.MESSAGE_FROM_SERIAL_PORT, String.valueOf(samples.size())).sendToTarget();
                }
            } catch (UnsupportedEncodingException e) {
                e.printStackTrace();
            }
        }
    };

    private Integer[] readSample(byte[] data) {
        if(data[0] != START_BYTE)
            return null;
        Integer[] result = new Integer[11];
        int sampleNum = (int) data[1];  // sample number
        for(int i=0; i<8; i++) {    // 8 eeg channels
            int val = interpret24bitAsInt32(new byte[]{data[i + 2], data[i + 3], data[i + 4]});
            // TODO scale factor
            result[i] = val;
        }
        for(int i=0; i<3; i++) {    // 3 acceleromeneter channels
            int val = interpret16bitAsInt32(new byte[]{data[i + 26], data[i + 27]});
            result[i+8] = val;
        }
        if(data[32] != END_BYTE)
            return null;
        return result;
    }

    /*
     * State changes in the CTS line will be received here
     */
    private UsbSerialInterface.UsbCTSCallback ctsCallback = new UsbSerialInterface.UsbCTSCallback() {
        @Override
        public void onCTSChanged(boolean state) {
            if(handler != null)
                handler.obtainMessage(OpenBciService.CTS_CHANGE).sendToTarget();
        }
    };

    /*
     * State changes in the DSR line will be received here
     */
    private UsbSerialInterface.UsbDSRCallback dsrCallback = new UsbSerialInterface.UsbDSRCallback() {
        @Override
        public void onDSRChanged(boolean state) {
            if(handler != null)
                handler.obtainMessage(OpenBciService.DSR_CHANGE).sendToTarget();
        }
    };
    /*
     * Different notifications from OS will be received here (USB attached, detached, permission responses...)
     * About BroadcastReceiver: http://developer.android.com/reference/android/content/BroadcastReceiver.html
     */
    private final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context arg0, Intent arg1) {
            if (arg1.getAction().equals(UsbService.ACTION_USB_PERMISSION)) {
                boolean granted = arg1.getExtras().getBoolean(UsbManager.EXTRA_PERMISSION_GRANTED);
                if (granted) // User accepted our USB connection. Try to open the device as a serial port
                {
                    Intent intent = new Intent(UsbService.ACTION_USB_PERMISSION_GRANTED);
                    arg0.sendBroadcast(intent);
                    connection = usbManager.openDevice(device);
                    serialPortConnected = true;
                    handler.obtainMessage(OpenBciService.SERIAL_PORT_CONNECTED).sendToTarget();
                    new ConnectionThread().run();
                } else // User not accepted our USB connection. Send an Intent to the Main Activity
                {
                    Intent intent = new Intent(UsbService.ACTION_USB_PERMISSION_NOT_GRANTED);
                    arg0.sendBroadcast(intent);
                }
            } else if (arg1.getAction().equals(UsbService.ACTION_USB_ATTACHED)) {
                if (!serialPortConnected)
                    findSerialPortDevice(); // A USB device has been attached. Try to open it as a Serial port
            } else if (arg1.getAction().equals(UsbService.ACTION_USB_DETACHED)) {
                // Usb device was disconnected. send an intent to the Main Activity
                Intent intent = new Intent(UsbService.ACTION_USB_DISCONNECTED);
                arg0.sendBroadcast(intent);
                serialPortConnected = false;
                serialPort.close();
                handler.obtainMessage(OpenBciService.SERIAL_PORT_DISCONNECTED).sendToTarget();
            }
        }
    };

    /*
     * onCreate will be executed when service is started. It configures an IntentFilter to listen for
     * incoming Intents (USB ATTACHED, USB DETACHED...) and it tries to open a serial port.
     */
    @Override
    public void onCreate() {
        this.context = this;
        serialPortConnected = false;
        OpenBciService.SERVICE_CONNECTED = true;
        setFilter();
        usbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
        findSerialPortDevice();
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        return Service.START_NOT_STICKY;
    }

    @Override
    public void onDestroy() {
        super.onDestroy();
        OpenBciService.SERVICE_CONNECTED = false;
    }

    /*
     * This function will be called from MainActivity to write data through Serial Port
     */
    public void write(byte[] data) {
        if (serialPort != null)
            serialPort.write(data);
    }

    private void findSerialPortDevice() {
        // This snippet will try to open the first encountered usb device connected, excluding usb root hubs
        HashMap<String, UsbDevice> usbDevices = usbManager.getDeviceList();
        if (!usbDevices.isEmpty()) {
            boolean keep = true;
            for (Map.Entry<String, UsbDevice> entry : usbDevices.entrySet()) {
                device = entry.getValue();
                int deviceVID = device.getVendorId();
                int devicePID = device.getProductId();

                if(deviceVID == OPENBCI_VENDOR_ID) {
                    // There is a device connected to our Android device. Try to open it as a Serial Port.
                    requestUserPermission();
                    keep = false;
                } else {
                    connection = null;
                    device = null;
                }

                if (!keep)
                    break;
            }
            if (!keep) {
                // There is no USB devices connected (but usb host were listed). Send an intent to MainActivity.
                Intent intent = new Intent(UsbService.ACTION_NO_USB);
                sendBroadcast(intent);
            }
        } else {
            // There is no USB devices connected. Send an intent to MainActivity
            Intent intent = new Intent(UsbService.ACTION_NO_USB);
            sendBroadcast(intent);
        }
    }

    private void setFilter() {
        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbService.ACTION_USB_PERMISSION);
        filter.addAction(UsbService.ACTION_USB_DETACHED);
        filter.addAction(UsbService.ACTION_USB_ATTACHED);
        registerReceiver(usbReceiver, filter);
    }

    /*
     * Request user permission. The response will be received in the BroadcastReceiver
     */
    private void requestUserPermission() {
        PendingIntent mPendingIntent = PendingIntent.getBroadcast(this, 0, new Intent(UsbService.ACTION_USB_PERMISSION), 0);
        usbManager.requestPermission(device, mPendingIntent);
    }

    /*
     * A simple thread to open a serial port.
     * Although it should be a fast operation. moving usb operations away from UI thread is a good thing.
     */
    private class ConnectionThread extends Thread {
        @Override
        public void run() {
            serialPort = UsbSerialDevice.createUsbSerialDevice(device, connection);
            if (serialPort != null) {
                if (serialPort.open()) {
                    serialPort.setBaudRate(BAUD_RATE);
                    serialPort.setDataBits(UsbSerialInterface.DATA_BITS_8);
                    serialPort.setStopBits(UsbSerialInterface.STOP_BITS_1);
                    serialPort.setParity(UsbSerialInterface.PARITY_NONE);
                    /**
                     * Current flow control Options:
                     * UsbSerialInterface.FLOW_CONTROL_OFF
                     * UsbSerialInterface.FLOW_CONTROL_RTS_CTS only for CP2102 and FT232
                     * UsbSerialInterface.FLOW_CONTROL_DSR_DTR only for CP2102 and FT232
                     */
                    serialPort.setFlowControl(UsbSerialInterface.FLOW_CONTROL_OFF);
                    serialPort.read(mCallback);
                    serialPort.getCTS(ctsCallback);
                    serialPort.getDSR(dsrCallback);
                    // Everything went as expected. Send an intent to MainActivity
                    Intent intent = new Intent(UsbService.ACTION_USB_READY);
                    context.sendBroadcast(intent);
                } else {
                    // Serial port could not be opened, maybe an I/O error or if CDC driver was chosen, it does not really fit
                    // Send an Intent to Main Activity
                    if (serialPort instanceof CDCSerialDevice) {
                        Intent intent = new Intent(UsbService.ACTION_CDC_DRIVER_NOT_WORKING);
                        context.sendBroadcast(intent);
                    } else {
                        Intent intent = new Intent(UsbService.ACTION_USB_DEVICE_NOT_WORKING);
                        context.sendBroadcast(intent);
                    }
                }
            } else {
                // No driver for given device, even generic CDC driver could not be loaded
                Intent intent = new Intent(UsbService.ACTION_USB_NOT_SUPPORTED);
                context.sendBroadcast(intent);
            }
        }
    }

    public void stop() {
        write("s".getBytes());
        //write("v".getBytes());
        isStreaming = false;
    }

    public void start() {
        write("b".getBytes());
    }

    public boolean isStreaming() {
        return isStreaming;
    }

    public boolean isSerialPortConnected() {
        return serialPortConnected;
    }

    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return binder;
    }

    private int interpret24bitAsInt32(byte[] byteArray) {
        int newInt = (
                ((0xFF & byteArray[0]) << 16) |
                        ((0xFF & byteArray[1]) << 8) |
                        (0xFF & byteArray[2])
        );
        if ((newInt & 0x00800000) > 0) {
            newInt |= 0xFF000000;
        } else {
            newInt &= 0x00FFFFFF;
        }
        return newInt;
    }

    int interpret16bitAsInt32(byte[] byteArray) {
        int newInt = (
                ((0xFF & byteArray[0]) << 8) |
                        (0xFF & byteArray[1])
        );
        if ((newInt & 0x00008000) > 0) {
            newInt |= 0xFFFF0000;
        } else {
            newInt &= 0x0000FFFF;
        }
        return newInt;
    }

    public void setHandler(Handler handler) {
        this.handler = handler;
    }
}
