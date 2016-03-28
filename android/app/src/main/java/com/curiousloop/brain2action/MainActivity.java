package com.curiousloop.brain2action;

import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.content.res.ColorStateList;
import android.graphics.Color;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.support.v7.widget.Toolbar;
import android.view.View;
import android.widget.Button;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import java.lang.ref.WeakReference;
import java.util.Set;

public class MainActivity extends AppCompatActivity {

    private final BroadcastReceiver usbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            switch (intent.getAction()) {
                case UsbService.ACTION_USB_PERMISSION_GRANTED: // USB PERMISSION GRANTED
                    Toast.makeText(context, "USB Ready", Toast.LENGTH_SHORT).show();
                    break;
                case UsbService.ACTION_USB_PERMISSION_NOT_GRANTED: // USB PERMISSION NOT GRANTED
                    Toast.makeText(context, "USB Permission not granted", Toast.LENGTH_SHORT).show();
                    break;
                case UsbService.ACTION_NO_USB: // NO USB CONNECTED
                    Toast.makeText(context, "No USB connected", Toast.LENGTH_SHORT).show();
                    break;
                case UsbService.ACTION_USB_DISCONNECTED: // USB DISCONNECTED
                    Toast.makeText(context, "USB disconnected", Toast.LENGTH_SHORT).show();
                    break;
                case UsbService.ACTION_USB_NOT_SUPPORTED: // USB NOT SUPPORTED
                    Toast.makeText(context, "USB device not supported", Toast.LENGTH_SHORT).show();
                    break;
            }
        }
    };

    private OpenBciService bciService;
    private MotionSuitService suitService;

    private MyHandler handler;

    private Button findDevicesButton;
    private Button startBciButton;
    private Button startSuitButton;
    private TextView bciConnectedTextview, bciReceivedTextview, bciSentTextview;
    private TextView suitConnectedTextview, suitReceivedTextview, suitSentTextview;


    private final ServiceConnection bciUsbConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName arg0, IBinder arg1) {
            bciService = ((OpenBciService.UsbBinder) arg1).getService();
            bciService.setHandler(handler);
        }

        @Override
        public void onServiceDisconnected(ComponentName arg0) {
            bciService = null;
        }
    };
    private final ServiceConnection suitUsbConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName arg0, IBinder arg1) {
            suitService = ((MotionSuitService.UsbBinder) arg1).getService();
            suitService.setHandler(handler);
        }

        @Override
        public void onServiceDisconnected(ComponentName arg0) {
            suitService = null;
        }
    };


    private static class MyHandler extends Handler {
        private final WeakReference<MainActivity> mActivity;

        public MyHandler(MainActivity activity) {
            mActivity = new WeakReference<>(activity);
        }

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case OpenBciService.MESSAGE_FROM_SERIAL_PORT:
                    mActivity.get().setBciSamplesReceived((String)msg.obj);
                    break;
                case OpenBciService.SERIAL_PORT_CONNECTED:
                    mActivity.get().setUIBciConnected();
                    break;
                case OpenBciService.SERIAL_PORT_DISCONNECTED:
                    mActivity.get().setUIBciDisconnected();
                    break;
                case OpenBciService.CTS_CHANGE:
                    Toast.makeText(mActivity.get(), "CTS_CHANGE",Toast.LENGTH_LONG).show();
                    break;
                case OpenBciService.DSR_CHANGE:
                    Toast.makeText(mActivity.get(), "DSR_CHANGE",Toast.LENGTH_LONG).show();
                    break;
            }
        }
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        handler = new MyHandler(this);

        findDevicesButton = (Button) findViewById(R.id.find_devices_button);
        startBciButton = (Button) findViewById(R.id.start_bci_button);
        startSuitButton = (Button) findViewById(R.id.start_suit_button);
        bciConnectedTextview = (TextView) findViewById(R.id.bci_connected_textview);
        bciReceivedTextview = (TextView) findViewById(R.id.bci_received_textview);
        bciSentTextview = (TextView) findViewById(R.id.bci_sent_textview);
        suitConnectedTextview = (TextView) findViewById(R.id.suit_connected_textview);
        suitReceivedTextview = (TextView) findViewById(R.id.suit_received_textview);
        suitSentTextview = (TextView) findViewById(R.id.suit_sent_textview);

        setUIBciDisconnected();
        setUISuitDisconnected();
    }

    private void setUIBciConnected() {
        bciConnectedTextview.setText("yes");
        bciConnectedTextview.setTextColor(Color.parseColor("#006400"));
        startBciButton.setEnabled(true);
    }

    private void setUIBciDisconnected() {
        bciConnectedTextview.setText("no");
        bciConnectedTextview.setTextColor(Color.parseColor("#ff0000"));
        startBciButton.setEnabled(false);
    }

    private void setUISuitConnected() {
        suitConnectedTextview.setText("yes");
        suitConnectedTextview.setTextColor(Color.parseColor("#006400"));
        startSuitButton.setEnabled(true);
    }

    private void setUISuitDisconnected() {
        suitConnectedTextview.setText("no");
        suitConnectedTextview.setTextColor(Color.parseColor("#ff0000"));
        startSuitButton.setEnabled(false);
    }

    private void setBciSamplesReceived(String value) {
        bciReceivedTextview.setText(value);
        bciReceivedTextview.setTextColor(Color.parseColor("#000080"));
    }

    private void setSuitSamplesReceived(int value) {
        suitReceivedTextview.setText(value);
        suitReceivedTextview.setTextColor(Color.parseColor("#000080"));
    }

    @Override
    public void onResume() {
        super.onResume();
        setFilters();  // Start listening notifications from OpenBciService
        startService(OpenBciService.class, bciUsbConnection, null);
        startService(MotionSuitService.class, suitUsbConnection, null);
    }

    @Override
    public void onPause() {
        super.onPause();
        unregisterReceiver(usbReceiver);
        unbindService(bciUsbConnection);
        unbindService(suitUsbConnection);
    }

    private void setFilters() {
        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbService.ACTION_USB_PERMISSION_GRANTED);
        filter.addAction(UsbService.ACTION_NO_USB);
        filter.addAction(UsbService.ACTION_USB_DISCONNECTED);
        filter.addAction(UsbService.ACTION_USB_NOT_SUPPORTED);
        filter.addAction(UsbService.ACTION_USB_PERMISSION_NOT_GRANTED);
        registerReceiver(usbReceiver, filter);
    }

    private void startService(Class<?> service, ServiceConnection serviceConnection, Bundle extras) {
        boolean serviceConnected = service == OpenBciService.class ? !OpenBciService.SERVICE_CONNECTED : !MotionSuitService.SERVICE_CONNECTED;
        if (serviceConnected) {
            Intent startService = new Intent(this, service);
            if (extras != null && !extras.isEmpty()) {
                Set<String> keys = extras.keySet();
                for (String key : keys) {
                    String extra = extras.getString(key);
                    startService.putExtra(key, extra);
                }
            }
            startService(startService);
        }
        Intent bindingIntent = new Intent(this, service);
        bindService(bindingIntent, serviceConnection, Context.BIND_AUTO_CREATE);
    }

    public void onClickFindDevices(View view) {

    }

    public void onClickStartBci(View view) {
        if(bciService != null) {
            if(bciService.isStreaming()) {
                bciService.stop();
                startBciButton.setText("Start bci");
            } else {
                bciService.start();
                startBciButton.setText("Stop bci");
            }
        }
    }

    public void onClickStartSuit(View view) {
        if(suitService != null) {
            if(suitService.isStreaming()) {
                suitService.stop();
                startSuitButton.setText("Start suit");
            } else {
                suitService.start();
                startSuitButton.setText("Stop suit");
            }
        }
    }

}
