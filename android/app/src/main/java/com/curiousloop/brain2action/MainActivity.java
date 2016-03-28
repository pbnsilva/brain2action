package com.curiousloop.brain2action;

import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
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
                case OpenBciService.ACTION_USB_PERMISSION_GRANTED: // USB PERMISSION GRANTED
                    Toast.makeText(context, "USB Ready", Toast.LENGTH_SHORT).show();
                    break;
                case OpenBciService.ACTION_USB_PERMISSION_NOT_GRANTED: // USB PERMISSION NOT GRANTED
                    Toast.makeText(context, "USB Permission not granted", Toast.LENGTH_SHORT).show();
                    break;
                case OpenBciService.ACTION_NO_USB: // NO USB CONNECTED
                    Toast.makeText(context, "No USB connected", Toast.LENGTH_SHORT).show();
                    break;
                case OpenBciService.ACTION_USB_DISCONNECTED: // USB DISCONNECTED
                    Toast.makeText(context, "USB disconnected", Toast.LENGTH_SHORT).show();
                    break;
                case OpenBciService.ACTION_USB_NOT_SUPPORTED: // USB NOT SUPPORTED
                    Toast.makeText(context, "USB device not supported", Toast.LENGTH_SHORT).show();
                    break;
            }
        }
    };

    private OpenBciService bciService;
    private MyHandler handler;

    private Button findDevicesButton;
    private Button startBciButton;
    private TextView bciTextview;
    private ScrollView bciScrollview;

    private boolean isReset = false;
    private boolean isStart = false;

    private final ServiceConnection usbConnection = new ServiceConnection() {
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

    private static class MyHandler extends Handler {
        private final WeakReference<MainActivity> mActivity;

        public MyHandler(MainActivity activity) {
            mActivity = new WeakReference<>(activity);
        }

        @Override
        public void handleMessage(Message msg) {
            switch (msg.what) {
                case OpenBciService.MESSAGE_FROM_SERIAL_PORT:
                    String data = (String) msg.obj;
                    mActivity.get().bciTextview.append(data);
                    mActivity.get().bciScrollview.fullScroll(View.FOCUS_DOWN);
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
        bciTextview = (TextView) findViewById(R.id.bci_textview);
        bciScrollview = (ScrollView) findViewById(R.id.bci_scrollview);
    }

    @Override
    public void onResume() {
        super.onResume();
        setFilters();  // Start listening notifications from OpenBciService
        startService(OpenBciService.class, usbConnection, null); // Start OpenBciService(if it was not started before) and Bind it
    }

    @Override
    public void onPause() {
        super.onPause();
        unregisterReceiver(usbReceiver);
        unbindService(usbConnection);
    }

    private void setFilters() {
        IntentFilter filter = new IntentFilter();
        filter.addAction(OpenBciService.ACTION_USB_PERMISSION_GRANTED);
        filter.addAction(OpenBciService.ACTION_NO_USB);
        filter.addAction(OpenBciService.ACTION_USB_DISCONNECTED);
        filter.addAction(OpenBciService.ACTION_USB_NOT_SUPPORTED);
        filter.addAction(OpenBciService.ACTION_USB_PERMISSION_NOT_GRANTED);
        registerReceiver(usbReceiver, filter);
    }

    private void startService(Class<?> service, ServiceConnection serviceConnection, Bundle extras) {
        if (!OpenBciService.SERVICE_CONNECTED) {
            Toast.makeText(MainActivity.this, "starting service", Toast.LENGTH_SHORT).show();
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
        /*if(!isReset) {
            bciService.write("v".getBytes());
            isReset = true;
        } else */if(!isStart) {
            bciService.write("b".getBytes());
            isStart = true;
        } else {
            bciService.write("s".getBytes());
            isStart = false;
        }
    }
}
