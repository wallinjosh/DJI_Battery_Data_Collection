package utm.aeroe.edu.dji_battery_data_collection;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.IntentFilter;
import android.os.AsyncTask;
import android.os.Build;
import android.os.Environment;
import android.support.v4.app.ActivityCompat;
import android.support.v4.app.FragmentActivity;
import android.os.Bundle;

import android.content.Intent;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;

import dji.common.battery.BatteryState;
import dji.common.error.DJIError;
import dji.common.error.DJISDKError;
import dji.common.flightcontroller.FlightControllerState;
import dji.common.mission.waypoint.Waypoint;
import dji.common.mission.waypoint.WaypointMission;
import dji.common.mission.waypoint.WaypointMissionFinishedAction;
import dji.common.mission.waypoint.WaypointMissionHeadingMode;
import dji.common.model.LocationCoordinate2D;
import dji.common.util.CommonCallbacks;
import dji.sdk.base.BaseComponent;
import dji.sdk.base.BaseProduct;
import dji.sdk.battery.Battery;
import dji.sdk.flightcontroller.FlightController;
import dji.sdk.mission.MissionControl;
import dji.sdk.mission.timeline.TimelineElement;
import dji.sdk.mission.timeline.actions.GoToAction;
import dji.sdk.mission.timeline.actions.TakeOffAction;
import dji.sdk.mission.waypoint.WaypointMissionOperator;
import dji.sdk.products.Aircraft;
import dji.sdk.sdkmanager.DJISDKManager;

import static utm.aeroe.edu.dji_battery_data_collection.BuildConfig.DEBUG;

public class MainActivity extends FragmentActivity implements View.OnClickListener, OnMapReadyCallback {

    private static final String TAG = MainActivity.class.getName();
    public static final String connection_change = "Connection change in DJI SDK";
    private static final String CSV_FILE_HEADER_LOG_FILE = "Time, Lat, Long, V_X, V_Y, V_Z, VOLTAGE(mV), CURRENT(mA)\n";
    private static final String CSV_FILE_HEADER_WAYPOINTS_FILE = "Lat, Long";
    //This is to help prevent possible mistakes in giving waypoint distances (e.g. accidentally giving a distance of 10000m)
    private static final double MAX_WAYPOINT_DISTANCE_M = 1000.0;

    /**
    private static final boolean DEBUG = false;
    private static final double[][] waypoint_coords = {{42.0736891, -93.6250437}, {42.0736891, -93.6250437}, {42.0736891, -93.6250437}};
    **/

    private static BaseProduct mProduct;
    private FlightController mFlightController;
    private Battery mDJIBattery;
    public static WaypointMission.Builder waypointMissionBuilder;
    private WaypointMissionOperator instance;
    private WaypointMissionFinishedAction mFinishedAction = WaypointMissionFinishedAction.NO_ACTION;
    private WaypointMissionHeadingMode mHeadingMode = WaypointMissionHeadingMode.USING_WAYPOINT_HEADING;

    public long initialTime;
    private double drone_lat = 42.0266795;
    private double drone_long = -93.6535388;
    private double drone_v_x = 0.0;
    private double drone_v_y = 0.0;
    private double drone_v_z = 0.0;
    private double drone_battery_voltage = 0.0;


    private double drone_battery_current = 0.0;
    private LatLng drone_pos;

    private ArrayList<Waypoint> mission_waypoints;
    private boolean mission_useDistance;
    private boolean mission_started;
    private float mission_velocity;
    private double mission_distance;
    private double mission_duration;
    private double mission_heading;
    private float mission_altitude;
    private int mission_laps;

    private Date FLIGHT_START_TIME;

    private boolean first = false;

    MissionControl mc;

    private Handler mHandler;
    File log_file;
    File waypoints_file;
    private RecordData rd;
    private WriteWaypoints ww;

    private GoogleMap gmap;
    private Marker drone_marker;
    private ArrayList<Marker> waypoint_markers;
    private EditText i_velocity;
    private EditText i_distance;
    private EditText i_duration;
    private EditText i_heading;
    private EditText i_altitude;
    private EditText i_laps;
    private Button start;
    private Button stop;
    private RadioButton selectDist;
    private RadioButton selectTime;

    private TimelineElement cur;

    private final String reg_success = "SDK Registration Success";

    public static synchronized BaseProduct getProductInstance() {
        if (mProduct == null) {
            mProduct = DJISDKManager.getInstance().getProduct();
        }
        return mProduct;
    }

    private DJISDKManager.SDKManagerCallback mDJISDKManagerCallback = new DJISDKManager.SDKManagerCallback() {
        @Override
        public void onRegister(DJIError error) {
            Log.d(TAG, error == null ? reg_success : error.getDescription());
            Handler handler = new Handler(Looper.getMainLooper());
            if(error == DJISDKError.REGISTRATION_SUCCESS) {
                DJISDKManager.getInstance().startConnectionToProduct();
                handler.post(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(getApplicationContext(), reg_success, Toast.LENGTH_LONG).show();
                    }
                });
            } else {
                handler.post(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(getApplicationContext(), "SDK Registration Failed: Attempt to restart application", Toast.LENGTH_LONG).show();
                    }
                });
            }
            Log.e("TAG", error.toString());
        }
        @Override
        public void onProductChange(BaseProduct oldProduct, BaseProduct newProduct) {
            mProduct = newProduct;
            if(mProduct != null) {
                mProduct.setBaseProductListener(mDJIBaseProductListener);
            }
            notifyStatusChange();
        }
    };

    public WaypointMissionOperator getWaypointMissionOperator() {
        if (instance == null) {
            instance = DJISDKManager.getInstance().getMissionControl().getWaypointMissionOperator();
        }
        return instance;
    }

    private BaseProduct.BaseProductListener mDJIBaseProductListener = new BaseProduct.BaseProductListener() {
        @Override
        public void onComponentChange(BaseProduct.ComponentKey key, BaseComponent oldComponent, BaseComponent newComponent) {
            if(newComponent != null) {
                newComponent.setComponentListener(mDJIComponentListener);
            }
            notifyStatusChange();
        }
        @Override
        public void onConnectivityChange(boolean isConnected) {
            notifyStatusChange();
        }
    };
    private BaseComponent.ComponentListener mDJIComponentListener = new BaseComponent.ComponentListener() {
        @Override
        public void onConnectivityChange(boolean isConnected) {
            notifyStatusChange();
        }
    };

    public void constructLapMission(){

        double distanceToTravel;

        if (mission_useDistance)
            distanceToTravel = mission_distance;
        else {
            distanceToTravel = mission_duration * mission_velocity;
        }

        double Lat = drone_lat;
        double Long = drone_long;

        //Calculates final destination of drone (accounts for curve of Earth -- may not be necessary on this scale)
        double d_lat = (distanceToTravel * Math.cos(mission_heading * 180.0 / Math.PI)) / 110540.0;
        double d_long = (distanceToTravel * Math.sin(mission_heading * 180.0 / Math.PI)) / (111320.0 * Math.cos(Lat));
        GoToAction new_point;

        //Take off
        DJISDKManager.getInstance().getMissionControl().scheduleElement(new TakeOffAction());
        //Ensure rise to mission altitude -- app currently not designed to change altitude during flight
        DJISDKManager.getInstance().getMissionControl().scheduleElement(new GoToAction(new LocationCoordinate2D(drone_lat, drone_long), mission_altitude));

        //Add all waypoints to mission -- for each lap, just adds point at given distance and initial point repeatedly
        for(int i = 0; i < mission_laps; i++){
            new_point = new GoToAction(new LocationCoordinate2D(Lat + d_lat, Long + d_long), mission_altitude);
            //new_point.setFlightSpeed(mission_velocity);
            DJISDKManager.getInstance().getMissionControl().scheduleElement(new_point);

            new_point = new GoToAction(new LocationCoordinate2D(Lat, Long), mission_altitude);
            //new_point.setFlightSpeed(mission_velocity);
            DJISDKManager.getInstance().getMissionControl().scheduleElement(new_point);
        }

        //Executes mission after adding all points
        DJISDKManager.getInstance().getMissionControl().startTimeline();

    }

    /*
        Currently unused method
     */
    public void constructWayPointMission(){

        if(DEBUG){
            Waypoint w;
            for(int i = 0; i < 3; i++){
                w = new Waypoint(drone_lat + (i/1000.0), drone_long + (i/1000.0), mission_altitude);
                mission_waypoints.add(w);
            }
        }else {
            double distanceToTravel;

            if (mission_useDistance)
                distanceToTravel = mission_distance;
            else {
                distanceToTravel = mission_duration * mission_velocity;
            }

            double Lat = drone_lat;
            double Long = drone_long;

            DJISDKManager.getInstance().getMissionControl().scheduleElement(new TakeOffAction());
            DJISDKManager.getInstance().getMissionControl().scheduleElement(new GoToAction(new LocationCoordinate2D(drone_lat, drone_long), (float) mission_altitude));

            while (distanceToTravel > MAX_WAYPOINT_DISTANCE_M) {
                //Assume heading is degree measure with 0 = NORTH, 90 = EAST, 180=SOUTH, 270=WEST
//            Lat = Math.asin(Math.sin(Lat) * Math.cos(MAX_WAYPOINT_DISTANCE_M)
//                + Math.cos(Lat)*Math.sin(MAX_WAYPOINT_DISTANCE_M)*Math.cos((mission_heading) * Math.PI / 180.0));
//            if(Math.cos(Lat) != 0.0){
//                Long = ((Long - Math.asin(Math.sin(((mission_heading) * Math.PI / 180.0)))
//                    *Math.sin(MAX_WAYPOINT_DISTANCE_M) + Math.PI) % (2*Math.PI)) - Math.PI;

                //Let's de-complicate the math; it's just planar geometry
                double d_lat = (MAX_WAYPOINT_DISTANCE_M * Math.cos(mission_heading * 180.0 / Math.PI)) / 110540;
                double d_long = (MAX_WAYPOINT_DISTANCE_M * Math.sin(mission_heading * 180.0 / Math.PI)) / (111320 * Math.cos(Lat));
                Lat += d_lat;
                Long += d_long;
                Log.d("POINT TO VISIT: ", Lat + "," + Long);
                //mission_waypoints.add(new Waypoint(Lat, Long, mission_altitude));

                GoToAction new_point = new GoToAction(new LocationCoordinate2D(Lat, Long), mission_altitude);
                DJISDKManager.getInstance().getMissionControl().scheduleElement(new_point);

                distanceToTravel -= MAX_WAYPOINT_DISTANCE_M;

            }

            double d_long = (distanceToTravel * Math.cos(mission_heading * 180.0 / Math.PI)) / (111320 * Math.cos(Lat));
            double d_lat = (distanceToTravel * Math.sin(mission_heading * 180.0 / Math.PI)) / 110540;
            Lat += d_lat;
            Long += d_long;
            Log.d("POINT TO VISIT: ", Lat + "," + Long);
            GoToAction new_point = new GoToAction(new LocationCoordinate2D(Lat, Long), mission_altitude);
            DJISDKManager.getInstance().getMissionControl().scheduleElement(new_point);

            //mission_waypoints.add(new Waypoint(Lat, Long, mission_altitude));

//            if(mission_waypoints.size() < 2){
//                mission_waypoints.add(new Waypoint(mission_waypoints.get(0).coordinate.getLatitude() + 0.0001,
//                        mission_waypoints.get(0).coordinate.getLongitude() + 0.0001,
//                        mission_waypoints.get(0).altitude));
//                mission_waypoints.add(new Waypoint(mission_waypoints.get(0).coordinate.getLatitude() + 0.0002,
//                        mission_waypoints.get(0).coordinate.getLongitude() + 0.0002,
//                        mission_waypoints.get(0).altitude));
//            }


        }
        runOnUiThread(new Runnable(){
            @Override
            public void run(){
                new WriteWaypoints().execute();
            }
        });

//        if (waypointMissionBuilder == null){
//            waypointMissionBuilder = new WaypointMission.Builder().finishedAction(mFinishedAction)
//                    .headingMode(mHeadingMode)
//                    .autoFlightSpeed(mission_velocity)
//                    .maxFlightSpeed(mission_velocity)
//                    .flightPathMode(WaypointMissionFlightPathMode.NORMAL);
//        }else
//        {
//            waypointMissionBuilder.finishedAction(mFinishedAction)
//                    .headingMode(mHeadingMode)
//                    .autoFlightSpeed(mission_velocity)
//                    .maxFlightSpeed(mission_velocity)
//                    .flightPathMode(WaypointMissionFlightPathMode.NORMAL);
//        }
//
//        for(Waypoint i : mission_waypoints){
//            waypointMissionBuilder.addWaypoint(i);
//        }
//
//        DJIError param_error = waypointMissionBuilder.checkParameters();
//
//        Log.d("PARAM ERROR: ", param_error.getDescription());
//
//        DJIError error = getWaypointMissionOperator().loadMission(waypointMissionBuilder.build());
//
//        uploadWayPointMission();

        DJISDKManager.getInstance().getMissionControl().startTimeline();
        Log.d("TIMELINE: ", "" + DJISDKManager.getInstance().getMissionControl().isTimelineRunning());
            Log.d("TIMELINE EVENT", "" + DJISDKManager.getInstance().getMissionControl().getCurrentTimelineMarker());
        Log.d("EVENTS SCHEDULED: ", "" + DJISDKManager.getInstance().getMissionControl().scheduledCount());

    }

    private void uploadWayPointMission(){
        getWaypointMissionOperator().uploadMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                final DJIError ecopy = error;
                if (error == null) {
                    runOnUiThread(new Runnable(){
                        public void run(){
                            Toast.makeText(getApplicationContext(), "Mission upload successfully!", Toast.LENGTH_LONG).show();}
                    });
                    startWaypointMission();
                } else {
                    runOnUiThread(new Runnable(){
                        public void run(){
                            Toast.makeText(getApplicationContext(), "Mission upload failed, error: " + ecopy.getDescription() + " retrying...", Toast.LENGTH_LONG).show();}
                    });
                    getWaypointMissionOperator().retryUploadMission(null);
                }
            }
        });
    }

    private void startWaypointMission(){
        getWaypointMissionOperator().startMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                final DJIError ecopy = error;
                Log.d("ERROR: ", ecopy.getDescription());
                runOnUiThread(new Runnable(){
                    public void run(){
                         Toast.makeText(getApplicationContext(), "Mission Start: " + (ecopy == null ? "Successfully" : ecopy .getDescription()), Toast.LENGTH_LONG).show();}

                    });
                    mission_started = true;
            }
        });
    }
    private void stopWaypointMission(){
        getWaypointMissionOperator().stopMission(new CommonCallbacks.CompletionCallback() {
            @Override
            public void onResult(DJIError error) {
                Toast.makeText(getApplicationContext(), "Mission Stop: " + (error == null ? "Successfully" : error.getDescription()), Toast.LENGTH_LONG).show();
                mission_started = false;
            }
        });
    }

    private void notifyStatusChange() {
        mHandler.removeCallbacks(updateRunnable);
        mHandler.postDelayed(updateRunnable, 500);
    }
    private Runnable updateRunnable = new Runnable() {
        @Override
        public void run() {
            Intent intent = new Intent(connection_change);
            sendBroadcast(intent);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        mHandler = new Handler(Looper.getMainLooper());

       if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            ActivityCompat.requestPermissions(this,
                    new String[]{"android.permission.WRITE_EXTERNAL_STORAGE", "android.permission.VIBRATE",
                            "android.permission.INTERNET", "android.permission.ACCESS_WIFI_STATE",
                            "android.permission.WAKE_LOCK", "android.permission.ACCESS_COARSE_LOCATION",
                            "android.permission.ACCESS_NETWORK_STATE", "android.permission.ACCESS_FINE_LOCATION",
                            "android.permission.CHANGE_WIFI_STATE", "android.permission.MOUNT_UNMOUNT_FILESYSTEMS",
                            "android.permission.READ_EXTERNAL_STORAGE", "android.permission.WRITE_EXTERNAL_STORAGE", "android.permission.SYSTEM_ALERT_WINDOW",
                            "android.permission.READ_PHONE_STATE"
                    }
                    , 1);
        }

        DJISDKManager.getInstance().registerApp(this, mDJISDKManagerCallback);

        setContentView(R.layout.activity_main);
        mHandler = new Handler(Looper.getMainLooper());

        start = (Button) findViewById(R.id.start);
        start.setOnClickListener(this);

        stop = (Button) findViewById(R.id.stop);
        stop.setOnClickListener(this);

        mission_started = false;

        mission_waypoints = new ArrayList<Waypoint>();

        IntentFilter filter = new IntentFilter();
        filter.addAction(connection_change);
        registerReceiver(mReceiver, filter);

        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager()
                .findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);

        if(FLIGHT_START_TIME == null){
            FLIGHT_START_TIME = Calendar.getInstance().getTime();
        }

        //If it doesn't already exist, makes directory for storing DJI Log Files
        log_file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), "DJI_LOG_FILES");
        log_file.mkdirs();
        //Makes particular log file used for this flight
        log_file = new File(log_file, "FLIGHT_LOG-" + FLIGHT_START_TIME + ".csv");
        waypoints_file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOCUMENTS), "DJI_LOG_FILES");
        waypoints_file = new File(waypoints_file, "WAYPOINTS-" + FLIGHT_START_TIME + ".csv");

        try {
            FileWriter mFileWriter = new FileWriter(log_file.getAbsolutePath(), true);
            mFileWriter.write(CSV_FILE_HEADER_LOG_FILE);
            mFileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        try {
            FileWriter mFileWriter = new FileWriter(waypoints_file.getAbsolutePath(), true);
            mFileWriter.write(CSV_FILE_HEADER_WAYPOINTS_FILE);
            mFileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        initialTime = Calendar.getInstance().getTimeInMillis();
        rd = new RecordData();
        rd.execute();
    }

    @Override
    public void onDestroy(){
        super.onDestroy();
        DJISDKManager.getInstance().stopConnectionToProduct();
    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        if (gmap == null) {
            gmap = googleMap;
        }
        gmap.moveCamera(CameraUpdateFactory.newLatLngZoom(new LatLng(drone_lat, drone_long), 18.0f));
    }

    protected BroadcastReceiver mReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            onProductConnectionChange();
        }
    };

    private void onProductConnectionChange()
    {
            initFlightBatteryController();
    }

    private void initFlightBatteryController() {
        BaseProduct product = getProductInstance();
        if (product != null && product.isConnected() && product instanceof Aircraft) {
                mFlightController = ( (Aircraft) product).getFlightController();
                mDJIBattery = product.getBattery();
        }
        if (mFlightController != null) {
            mFlightController.setStateCallback(new FlightControllerState.Callback() {
                @Override
                public void onUpdate(FlightControllerState djiFlightControllerCurrentState) {
                    drone_lat = djiFlightControllerCurrentState.getAircraftLocation().getLatitude();
                    drone_long = djiFlightControllerCurrentState.getAircraftLocation().getLongitude();
                    drone_pos = new LatLng(drone_lat, drone_long);
                    drone_v_x = djiFlightControllerCurrentState.getVelocityX();
                    drone_v_y = djiFlightControllerCurrentState.getVelocityY();
                    drone_v_z = djiFlightControllerCurrentState.getVelocityZ();
                    if(DJISDKManager.getInstance().getMissionControl().isTimelineRunning())
                        Log.d("TIMELINE EVENT", "" + DJISDKManager.getInstance().getMissionControl().getCurrentTimelineMarker());
                    updateMap();
                    new RecordData().execute();
                }
            });
        }
        if (mDJIBattery != null){
            mDJIBattery.setStateCallback(new BatteryState.Callback() {
                @Override
                public void onUpdate(BatteryState batteryState) {
                    drone_battery_voltage = batteryState.getVoltage();
                    /*
                        FROM DJI DOCUMENTATION:
                        "Returns the real time current draw of the battery (mA). A negative value means the battery is being discharged, and a positive value means it is being charged."
                     */
                    drone_battery_current = batteryState.getCurrent();
                    new RecordData().execute();
                }
            });
        }
    }

    @Override
    public void onClick(View v) {
        if(v.getId() == R.id.start){
            i_velocity = (EditText) findViewById(R.id.velocity);
            i_heading = (EditText) findViewById(R.id.heading);
            i_duration = (EditText) findViewById(R.id.duration);
            i_distance = (EditText) findViewById(R.id.distance);
            i_altitude = (EditText) findViewById(R.id.altitude);
            i_laps = (EditText) findViewById(R.id.laps);
            selectDist = (RadioButton) findViewById(R.id.selectDistance);
            selectTime = (RadioButton) findViewById(R.id.selectTime);
            if(validateInput()){
                mission_velocity = Float.valueOf(i_velocity.getText().toString());
                mission_distance = Double.valueOf(i_distance.getText().toString());
                mission_duration = Double.valueOf(i_duration.getText().toString());
                mission_heading = Double.valueOf(i_heading.getText().toString());
                mission_altitude = Float.valueOf(i_altitude.getText().toString());
                mission_laps = Integer.valueOf(i_laps.getText().toString());
                mission_useDistance = selectDist.isChecked();
                mission_started = true;
                Toast.makeText(getApplicationContext(), "Valid Input. Starting flight.", Toast.LENGTH_LONG).show();
                //constructWayPointMission();
                constructLapMission();
            }else{
                Toast.makeText(getApplicationContext(), "Please complete all fields with valid values.", Toast.LENGTH_LONG).show();
            }

        } else if(v.getId() == R.id.stop){
            if(mission_started) stopWaypointMission();
        }
    }

    private boolean isEditTextEmpty(EditText e_text){
        String test = e_text.getText().toString().trim();
        return (test.isEmpty() || test.equals("") || test == null || test.length() == 0);
    }

    /*
        Checks text input for validity, and implicitly captures bounds on possible values;
        These can be altered as needed but DO SO CAREFUllY
     */
    public boolean validateInput(){
        if(isEditTextEmpty(i_velocity)) return false;
        //Velocity must be greater than 0m/s and is limited to a max of 8m/s
        if(!(Float.valueOf(i_velocity.getText().toString()) > 0 && Double.valueOf(i_velocity.getText().toString()) <= 8)) return false;
        if(isEditTextEmpty(i_altitude)) return false;
        //Minimum altitude set to (hopefully) avoid hitting anything in the area at 15m; capped at 25m to ensure aircraft is always clearly visible
        if(!(Float.valueOf(i_altitude.getText().toString()) > 15 && Double.valueOf(i_altitude.getText().toString()) <= 25)) return false;
        if(isEditTextEmpty(i_distance)) return false;
        //Maximum distance is limited to 1000m; this translates to ensuring that the aircraft, when performing laps, is never more than 1000m away
        if(!(Double.valueOf(i_distance.getText().toString()) >= 0 && Double.valueOf(i_distance.getText().toString()) <= 1000)) return false;
        if(isEditTextEmpty(i_heading)) return false;
        //For the sake of simplicity, headings beyond 360 degrees are not allowed (if desired, could add calculations to accept any heading)
        if(!(Double.valueOf(i_heading.getText().toString()) >= 0 && Double.valueOf(i_heading.getText().toString()) <= 360)) return false;
        if(isEditTextEmpty(i_duration)) return false;
        //Duration (in seconds) limited to 900 seconds (15 minutes)
        if(!(Double.valueOf(i_duration.getText().toString()) >= 0 && Double.valueOf(i_duration.getText().toString()) <= 900)) return false;
        //Must decide to use either time or distance for determining how to fly
        if(!(selectTime.isChecked() || selectDist.isChecked())) return false;
        return true;
    }

    private void updateMap(){
        final MarkerOptions markerOptions = new MarkerOptions();
        markerOptions.position(drone_pos);
        markerOptions.icon(BitmapDescriptorFactory.fromResource(R.drawable.aircraft));

        runOnUiThread(new Runnable(){
            @Override
            public void run(){
                if(drone_marker != null){
                    drone_marker.remove();
                }

                drone_marker = gmap.addMarker(markerOptions);
                //gmap.moveCamera(CameraUpdateFactory.newLatLngZoom(drone_pos, (float) 18.0));
            }

        });


    }

    private void addWaypointMarker(Waypoint w){
        final MarkerOptions markerOptions = new MarkerOptions();
        markerOptions.position(new LatLng(w.coordinate.getLatitude(), w.coordinate.getLongitude()));
        markerOptions.icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_BLUE));

        runOnUiThread(new Runnable(){
            @Override
            public void run(){

                if(waypoint_markers == null){
                    waypoint_markers = new ArrayList<Marker>();
                }

                waypoint_markers.add(gmap.addMarker(markerOptions));
            }

        });
    }

    private class RecordData extends AsyncTask<Void, Void, Void> {

        @Override
        protected Void doInBackground(Void... params) {
            FileWriter mFileWriter = null;

            if (log_file.exists()) {
                try {
                    mFileWriter = new FileWriter(log_file.getAbsolutePath(), true);
                } catch (IOException ie) {
                    ie.printStackTrace();
                }
            }

            if (mFileWriter != null) {
                if(mProduct != null) {
                    final String newData = (Calendar.getInstance().getTimeInMillis() - initialTime) + ", " + drone_lat + ", " + drone_long + "," +
                            drone_v_x + ", " + drone_v_y + ", " + drone_v_z + ", " + drone_battery_voltage + "," + drone_battery_current + ", " +
                            "\n";

                    try {
                        mFileWriter.write(newData);
                        mFileWriter.close();
                    } catch (IOException ie) {
                        ie.printStackTrace();
                    }

                }else{
                    final String newData = (Calendar.getInstance().getTimeInMillis() - initialTime) + ", " + drone_lat + ", " + drone_long + "," +
                            drone_v_x + ", " + drone_v_y + ", " + drone_v_z + ", " + drone_battery_voltage + "," + drone_battery_current + ", ";

                    try {
                        mFileWriter.write(newData);
                        mFileWriter.close();
                    } catch (IOException ie) {
                        ie.printStackTrace();
                    }
                }

            }

            return null;
        }
    }

    private class WriteWaypoints extends AsyncTask<Void, Void, Void> {

        @Override
        protected Void doInBackground(Void... params) {
            FileWriter mFileWriter = null;

            if (waypoints_file.exists()) {
                try {
                    mFileWriter = new FileWriter(waypoints_file.getAbsolutePath(), true);
                } catch (IOException ie) {
                    ie.printStackTrace();
                }
            }

            for(Waypoint i : mission_waypoints){
                if (mFileWriter != null) {
                    final String newData = i.altitude + "," + i.coordinate.getLatitude() + "," + i.coordinate.getLongitude() + "\n";
                    try {
                        mFileWriter.write(newData);
                        mFileWriter.close();
                    } catch (IOException ie) {
                        ie.printStackTrace();
                    }
            }

            }

            return null;
        }
    }

}
