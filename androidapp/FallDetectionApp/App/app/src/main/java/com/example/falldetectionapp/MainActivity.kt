package com.example.falldetectionapp

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.graphics.Color
import android.media.RingtoneManager
import android.net.Uri
import android.os.Build
import android.os.Bundle
import android.os.VibrationEffect
import android.os.Vibrator
import android.provider.Settings
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.core.app.ActivityCompat
import androidx.core.app.NotificationCompat
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.DatabaseError
import com.google.firebase.database.DatabaseReference
import com.google.firebase.database.FirebaseDatabase
import com.google.firebase.database.ValueEventListener
import com.hivemq.client.mqtt.MqttClient
import com.hivemq.client.mqtt.datatypes.MqttQos
import com.hivemq.client.mqtt.mqtt3.Mqtt3AsyncClient
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.json.JSONObject
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class MainActivity : AppCompatActivity() {

    private lateinit var statusTextView: TextView
    private lateinit var batteryTextView: TextView
    private lateinit var resetAlarmButton: Button
    private lateinit var phoneEditText: EditText
    private lateinit var sendConfigButton: Button
    private lateinit var mapContainer: LinearLayout
    private lateinit var coordsTextView: TextView
    private lateinit var historyButton: Button
    
    private var isAlarmActive = false
    private lateinit var database: DatabaseReference
    
    private var currentLat: Double = 0.0
    private var currentLon: Double = 0.0

    private val mqttUpdateReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context?, intent: Intent?) {
            val type = intent?.getStringExtra("type")
            val data = intent?.getStringExtra("data") ?: return
            
            try {
                val json = JSONObject(data)
                if (type == "telemetry") {
                    val lat = json.optDouble("lat", 0.0)
                    val lon = json.optDouble("lon", 0.0)
                    val battery = json.optInt("bat", -1)
                    val motionStatus = json.optString("status", "UNKNOWN")
                    updateTelemetryUI(lat, lon, battery, motionStatus)
                } else if (type == "alarm") {
                    handleAlarmUI(json)
                }
            } catch (e: Exception) {
                Log.e("MainActivity", "Error parsing broadcast: ${e.message}")
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        checkPermissions()
        initViews()
        
        val databaseUrl = "https://falldetectionapp-83f9e-default-rtdb.asia-southeast1.firebasedatabase.app/"
        database = FirebaseDatabase.getInstance(databaseUrl).reference

        startMqttService()
        observeSystemStatus()

        resetAlarmButton.setOnClickListener { resetSystemStatus() }
        sendConfigButton.setOnClickListener { handleSendConfig() }
        mapContainer.setOnClickListener { openGoogleMaps() }
        historyButton.setOnClickListener {
            val intent = Intent(this, HistoryActivity::class.java)
            startActivity(intent)
        }
        findViewById<Button>(R.id.logoutButton).setOnClickListener { logout() }
    }

    override fun onResume() {
        super.onResume()
        val filter = IntentFilter("COM_EXAMPLE_MQTT_UPDATE")
        ContextCompat.registerReceiver(this, mqttUpdateReceiver, filter, ContextCompat.RECEIVER_NOT_EXPORTED)
    }

    override fun onPause() {
        super.onPause()
        unregisterReceiver(mqttUpdateReceiver)
    }

    private fun startMqttService() {
        val serviceIntent = Intent(this, MqttService::class.java)
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            startForegroundService(serviceIntent)
        } else {
            startService(serviceIntent)
        }
    }

    private fun observeSystemStatus() {
        database.child("devices").child("DEV_01").addValueEventListener(object : ValueEventListener {
            override fun onDataChange(snapshot: DataSnapshot) {
                val alertStatus = snapshot.child("alert_status").getValue(String::class.java)
                val lastMessage = snapshot.child("last_alert_message").getValue(String::class.java) ?: "Phát hiện té ngã!"
                val telemetry = snapshot.child("telemetry")
                val lat = telemetry.child("lat").getValue(Double::class.java) ?: 0.0
                val lon = telemetry.child("lon").getValue(Double::class.java) ?: 0.0
                val battery = telemetry.child("bat").getValue(Int::class.java) ?: -1
                val motionStatus = telemetry.child("status").getValue(String::class.java) ?: "UNKNOWN"
                val gpsStatus = telemetry.child("gps").getValue(String::class.java) ?: "NONE"

                if (alertStatus == "ALARM") {
                    updateAlarmUI(lastMessage, lat, lon, battery, gpsStatus)
                } else if (isAlarmActive && alertStatus == "NORMAL") {
                    val intent = Intent(this@MainActivity, MqttService::class.java)
                    intent.putExtra("action", "STOP_ALARM_EFFECTS")
                    startService(intent)
                    resetUIAppearance()
                }
                
                // Luôn cập nhật tọa độ và pin ngầm
                if (!isAlarmActive) {
                    updateTelemetryUI(lat, lon, battery, motionStatus)
                }
            }

            override fun onCancelled(error: DatabaseError) {
                Log.e("MainActivity", "Firebase error: ${error.message}")
            }
        })
    }

    private fun initViews() {
        statusTextView = findViewById(R.id.statusTextView)
        batteryTextView = findViewById(R.id.batteryTextView)
        resetAlarmButton = findViewById(R.id.resetAlarmButton)
        phoneEditText = findViewById(R.id.phoneEditText)
        sendConfigButton = findViewById(R.id.sendConfigButton)
        mapContainer = findViewById(R.id.mapContainer)
        coordsTextView = findViewById(R.id.coordsTextView)
        historyButton = findViewById(R.id.historyButton)
    }

    private fun checkPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            if (ContextCompat.checkSelfPermission(this, android.Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED) {
                ActivityCompat.requestPermissions(this, arrayOf(android.Manifest.permission.POST_NOTIFICATIONS), 101)
            }
        }
    }

    private fun updateTelemetryUI(lat: Double, lon: Double, battery: Int, motionStatus: String) {
        if (!isAlarmActive) {
            val displayStatus = when (motionStatus) {
                "MOVING" -> "Đang di chuyển"
                "STILL" -> "Đang đứng yên"
                else -> "Đang chờ tín hiệu..."
            }
            statusTextView.text = "Hệ thống đã kết nối\nTrạng thái: $displayStatus"
            statusTextView.setTextColor(Color.parseColor("#0071e3"))
        }

        if (battery != -1) {
            batteryTextView.text = "🔋 $battery%"
            batteryTextView.setTextColor(if (battery < 20) Color.RED else Color.parseColor("#8E8E93"))
        }

        if (lat != 0.0 && lon != 0.0) {
            currentLat = lat
            currentLon = lon
            if (!isAlarmActive) {
                coordsTextView.text = "📍 Vị trí thiết bị: $currentLat, $currentLon"
                coordsTextView.setTextColor(Color.parseColor("#8E8E93"))
            }
        }
    }

    private fun handleAlarmUI(json: JSONObject) {
        val alert = json.optString("alert", "Cảnh báo khẩn cấp")
        val lat = json.optDouble("lat", 0.0)
        val lon = json.optDouble("lon", 0.0)
        val battery = json.optInt("bat", -1)
        val gpsStatus = json.optString("gps", "NONE")
        updateAlarmUI(alert, lat, lon, battery, gpsStatus)
    }

    private fun updateAlarmUI(alert: String, lat: Double, lon: Double, battery: Int, gpsStatus: String) {
        isAlarmActive = true
        if (lat != 0.0) currentLat = lat
        if (lon != 0.0) currentLon = lon

        if (battery != -1) {
            batteryTextView.text = "🔋 $battery%"
            if (battery < 20) batteryTextView.setTextColor(Color.RED)
        }

        statusTextView.text = "⚠️ CẢNH BÁO: $alert\nGPS: $gpsStatus"
        statusTextView.setTextColor(Color.WHITE)
        statusTextView.setBackgroundColor(Color.RED)
        resetAlarmButton.visibility = View.VISIBLE

        if (currentLat != 0.0 && currentLon != 0.0) {
            coordsTextView.text = "📍 Vị trí: $currentLat, $currentLon"
            coordsTextView.setTextColor(Color.BLACK)
        } else {
            coordsTextView.text = "📍 Chưa có tín hiệu GPS chính xác"
            coordsTextView.setTextColor(Color.WHITE)
        }
        
        if (gpsStatus == "NONE" && isAlarmActive) {
            Toast.makeText(this, "Cảnh báo: Mất tín hiệu GPS!", Toast.LENGTH_LONG).show()
        }
    }

    private fun resetSystemStatus() {
        // Gửi lệnh ép Service tắt còi ngay lập tức
        val intent = Intent(this, MqttService::class.java)
        intent.putExtra("action", "STOP_ALARM_EFFECTS")
        startService(intent)

        lifecycleScope.launch(Dispatchers.IO) {
            database.child("devices").child("DEV_01").child("alert_status").setValue("NORMAL")
        }
        // Giao diện sẽ tự cập nhật thông qua observeSystemStatus()
    }

    private fun resetUIAppearance() {
        isAlarmActive = false
        statusTextView.text = "Hệ thống hoạt động bình thường"
        statusTextView.setTextColor(Color.parseColor("#0071e3"))
        statusTextView.setBackgroundResource(R.drawable.edit_text_background)
        resetAlarmButton.visibility = View.GONE
        coordsTextView.setTextColor(Color.parseColor("#8E8E93"))
    }

    private fun handleSendConfig() {
        val phone = phoneEditText.text.toString().trim()
        if (phone.isEmpty()) {
            Toast.makeText(this, "Vui lòng nhập số điện thoại", Toast.LENGTH_SHORT).show()
            return
        }

        if (!phone.matches(Regex("^\\+?[0-9]{9,15}$"))) {
            Toast.makeText(this, "Số điện thoại không hợp lệ (9-15 chữ số)", Toast.LENGTH_SHORT).show()
            return
        }

        val intent = Intent(this, MqttService::class.java)
        intent.putExtra("action", "PUBLISH_CONFIG")
        intent.putExtra("payload", phone)
        startService(intent)

        database.child("devices").child("DEV_01")
            .child("config").child("emergency_phone")
            .setValue(phone)
            .addOnCompleteListener { task ->
                if (task.isSuccessful) {
                    Log.i("Firebase", "Đã lưu số điện thoại")
                } else {
                    Log.e("Firebase", "Lỗi lưu số điện thoại", task.exception)
                }
            }

        phoneEditText.text.clear()
        Toast.makeText(this, "Đang gửi cấu hình...", Toast.LENGTH_SHORT).show()
    }

    private fun openGoogleMaps() {
        if (currentLat != 0.0 && currentLon != 0.0) {
            val uri = "google.navigation:q=$currentLat,$currentLon"
            val mapIntent = Intent(Intent.ACTION_VIEW, Uri.parse(uri))
            mapIntent.setPackage("com.google.android.apps.maps")
            try {
                startActivity(mapIntent)
            } catch (e: Exception) {
                val browserIntent = Intent(Intent.ACTION_VIEW, Uri.parse("http://maps.google.com/maps?q=$currentLat,$currentLon"))
                startActivity(browserIntent)
            }
        } else {
            Toast.makeText(this, "Chưa có tọa độ vị trí", Toast.LENGTH_SHORT).show()
        }
    }

    private fun logout() {
        FirebaseAuth.getInstance().signOut()
        stopService(Intent(this, MqttService::class.java))
        val intent = Intent(this, LoginActivity::class.java)
        intent.flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
        startActivity(intent)
        finish()
    }

    override fun onDestroy() {
        super.onDestroy()
    }
}

