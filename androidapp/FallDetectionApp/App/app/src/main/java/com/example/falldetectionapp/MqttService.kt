package com.example.falldetectionapp

import android.app.*
import android.content.Context
import android.content.Intent
import android.graphics.Color
import android.media.RingtoneManager
import android.net.Uri
import android.os.*
import android.provider.Settings
import android.util.Log
import androidx.core.app.NotificationCompat
import com.google.firebase.database.FirebaseDatabase
import com.hivemq.client.mqtt.MqttClient
import com.hivemq.client.mqtt.datatypes.MqttQos
import com.hivemq.client.mqtt.mqtt3.Mqtt3AsyncClient
import org.json.JSONObject
import java.text.SimpleDateFormat
import java.util.*

class MqttService : Service() {

    private lateinit var mqttClient: Mqtt3AsyncClient
    private val channelId = "fall_detection_service"
    private val alarmChannelId = "fall_alerts"
    private var isAlarmActive = false
    private var ringtone: android.media.Ringtone? = null
    private var vibrator: Vibrator? = null

    override fun onCreate() {
        super.onCreate()
        createNotificationChannels()
        startForeground(100, createForegroundNotification())
        setupMqtt()
    }

    private fun createNotificationChannels() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val serviceChannel = NotificationChannel(
                channelId, "Fall Detection Service",
                NotificationManager.IMPORTANCE_LOW
            )
            val alarmChannel = NotificationChannel(
                alarmChannelId, "Fall Alerts",
                NotificationManager.IMPORTANCE_HIGH
            ).apply {
                description = "Thông báo khi phát hiện té ngã hoặc SOS"
                enableLights(true)
                lightColor = Color.RED
                enableVibration(true)
            }
            val manager = getSystemService(NotificationManager::class.java)
            manager.createNotificationChannel(serviceChannel)
            manager.createNotificationChannel(alarmChannel)
        }
    }

    private fun createForegroundNotification(): Notification {
        val intent = Intent(this, MainActivity::class.java)
        val pendingIntent = PendingIntent.getActivity(
            this, 0, intent, PendingIntent.FLAG_IMMUTABLE
        )
        return NotificationCompat.Builder(this, channelId)
            .setContentTitle("Hệ thống bảo vệ đang chạy")
            .setContentText("Đang theo dõi trạng thái thiết bị...")
            .setSmallIcon(android.R.drawable.ic_menu_mylocation)
            .setContentIntent(pendingIntent)
            .build()
    }

    private fun setupMqtt() {
        val androidId = Settings.Secure.getString(contentResolver, Settings.Secure.ANDROID_ID)
        mqttClient = MqttClient.builder()
            .useMqttVersion3()
            .identifier("Mobile_App_Service_$androidId")
            .serverHost("broker.emqx.io")
            .serverPort(1883)
            .automaticReconnectWithDefaultConfig()
            .addConnectedListener {
                Log.d("MqttService", "MQTT Connected/Reconnected")
                subscribeToTopics()
                publishPendingConfig()
            }
            .buildAsync()

        mqttClient.connectWith()
            .cleanSession(false)
            .send()
    }

    private fun publishPendingConfig() {
        val prefs = getSharedPreferences("AppConfig", Context.MODE_PRIVATE)
        val pendingPhone = prefs.getString("pending_emergency_phone", null)
        if (pendingPhone != null && ::mqttClient.isInitialized && mqttClient.state.isConnected) {
            mqttClient.publishWith()
                .topic("thiet_bi/cai_dat")
                .qos(MqttQos.AT_LEAST_ONCE)
                .retain(true)
                .payload(pendingPhone.toByteArray())
                .send()
                .whenComplete { _, throwable ->
                    if (throwable == null) {
                        Log.d("MqttService", "Published retained config: $pendingPhone")
                    }
                }
        }
    }

    private fun subscribeToTopics() {
        mqttClient.subscribeWith()
            .topicFilter("thiet_bi/telemetry")
            .callback { publish ->
                val message = String(publish.payloadAsBytes)
                handleTelemetry(message)
            }.send()

        mqttClient.subscribeWith()
            .topicFilter("thiet_bi/bao_dong")
            .qos(MqttQos.AT_LEAST_ONCE)
            .callback { publish ->
                val message = String(publish.payloadAsBytes)
                handleAlarm(message)
            }.send()
    }

    private fun handleTelemetry(message: String) {
        try {
            val json = JSONObject(message)
            val databaseUrl = "https://falldetectionapp-83f9e-default-rtdb.asia-southeast1.firebasedatabase.app/"
            val database = FirebaseDatabase.getInstance(databaseUrl).reference
            
            val telemetryData = mutableMapOf<String, Any>(
                "lat" to json.optDouble("lat", 0.0),
                "lon" to json.optDouble("lon", 0.0),
                "status" to json.optString("status", "UNKNOWN"),
                "timestamp" to System.currentTimeMillis()
            )
            if (json.has("gps")) telemetryData["gps"] = json.optString("gps")
            if (json.has("bat")) telemetryData["bat"] = json.optInt("bat", -1)
            
            database.child("devices").child("DEV_01").child("telemetry").updateChildren(telemetryData)
            
            // Broadcast to Activity if open
            val intent = Intent("COM_EXAMPLE_MQTT_UPDATE")
            intent.putExtra("type", "telemetry")
            intent.putExtra("data", message)
            sendBroadcast(intent)
            
        } catch (e: Exception) {
            Log.e("MqttService", "Telemetry error: ${e.message}")
        }
    }

    private fun handleAlarm(message: String) {
        val json = try { JSONObject(message) } catch (e: Exception) { null }
        if (json == null) return

        // Mức 2: Chống replay tin cũ (nếu quá 5 phút thì bỏ qua)
        val msgTimestamp = json.optLong("ts", 0) // Giả định firmware có gửi kèm timestamp rút gọn hoặc dùng System.currentTimeMillis() nếu đồng bộ
        if (msgTimestamp > 0 && (System.currentTimeMillis() / 1000 - msgTimestamp) > 300) {
            Log.w("MqttService", "Bỏ qua cảnh báo cũ: $message")
            return
        }

        val alert = json.optString("alert", "Cảnh báo khẩn cấp")
        val lat = json.optDouble("lat", 0.0)
        val lon = json.optDouble("lon", 0.0)
        val battery = json.optInt("bat", -1)
        val gpsStatus = json.optString("gps", "NONE")

        triggerAlarmEffects(alert)

        val databaseUrl = "https://falldetectionapp-83f9e-default-rtdb.asia-southeast1.firebasedatabase.app/"
        val database = FirebaseDatabase.getInstance(databaseUrl).reference
        val currentTime = SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.getDefault()).format(Date())

        val logData = mutableMapOf<String, Any>(
            "timestamp" to currentTime,
            "status" to alert,
            "lat" to lat,
            "lon" to lon,
            "gps" to gpsStatus
        )
        database.child("devices").child("DEV_01").child("logs").push().setValue(logData)
        
        // Cập nhật trạng thái và tin nhắn cảnh báo
        val alarmStatus = mutableMapOf<String, Any>(
            "alert_status" to "ALARM",
            "last_alert_message" to alert
        )
        database.child("devices").child("DEV_01").updateChildren(alarmStatus)
        
        // Đồng bộ tọa độ khẩn cấp và pin vào telemetry để Activity hiển thị ngay
        val telemetryUpdate = mutableMapOf<String, Any>(
            "timestamp" to System.currentTimeMillis(),
            "gps" to gpsStatus
        )
        if (lat != 0.0) {
            telemetryUpdate["lat"] = lat
            telemetryUpdate["lon"] = lon
        }
        if (battery != -1) {
            telemetryUpdate["bat"] = battery
        }
        database.child("devices").child("DEV_01").child("telemetry").updateChildren(telemetryUpdate)

        // Broadcast to Activity
        val intent = Intent("COM_EXAMPLE_MQTT_UPDATE")
        intent.putExtra("type", "alarm")
        val dataJson = JSONObject().apply {
            put("alert", alert)
            put("lat", lat)
            put("lon", lon)
            put("bat", battery)
            put("gps", gpsStatus)
        }
        intent.putExtra("data", dataJson.toString())
        sendBroadcast(intent)
    }

    private fun triggerAlarmEffects(message: String) {
        // Vibrate
        vibrator = getSystemService(Context.VIBRATOR_SERVICE) as Vibrator
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            vibrator?.vibrate(VibrationEffect.createWaveform(longArrayOf(0, 500, 200, 500), 0))
        } else {
            vibrator?.vibrate(longArrayOf(0, 500, 200, 500), 0)
        }

        // Sound
        val alarmUri: Uri = RingtoneManager.getDefaultUri(RingtoneManager.TYPE_ALARM)
        ringtone = RingtoneManager.getRingtone(applicationContext, alarmUri)
        ringtone?.play()

        // Tự tắt sau 15s nếu không ai đụng vào
        Handler(Looper.getMainLooper()).postDelayed({
            stopAlarmEffects()
        }, 15000)

        // Notification
        val intent = Intent(this, MainActivity::class.java).apply {
            flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TOP
        }
        val pendingIntent = PendingIntent.getActivity(this, 0, intent, PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE)

        val notification = NotificationCompat.Builder(this, alarmChannelId)
            .setSmallIcon(android.R.drawable.ic_dialog_alert)
            .setContentTitle("⚠️ CẢNH BÁO KHẨN CẤP")
            .setContentText(message)
            .setPriority(NotificationCompat.PRIORITY_MAX)
            .setCategory(NotificationCompat.CATEGORY_ALARM)
            .setFullScreenIntent(pendingIntent, true)
            .setAutoCancel(true)
            .build()

        val notificationManager = getSystemService(Context.NOTIFICATION_SERVICE) as NotificationManager
        notificationManager.notify(1, notification)
    }

    private fun stopAlarmEffects() {
        ringtone?.takeIf { it.isPlaying }?.stop()
        vibrator?.cancel()
    }

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val action = intent?.getStringExtra("action")
        if (action == "PUBLISH_CONFIG") {
            val payload = intent.getStringExtra("payload") ?: ""
            if (payload.isNotEmpty()) {
                // Lưu vào SharedPreferences để retry nếu mất mạng và để sync với firmware (Retain)
                getSharedPreferences("AppConfig", Context.MODE_PRIVATE)
                    .edit()
                    .putString("pending_emergency_phone", payload)
                    .apply()

                if (::mqttClient.isInitialized && mqttClient.state.isConnected) {
                    mqttClient.publishWith()
                        .topic("thiet_bi/cai_dat")
                        .qos(MqttQos.AT_LEAST_ONCE)
                        .retain(true)
                        .payload(payload.toByteArray())
                        .send()
                } else {
                    Log.w("MqttService", "MQTT chưa kết nối, cấu hình đã được lưu và sẽ gửi lại khi có mạng.")
                }
            }
        } else if (action == "STOP_ALARM_EFFECTS") {
            stopAlarmEffects()
        }
        return START_STICKY
    }

    override fun onDestroy() {
        if (::mqttClient.isInitialized) mqttClient.disconnect()
        super.onDestroy()
    }
}
