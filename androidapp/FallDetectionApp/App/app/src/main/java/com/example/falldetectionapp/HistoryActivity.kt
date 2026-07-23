package com.example.falldetectionapp

import android.os.Bundle
import android.widget.Button
import androidx.appcompat.app.AppCompatActivity
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.google.firebase.database.*

class HistoryActivity : AppCompatActivity() {

    private lateinit var recyclerView: RecyclerView
    private lateinit var adapter: HistoryAdapter
    private lateinit var database: DatabaseReference
    private val logList = mutableListOf<FallLog>()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_history)

        val databaseUrl = "https://falldetectionapp-83f9e-default-rtdb.asia-southeast1.firebasedatabase.app/"
        database = FirebaseDatabase.getInstance(databaseUrl).reference.child("devices").child("DEV_01").child("logs")

        recyclerView = findViewById(R.id.historyRecyclerView)
        recyclerView.layoutManager = LinearLayoutManager(this)
        adapter = HistoryAdapter(logList)
        recyclerView.adapter = adapter

        loadHistory()

        findViewById<Button>(R.id.backButton).setOnClickListener {
            finish()
        }
    }

    private fun loadHistory() {
        database.orderByChild("timestamp").addValueEventListener(object : ValueEventListener {
            override fun onDataChange(snapshot: DataSnapshot) {
                logList.clear()
                for (data in snapshot.children) {
                    val log = data.getValue(FallLog::class.java)
                    if (log != null) {
                        // Gán ID thủ công từ key của Firebase để có thể dùng sau này (xóa/sửa)
                        val logWithId = log.copy(id = data.key ?: "")
                        logList.add(0, logWithId)
                    }
                }
                adapter.updateData(logList)
            }

            override fun onCancelled(error: DatabaseError) {
                // Xử lý lỗi nếu cần
            }
        })
    }
}
