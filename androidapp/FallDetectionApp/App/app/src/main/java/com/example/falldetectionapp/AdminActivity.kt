package com.example.falldetectionapp

import android.content.Intent
import android.os.Bundle
import android.widget.Button
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.firestore.FirebaseFirestore

class AdminActivity : AppCompatActivity() {
    private lateinit var db: FirebaseFirestore
    private lateinit var adapter: UserAdapter
    private var userList = mutableListOf<User>()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_admin)

        db = FirebaseFirestore.getInstance()

        // Kiểm tra quyền Admin
        val currentUser = FirebaseAuth.getInstance().currentUser
        if (currentUser == null) {
            finish()
            return
        }
        db.collection("users").document(currentUser.uid).get()
            .addOnSuccessListener { doc ->
                if (doc.getString("role") != "admin") {
                    Toast.makeText(this, "Bạn không có quyền truy cập!", Toast.LENGTH_SHORT).show()
                    finish()
                }
            }
            .addOnFailureListener {
                finish()
            }

        // Setup RecyclerView
        val recyclerView = findViewById<RecyclerView>(R.id.userRecyclerView)
        recyclerView.layoutManager = LinearLayoutManager(this)
        adapter = UserAdapter(userList) { user ->
            showDeleteConfirmDialog(user)
        }
        recyclerView.adapter = adapter

        // Load users from Firestore
        loadUsers()

        val logoutButton = findViewById<Button>(R.id.adminLogoutButton)
        logoutButton.setOnClickListener {
            FirebaseAuth.getInstance().signOut()
            val intent = Intent(this, LoginActivity::class.java)
            intent.flags = Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK
            startActivity(intent)
            finish()
        }
    }

    private fun loadUsers() {
        db.collection("users")
            .addSnapshotListener { value, error ->
                if (error != null) {
                    Toast.makeText(this, "Lỗi tải dữ liệu: ${error.message}", Toast.LENGTH_SHORT).show()
                    return@addSnapshotListener
                }

                userList.clear()
                for (doc in value!!) {
                    val user = User(
                        uid = doc.id,
                        name = doc.getString("name") ?: "",
                        email = doc.getString("email") ?: "",
                        phone = doc.getString("phone") ?: "",
                        role = doc.getString("role") ?: "user"
                    )
                    // Không hiển thị chính mình (admin hiện tại) trong danh sách xóa
                    if (user.uid != FirebaseAuth.getInstance().currentUser?.uid) {
                        userList.add(user)
                    }
                }
                adapter.updateData(userList)
            }
    }

    private fun showDeleteConfirmDialog(user: User) {
        AlertDialog.Builder(this)
            .setTitle("Xóa người dùng")
            .setMessage("Bạn có chắc chắn muốn xóa người dùng ${user.name} không?")
            .setPositiveButton("Xóa") { _, _ ->
                deleteUser(user)
            }
            .setNegativeButton("Hủy", null)
            .show()
    }

    private fun deleteUser(user: User) {
        db.collection("users").document(user.uid)
            .delete()
            .addOnSuccessListener {
                Toast.makeText(this, "Đã xóa người dùng thành công", Toast.LENGTH_SHORT).show()
            }
            .addOnFailureListener { e ->
                Toast.makeText(this, "Lỗi: ${e.message}", Toast.LENGTH_SHORT).show()
            }
    }
}