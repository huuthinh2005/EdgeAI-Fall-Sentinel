package com.example.falldetectionapp

import android.content.Intent
import android.net.Uri
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import android.widget.Toast
import androidx.recyclerview.widget.RecyclerView

class HistoryAdapter(private var logs: List<FallLog>) :
    RecyclerView.Adapter<HistoryAdapter.ViewHolder>() {

    class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
        val logStatus: TextView = view.findViewById(R.id.logStatus)
        val logTime: TextView = view.findViewById(R.id.logTime)
        val logCoords: TextView = view.findViewById(R.id.logCoords)
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val view = LayoutInflater.from(parent.context)
            .inflate(R.layout.item_fall_log, parent, false)
        return ViewHolder(view)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        val log = logs[position]
        holder.logStatus.text = "${log.status} (GPS: ${log.gps})"
        holder.logTime.text = log.timestamp
        holder.logCoords.text = "📍 ${log.lat}, ${log.lon}"

        holder.itemView.setOnClickListener {
            if (log.lat != 0.0 && log.lon != 0.0) {
                val uri = "google.navigation:q=${log.lat},${log.lon}"
                val mapIntent = Intent(Intent.ACTION_VIEW, Uri.parse(uri))
                mapIntent.setPackage("com.google.android.apps.maps")
                try {
                    holder.itemView.context.startActivity(mapIntent)
                } catch (e: Exception) {
                    val browserIntent = Intent(Intent.ACTION_VIEW, Uri.parse("http://maps.google.com/maps?q=${log.lat},${log.lon}"))
                    holder.itemView.context.startActivity(browserIntent)
                }
            } else {
                Toast.makeText(holder.itemView.context, "Bản ghi này không có tọa độ GPS", Toast.LENGTH_SHORT).show()
            }
        }
    }

    override fun getItemCount() = logs.size

    fun updateData(newLogs: List<FallLog>) {
        logs = newLogs
        notifyDataSetChanged()
    }
}
