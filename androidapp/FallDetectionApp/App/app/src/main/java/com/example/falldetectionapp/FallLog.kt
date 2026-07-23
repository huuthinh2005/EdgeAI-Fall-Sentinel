package com.example.falldetectionapp

data class FallLog(
    val id: String = "",
    val timestamp: String = "",
    val status: String = "",
    val lat: Double = 0.0,
    val lon: Double = 0.0,
    val gps: String = ""
)
