#!/usr/bin/python3
"""
Live Aquaculture Water Quality Web Dashboard Server Node (ROS 2 + Embedded HTTP/MJPEG Server)
===========================================================================================
Serves a clean, real-time water quality web dashboard streaming live telemetry
directly from the autonomous camera-guided robot:
  - /esp32/sensors_filtered (DO, pH, Temperature, Turbidity %, Hopper Weight, Valve)
  - /aqua/decision_report   (Live buoy inspection, missing parameters, valve actions)
  - /vision/feeding_demand  (OpenCV fish feeding demand score)
  - /vision/debug_image     (Live MJPEG stream at /stream.mjpg)
  - /ultrasonic/{front,left,right,rear} (4-side ultrasonic distances)

Interactive Controls (Browser -> ROS 2):
  - Edit Safe Limits per species -> publishes /aqua/safe_limits
  - Manual Valve Open / Close -> publishes /feeder_valve_cmd
  - Hopper Refill -> publishes /feeder_refill_cmd
  - Seek Buoy / Auto / Pause -> publishes /aqua/dispatch_cmd

Default URL: http://localhost:8080
"""

from collections import deque
from datetime import datetime
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import subprocess
import threading
import time
from typing import Deque, Dict, List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Float32, String


DASHBOARD_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Water Quality — Autonomous Aquaculture Robot</title>
  <style>
    :root {
      --bg-page: #F7F9FB;
      --bg-card: #FFFFFF;
      --border: #E5E7EB;
      --text-main: #111827;
      --text-sub: #6B7280;
      --text-muted: #9CA3AF;
      --green-bg: #EAF9EE;
      --green-border: #C3EFD0;
      --green-text: #146C38;
      --green-pill: #DCFCE7;
      --green-line: #16A34A;
      --amber-bg: #FFFBEB;
      --amber-border: #FDE68A;
      --amber-text: #B45309;
      --amber-pill: #FEF3C7;
      --red-bg: #FEF2F2;
      --red-border: #FECACA;
      --red-text: #B91C1C;
      --red-pill: #FEE2E2;
      --blue-bg: #EFF6FF;
      --blue-border: #BFDBFE;
      --blue-text: #1D4ED8;
      --blue-pill: #DBEAFE;
      --primary: #0066CC;
      --primary-hover: #0052A3;
    }

    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }

    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
      background: var(--bg-page);
      color: var(--text-main);
      line-height: 1.45;
      -webkit-font-smoothing: antialiased;
    }

    /* Top Header Bar */
    .navbar {
      background: #FFFFFF;
      border-bottom: 1px solid var(--border);
      padding: 12px 32px;
      display: flex;
      align-items: center;
      justify-content: space-between;
    }

    .brand {
      display: flex;
      flex-direction: column;
      align-items: center;
      user-select: none;
    }

    .brand-title {
      font-size: 16px;
      letter-spacing: 0.08em;
      font-weight: 700;
      color: #374151;
    }

    .brand-title span {
      color: #111827;
      font-weight: 800;
    }

    .brand-subicon {
      margin-top: -1px;
      opacity: 0.78;
    }

    .nav-right {
      display: flex;
      align-items: center;
      gap: 12px;
      font-size: 13px;
      color: #4B5563;
    }

    .pill {
      display: inline-flex;
      align-items: center;
      gap: 5px;
      padding: 4px 10px;
      border-radius: 999px;
      font-size: 12px;
      font-weight: 600;
      white-space: nowrap;
    }

    .pill-green {
      background: var(--green-pill);
      color: var(--green-text);
    }

    .pill-amber {
      background: var(--amber-pill);
      color: var(--amber-text);
    }

    .pill-red {
      background: var(--red-pill);
      color: var(--red-text);
    }

    .pill-blue {
      background: var(--blue-pill);
      color: var(--blue-text);
    }

    /* Page Container */
    .container {
      max-width: 1180px;
      margin: 0 auto;
      padding: 28px 32px 36px;
    }

    .page-header h1 {
      font-size: 28px;
      font-weight: 700;
      color: #0F172A;
      letter-spacing: -0.02em;
    }

    .page-header p {
      margin-top: 4px;
      font-size: 13.5px;
      color: var(--text-sub);
    }

    /* Status Banner */
    .banner {
      margin-top: 20px;
      border-radius: 10px;
      padding: 16px 20px;
      display: flex;
      align-items: flex-start;
      gap: 12px;
      transition: background 0.25s ease, border-color 0.25s ease;
      background: var(--green-bg);
      border: 1px solid var(--green-border);
      color: var(--green-text);
    }

    .banner.status-FEEDING {
      background: var(--blue-bg);
      border-color: var(--blue-border);
      color: var(--blue-text);
    }

    .banner.status-WARNING {
      background: var(--amber-bg);
      border-color: var(--amber-border);
      color: var(--amber-text);
    }

    .banner.status-CRITICAL {
      background: var(--red-bg);
      border-color: var(--red-border);
      color: var(--red-text);
    }

    .banner-icon {
      font-size: 16px;
      font-weight: 700;
      margin-top: 1px;
      flex-shrink: 0;
    }

    .banner-title {
      font-size: 15px;
      font-weight: 700;
    }

    .banner-sub {
      font-size: 13px;
      margin-top: 2px;
      opacity: 0.92;
    }

    /* 4 Metric Cards Row */
    .metrics-grid {
      margin-top: 20px;
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 16px;
    }

    .card {
      background: var(--bg-card);
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 18px 20px;
      box-shadow: 0 1px 2px rgba(15, 23, 42, 0.03);
    }

    .metric-top {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 8px;
    }

    .metric-label {
      font-size: 13.5px;
      font-weight: 600;
      color: #374151;
    }

    .metric-val-row {
      margin-top: 10px;
      display: flex;
      align-items: baseline;
      gap: 5px;
    }

    .metric-value {
      font-size: 38px;
      font-weight: 700;
      letter-spacing: -0.02em;
      color: #0F172A;
      line-height: 1.05;
      font-variant-numeric: tabular-nums;
    }

    .metric-unit {
      font-size: 15px;
      font-weight: 500;
      color: #4B5563;
    }

    .metric-limit {
      margin-top: 8px;
      font-size: 12px;
      color: #64748B;
    }

    .sparkline-box {
      margin-top: 12px;
      height: 28px;
      width: 100%;
    }

    .sparkline-box svg {
      width: 100%;
      height: 100%;
      overflow: visible;
    }

    /* Bottom 2-Column Row */
    .bottom-grid {
      margin-top: 20px;
      display: grid;
      grid-template-columns: 1.55fr 1fr;
      gap: 18px;
      align-items: stretch;
    }

    .section-card-title {
      font-size: 17px;
      font-weight: 700;
      color: #0F172A;
    }

    .action-list {
      margin-top: 14px;
      padding-left: 20px;
      color: #1F2937;
      font-size: 13.5px;
    }

    .action-list li {
      margin-bottom: 9px;
      padding-left: 4px;
    }

    .btn-row {
      margin-top: 20px;
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      align-items: center;
    }

    .btn {
      font-family: inherit;
      font-size: 13px;
      font-weight: 600;
      padding: 9px 18px;
      border-radius: 8px;
      cursor: pointer;
      border: 1px solid transparent;
      transition: all 0.15s ease;
    }

    .btn-primary {
      background: var(--primary);
      color: #FFFFFF;
    }

    .btn-primary:hover {
      background: var(--primary-hover);
    }

    .btn-outline {
      background: #FFFFFF;
      border-color: #D1D5DB;
      color: #1F2937;
    }

    .btn-outline:hover {
      background: #F3F4F6;
    }

    .btn-soft {
      background: #F8FAFC;
      border-color: #CBD5E1;
      color: #0F172A;
    }

    .btn-soft:hover {
      background: #EFF6FF;
      border-color: #93C5FD;
      color: #1D4ED8;
    }

    /* Automation Panel Rows */
    .auto-list {
      margin-top: 10px;
    }

    .auto-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
      padding: 12px 0;
      border-bottom: 1px solid #F1F5F9;
    }

    .auto-row:last-child {
      border-bottom: none;
      padding-bottom: 2px;
    }

    .auto-name {
      font-size: 14px;
      font-weight: 600;
      color: #111827;
    }

    .auto-sub {
      font-size: 12px;
      color: var(--text-sub);
      margin-top: 2px;
    }

    /* Modals */
    .modal-backdrop {
      position: fixed;
      inset: 0;
      background: rgba(15, 23, 42, 0.45);
      display: none;
      align-items: center;
      justify-content: center;
      z-index: 1000;
      padding: 20px;
    }

    .modal-backdrop.open {
      display: flex;
    }

    .modal {
      background: #FFFFFF;
      border-radius: 14px;
      width: 100%;
      max-width: 780px;
      max-height: 90vh;
      overflow-y: auto;
      box-shadow: 0 20px 40px rgba(15, 23, 42, 0.18);
      border: 1px solid var(--border);
    }

    .modal-header {
      padding: 16px 22px;
      border-bottom: 1px solid var(--border);
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    .modal-header h2 {
      font-size: 17px;
      font-weight: 700;
    }

    .modal-close {
      background: none;
      border: none;
      font-size: 20px;
      cursor: pointer;
      color: #6B7280;
    }

    .modal-body {
      padding: 20px 22px;
    }

    .form-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 14px;
    }

    .form-group label {
      display: block;
      font-size: 12.5px;
      font-weight: 600;
      color: #374151;
      margin-bottom: 4px;
    }

    .form-group input, .form-group select {
      width: 100%;
      padding: 8px 10px;
      border: 1px solid #D1D5DB;
      border-radius: 8px;
      font-size: 13.5px;
    }

    table.hist-table {
      width: 100%;
      border-collapse: collapse;
      font-size: 12.5px;
    }

    table.hist-table th, table.hist-table td {
      text-align: left;
      padding: 9px 10px;
      border-bottom: 1px solid #F1F5F9;
    }

    table.hist-table th {
      color: #64748B;
      font-weight: 600;
      background: #F8FAFC;
    }

    /* Robot Camera & 4-Side Ultrasonic Modal Grid */
    .robot-grid {
      display: grid;
      grid-template-columns: 1.35fr 1fr;
      gap: 16px;
    }

    .cam-frame {
      width: 100%;
      border-radius: 10px;
      border: 1px solid var(--border);
      background: #0F172A;
      min-height: 280px;
      object-fit: cover;
    }

    .sonar-box {
      background: #F8FAFC;
      border: 1px solid var(--border);
      border-radius: 10px;
      padding: 12px;
      font-size: 12.5px;
    }

    .sonar-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
      margin-top: 8px;
    }

    .sonar-item {
      background: #FFFFFF;
      border: 1px solid #E2E8F0;
      border-radius: 8px;
      padding: 8px 10px;
      display: flex;
      justify-content: space-between;
      font-weight: 600;
    }

    .ctrl-btns {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
      margin-top: 12px;
    }

    @media (max-width: 960px) {
      .metrics-grid { grid-template-columns: repeat(2, 1fr); }
      .bottom-grid { grid-template-columns: 1fr; }
      .robot-grid { grid-template-columns: 1fr; }
    }
  </style>
</head>
<body>

  <!-- Top Navbar -->
  <header class="navbar">
    <div class="brand">
      <div class="brand-title">AQUACULTURE <span>MONITOR</span></div>
      <svg class="brand-subicon" width="72" height="12" viewBox="0 0 72 12" fill="none">
        <line x1="2" y1="6" x2="70" y2="6" stroke="#374151" stroke-width="1.2"/>
        <circle cx="18" cy="6" r="2.2" fill="#FFFFFF" stroke="#374151" stroke-width="1.2"/>
        <polygon points="36,2 40,9 32,9" fill="#0F172A"/>
        <circle cx="54" cy="6" r="2.2" fill="#FFFFFF" stroke="#374151" stroke-width="1.2"/>
      </svg>
    </div>

    <div class="nav-right">
      <span id="nav-updated">Pond 1 · Updated --:--:--</span>
      <span id="nav-pill" class="pill pill-green">✓ Stable</span>
    </div>
  </header>

  <main class="container">
    <!-- Main Page Title -->
    <div class="page-header">
      <h1>Water quality</h1>
      <p>Safe limits are defaults and can be edited per species.</p>
    </div>

    <!-- Top Status Banner -->
    <section id="status-banner" class="banner status-STABLE">
      <div id="banner-icon" class="banner-icon">✓</div>
      <div>
        <div id="banner-title" class="banner-title">All parameters are stable</div>
        <div id="banner-sub" class="banner-sub">Aerators stay off to save energy. You will be alerted if anything changes.</div>
      </div>
    </section>

    <!-- 4 Live Water Parameter Cards -->
    <section class="metrics-grid">
      <!-- 1. Dissolved Oxygen -->
      <div class="card">
        <div class="metric-top">
          <span class="metric-label">Dissolved oxygen</span>
          <span id="pill-do" class="pill pill-green">✓ Stable</span>
        </div>
        <div class="metric-val-row">
          <span id="val-do" class="metric-value">6.8</span>
          <span class="metric-unit">mg/L</span>
        </div>
        <div id="limit-do" class="metric-limit">Safe: above 5.0 mg/L</div>
        <div class="sparkline-box">
          <svg id="spark-do" viewBox="0 0 220 28" preserveAspectRatio="none"></svg>
        </div>
      </div>

      <!-- 2. pH -->
      <div class="card">
        <div class="metric-top">
          <span class="metric-label">pH</span>
          <span id="pill-ph" class="pill pill-green">✓ Stable</span>
        </div>
        <div class="metric-val-row">
          <span id="val-ph" class="metric-value">7.6</span>
        </div>
        <div id="limit-ph" class="metric-limit">Safe: 6.5 to 9.0</div>
        <div class="sparkline-box">
          <svg id="spark-ph" viewBox="0 0 220 28" preserveAspectRatio="none"></svg>
        </div>
      </div>

      <!-- 3. Temperature -->
      <div class="card">
        <div class="metric-top">
          <span class="metric-label">Temperature</span>
          <span id="pill-temp" class="pill pill-green">✓ Stable</span>
        </div>
        <div class="metric-val-row">
          <span id="val-temp" class="metric-value">28.1</span>
          <span class="metric-unit">°C</span>
        </div>
        <div id="limit-temp" class="metric-limit">Safe: 25 to 30 °C</div>
        <div class="sparkline-box">
          <svg id="spark-temp" viewBox="0 0 220 28" preserveAspectRatio="none"></svg>
        </div>
      </div>

      <!-- 4. Turbidity -->
      <div class="card">
        <div class="metric-top">
          <span class="metric-label">Turbidity (relative)</span>
          <span id="pill-turb" class="pill pill-green">✓ Stable</span>
        </div>
        <div class="metric-val-row">
          <span id="val-turb" class="metric-value">22</span>
          <span class="metric-unit">%</span>
        </div>
        <div id="limit-turb" class="metric-limit">Safe: below 40%</div>
        <div class="sparkline-box">
          <svg id="spark-turb" viewBox="0 0 220 28" preserveAspectRatio="none"></svg>
        </div>
      </div>
    </section>

    <!-- Bottom 2 Columns: Action Card & Automation Card -->
    <section class="bottom-grid">
      <!-- Left: Sensor Decisions & Action Card -->
      <div class="card">
        <h2 id="action-title" class="section-card-title">No action needed</h2>
        <ol id="action-steps" class="action-list">
          <li>No action needed</li>
          <li>Keep feeding on the normal schedule.</li>
          <li>Aerators run only when oxygen falls below its limit.</li>
        </ol>

        <div class="btn-row">
          <button class="btn btn-primary" onclick="openHistoryModal()">View history</button>
          <button class="btn btn-outline" onclick="openLimitsModal()">Edit limits</button>
          <button class="btn btn-soft" onclick="openRobotModal()">📷 Robot Vision &amp; 4x Ultrasonics</button>
        </div>
      </div>

      <!-- Right: Automation Status Card -->
      <div class="card">
        <h2 class="section-card-title">Automation</h2>
        <div class="auto-list">
          <div class="auto-row">
            <div>
              <div class="auto-name">Aerators</div>
              <div id="auto-aer-sub" class="auto-sub">Start automatically when oxygen is low.</div>
            </div>
            <span id="auto-aer-pill" class="pill pill-green">✓ OFF · AUTO</span>
          </div>

          <div class="auto-row">
            <div>
              <div class="auto-name">Feeder</div>
              <div id="auto-feed-sub" class="auto-sub">On schedule.</div>
            </div>
            <span id="auto-feed-pill" class="pill pill-green">✓ Running</span>
          </div>

          <div class="auto-row">
            <div>
              <div class="auto-name">Phone alerts</div>
              <div id="auto-alert-sub" class="auto-sub">Nothing to report.</div>
            </div>
            <span id="auto-alert-pill" class="pill pill-green">✓ None</span>
          </div>
        </div>
      </div>
    </section>
  </main>

  <!-- Modal: View History -->
  <div id="modal-history" class="modal-backdrop">
    <div class="modal">
      <div class="modal-header">
        <h2>Pond Inspection &amp; Valve Action History</h2>
        <button class="modal-close" onclick="closeModal('modal-history')">&times;</button>
      </div>
      <div class="modal-body">
        <table class="hist-table">
          <thead>
            <tr>
              <th>Time</th>
              <th>Buoy / Zone</th>
              <th>DO</th>
              <th>pH</th>
              <th>Temp</th>
              <th>Turb</th>
              <th>Action / Decision</th>
            </tr>
          </thead>
          <tbody id="history-tbody">
            <tr><td colspan="7" style="color:#6B7280">Waiting for robot buoy inspections...</td></tr>
          </tbody>
        </table>
      </div>
    </div>
  </div>

  <!-- Modal: Edit Safe Limits -->
  <div id="modal-limits" class="modal-backdrop">
    <div class="modal" style="max-width:540px">
      <div class="modal-header">
        <h2>Edit Safe Water Limits (Per Species)</h2>
        <button class="modal-close" onclick="closeModal('modal-limits')">&times;</button>
      </div>
      <div class="modal-body">
        <div class="form-group" style="margin-bottom:14px">
          <label>Species Preset</label>
          <select id="species-preset" onchange="applyPreset()">
            <option value="default">Nile Tilapia (Default Safe Limits)</option>
            <option value="catfish">African Catfish (High Tolerance)</option>
            <option value="seabass">European Sea Bass (Sensitive DO)</option>
          </select>
        </div>
        <div class="form-grid">
          <div class="form-group">
            <label>Min Dissolved Oxygen (mg/L)</label>
            <input type="number" step="0.1" id="inp-min-do" value="5.0" />
          </div>
          <div class="form-group">
            <label>Max Turbidity (%)</label>
            <input type="number" step="1" id="inp-max-turb" value="40" />
          </div>
          <div class="form-group">
            <label>Min pH</label>
            <input type="number" step="0.1" id="inp-min-ph" value="6.5" />
          </div>
          <div class="form-group">
            <label>Max pH</label>
            <input type="number" step="0.1" id="inp-max-ph" value="9.0" />
          </div>
          <div class="form-group">
            <label>Min Temperature (°C)</label>
            <input type="number" step="0.5" id="inp-min-temp" value="25.0" />
          </div>
          <div class="form-group">
            <label>Max Temperature (°C)</label>
            <input type="number" step="0.5" id="inp-max-temp" value="30.0" />
          </div>
        </div>
        <div class="btn-row" style="justify-content:flex-end">
          <button class="btn btn-outline" onclick="closeModal('modal-limits')">Cancel</button>
          <button class="btn btn-primary" onclick="saveLimits()">Save &amp; Apply Live</button>
        </div>
      </div>
    </div>
  </div>

  <!-- Modal: Live Robot Vision, Valve & 4-Side Ultrasonics -->
  <div id="modal-robot" class="modal-backdrop">
    <div class="modal" style="max-width:920px">
      <div class="modal-header">
        <h2>Autonomous Robot Live Camera &amp; 4-Side Ultrasonic Obstacle Avoidance</h2>
        <button class="modal-close" onclick="closeModal('modal-robot')">&times;</button>
      </div>
      <div class="modal-body">
        <div class="robot-grid">
          <div>
            <img id="live-cam-img" class="cam-frame" src="" alt="Robot Vision Feed" />
            <div style="margin-top:8px; font-size:12.5px; color:#475569" id="robot-mission-line">
              FSM: RANDOM_SEARCH · Camera Visual Navigation &amp; 4x Ultrasonics
            </div>
          </div>
          <div>
            <div class="sonar-box">
              <strong>4-Side Waterproof Ultrasonic Sensors</strong>
              <div class="sonar-grid">
                <div class="sonar-item"><span>Front</span><span id="us-front">4.50 m</span></div>
                <div class="sonar-item"><span>Rear</span><span id="us-rear">4.50 m</span></div>
                <div class="sonar-item"><span>Left</span><span id="us-left">4.50 m</span></div>
                <div class="sonar-item"><span>Right</span><span id="us-right">4.50 m</span></div>
              </div>
              <div style="margin-top:10px; display:flex; justify-content:space-between">
                <span><strong>Hopper Weight:</strong> <span id="rob-weight">5000 g</span></span>
                <span><strong>Fish Demand:</strong> <span id="rob-demand">0 %</span></span>
              </div>
              <div style="margin-top:6px">
                <strong>Feeder Valve:</strong> <span id="rob-valve">CLOSED</span> ·
                <strong>Target Buoy:</strong> <span id="rob-buoy">Scanning...</span>
              </div>
            </div>

            <div class="ctrl-btns">
              <button class="btn btn-primary" onclick="sendCmd('AUTO')">▶ Auto Random Search</button>
              <button class="btn btn-outline" onclick="sendCmd('STOP')">⏸ Pause Robot</button>
              <button class="btn btn-soft" onclick="sendCmd('SEEK_GREEN')">🟢 Seek Green Buoy</button>
              <button class="btn btn-soft" onclick="sendCmd('SEEK_YELLOW')">🟡 Seek Yellow Buoy</button>
              <button class="btn btn-soft" onclick="sendCmd('SEEK_RED')">🔴 Seek Red Buoy</button>
              <button class="btn btn-soft" onclick="sendCmd('SEEK_MAGENTA')">🟣 Seek Magenta Buoy</button>
              <button class="btn btn-outline" onclick="sendValve(true)">Open Valve</button>
              <button class="btn btn-outline" onclick="sendValve(false)">Close Valve</button>
            </div>
            <button class="btn btn-outline" style="width:100%; margin-top:8px" onclick="sendRefill()">
              🔄 Refill Hopper Tank (5000g)
            </button>
          </div>
        </div>
      </div>
    </div>
  </div>

  <script>
    function renderSparkline(svgId, values, minY, maxY, isOk) {
      const svg = document.getElementById(svgId);
      if (!svg || !values || values.length < 2) return;
      const w = 220, h = 28;
      const span = Math.max(0.01, maxY - minY);
      const pts = values.map((v, i) => {
        const x = (i / (values.length - 1)) * w;
        const clamped = Math.max(minY, Math.min(maxY, v));
        const y = h - 4 - ((clamped - minY) / span) * (h - 8);
        return `${x.toFixed(1)},${y.toFixed(1)}`;
      }).join(' ');
      const color = isOk ? '#16A34A' : '#DC2626';
      svg.innerHTML = `<polyline fill="none" stroke="${color}" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" points="${pts}" />`;
    }

    function setPill(elId, ok, okText, badText, overrideClass) {
      const el = document.getElementById(elId);
      if (!el) return;
      el.className = 'pill ' + (overrideClass || (ok ? 'pill-green' : 'pill-red'));
      el.textContent = ok ? okText : badText;
    }

    async function refreshTelemetry() {
      try {
        const res = await fetch('/api/telemetry');
        if (!res.ok) return;
        const data = await res.json();
        const s = data.sensors;
        const lim = data.limits;
        const rep = data.report;

        document.getElementById('nav-updated').textContent = `Pond 1 · Updated ${data.updated_at}`;

        // Top nav pill & overall banner
        const st = rep.overall_status || 'STABLE';
        const navPill = document.getElementById('nav-pill');
        if (st === 'STABLE') {
          navPill.className = 'pill pill-green';
          navPill.textContent = `✓ ${rep.status_badge || 'Stable'}`;
        } else if (st === 'FEEDING') {
          navPill.className = 'pill pill-blue';
          navPill.textContent = `● ${rep.status_badge || 'Valve OPEN'}`;
        } else if (st === 'WARNING') {
          navPill.className = 'pill pill-amber';
          navPill.textContent = `⚠ ${rep.status_badge || 'Monitor'}`;
        } else {
          navPill.className = 'pill pill-red';
          navPill.textContent = `⚠ ${rep.status_badge || 'Action needed'}`;
        }

        const banner = document.getElementById('status-banner');
        banner.className = `banner status-${st}`;
        document.getElementById('banner-icon').textContent = st === 'STABLE' ? '✓' : (st === 'FEEDING' ? '⚙' : '⚠');
        document.getElementById('banner-title').textContent = rep.banner_title;
        document.getElementById('banner-sub').textContent = rep.banner_subtitle;

        // Metric Values & Pills
        const doOk = s.do >= lim.min_do;
        document.getElementById('val-do').textContent = s.do.toFixed(1);
        document.getElementById('limit-do').textContent = `Safe: above ${lim.min_do.toFixed(1)} mg/L`;
        setPill('pill-do', doOk, '✓ Stable', '⚠ Low');
        renderSparkline('spark-do', data.history_series.do, 1.5, 9.5, doOk);

        const phOk = s.ph >= lim.min_ph && s.ph <= lim.max_ph;
        document.getElementById('val-ph').textContent = s.ph.toFixed(1);
        document.getElementById('limit-ph').textContent = `Safe: ${lim.min_ph.toFixed(1)} to ${lim.max_ph.toFixed(1)}`;
        setPill('pill-ph', phOk, '✓ Stable', '⚠ Out of range');
        renderSparkline('spark-ph', data.history_series.ph, 5.0, 9.5, phOk);

        const tempOk = s.temp >= lim.min_temp && s.temp <= lim.max_temp;
        document.getElementById('val-temp').textContent = s.temp.toFixed(1);
        document.getElementById('limit-temp').textContent = `Safe: ${lim.min_temp.toFixed(0)} to ${lim.max_temp.toFixed(0)} °C`;
        setPill('pill-temp', tempOk, '✓ Stable', '⚠ High');
        renderSparkline('spark-temp', data.history_series.temp, 22.0, 34.0, tempOk);

        const turbPct = s.turbidity_pct;
        const turbOk = turbPct <= lim.max_turb_pct;
        document.getElementById('val-turb').textContent = Math.round(turbPct);
        document.getElementById('limit-turb').textContent = `Safe: below ${Math.round(lim.max_turb_pct)}%`;
        setPill('pill-turb', turbOk, '✓ Stable', '⚠ High');
        renderSparkline('spark-turb', data.history_series.turbidity_pct, 0.0, 85.0, turbOk);

        // Left Action Card
        document.getElementById('action-title').textContent = rep.action_title;
        const ol = document.getElementById('action-steps');
        ol.innerHTML = (rep.action_steps || []).map(step => `<li>${step}</li>`).join('');

        // Right Automation Panel
        const aerOn = (rep.aerator_state || '').startsWith('ON');
        document.getElementById('auto-aer-sub').textContent = rep.aerator_sub;
        setPill('auto-aer-pill', !aerOn, `✓ ${rep.aerator_state}`, `⚡ ${rep.aerator_state}`, aerOn ? 'pill-amber' : 'pill-green');

        const valveOpenNow = s.valve === 1 || (rep.feeder_state || '').includes('OPEN');
        document.getElementById('auto-feed-sub').textContent = rep.feeder_sub;
        setPill('auto-feed-pill', !valveOpenNow, `✓ ${rep.feeder_state}`, `● ${rep.feeder_state}`, valveOpenNow ? 'pill-blue' : 'pill-green');

        const hasAlert = rep.alert_state && rep.alert_state !== 'None';
        document.getElementById('auto-alert-sub').textContent = rep.alert_sub;
        setPill('auto-alert-pill', !hasAlert, '✓ None', `⚠ ${rep.alert_state}`, hasAlert ? 'pill-red' : 'pill-green');

        // Robot Modal live stats
        const us = rep.ultrasonic || {};
        document.getElementById('us-front').textContent = `${(us.front ?? 4.5).toFixed(2)} m`;
        document.getElementById('us-rear').textContent = `${(us.rear ?? 4.5).toFixed(2)} m`;
        document.getElementById('us-left').textContent = `${(us.left ?? 4.5).toFixed(2)} m`;
        document.getElementById('us-right').textContent = `${(us.right ?? 4.5).toFixed(2)} m`;
        document.getElementById('rob-weight').textContent = `${Math.round(s.weight)} g`;
        document.getElementById('rob-demand').textContent = `${Math.round(data.demand)} %`;
        document.getElementById('rob-valve').textContent = s.valve ? 'OPEN' : 'CLOSED';
        document.getElementById('rob-buoy').textContent = rep.target_buoy || 'Random Exploring...';
        document.getElementById('robot-mission-line').textContent = `FSM: ${rep.fsm_state} · ${data.mission_text}`;
      } catch (err) {
        console.error(err);
      }
    }

    async function openHistoryModal() {
      document.getElementById('modal-history').classList.add('open');
      const res = await fetch('/api/history');
      const rows = await res.json();
      const tbody = document.getElementById('history-tbody');
      if (!rows.length) {
        tbody.innerHTML = '<tr><td colspan="7" style="color:#6B7280">No buoy inspections logged yet — robot will record events automatically as it visits buoys.</td></tr>';
        return;
      }
      tbody.innerHTML = rows.slice().reverse().map(r => `
        <tr>
          <td>${r.time}</td>
          <td><strong>${r.buoy}</strong></td>
          <td>${r.do.toFixed(1)} mg/L</td>
          <td>${r.ph.toFixed(1)}</td>
          <td>${r.temp.toFixed(1)}°C</td>
          <td>${Math.round(r.turb)}%</td>
          <td>${r.summary}</td>
        </tr>
      `).join('');
    }

    async function openLimitsModal() {
      const res = await fetch('/api/telemetry');
      const data = await res.json();
      const lim = data.limits;
      document.getElementById('inp-min-do').value = lim.min_do;
      document.getElementById('inp-min-ph').value = lim.min_ph;
      document.getElementById('inp-max-ph').value = lim.max_ph;
      document.getElementById('inp-min-temp').value = lim.min_temp;
      document.getElementById('inp-max-temp').value = lim.max_temp;
      document.getElementById('inp-max-turb').value = lim.max_turb_pct;
      document.getElementById('modal-limits').classList.add('open');
    }

    function applyPreset() {
      const v = document.getElementById('species-preset').value;
      if (v === 'default') {
        document.getElementById('inp-min-do').value = 5.0;
        document.getElementById('inp-min-ph').value = 6.5;
        document.getElementById('inp-max-ph').value = 9.0;
        document.getElementById('inp-min-temp').value = 25;
        document.getElementById('inp-max-temp').value = 30;
        document.getElementById('inp-max-turb').value = 40;
      } else if (v === 'catfish') {
        document.getElementById('inp-min-do').value = 4.0;
        document.getElementById('inp-min-ph').value = 6.0;
        document.getElementById('inp-max-ph').value = 8.5;
        document.getElementById('inp-min-temp').value = 24;
        document.getElementById('inp-max-temp').value = 32;
        document.getElementById('inp-max-turb').value = 50;
      } else if (v === 'seabass') {
        document.getElementById('inp-min-do').value = 5.8;
        document.getElementById('inp-min-ph').value = 7.2;
        document.getElementById('inp-max-ph').value = 8.5;
        document.getElementById('inp-min-temp').value = 18;
        document.getElementById('inp-max-temp').value = 27;
        document.getElementById('inp-max-turb').value = 30;
      }
    }

    async function saveLimits() {
      const payload = {
        min_do: parseFloat(document.getElementById('inp-min-do').value),
        min_ph: parseFloat(document.getElementById('inp-min-ph').value),
        max_ph: parseFloat(document.getElementById('inp-max-ph').value),
        min_temp: parseFloat(document.getElementById('inp-min-temp').value),
        max_temp: parseFloat(document.getElementById('inp-max-temp').value),
        max_turb_pct: parseFloat(document.getElementById('inp-max-turb').value),
      };
      await fetch('/api/limits', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify(payload),
      });
      closeModal('modal-limits');
      refreshTelemetry();
    }

    function openRobotModal() {
      document.getElementById('live-cam-img').src = '/stream.mjpg?t=' + Date.now();
      document.getElementById('modal-robot').classList.add('open');
    }

    function closeModal(id) {
      if (id === 'modal-robot') {
        document.getElementById('live-cam-img').src = '';
      }
      document.getElementById(id).classList.remove('open');
    }

    async function sendCmd(cmd) {
      await fetch('/api/command', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({type: 'dispatch', value: cmd}),
      });
    }

    async function sendValve(openValve) {
      await fetch('/api/command', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({type: 'valve', value: openValve}),
      });
    }

    async function sendRefill() {
      await fetch('/api/command', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({type: 'refill', value: true}),
      });
    }

    setInterval(refreshTelemetry, 450);
    refreshTelemetry();
  </script>
</body>
</html>
"""


class AquaWebDashboardNode(Node):
    """ROS 2 node storing live robot telemetry & driving the Live Water Quality Web UI."""

    def __init__(self) -> None:
        super().__init__("aqua_web_dashboard_node")

        self.declare_parameter("port", 8080)
        self.declare_parameter("open_browser", True)
        self.port: int = int(self.get_parameter("port").value)
        self.open_browser: bool = bool(self.get_parameter("open_browser").value)

        self.lock = threading.Lock()
        self.sensors: Dict[str, float] = {
            "weight": 5000.0,
            "ph": 7.60,
            "do": 6.80,
            "turbidity": 33.0,
            "turbidity_pct": 22.0,
            "temp": 28.1,
            "valve": 0,
        }
        self.limits: Dict[str, float] = {
            "min_do": 5.0,
            "min_ph": 6.5,
            "max_ph": 9.0,
            "min_temp": 25.0,
            "max_temp": 30.0,
            "max_turb_pct": 40.0,
        }
        self.demand_pct: float = 0.0
        self.mission_text: str = "Random Pond Search (Camera + 4x Ultrasonic)"
        self.report: dict = {
            "seq": 0,
            "fsm_state": "RANDOM_SEARCH",
            "overall_status": "STABLE",
            "status_badge": "Stable",
            "banner_title": "All parameters are stable",
            "banner_subtitle": "Aerators stay off to save energy. You will be alerted if anything changes.",
            "action_title": "No action needed",
            "action_steps": [
                "No action needed",
                "Keep feeding on the normal schedule.",
                "Aerators run only when oxygen falls below its limit.",
            ],
            "aerator_state": "OFF · AUTO",
            "aerator_sub": "Start automatically when oxygen is low.",
            "feeder_state": "Running",
            "feeder_sub": "On schedule.",
            "alert_state": "None",
            "alert_sub": "Nothing to report.",
            "target_buoy": "",
            "last_buoy": "Pond 1",
            "ultrasonic": {"front": 4.5, "left": 4.5, "right": 4.5, "rear": 4.5},
        }
        self.last_recorded_seq: int = 0
        self.event_history: List[dict] = []

        # Rolling sparkline buffers (initialized smoothly)
        self.series: Dict[str, Deque[float]] = {
            "do": deque([6.8] * 28, maxlen=35),
            "ph": deque([7.6] * 28, maxlen=35),
            "temp": deque([28.1] * 28, maxlen=35),
            "turbidity_pct": deque([22.0] * 28, maxlen=35),
        }
        self.last_series_time: float = 0.0
        self.latest_jpeg: Optional[bytes] = self._make_placeholder_jpeg()

        # Publishers
        self.limits_pub = self.create_publisher(String, "/aqua/safe_limits", 10)
        self.dispatch_pub = self.create_publisher(String, "/aqua/dispatch_cmd", 10)
        self.valve_pub = self.create_publisher(Bool, "/feeder_valve_cmd", 10)
        self.refill_pub = self.create_publisher(Bool, "/feeder_refill_cmd", 10)

        # Subscribers
        self.create_subscription(String, "/esp32/sensors_filtered", self._sensors_cb, 10)
        self.create_subscription(String, "/aqua/decision_report", self._report_cb, 10)
        self.create_subscription(String, "/aqua/mission_status", self._mission_cb, 10)
        self.create_subscription(Float32, "/vision/feeding_demand", self._demand_cb, 10)
        self.create_subscription(RosImage, "/vision/debug_image", self._image_cb, 10)

    @staticmethod
    def _make_placeholder_jpeg() -> bytes:
        canvas = np.zeros((360, 640, 3), dtype=np.uint8)
        canvas[:] = (22, 32, 48)
        cv2.putText(
            canvas,
            "Waiting for Robot Camera Stream (/vision/debug_image)...",
            (55, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (140, 220, 255),
            1,
            cv2.LINE_AA,
        )
        ok, buf = cv2.imencode(".jpg", canvas)
        return buf.tobytes() if ok else b""

    def _sensors_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        now = time.monotonic()
        with self.lock:
            for k in ["weight", "ph", "do", "turbidity", "turbidity_pct", "temp", "valve"]:
                if k in data:
                    self.sensors[k] = float(data[k])
            if "turbidity_pct" not in data and "turbidity" in data:
                self.sensors["turbidity_pct"] = round(
                    min(100.0, max(0.0, self.sensors["turbidity"] / 1.5)), 1
                )
            if (now - self.last_series_time) >= 0.75:
                self.last_series_time = now
                self.series["do"].append(round(self.sensors["do"], 2))
                self.series["ph"].append(round(self.sensors["ph"], 2))
                self.series["temp"].append(round(self.sensors["temp"], 2))
                self.series["turbidity_pct"].append(round(self.sensors["turbidity_pct"], 1))

    def _report_cb(self, msg: String) -> None:
        try:
            rep = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        with self.lock:
            self.report = rep
            seq = int(rep.get("seq", 0))
            ev = rep.get("event_log", "")
            if seq > self.last_recorded_seq and ev:
                self.last_recorded_seq = seq
                self.event_history.append(
                    {
                        "time": datetime.now().strftime("%H:%M:%S"),
                        "buoy": rep.get("last_buoy", "Pond 1"),
                        "do": round(self.sensors["do"], 2),
                        "ph": round(self.sensors["ph"], 2),
                        "temp": round(self.sensors["temp"], 1),
                        "turb": round(self.sensors["turbidity_pct"], 1),
                        "summary": ev,
                    }
                )
                if len(self.event_history) > 100:
                    self.event_history.pop(0)

    def _mission_cb(self, msg: String) -> None:
        with self.lock:
            self.mission_text = msg.data

    def _demand_cb(self, msg: Float32) -> None:
        with self.lock:
            self.demand_pct = float(msg.data)

    def _image_cb(self, msg: RosImage) -> None:
        if msg.width == 0 or msg.height == 0:
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if arr.size < msg.width * msg.height * 3:
            return
        bgr = arr.reshape((msg.height, msg.width, 3))
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ok:
            with self.lock:
                self.latest_jpeg = buf.tobytes()

    def get_telemetry_json(self) -> dict:
        with self.lock:
            return {
                "updated_at": datetime.now().strftime("%H:%M:%S"),
                "sensors": dict(self.sensors),
                "limits": dict(self.limits),
                "report": dict(self.report),
                "demand": self.demand_pct,
                "mission_text": self.mission_text,
                "history_series": {k: list(v) for k, v in self.series.items()},
            }

    def get_history_list(self) -> List[dict]:
        with self.lock:
            return list(self.event_history)

    def update_limits(self, payload: dict) -> None:
        with self.lock:
            for k in ["min_do", "min_ph", "max_ph", "min_temp", "max_temp", "max_turb_pct"]:
                if k in payload:
                    self.limits[k] = float(payload[k])
            lim_copy = dict(self.limits)
        msg = String()
        msg.data = json.dumps(lim_copy)
        self.limits_pub.publish(msg)

    def handle_command(self, payload: dict) -> None:
        cmd_type = payload.get("type", "")
        val = payload.get("value")
        if cmd_type == "dispatch":
            m = String()
            m.data = str(val)
            self.dispatch_pub.publish(m)
        elif cmd_type == "valve":
            m = Bool()
            m.data = bool(val)
            self.valve_pub.publish(m)
        elif cmd_type == "refill":
            m = Bool()
            m.data = True
            self.refill_pub.publish(m)


def make_handler(ros_node: AquaWebDashboardNode):
    class DashboardHandler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args) -> None:
            return  # Keep terminal output clean

        def _send_json(self, data, status=HTTPStatus.OK) -> None:
            body = json.dumps(data).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = self.path.split("?")[0]
            if path == "/":
                body = DASHBOARD_HTML.encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif path == "/api/telemetry":
                self._send_json(ros_node.get_telemetry_json())
            elif path == "/api/history":
                self._send_json(ros_node.get_history_list())
            elif path == "/stream.mjpg":
                self.send_response(HTTPStatus.OK)
                self.send_header(
                    "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                )
                self.end_headers()
                try:
                    while rclpy.ok():
                        with ros_node.lock:
                            jpg = ros_node.latest_jpeg
                        if jpg:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(
                                b"Content-Type: image/jpeg\r\n"
                                + f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
                            )
                            self.wfile.write(jpg)
                            self.wfile.write(b"\r\n")
                        time.sleep(0.08)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(HTTPStatus.NOT_FOUND, "Not found")

        def do_POST(self) -> None:
            path = self.path.split("?")[0]
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length > 0 else b"{}"
            try:
                payload = json.loads(raw.decode("utf-8"))
            except json.JSONDecodeError:
                self._send_json({"error": "Invalid JSON"}, HTTPStatus.BAD_REQUEST)
                return

            if path == "/api/limits":
                ros_node.update_limits(payload)
                self._send_json({"ok": True})
            elif path == "/api/command":
                ros_node.handle_command(payload)
                self._send_json({"ok": True})
            else:
                self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

    return DashboardHandler


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AquaWebDashboardNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    server = ReusableThreadingHTTPServer(("0.0.0.0", node.port), make_handler(node))
    node.get_logger().info(
        f"Live Water Quality Web Dashboard running at http://localhost:{node.port}"
    )

    if node.open_browser and os.environ.get("DISPLAY"):
        def _open_later():
            time.sleep(1.5)
            try:
                subprocess.Popen(
                    ["xdg-open", f"http://localhost:{node.port}"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass
        threading.Thread(target=_open_later, daemon=True).start()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
