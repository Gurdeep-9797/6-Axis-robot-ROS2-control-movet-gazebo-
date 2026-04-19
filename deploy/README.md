# RoboForge — Deployment Guide

## Deploying to Another PC

### Prerequisites

- Windows 10/11 with Docker Desktop (24+)
- 8 GB RAM minimum (16 GB recommended)
- 10 GB free disk space

### Steps

1. Copy the entire project folder to the target PC
2. Open PowerShell in the project root
3. Run:

```powershell
docker compose up -d
```

4. Wait 60–90 seconds for all services to initialize
5. Open browser: **http://localhost:8080**

### With Gazebo Simulation

```powershell
docker compose --profile sim up -d
```

### Verify Deployment

```powershell
# Check all containers are running
docker ps --filter "name=roboforge"

# Check bridge health
curl http://localhost:8765/health

# Run pipeline test
python tools/test_pipeline.py
```

### Connecting Physical Hardware

```powershell
# Set ESP32 IP address
$env:CONTROLLER_IP = "192.168.1.100"

# Restart bridge to pick up new config
docker restart roboforge_bridge
```

---

For full documentation, see the [main README](../README.md).
