# Dockerizing WDPJone

This guide shows how to build and run the backend (Python) and frontend (React) using Docker, and how to run the images on another machine.

## Prerequisites
- Docker Desktop (or docker engine) installed
- Optionally: Git to clone the repo

## Build and Run with docker-compose (recommended)

From the project root:

```powershell
# Build both images
docker compose build

# Start containers
docker compose up -d

# View logs
docker compose logs -f
```

Once up:
- Backend WebSocket bridge: ws://localhost:9003
- Frontend UI: http://localhost:8080

To stop:
```powershell
docker compose down
```

## Building images separately

Backend:
```powershell
# Build
docker build -f Dockerfile.backend -t wdpjone/backend:latest .
# Run
docker run -d --name wdpjone-backend -p 9003:9003 -p 12346:12346 -p 12345:12345 wdpjone/backend:latest
```

Frontend:
```powershell
# Build with custom WS URL (if backend on different host)
$ws="ws://localhost:9003"
docker build -f Dockerfile.frontend --build-arg REACT_APP_WS_BRIDGE_URL=$ws -t wdpjone/frontend:latest .
# Run
docker run -d --name wdpjone-frontend -p 8080:80 wdpjone/frontend:latest
```

## Using the images on another machine

1. Save images to tar files on your build machine:
```powershell
docker save wdpjone/backend:latest -o wdpjone-backend.tar
docker save wdpjone/frontend:latest -o wdpjone-frontend.tar
```
2. Copy the `.tar` files to the target machine (USB, network, etc.).
3. On the target machine, load the images:
```powershell
docker load -i wdpjone-backend.tar
docker load -i wdpjone-frontend.tar
```
4. Run the containers (adjust host as needed):
```powershell
# Backend
docker run -d --name wdpjone-backend -p 9003:9003 -p 12346:12346 -p 12345:12345 wdpjone/backend:latest
# Frontend
# If backend is on the same machine, default build arg works. If not, rebuild frontend with correct WS URL.
docker run -d --name wdpjone-frontend -p 8080:80 wdpjone/frontend:latest
```

If you need the frontend to connect to a remote backend, rebuild the frontend image with:
```powershell
$ws="ws://<backend-host>:9003"
docker build -f Dockerfile.frontend --build-arg REACT_APP_WS_BRIDGE_URL=$ws -t wdpjone/frontend:latest .
```

## Environment and Ports
- Backend environment variables:
  - `WS_BRIDGE_PORT` (default 9003)
  - `TCP_PORT` (default 12346)
  - `OTA_PORT` (default 12345)
  - `LOG_LEVEL` (default INFO)
- Frontend is a static site served by Nginx; the WebSocket URL is baked in at build time via `REACT_APP_WS_BRIDGE_URL`.

## Notes
- The backend Docker image runs `back/direct_bridge.py`. If you need to run the GUI runner (`back/start.py`), that requires a desktop environment and is not containerized here.
- If you need to persist logs or databases, mount a host volume path to `/app/back/logs` or `/app/back/telemetry.db` as needed.
  Example:
  ```powershell
  docker run -d --name wdpjone-backend -p 9003:9003 -p 12346:12346 -p 12345:12345 -v $PWD\logs:/app/back/logs wdpjone/backend:latest
  ```
