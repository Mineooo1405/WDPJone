import React, { useEffect, useRef, useState, useContext, useCallback } from 'react';
import { Chart as ChartJS, LineController, LineElement, PointElement, LinearScale, Title, Tooltip, Legend, CategoryScale } from 'chart.js';
import zoomPlugin from 'chartjs-plugin-zoom';
import { useRobotContext, ReadyState } from './RobotContext'; 
import { GlobalAppContext } from '../contexts/GlobalAppContext'; 
import { RotateCcw, Maximize } from 'lucide-react';
import WidgetConnectionHeader from './WidgetConnectionHeader';
import { appConfig } from '../config/appConfig';

// Register Chart.js components
ChartJS.register(
  LineController,
  LineElement,
  PointElement,
  LinearScale,
  CategoryScale, // Added for x-axis if using labels, or for scatter plot type if x is also linear
  Title,
  Tooltip,
  Legend,
  zoomPlugin
);

// Interface for a single point in the trajectory path
interface TrajectoryPoint {
  x: number;
  y: number;
  theta?: number; // Optional: robot's orientation at this point
}

// Interface for the current pose of the robot
interface RobotPose extends TrajectoryPoint {}

const MAX_PATH_POINTS_DISPLAY = appConfig.trajectory.maxPathPointsDisplay; // Max points to keep in the chart for performance

const TrajectoryWidget: React.FC<{ compact?: boolean }> = ({ compact = false }) => {
  const { selectedRobotId, setSelectedRobotId, connectedRobots, sendJsonMessage, lastJsonMessage, readyState } = useRobotContext();
  const { firmwareUpdateMode } = useContext(GlobalAppContext);

  const chartRef = useRef<ChartJS<'line', TrajectoryPoint[], unknown> | null>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  // Multi-robot state
  const [robotPaths, setRobotPaths] = useState<Record<string, TrajectoryPoint[]>>({});
  const [robotPoses, setRobotPoses] = useState<Record<string, RobotPose | null>>({});
  const robotPosesRef = useRef<Record<string, RobotPose | null>>({}); // For plugin access
  // Track robot types per alias for custom icon rendering (omni vs mecanum)
  const robotTypesRef = useRef<Record<string, 'omni' | 'mecanum'>>({});
  const subscribedAliasesRef = useRef<Set<string>>(new Set());
  const colorMapRef = useRef<Record<string, string>>({});
  const [widgetError, setWidgetError] = useState<string | null>(null);
  // Map and navigation state
  const [mapImageUrl, setMapImageUrl] = useState<string | null>(null);
  const [mapMeta, setMapMeta] = useState<{ width: number; height: number; resolution: number; origin_x: number; origin_y: number } | null>(null);
  const [plannedPaths, setPlannedPaths] = useState<Record<string, { x: number; y: number }[]>>({});
  const [navStatus, setNavStatus] = useState<Record<string, string>>({});

  // Color palette for different robots
  const palette = React.useMemo(() => [
    // Colorblind-safe (Okabe-Ito) + a few vivid extras suitable for dark backgrounds
    '#0072B2', // blue
    '#E69F00', // orange
    '#009E73', // green
    '#D55E00', // vermillion
    '#CC79A7', // reddish purple
    '#56B4E9', // sky blue
    '#F0E442', // yellow
    '#9400D3', // dark violet
    '#00CED1', // dark turquoise
    '#FF1493', // deep pink
    '#7FFF00', // chartreuse
    '#FF4500', // orange red
    '#00FF7F', // spring green
    '#00BFFF', // deep sky blue
    '#FF00FF', // magenta
    '#ADFF2F', // green yellow
  ], []);
  const getColorForAlias = useCallback((alias: string) => {
    // Reuse if already assigned
    if (colorMapRef.current[alias]) return colorMapRef.current[alias];
    // Deterministic base index from alias hash
    let h = 0;
    for (let i = 0; i < alias.length; i++) h = ((h << 5) - h) + alias.charCodeAt(i);
    let idx = Math.abs(h) % palette.length;
    // Ensure uniqueness among currently assigned colors when possible
    const used = new Set(Object.values(colorMapRef.current));
    let attempts = 0;
    while (used.has(palette[idx]) && attempts < palette.length) {
      idx = (idx + 1) % palette.length;
      attempts++;
    }
    const color = palette[idx];
    colorMapRef.current[alias] = color;
    return color;
  }, [palette]);

  // Additional line/point styles to increase contrast beyond color alone
  const getStyleForAlias = useCallback((alias: string) => {
    let h = 0;
    for (let i = 0; i < alias.length; i++) h = ((h << 5) - h) + alias.charCodeAt(i);
    const dashPatterns: number[][] = [
      [], [6, 4], [3, 3], [10, 4], [2, 4], [1, 3], [8, 3, 2, 3]
    ];
    const pointStyles: any[] = ['circle', 'rect', 'triangle', 'rectRounded', 'star', 'crossRot', 'line'];
    const dash = dashPatterns[Math.abs(h) % dashPatterns.length];
    const pointStyle = pointStyles[Math.abs(h >> 3) % pointStyles.length];
    return { dash, pointStyle } as { dash: number[]; pointStyle: any };
  }, []);

  // Effect to handle WebSocket messages for real-time trajectory updates (position-only),
  // plus lightweight mecanum detection from encoder_data, navigation, and map
  useEffect(() => {
    if (lastJsonMessage) {
      try {
        const message = lastJsonMessage as any;
        // Position-only updates from firmware (authoritative)
        if (message.type === 'position_update' && message.robot_alias) {
          const alias = message.robot_alias as string;
          const position = message.position || message.data; // tolerate either shape
          if (position && typeof position.x === 'number' && typeof position.y === 'number') {
            const newPose: RobotPose = {
              x: position.x,
              y: position.y,
              theta: typeof position.theta === 'number' ? position.theta : 0,
            };
            setRobotPoses(prev => {
              const next = { ...prev, [alias]: newPose };
              robotPosesRef.current = next;
              return next;
            });

            // Append to path (position-only history)
            setRobotPaths(prev => {
              const prevPath = prev[alias] || [];
              const last = prevPath[prevPath.length - 1];
              // Avoid duplicate consecutive points
              const shouldAppend = !last || last.x !== newPose.x || last.y !== newPose.y || last.theta !== newPose.theta;
              const nextPath = shouldAppend ? [...prevPath, { x: newPose.x, y: newPose.y, theta: newPose.theta }] : prevPath;
              // Cap length
              const capped = nextPath.length > MAX_PATH_POINTS_DISPLAY ? nextPath.slice(-MAX_PATH_POINTS_DISPLAY) : nextPath;
              return { ...prev, [alias]: capped };
            });
            if (widgetError) setWidgetError(null);
          } else {
            console.warn('[TrajectoryWidget] Malformed position_update:', message);
          }
        } else if (message.type === 'realtime_trajectory' && message.robot_alias) {
          // Intentionally ignored: only position_update drives drawing per user request.
        } else if (message.type === 'encoder_data' && message.robot_alias) {
          // Type detection only: if 4+ encoders seen, render mecanum icon for this alias
          const alias = message.robot_alias as string;
          const rpms = Array.isArray(message.data) ? message.data : [];
          if (appConfig.features.encoderTypeDetection && rpms.length >= 4 && robotTypesRef.current[alias] !== 'mecanum') {
            robotTypesRef.current[alias] = 'mecanum';
            chartRef.current?.update('none');
          }
        } else if (message.type === 'planned_path' && message.robot_alias) {
          const alias = message.robot_alias as string;
          const points = Array.isArray(message.points) ? message.points.filter((p: any) => typeof p.x === 'number' && typeof p.y === 'number') : [];
          setPlannedPaths(prev => ({ ...prev, [alias]: points }));
        } else if (message.type === 'navigation_status' && message.robot_alias) {
          const alias = message.robot_alias as string;
          if (typeof message.status === 'string') setNavStatus(prev => ({ ...prev, [alias]: message.status }));
        } else if (message.type === 'map_loaded') {
          const { width, height, resolution, origin_x, origin_y } = message;
          if (typeof width === 'number' && typeof height === 'number' && typeof resolution === 'number') {
            setMapMeta({ width, height, resolution, origin_x: origin_x ?? 0, origin_y: origin_y ?? 0 });
            // Keep existing imageUrl; UI triggers upload and stores preview already
          }
        }
      } catch (error) {
        console.error("Error processing trajectory message:", error);
        setWidgetError("Error processing trajectory data.");
      }
    }
  }, [lastJsonMessage, widgetError]);

  // Effect for subscribing to all connected robots (position_update for drawing, encoder_data for type detection, navigation topics)
  useEffect(() => {
    if (readyState !== ReadyState.OPEN) return;
    const currentSubs = subscribedAliasesRef.current;
    const targetAliases = new Set((connectedRobots || []).map(r => r.alias));

  // Unsubscribe removed
    currentSubs.forEach(alias => {
      if (!targetAliases.has(alias)) {
        sendJsonMessage({ command: 'unsubscribe', type: 'position_update', robot_alias: alias });
        sendJsonMessage({ command: 'unsubscribe', type: 'planned_path', robot_alias: alias });
        sendJsonMessage({ command: 'unsubscribe', type: 'navigation_status', robot_alias: alias });
        if (appConfig.features.encoderTypeDetection) {
          sendJsonMessage({ command: 'unsubscribe', type: 'encoder_data', robot_alias: alias });
        }
        currentSubs.delete(alias);
        setRobotPaths(prev => { const n = { ...prev }; delete n[alias]; return n; });
        setRobotPoses(prev => { const n = { ...prev }; delete n[alias]; robotPosesRef.current = n; return n; });
      }
    });

  // Subscribe new
    targetAliases.forEach(alias => {
      if (!currentSubs.has(alias)) {
        sendJsonMessage({ command: 'subscribe', type: 'position_update', robot_alias: alias });
        sendJsonMessage({ command: 'subscribe', type: 'planned_path', robot_alias: alias });
        sendJsonMessage({ command: 'subscribe', type: 'navigation_status', robot_alias: alias });
        // Subscribe to encoder_data solely for mecanum detection (4 channels)
        if (appConfig.features.encoderTypeDetection) {
          sendJsonMessage({ command: 'subscribe', type: 'encoder_data', robot_alias: alias });
        }
        currentSubs.add(alias);
      }
    });

    // Cleanup on unmount: unsubscribe all
    return () => {
      currentSubs.forEach(alias => {
        sendJsonMessage({ command: 'unsubscribe', type: 'position_update', robot_alias: alias });
        sendJsonMessage({ command: 'unsubscribe', type: 'planned_path', robot_alias: alias });
        sendJsonMessage({ command: 'unsubscribe', type: 'navigation_status', robot_alias: alias });
        if (appConfig.features.encoderTypeDetection) {
          sendJsonMessage({ command: 'unsubscribe', type: 'encoder_data', robot_alias: alias });
        }
      });
      currentSubs.clear();
    };
  }, [connectedRobots, readyState, sendJsonMessage]);

  // Initialize chart once on mount
  useEffect(() => {
    if (!canvasRef.current) return;
    const ctx = canvasRef.current.getContext('2d');
    if (!ctx) return;
    // Plugin to draw robot icons:
    // - Omni: circular body + 3 wheels at 120°
    // - Mecanum: circular body + 4 mecanum wheels with 45° rollers
  const robotIconPlugin: any = {
      id: 'robotIconPlugin',
      afterDatasetsDraw: (chart: any) => {
        const posesMap = robotPosesRef.current || {};
        const xScale = chart.scales['x'];
        const yScale = chart.scales['y'];
        if (!xScale || !yScale) return;
        const ctx2 = chart.ctx as CanvasRenderingContext2D;
        const wheelAngles = [0, (2*Math.PI)/3, (4*Math.PI)/3];
        const drawOmni = (pose: RobotPose, color: string) => {
          const cx = xScale.getPixelForValue(pose.x);
          const cy = yScale.getPixelForValue(pose.y);
          const bodyR = 10;
          const wheelR = bodyR + 6;
          const wheelLen = 12;
          const wheelThk = 4;
          const arrowLen = 14;
          const arrowBase = 8;
          ctx2.save();
          ctx2.translate(cx, cy);
          ctx2.rotate(-(pose.theta || 0));
          // Body
          ctx2.beginPath();
          ctx2.arc(0, 0, bodyR, 0, Math.PI * 2);
          ctx2.fillStyle = `${color}33`;
          ctx2.fill();
          ctx2.lineWidth = 2;
          ctx2.strokeStyle = color;
          ctx2.stroke();
          // Wheels
          for (const ang of wheelAngles) {
            const wx = wheelR * Math.cos(ang);
            const wy = wheelR * Math.sin(ang);
            ctx2.save(); ctx2.translate(wx, wy); ctx2.rotate(ang + Math.PI/2);
            const w = wheelLen, h = wheelThk, rx = -w/2, ry = -h/2, r = Math.min(2, h/2);
            ctx2.beginPath();
            ctx2.moveTo(rx + r, ry);
            ctx2.lineTo(rx + w - r, ry);
            ctx2.quadraticCurveTo(rx + w, ry, rx + w, ry + r);
            ctx2.lineTo(rx + w, ry + h - r);
            ctx2.quadraticCurveTo(rx + w, ry + h, rx + w - r, ry + h);
            ctx2.lineTo(rx + r, ry + h);
            ctx2.quadraticCurveTo(rx, ry + h, rx, ry + h - r);
            ctx2.lineTo(rx, ry + r);
            ctx2.quadraticCurveTo(rx, ry, rx + r, ry);
            ctx2.closePath();
            ctx2.fillStyle = 'rgba(200, 200, 200, 0.85)';
            ctx2.fill();
            ctx2.lineWidth = 1.5;
            ctx2.strokeStyle = 'rgba(80, 80, 80, 0.9)';
            ctx2.stroke();
            ctx2.restore();
          }
          // Heading
          ctx2.beginPath();
          ctx2.moveTo(arrowLen, 0);
          ctx2.lineTo(-arrowLen * 0.35, arrowBase / 2);
          ctx2.lineTo(-arrowLen * 0.35, -arrowBase / 2);
          ctx2.closePath();
          ctx2.fillStyle = color;
          ctx2.fill();
          ctx2.restore();
        };
        const drawMecanum = (pose: RobotPose, color: string) => {
          const cx = xScale.getPixelForValue(pose.x);
          const cy = yScale.getPixelForValue(pose.y);
          const bodyR = 10;
          const wheelR = bodyR + 7;
          const wheelLen = 14;
          const wheelThk = 4;
          const arrowLen = 14;
          const arrowBase = 8;
          ctx2.save();
          ctx2.translate(cx, cy);
          ctx2.rotate(-(pose.theta || 0));
          // Body
          ctx2.beginPath();
          ctx2.arc(0, 0, bodyR, 0, Math.PI * 2);
          ctx2.fillStyle = `${color}33`;
          ctx2.fill();
          ctx2.lineWidth = 2;
          ctx2.strokeStyle = color;
          ctx2.stroke();
          // Four mecanum wheels at N/E/S/W with ±45° roller orientation
          const wheels = [
            { x: wheelR,  y: 0,        ang: Math.PI/4 },   // right
            { x: -wheelR, y: 0,        ang: -Math.PI/4 },  // left
            { x: 0,       y: -wheelR,  ang: -Math.PI/4 },  // top
            { x: 0,       y: wheelR,   ang: Math.PI/4 },   // bottom
          ];
          for (const wpos of wheels) {
            ctx2.save(); ctx2.translate(wpos.x, wpos.y); ctx2.rotate(wpos.ang);
            const w = wheelLen, h = wheelThk, rx = -w/2, ry = -h/2, r = Math.min(2, h/2);
            // Base wheel rectangle
            ctx2.beginPath();
            ctx2.moveTo(rx + r, ry);
            ctx2.lineTo(rx + w - r, ry);
            ctx2.quadraticCurveTo(rx + w, ry, rx + w, ry + r);
            ctx2.lineTo(rx + w, ry + h - r);
            ctx2.quadraticCurveTo(rx + w, ry + h, rx + w - r, ry + h);
            ctx2.lineTo(rx + r, ry + h);
            ctx2.quadraticCurveTo(rx, ry + h, rx, ry + h - r);
            ctx2.lineTo(rx, ry + r);
            ctx2.quadraticCurveTo(rx, ry, rx + r, ry);
            ctx2.closePath();
            ctx2.fillStyle = 'rgba(210, 210, 210, 0.95)';
            ctx2.fill();
            ctx2.lineWidth = 1.5;
            ctx2.strokeStyle = 'rgba(70, 70, 70, 0.95)';
            ctx2.stroke();
            // Diagonal rollers hint lines
            ctx2.save();
            ctx2.strokeStyle = 'rgba(70,70,70,0.9)';
            ctx2.lineWidth = 1;
            const lines = 3; // small diagonal grooves
            for (let i = 0; i < lines; i++) {
              const t = (i + 1) / (lines + 1);
              const lx = rx + t * w;
              ctx2.beginPath();
              ctx2.moveTo(lx - 3, ry);
              ctx2.lineTo(lx + 3, ry + h);
              ctx2.stroke();
            }
            ctx2.restore();
            ctx2.restore();
          }
          // Heading
          ctx2.beginPath();
          ctx2.moveTo(arrowLen, 0);
          ctx2.lineTo(-arrowLen * 0.35, arrowBase / 2);
          ctx2.lineTo(-arrowLen * 0.35, -arrowBase / 2);
          ctx2.closePath();
          ctx2.fillStyle = color;
          ctx2.fill();
          ctx2.restore();
        };
        Object.entries(posesMap).forEach(([alias, pose]) => {
          if (!pose) return;
          const color = getColorForAlias(alias);
          const rtype = robotTypesRef.current[alias] || 'omni';
          if (rtype === 'mecanum') drawMecanum(pose, color); else drawOmni(pose, color);
        });
      }
    };

    // Background plugin to improve contrast on dark UI
    const chartBgPlugin: any = {
      id: 'chartBgPlugin',
      beforeDraw(chart: any) {
        const { ctx } = chart;
        const { chartArea } = chart;
        if (!chartArea) return;
        ctx.save();
        ctx.fillStyle = '#0b1220'; // deep navy for higher contrast
        ctx.fillRect(chartArea.left, chartArea.top, chartArea.right - chartArea.left, chartArea.bottom - chartArea.top);
        ctx.restore();
      }
    };

    // Map overlay plugin draws uploaded map aligned with world coordinates
    const mapOverlayPlugin: any = {
      id: 'mapOverlayPlugin',
      beforeDatasetsDraw: (chart: any) => {
        if (!mapImageUrl || !mapMeta) return;
        const img = new Image();
        img.src = mapImageUrl;
        const xScale = chart.scales['x'];
        const yScale = chart.scales['y'];
        if (!xScale || !yScale) return;
        // Compute pixel bounds for world-aligned image
        const x0 = xScale.getPixelForValue(mapMeta.origin_x);
        const x1 = xScale.getPixelForValue(mapMeta.origin_x + mapMeta.width * mapMeta.resolution);
        const yTop = yScale.getPixelForValue(mapMeta.origin_y + mapMeta.height * mapMeta.resolution);
        const yBot = yScale.getPixelForValue(mapMeta.origin_y);
        const wpx = x1 - x0;
        const hpx = yBot - yTop;
        try {
          chart.ctx.save();
          chart.ctx.globalAlpha = 0.6;
          chart.ctx.drawImage(img, x0, yTop, wpx, hpx);
          chart.ctx.restore();
        } catch {}
      }
    };

    chartRef.current = new ChartJS(ctx, {
      type: 'line',
  data: { datasets: [] },
  options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: {
            type: 'linear',
            position: 'bottom',
    title: { display: true, text: 'X (meters)', color: '#e5e7eb' },
    grid: { color: 'rgba(148,163,184,0.25)' },
    ticks: { color: '#cbd5e1' },
          },
          y: {
            type: 'linear',
    title: { display: true, text: 'Y (meters)', color: '#e5e7eb' },
    grid: { color: 'rgba(148,163,184,0.25)' },
    ticks: { color: '#cbd5e1' },
          },
        },
        plugins: {
      legend: { position: 'top' as const, labels: { color: '#e5e7eb' } },
          tooltip: {
            callbacks: {
              label: function (context) {
                const point = context.raw as TrajectoryPoint;
                let label = `X: ${point.x.toFixed(2)}, Y: ${point.y.toFixed(2)}`;
                if (point.theta !== undefined) {
                  label += `, Theta: ${(point.theta * 180 / Math.PI).toFixed(1)}°`;
                }
                return label;
              }
            }
          },
          zoom: { pan: { enabled: true, mode: 'xy' as const }, zoom: { wheel: { enabled: true }, pinch: { enabled: true }, mode: 'xy' as const } },
        },
        aspectRatio: 1,
      },
  plugins: [robotIconPlugin, chartBgPlugin, mapOverlayPlugin],
    });
    return () => {
      chartRef.current?.destroy();
      chartRef.current = null;
    };
  }, [getColorForAlias, mapImageUrl, mapMeta]);

  // Update chart datasets when paths change (multi-robot)
  useEffect(() => {
    const chart = chartRef.current;
    if (!chart) return;
    const aliases = Object.keys(robotPaths);
    const datasets: any[] = [];
    // Planned paths first (thin, dashed)
    Object.keys(plannedPaths).forEach(alias => {
      const pathPts = plannedPaths[alias] || [];
      if (pathPts.length === 0) return;
      const color = getColorForAlias(alias);
      datasets.push({
        label: `${alias} planned`,
        data: pathPts as any,
        borderColor: color,
        pointBackgroundColor: color,
        borderDash: [6, 4],
        borderWidth: 2,
        pointRadius: 0,
        fill: false,
        tension: 0,
      });
    });
    // Actual paths
    aliases.forEach((alias) => {
      const color = getColorForAlias(alias);
      const { dash, pointStyle } = getStyleForAlias(alias);
      datasets.push({
        label: alias,
        data: (robotPaths[alias] || []) as any,
        borderColor: color,
        pointBackgroundColor: color,
        fill: false,
        borderWidth: selectedRobotId === alias ? 3.5 : 2.25,
        borderDash: dash,
        pointStyle,
        pointRadius: selectedRobotId === alias ? 3 : 2,
        tension: 0.1,
      });
    });
    chart.data.datasets = datasets;
    chart.update('none');
  }, [robotPaths, plannedPaths, selectedRobotId, getColorForAlias, getStyleForAlias]);

  // Repaint icons when poses change
  useEffect(() => {
    chartRef.current?.update('none');
  }, [robotPoses]);

  // Keep robot types in a ref for the icon plugin. Trigger repaint when the list changes.
  useEffect(() => {
    const map: Record<string, 'omni' | 'mecanum'> = {};
    (connectedRobots || []).forEach(r => {
      map[r.alias] = (r.robot_type === 'mecanum' ? 'mecanum' : 'omni');
    });
    robotTypesRef.current = map;
    // Repaint to reflect icon changes
    chartRef.current?.update('none');
  }, [connectedRobots]);

  const resetZoom = () => chartRef.current?.resetZoom();
  const fileInputRef = useRef<HTMLInputElement>(null);
  const [mapConfig, setMapConfig] = useState({ resolution: 0.02, origin_x: 0, origin_y: 0, threshold: 127 });
  const onUploadMapClick = () => fileInputRef.current?.click();
  const onMapFileSelected = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = () => {
      const result = reader.result as string;
      setMapImageUrl(result);
      const base64 = result.split(',')[1] || '';
      if (readyState === WebSocket.OPEN) {
        sendJsonMessage({ command: 'upload_map', data: base64, ...mapConfig });
      }
    };
    reader.readAsDataURL(file);
  };
  const handleCanvasClick = (evt: React.MouseEvent<HTMLCanvasElement>) => {
    if (!selectedRobotId || firmwareUpdateMode) return;
    const chart = chartRef.current as any;
    if (!chart) return;
    const rect = chart.canvas.getBoundingClientRect();
    const px = evt.clientX - rect.left;
    const py = evt.clientY - rect.top;
    const xs = chart.scales['x'];
    const ys = chart.scales['y'];
    if (!xs || !ys) return;
    const xVal = xs.getValueForPixel(px);
    const yVal = ys.getValueForPixel(py);
    if (readyState === WebSocket.OPEN) {
      sendJsonMessage({ command: 'navigate_to', robot_alias: selectedRobotId, x: xVal, y: yVal, speed: 0.2 });
    }
  };
  const clearPath = () => {
    if (selectedRobotId && readyState === WebSocket.OPEN) {
      // Send a command to the backend to clear its trajectory history
      sendJsonMessage({
        command: "clear_trajectory", // New command type
        robot_alias: selectedRobotId,
      });
      
      // Optimistically clear the frontend state as well.
      setRobotPaths(prev => ({ ...prev, [selectedRobotId]: [] }));
      setRobotPoses(prev => { const n = { ...prev, [selectedRobotId]: null }; robotPosesRef.current = n; return n; });
      chartRef.current?.update('none');
    } else {
      // Fallback for local clear if WS not ready or no robot selected
      console.warn("[TrajectoryWidget] Clear path attempted but no selected robot.");
      console.warn("[TrajectoryWidget] Clear path attempted locally (WS not open or no robot selected).");
    }
  };

  let derivedStatusText: string;
  if (readyState === WebSocket.CONNECTING) {
    derivedStatusText = "WS: Connecting...";
  } else if (readyState !== WebSocket.OPEN) {
    derivedStatusText = "WS: Disconnected";
  } else if (!selectedRobotId) {
    derivedStatusText = "No robot selected";
  } else if (firmwareUpdateMode) {
    derivedStatusText = "Firmware Update Mode - Trajectory Disabled";
  } else {
    derivedStatusText = `Tracking ${selectedRobotId}`;
  }

  return (
  <div className={`flex flex-col h-full ${compact ? 'p-2' : 'p-3'} bg-white dark:bg-gray-800 text-gray-900 dark:text-gray-200 rounded-lg shadow-xl`}>
      <WidgetConnectionHeader
  title={`Real-time Trajectory (Position-only)`}
        statusTextOverride={derivedStatusText}
        isConnected={readyState === WebSocket.OPEN && !!selectedRobotId && !firmwareUpdateMode}
        error={widgetError}
      />

      <div className={`flex gap-2 mb-2 items-center flex-wrap ${compact ? 'mt-1' : ''}`}>
  {/* Follow/highlight buttons for connected robots */}
        {connectedRobots && connectedRobots.length > 0 && (
          <div className="flex items-center gap-1 flex-wrap">
            {connectedRobots.map((r) => {
              const isActive = r.alias === selectedRobotId;
              return (
                <button
                  key={r.key || r.alias}
                  onClick={() => !firmwareUpdateMode && setSelectedRobotId(r.alias)}
                  className={`px-2 py-1 rounded-md text-xs border transition-colors ${
                    isActive
                      ? 'bg-emerald-600 text-white border-emerald-700'
                      : 'bg-gray-100 dark:bg-gray-700 text-gray-800 dark:text-gray-200 border-gray-300 dark:border-gray-600 hover:bg-gray-200 dark:hover:bg-gray-600'
                  } ${firmwareUpdateMode ? 'opacity-50 cursor-not-allowed' : ''}`}
                  disabled={firmwareUpdateMode}
                  title={r.ip}
                >
                  {r.alias}
                </button>
              );
            })}
          </div>
        )}

        {/* Utility controls */}
        <div className="flex items-center gap-2 ml-auto">
          {/* Map controls */}
          <div className="flex items-center gap-2">
            <button
              onClick={onUploadMapClick}
              className="px-3 py-1.5 bg-emerald-600 text-white rounded-md flex items-center gap-1 hover:bg-emerald-700 disabled:opacity-50 text-xs"
              disabled={firmwareUpdateMode}
              title="Upload occupancy map image"
            >
              Upload Map
            </button>
            <input ref={fileInputRef} type="file" accept="image/*" className="hidden" onChange={onMapFileSelected} />
            <div className="flex items-center gap-1 text-xs text-gray-600 dark:text-gray-300">
              <label>res:</label>
              <input type="number" step="0.001" className="w-16 px-1 py-0.5 rounded border dark:bg-gray-700"
                value={mapConfig.resolution} onChange={e => setMapConfig(cfg => ({ ...cfg, resolution: parseFloat(e.target.value) }))} />
              <label>ox:</label>
              <input type="number" step="0.01" className="w-16 px-1 py-0.5 rounded border dark:bg-gray-700"
                value={mapConfig.origin_x} onChange={e => setMapConfig(cfg => ({ ...cfg, origin_x: parseFloat(e.target.value) }))} />
              <label>oy:</label>
              <input type="number" step="0.01" className="w-16 px-1 py-0.5 rounded border dark:bg-gray-700"
                value={mapConfig.origin_y} onChange={e => setMapConfig(cfg => ({ ...cfg, origin_y: parseFloat(e.target.value) }))} />
            </div>
          </div>
          <button
            onClick={resetZoom}
            className="px-3 py-1.5 bg-blue-600 text-white rounded-md flex items-center gap-1 hover:bg-blue-700 disabled:opacity-50 text-xs"
            disabled={firmwareUpdateMode}
          >
            <Maximize size={14} /> Reset Zoom
          </button>
          <button
            onClick={clearPath}
            className="px-3 py-1.5 bg-red-600 text-white rounded-md flex items-center gap-1 hover:bg-red-700 disabled:opacity-50 text-xs"
            disabled={!selectedRobotId || firmwareUpdateMode}
          >
            <RotateCcw size={14} /> Clear Path
          </button>
          <button
            onClick={() => {
              // Clear all
              Object.keys(robotPaths).forEach(alias => {
                if (readyState === WebSocket.OPEN) {
                  sendJsonMessage({ command: 'clear_trajectory', robot_alias: alias });
                }
              });
              setRobotPaths({}); setRobotPoses({}); robotPosesRef.current = {};
              chartRef.current?.update('none');
            }}
            className="px-3 py-1.5 bg-amber-600 text-white rounded-md flex items-center gap-1 hover:bg-amber-700 disabled:opacity-50 text-xs"
            disabled={firmwareUpdateMode}
          >
            <RotateCcw size={14} /> Clear All
          </button>
        </div>
      </div>

      {widgetError && (
        <div className="bg-red-100 border border-red-400 text-red-700 px-3 py-2 rounded relative mb-2 text-xs" role="alert">
          <strong className="font-bold">Error: </strong>
          <span className="block sm:inline">{widgetError}</span>
        </div>
      )}
      {selectedRobotId && navStatus[selectedRobotId] && (
        <div className="bg-amber-100 border border-amber-300 text-amber-700 px-3 py-1 rounded mb-2 text-xs">
          Navigation: {navStatus[selectedRobotId]}
        </div>
      )}

      <div className={`flex-grow relative w-full ${compact ? 'h-56 min-h-[220px]' : 'h-64 md:h-auto min-h-[300px]'}`}>
        {firmwareUpdateMode ? (
          <div className="absolute inset-0 flex items-center justify-center bg-gray-700 bg-opacity-80">
            <p className="text-lg font-semibold">Trajectory view disabled during Firmware Update.</p>
          </div>
        ) : (
          <canvas ref={canvasRef} onClick={handleCanvasClick}></canvas>
        )}
      </div>
    </div>
  );
};

export default TrajectoryWidget;