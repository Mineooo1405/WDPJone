import React, { useEffect, useRef, useState, useContext, useCallback } from 'react';
import { Chart as ChartJS, LineController, LineElement, PointElement, LinearScale, Title, Tooltip, Legend, CategoryScale } from 'chart.js';
import zoomPlugin from 'chartjs-plugin-zoom';
import { useRobotContext, ReadyState } from './RobotContext'; 
import { GlobalAppContext } from '../contexts/GlobalAppContext'; 
import { RotateCcw, Maximize } from 'lucide-react';
import WidgetConnectionHeader from './WidgetConnectionHeader';

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

const MAX_PATH_POINTS_DISPLAY = 500; // Max points to keep in the chart for performance

const TrajectoryWidget: React.FC<{ compact?: boolean }> = ({ compact = false }) => {
  const { selectedRobotId, setSelectedRobotId, connectedRobots, sendJsonMessage, lastJsonMessage, readyState } = useRobotContext();
  const { firmwareUpdateMode } = useContext(GlobalAppContext);

  const chartRef = useRef<ChartJS<'line', TrajectoryPoint[], unknown> | null>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);
  // Multi-robot state
  const [robotPaths, setRobotPaths] = useState<Record<string, TrajectoryPoint[]>>({});
  const [robotPoses, setRobotPoses] = useState<Record<string, RobotPose | null>>({});
  const robotPosesRef = useRef<Record<string, RobotPose | null>>({}); // For plugin access
  const subscribedAliasesRef = useRef<Set<string>>(new Set());
  const colorMapRef = useRef<Record<string, string>>({});
  const [widgetError, setWidgetError] = useState<string | null>(null);

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

  // Effect to handle WebSocket messages for real-time trajectory updates (multi-robot)
  useEffect(() => {
    if (lastJsonMessage) {
      try {
        const message = lastJsonMessage as any;
        // Check for the 'realtime_trajectory' message type from the backend
        if (message.type === 'realtime_trajectory' && message.robot_alias) {
          const alias = message.robot_alias as string;
          // console.log(`[TrajectoryWidget] Received realtime_trajectory for ${alias}:`, message);
          const { position, path } = message;

          if (position && typeof position.x === 'number' && typeof position.y === 'number' && (typeof position.theta === 'number' || typeof position.theta === 'undefined') && Array.isArray(path)) {
            const newPose: RobotPose = {
              x: position.x,
              y: position.y,
              theta: typeof position.theta === 'number' ? position.theta : 0, // Default theta if undefined
            };
            setRobotPoses(prev => {
              const next = { ...prev, [alias]: newPose };
              robotPosesRef.current = next;
              return next;
            });
            // console.log("[TrajectoryWidget] Updated currentPose:", newPose); // Log new pose

            const newPathPoints: TrajectoryPoint[] = path
              .filter(p => p && typeof p.x === 'number' && typeof p.y === 'number') // Filter out invalid points
              .map((p: any) => ({ x: p.x, y: p.y })) // Ensure points have x and y
              .slice(-MAX_PATH_POINTS_DISPLAY);
            
            setRobotPaths(prev => ({ ...prev, [alias]: newPathPoints }));
            // Log only the first point or length to avoid flooding console with large paths
            // console.log("[TrajectoryWidget] Updated currentPath (length " + newPathPoints.length + "):", newPathPoints.length > 0 ? newPathPoints[0] : 'empty', newPathPoints);


            if (widgetError) setWidgetError(null); // Clear any previous error
          } else {
            console.warn("[TrajectoryWidget] Received malformed realtime_trajectory data. Position:", position, "Path:", path, "Message:", message);
            // Optionally, set an error state for the user, but be mindful of spamming if messages are frequent
            // setWidgetError("Malformed trajectory data received."); 
          }
        } else if (message.type === 'trajectory_data' && message.robot_alias) {
          const alias = message.robot_alias as string;
          // Handle older 'trajectory_data' format if still in use, with similar logging
          // console.log(`[TrajectoryWidget] Received (old) trajectory_data for ${alias}:`, message);
          const { trajectory } = message; 
          if (Array.isArray(trajectory)) {
              const newPath: TrajectoryPoint[] = trajectory
                  .filter(p => p && typeof p.x === 'number' && typeof p.y === 'number')
                  .map((p: any) => ({ x: p.x, y: p.y }))
                  .slice(-MAX_PATH_POINTS_DISPLAY);
              setRobotPaths(prev => ({ ...prev, [alias]: newPath }));
              // console.log("[TrajectoryWidget] Updated currentPath from trajectory_data (length " + newPath.length + "):", newPath.length > 0 ? newPath[0] : 'empty', newPath);

              if (trajectory.length > 0) {
                  const lastPoint = trajectory[trajectory.length - 1];
                  if (lastPoint && typeof lastPoint.x === 'number' && typeof lastPoint.y === 'number') {
                      const newPose: RobotPose = {
                          x: lastPoint.x,
                          y: lastPoint.y,
                          theta: typeof lastPoint.theta === 'number' ? lastPoint.theta : 0,
                      };
                      setRobotPoses(prev => {
                        const next = { ...prev, [alias]: newPose };
                        robotPosesRef.current = next;
                        return next;
                      });
                      // console.log("[TrajectoryWidget] Updated currentPose from trajectory_data:", newPose);
                  }
              }
              if (widgetError) setWidgetError(null);
          } else {
              console.warn("[TrajectoryWidget] Received malformed trajectory_data. Trajectory:", trajectory, "Message:", message);
          }
        } else if (message.robot_alias) {
          // Log if message is for the selected robot but not a recognized trajectory type
          // console.log(`[TrajectoryWidget] Received message of type '${message.type}' for ${selectedRobotId}, not a trajectory type.`);
        }
      } catch (error) {
        console.error("Error processing trajectory message:", error);
        setWidgetError("Error processing trajectory data.");
      }
    }
  }, [lastJsonMessage, widgetError]);

  // Effect for subscribing to all connected robots and unsubscribing when they disappear
  useEffect(() => {
    if (readyState !== ReadyState.OPEN) return;
    const currentSubs = subscribedAliasesRef.current;
    const targetAliases = new Set((connectedRobots || []).map(r => r.alias));

    // Unsubscribe removed
    currentSubs.forEach(alias => {
      if (!targetAliases.has(alias)) {
        sendJsonMessage({ command: 'unsubscribe', type: 'realtime_trajectory', robot_alias: alias });
        currentSubs.delete(alias);
        setRobotPaths(prev => { const n = { ...prev }; delete n[alias]; return n; });
        setRobotPoses(prev => { const n = { ...prev }; delete n[alias]; robotPosesRef.current = n; return n; });
      }
    });

    // Subscribe new
    targetAliases.forEach(alias => {
      if (!currentSubs.has(alias)) {
        sendJsonMessage({ command: 'subscribe', type: 'realtime_trajectory', robot_alias: alias });
        currentSubs.add(alias);
      }
    });

    // Cleanup on unmount: unsubscribe all
    return () => {
      currentSubs.forEach(alias => {
        sendJsonMessage({ command: 'unsubscribe', type: 'realtime_trajectory', robot_alias: alias });
      });
      currentSubs.clear();
    };
  }, [connectedRobots, readyState, sendJsonMessage]);

  // Initialize chart once on mount
  useEffect(() => {
    if (!canvasRef.current) return;
    const ctx = canvasRef.current.getContext('2d');
    if (!ctx) return;
    // Plugin to draw a 3-wheel omni robot: body + 3 wheels + heading mark
  const robotIconPlugin: any = {
      id: 'robotIconPlugin',
      afterDatasetsDraw: (chart: any) => {
        const posesMap = robotPosesRef.current || {};
        const xScale = chart.scales['x'];
        const yScale = chart.scales['y'];
        if (!xScale || !yScale) return;
        const ctx2 = chart.ctx as CanvasRenderingContext2D;
        const wheelAngles = [0, (2*Math.PI)/3, (4*Math.PI)/3];
        const drawRobot = (pose: RobotPose, color: string) => {
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
        Object.entries(posesMap).forEach(([alias, pose]) => {
          if (!pose) return;
          const color = getColorForAlias(alias);
          drawRobot(pose, color);
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
  plugins: [robotIconPlugin, chartBgPlugin],
    });
    return () => {
      chartRef.current?.destroy();
      chartRef.current = null;
    };
  }, [getColorForAlias]);

  // Update chart datasets when paths change (multi-robot)
  useEffect(() => {
    const chart = chartRef.current;
    if (!chart) return;
    const aliases = Object.keys(robotPaths);
    // Rebuild datasets to match aliases
    chart.data.datasets = aliases.map((alias) => {
      const color = getColorForAlias(alias);
      const { dash, pointStyle } = getStyleForAlias(alias);
      return {
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
      };
    });
    chart.update('none');
  }, [robotPaths, selectedRobotId, getColorForAlias, getStyleForAlias]);

  // Repaint icons when poses change
  useEffect(() => {
    chartRef.current?.update('none');
  }, [robotPoses]);

  const resetZoom = () => chartRef.current?.resetZoom();
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
  title={`Real-time Trajectory (Multi)`}
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

      <div className={`flex-grow relative w-full ${compact ? 'h-56 min-h-[220px]' : 'h-64 md:h-auto min-h-[300px]'}`}>
        {firmwareUpdateMode ? (
          <div className="absolute inset-0 flex items-center justify-center bg-gray-700 bg-opacity-80">
            <p className="text-lg font-semibold">Trajectory view disabled during Firmware Update.</p>
          </div>
        ) : (
          <canvas ref={canvasRef}></canvas>
        )}
      </div>
    </div>
  );
};

export default TrajectoryWidget;