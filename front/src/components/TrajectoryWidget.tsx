import React, { useEffect, useRef, useState, useContext } from 'react';
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
  const { selectedRobotId, sendJsonMessage, lastJsonMessage, readyState } = useRobotContext();
  const { firmwareUpdateMode } = useContext(GlobalAppContext);

  const chartRef = useRef<ChartJS<'line', TrajectoryPoint[], unknown> | null>(null);
  const canvasRef = useRef<HTMLCanvasElement>(null);

  const [currentPath, setCurrentPath] = useState<TrajectoryPoint[]>([]);
  const [currentPose, setCurrentPose] = useState<RobotPose | null>(null);
  const currentPoseRef = useRef<RobotPose | null>(null); // For plugin access
  const [widgetError, setWidgetError] = useState<string | null>(null);
  const subscribedToRobotRef = useRef<string | null>(null);

  // Effect to handle WebSocket messages for real-time trajectory updates
  useEffect(() => {
    if (lastJsonMessage && selectedRobotId) {
      try {
        const message = lastJsonMessage as any;
        // Check for the 'realtime_trajectory' message type from the backend
        if (message.type === 'realtime_trajectory' && message.robot_alias === selectedRobotId) {
          console.log(`[TrajectoryWidget] Received realtime_trajectory for ${selectedRobotId}:`, message);
          const { position, path } = message;

          if (position && typeof position.x === 'number' && typeof position.y === 'number' && (typeof position.theta === 'number' || typeof position.theta === 'undefined') && Array.isArray(path)) {
            const newPose: RobotPose = {
              x: position.x,
              y: position.y,
              theta: typeof position.theta === 'number' ? position.theta : 0, // Default theta if undefined
            };
            setCurrentPose(newPose);
            currentPoseRef.current = newPose; // keep ref synced for plugin
            // console.log("[TrajectoryWidget] Updated currentPose:", newPose); // Log new pose

            const newPathPoints: TrajectoryPoint[] = path
              .filter(p => p && typeof p.x === 'number' && typeof p.y === 'number') // Filter out invalid points
              .map((p: any) => ({ x: p.x, y: p.y })) // Ensure points have x and y
              .slice(-MAX_PATH_POINTS_DISPLAY);
            
            setCurrentPath(newPathPoints);
            // Log only the first point or length to avoid flooding console with large paths
            // console.log("[TrajectoryWidget] Updated currentPath (length " + newPathPoints.length + "):", newPathPoints.length > 0 ? newPathPoints[0] : 'empty', newPathPoints);


            if (widgetError) setWidgetError(null); // Clear any previous error
          } else {
            console.warn("[TrajectoryWidget] Received malformed realtime_trajectory data. Position:", position, "Path:", path, "Message:", message);
            // Optionally, set an error state for the user, but be mindful of spamming if messages are frequent
            // setWidgetError("Malformed trajectory data received."); 
          }
        } else if (message.type === 'trajectory_data' && message.robot_alias === selectedRobotId) {
          // Handle older 'trajectory_data' format if still in use, with similar logging
          console.log(`[TrajectoryWidget] Received (old) trajectory_data for ${selectedRobotId}:`, message);
          const { trajectory } = message; 
          if (Array.isArray(trajectory)) {
              const newPath: TrajectoryPoint[] = trajectory
                  .filter(p => p && typeof p.x === 'number' && typeof p.y === 'number')
                  .map((p: any) => ({ x: p.x, y: p.y }))
                  .slice(-MAX_PATH_POINTS_DISPLAY);
              setCurrentPath(newPath);
              // console.log("[TrajectoryWidget] Updated currentPath from trajectory_data (length " + newPath.length + "):", newPath.length > 0 ? newPath[0] : 'empty', newPath);

              if (trajectory.length > 0) {
                  const lastPoint = trajectory[trajectory.length - 1];
                  if (lastPoint && typeof lastPoint.x === 'number' && typeof lastPoint.y === 'number') {
                      const newPose: RobotPose = {
                          x: lastPoint.x,
                          y: lastPoint.y,
                          theta: typeof lastPoint.theta === 'number' ? lastPoint.theta : 0,
                      };
                      setCurrentPose(newPose);
                      currentPoseRef.current = newPose;
                      // console.log("[TrajectoryWidget] Updated currentPose from trajectory_data:", newPose);
                  }
              }
              if (widgetError) setWidgetError(null);
          } else {
              console.warn("[TrajectoryWidget] Received malformed trajectory_data. Trajectory:", trajectory, "Message:", message);
          }
        } else if (message.robot_alias === selectedRobotId) {
          // Log if message is for the selected robot but not a recognized trajectory type
          // console.log(`[TrajectoryWidget] Received message of type '${message.type}' for ${selectedRobotId}, not a trajectory type.`);
        }
      } catch (error) {
        console.error("Error processing trajectory message:", error);
        setWidgetError("Error processing trajectory data.");
      }
    }
  }, [lastJsonMessage, selectedRobotId, widgetError]);

  // Effect for subscribing and unsubscribing to real-time trajectory data
  useEffect(() => {
    const robotIdToManage = selectedRobotId; // The robot we intend to track
    const currentlyTrackedRobot = subscribedToRobotRef.current; // The robot we are currently subscribed to

    if (robotIdToManage && readyState === ReadyState.OPEN) {
      // We want to track a robot, and WebSocket is open.
      if (currentlyTrackedRobot !== robotIdToManage) {
        // If we are not tracking any robot, or tracking a different one, we need to switch.
        
        // 1. Unsubscribe from the old robot (if any)
        if (currentlyTrackedRobot) {
          sendJsonMessage({
            command: "unsubscribe",
            type: "realtime_trajectory",
            robot_alias: currentlyTrackedRobot,
          });
        }

        // 2. Subscribe to the new robot
        sendJsonMessage({
          command: "subscribe",
          type: "realtime_trajectory", 
          robot_alias: robotIdToManage,
        });
        subscribedToRobotRef.current = robotIdToManage; // Update our tracking reference

        // 3. Reset state for the new robot
        setCurrentPath([]); 
        setCurrentPose(null);
        if (widgetError) setWidgetError(null); // Clear any previous error
      }
      // Else (currentlyTrackedRobot === robotIdToManage): Already tracking the correct robot. Do nothing.
    } else {
      // Conditions for active subscription are not met (no robot selected, or WebSocket is not open).
      // If we were subscribed to any robot, unsubscribe now.
      if (currentlyTrackedRobot) {
        sendJsonMessage({
          command: "unsubscribe",
          type: "realtime_trajectory",
          robot_alias: currentlyTrackedRobot,
        });
        subscribedToRobotRef.current = null; // Mark that we are no longer subscribed
        
        setCurrentPath([]);
        setCurrentPose(null);
      }
    }

    // Cleanup function
    return () => {
      const robotAtCleanup = subscribedToRobotRef.current;
      // Only unsubscribe if actually subscribed and WS was open during effect's main run.
      // Rely on readyState from the effect's closure.
      if (robotAtCleanup && readyState === ReadyState.OPEN) { 
        sendJsonMessage({
          command: "unsubscribe",
          type: "realtime_trajectory",
          robot_alias: robotAtCleanup,
        });
        // It's good practice to nullify the ref if the cleanup logic implies the subscription is truly gone.
        // However, the main logic handles setting it to null when appropriate.
        // For unmount, this ensures the ref state is consistent if it were somehow inspected post-unmount (unlikely).
      }
    };
  }, [selectedRobotId, readyState, sendJsonMessage, widgetError]);

  // Initialize chart once on mount
  useEffect(() => {
    if (!canvasRef.current) return;
    const ctx = canvasRef.current.getContext('2d');
    if (!ctx) return;
    // Plugin to draw a 3-wheel omni robot: body + 3 wheels + heading mark
    const robotIconPlugin: any = {
      id: 'robotIconPlugin',
      afterDatasetsDraw: (chart: any) => {
        const pose = currentPoseRef.current;
        if (!pose) return;
        const xScale = chart.scales['x'];
        const yScale = chart.scales['y'];
        if (!xScale || !yScale) return;
        const cx = xScale.getPixelForValue(pose.x);
        const cy = yScale.getPixelForValue(pose.y);
        const ctx2 = chart.ctx as CanvasRenderingContext2D;
        const bodyR = 10;          // robot body radius in px
        const wheelR = bodyR + 6;  // distance from center to wheel center
        const wheelLen = 12;       // wheel length in px
        const wheelThk = 4;        // wheel thickness in px
        const arrowLen = 14;       // heading arrow length
        const arrowBase = 8;       // heading arrow base width
        ctx2.save();
        ctx2.translate(cx, cy);
        // Convert math/IMU angle (CCW, +y up) to canvas (+y down): rotate by -theta
        ctx2.rotate(-(pose.theta || 0));
        // Body circle
        ctx2.beginPath();
        ctx2.arc(0, 0, bodyR, 0, Math.PI * 2);
        ctx2.fillStyle = 'rgba(54, 162, 235, 0.25)';
        ctx2.fill();
        ctx2.lineWidth = 2;
        ctx2.strokeStyle = 'rgb(54, 162, 235)';
        ctx2.stroke();
        // Draw 3 omni wheels at 120-degree intervals; wheel long axis tangent to body circle
        const wheelAngles = [0, (2*Math.PI)/3, (4*Math.PI)/3]; // radians in robot frame
        for (const ang of wheelAngles) {
          const wx = wheelR * Math.cos(ang);
          const wy = wheelR * Math.sin(ang);
          ctx2.save();
          ctx2.translate(wx, wy);
          // Wheel tangent orientation: rotate by ang + 90deg
          ctx2.rotate(ang + Math.PI/2);
          // Draw rounded rectangle to represent wheel module
          const w = wheelLen;
          const h = wheelThk;
          const rx = -w/2;
          const ry = -h/2;
          const r = Math.min(2, h/2);
          ctx2.beginPath();
          // Rounded rect path
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
        // Heading triangle pointing along +X in local frame (small, just a cue)
        ctx2.beginPath();
        ctx2.moveTo(arrowLen, 0);
        ctx2.lineTo(-arrowLen * 0.35, arrowBase / 2);
        ctx2.lineTo(-arrowLen * 0.35, -arrowBase / 2);
        ctx2.closePath();
        ctx2.fillStyle = 'rgb(54, 162, 235)';
        ctx2.fill();
        ctx2.restore();
      }
    };

    chartRef.current = new ChartJS(ctx, {
      type: 'line',
      data: {
        datasets: [
          {
            label: 'Robot Path',
            data: [],
            borderColor: 'rgb(54, 162, 235)',
            fill: false,
            borderWidth: 2,
            pointRadius: 2,
            pointBackgroundColor: 'rgb(54, 162, 235)',
            tension: 0.1,
          },
        ],
  },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: {
            type: 'linear',
            position: 'bottom',
            title: { display: true, text: 'X (meters)', color: '#CCC' },
            grid: { color: 'rgba(255,255,255,0.1)' },
            ticks: { color: '#AAA' },
          },
          y: {
            type: 'linear',
            title: { display: true, text: 'Y (meters)', color: '#CCC' },
            grid: { color: 'rgba(255,255,255,0.1)' },
            ticks: { color: '#AAA' },
          },
        },
        plugins: {
          legend: { position: 'top' as const, labels: { color: '#CCC' } },
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
      plugins: [robotIconPlugin],
    });
    return () => {
      chartRef.current?.destroy();
      chartRef.current = null;
    };
  }, []);

  // Update chart data when path changes
  useEffect(() => {
    if (!chartRef.current) return;
    chartRef.current.data.datasets[0].data = currentPath as any;
    chartRef.current.update('none');
  }, [currentPath]);

  // Repaint arrow when pose updates
  useEffect(() => {
    if (!chartRef.current) return;
    chartRef.current.update('none');
  }, [currentPose]);

  const resetZoom = () => chartRef.current?.resetZoom();
  const clearPath = () => {
    if (selectedRobotId && readyState === WebSocket.OPEN) {
      // Send a command to the backend to clear its trajectory history
      sendJsonMessage({
        command: "clear_trajectory", // New command type
        robot_alias: selectedRobotId,
      });
      
      // Optimistically clear the frontend state as well.
      // The backend should ideally confirm or send an empty path shortly.
      setCurrentPath([]);
      setCurrentPose(null);
      if (chartRef.current) {
          chartRef.current.data.datasets[0].data = [];
          // chartRef.current.data.datasets[1].data = []; // For current pose if used
          chartRef.current.update('none'); // Update chart with no animation
      }
    } else {
      // Fallback for local clear if WS not ready or no robot selected
      // (though button should ideally be disabled in these cases)
      setCurrentPath([]);
      setCurrentPose(null);
      if (chartRef.current) {
          chartRef.current.data.datasets[0].data = [];
          chartRef.current.update('none');
      }
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
    <div className={`flex flex-col h-full ${compact ? 'p-2' : 'p-3'} bg-gray-800 text-gray-200 rounded-lg shadow-xl`}>
      <WidgetConnectionHeader
        title={`Real-time Trajectory (${selectedRobotId || 'No Robot'})`}
        statusTextOverride={derivedStatusText}
        isConnected={readyState === WebSocket.OPEN && !!selectedRobotId && !firmwareUpdateMode}
        error={widgetError}
      />

      {!compact && (
        <div className="flex gap-2 mb-2 items-center flex-wrap">
          <button
            onClick={resetZoom}
            className="px-3 py-1.5 bg-blue-600 text-white rounded-md flex items-center gap-1 hover:bg-blue-700 disabled:opacity-50 text-xs"
            disabled={!selectedRobotId || firmwareUpdateMode}
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
        </div>
      )}

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