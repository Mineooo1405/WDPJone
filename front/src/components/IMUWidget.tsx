import React, { useState, useEffect, useCallback, useRef } from 'react';
import { useRobotContext } from './RobotContext';
import { ReadyState } from 'react-use-websocket';
import WidgetConnectionHeader from './WidgetConnectionHeader';
import { RotateCcw, Download } from 'lucide-react';
import { Line } from 'react-chartjs-2';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend
} from 'chart.js';
import zoomPlugin from 'chartjs-plugin-zoom';
import { appConfig } from '../config/appConfig';

// Register Chart.js components
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  zoomPlugin
);

// Performance optimization constants
const MAX_HISTORY_POINTS = appConfig.imu.maxHistoryPoints;
const UI_UPDATE_INTERVAL = appConfig.imu.uiUpdateIntervalMs; // Adjusted for consistency, IMU might send data fast

// Standardized IMU data structure expected from WebSocket
interface ImuData {
  roll: number;
  pitch: number;
  yaw: number;
  quat_w: number;
  quat_x: number;
  quat_y: number;
  quat_z: number;
  timestamp: number;
  robot_ip: string; // Added to store robot_ip from message
  robot_alias: string; // Added to store robot_alias from message
  calibrated: boolean;
}

// Replace the current SimpleCompassVisualizer with this simpler YPR visualization
const SimpleYPRVisualizer: React.FC<{ roll: number; pitch: number; yaw: number }> = ({ roll, pitch, yaw }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    
    const width = canvas.width;
    const height = canvas.height;
    const centerX = width / 2;
    const centerY = height / 2 - 20; // Adjusted centerY slightly more
    const scale = Math.min(width, height) * 0.28; // Slightly reduced scale for more padding

    // --- Matrix Math Utilities ---
    const multiplyMatrixAndPoint = (matrix: number[][], point: number[]): number[] => {
      const result = [0, 0, 0];
      for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 3; j++) {
          result[i] += matrix[i][j] * point[j];
        }
      }
      return result;
    };

    const rotationXMatrix = (angle: number): number[][] => [
      [1, 0, 0],
      [0, Math.cos(angle), -Math.sin(angle)],
      [0, Math.sin(angle), Math.cos(angle)],
    ];

    const rotationYMatrix = (angle: number): number[][] => [
      [Math.cos(angle), 0, Math.sin(angle)],
      [0, 1, 0],
      [-Math.sin(angle), 0, Math.cos(angle)],
    ];

    const rotationZMatrix = (angle: number): number[][] => [
      [Math.cos(angle), -Math.sin(angle), 0],
      [Math.sin(angle), Math.cos(angle), 0],
      [0, 0, 1],
    ];
    
    const multiplyMatrices = (m1: number[][], m2: number[][]): number[][] => {
        const result: number[][] = [[0,0,0],[0,0,0],[0,0,0]];
        for (let i = 0; i < 3; i++) {
            for (let j = 0; j < 3; j++) {
                for (let k = 0; k < 3; k++) {
                    result[i][j] += m1[i][k] * m2[k][j];
                }
            }
        }
        return result;
    };

    // --- Cube Definition ---
    const points: number[][] = [
      [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
      [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1],
    ];

    const lines: [number, number][] = [
      [0, 1], [1, 2], [2, 3], [3, 0],
      [4, 5], [5, 6], [6, 7], [7, 4],
      [0, 4], [1, 5], [2, 6], [3, 7],
    ];
    
    // Axes points (origin, X_end, Y_end, Z_end)
    const axisPoints: number[][] = [
        [0,0,0], // Origin
        [1.5,0,0], // X axis end
        [0,1.5,0], // Y axis end
        [0,0,1.5]  // Z axis end
    ];

    // --- Drawing Logic ---
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#f4f4f8'; // Slightly different light grey background
    ctx.fillRect(0, 0, width, height);

    // Initial view rotation to make all axes visible at rest
    const initialViewRotX = rotationXMatrix(Math.PI / 8); // Rotate slightly around X
    const initialViewRotY = rotationYMatrix(-Math.PI / 8); // Rotate slightly around Y
    let initialViewTransform = multiplyMatrices(initialViewRotY, initialViewRotX);

    // IMU rotation matrices
    const Rz_imu = rotationZMatrix(yaw);    // Yaw around Z
    const Ry_imu = rotationYMatrix(pitch);  // Pitch around Y
    const Rx_imu = rotationXMatrix(roll);   // Roll around X

    // Combined IMU rotation: R_imu = Rz * Ry * Rx
    let R_imu = multiplyMatrices(Rz_imu, Ry_imu);
    R_imu = multiplyMatrices(R_imu, Rx_imu);
    
    // Final transform: Apply IMU rotation then initial view transform
    // This means the object rotates in its own frame, then the whole scene is viewed from a slight angle.
    // Alternative: R_final = multiplyMatrices(initialViewTransform, R_imu); // View transformation first
    let R_final = multiplyMatrices(initialViewTransform, R_imu); 

    const projectedPoints: { x: number; y: number; z: number }[] = [];
      
    // Rotate and project cube points
    points.forEach(point => {
      const rotated = multiplyMatrixAndPoint(R_final, point);
      projectedPoints.push({
        x: rotated[0] * scale + centerX,
        y: rotated[1] * scale + centerY,
        z: rotated[2] * scale, // For potential depth sorting or effects
      });
    });
    
    // Rotate and project axis points
    const projectedAxisPoints: { x: number; y: number; z: number }[] = [];
    axisPoints.forEach(point => {
        const bodyRotated = multiplyMatrixAndPoint(R_imu, point);
        const finalRotated = multiplyMatrixAndPoint(initialViewTransform, bodyRotated);
        projectedAxisPoints.push({
            x: finalRotated[0] * scale + centerX,
            y: finalRotated[1] * scale + centerY,
            z: finalRotated[2] * scale,
        });
    });

    // Draw cube lines (thicker, slightly darker blue)
    ctx.strokeStyle = '#2980b9'; 
    ctx.lineWidth = 2;
    lines.forEach(line => {
      const p1 = projectedPoints[line[0]];
      const p2 = projectedPoints[line[1]];
    ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
    ctx.stroke();
    });
    
    // Draw Axes
    const axisColors = ['#c0392b', '#27ae60', '#2980b9']; // Darker Red, Green, Blue
    const axisLabels = ['R', 'P', 'Y']; // For Roll (X), Pitch (Y), Yaw (Z) axes of the body
    const originScreen = projectedAxisPoints[0];

    for (let i = 0; i < 3; i++) {
        const axisEndScreen = projectedAxisPoints[i+1];
      ctx.beginPath();
        ctx.moveTo(originScreen.x, originScreen.y);
        ctx.lineTo(axisEndScreen.x, axisEndScreen.y);
        ctx.strokeStyle = axisColors[i];
        ctx.lineWidth = 3; // Thicker axes
      ctx.stroke();
      
        ctx.fillStyle = axisColors[i];
        ctx.font = 'bold 13px Arial';
        const labelOffsetX = (axisEndScreen.x - originScreen.x) * 0.15; // Push labels out a bit more
        const labelOffsetY = (axisEndScreen.y - originScreen.y) * 0.15;
        // Draw label near the tip of the axis arrow
        ctx.fillText(axisLabels[i], axisEndScreen.x + labelOffsetX - (axisLabels[i].length > 1 ? 5 : 0), axisEndScreen.y + labelOffsetY + 5);
    }
    
    ctx.beginPath();
    ctx.arc(originScreen.x, originScreen.y, 4, 0, Math.PI * 2); // Slightly larger origin dot
    ctx.fillStyle = '#2c3e50'; // Dark grey for origin
    ctx.fill();

  }, [roll, pitch, yaw]); // Removed width, height as they are read once initially
  
  return (
    <canvas
      ref={canvasRef}
      width={280} // Adjusted for potentially smaller display area
      height={280} // Adjusted
      className="w-full h-full bg-gray-100 rounded-md shadow-inner border border-gray-300"
    />
  );
};

const IMUWidget: React.FC = () => {
  const { selectedRobotId, sendJsonMessage, lastJsonMessage, readyState, getIncomingForRobot } = useRobotContext() as any;

  const [currentImuData, setCurrentImuData] = useState<ImuData | null>(null);
  const [widgetError, setWidgetError] = useState<string | null>(null);
  const [activeChart, setActiveChart] = useState<'orientation' | 'quaternion'>('orientation');
  
  const messageBuffer = useRef<ImuData[]>([]);
  const lastUIUpdateTime = useRef(0);
  const animationFrameId = useRef<number | null>(null);
  
  const [history, setHistory] = useState({
    timestamps: [] as string[],
    orientation: { roll: [] as number[], pitch: [] as number[], yaw: [] as number[] },
    quaternion: { w: [] as number[], x: [] as number[], y: [] as number[], z: [] as number[] }
  });

  const chartRef = useRef<any>(null);
  // Removed pause/freeze; chart updates continuously

  const subscribedToRobotRef = useRef<string | null>(null);

  const formatTimestampForChart = (timestamp: number): string => {
    const date = new Date(timestamp > 2000000000 ? timestamp : timestamp * 1000); // Handle s or ms
    return date.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit', fractionalSecondDigits: 2 });
  };

  const processMessageBuffer = useCallback(() => {
    if (animationFrameId.current !== null) {
      cancelAnimationFrame(animationFrameId.current);
      animationFrameId.current = null;
    }
    
    if (messageBuffer.current.length === 0) return;
    
    const newMessages = [...messageBuffer.current];
    messageBuffer.current = [];

    if (newMessages.length > 0) {
      const latestMessage = newMessages[newMessages.length - 1];
      setCurrentImuData(latestMessage); // Update current display data

      setHistory(prev => {
          const newTimestamps = newMessages.map(msg => formatTimestampForChart(msg.timestamp));
          const newRolls = newMessages.map(msg => msg.roll);
          const newPitches = newMessages.map(msg => msg.pitch);
          const newYaws = newMessages.map(msg => msg.yaw);
          const newQuatW = newMessages.map(msg => msg.quat_w);
          const newQuatX = newMessages.map(msg => msg.quat_x);
          const newQuatY = newMessages.map(msg => msg.quat_y);
          const newQuatZ = newMessages.map(msg => msg.quat_z);
          
          return {
            timestamps: [...prev.timestamps, ...newTimestamps].slice(-MAX_HISTORY_POINTS),
            orientation: {
              roll: [...prev.orientation.roll, ...newRolls].slice(-MAX_HISTORY_POINTS),
              pitch: [...prev.orientation.pitch, ...newPitches].slice(-MAX_HISTORY_POINTS),
              yaw: [...prev.orientation.yaw, ...newYaws].slice(-MAX_HISTORY_POINTS)
            },
            quaternion: {
              w: [...prev.quaternion.w, ...newQuatW].slice(-MAX_HISTORY_POINTS),
              x: [...prev.quaternion.x, ...newQuatX].slice(-MAX_HISTORY_POINTS),
              y: [...prev.quaternion.y, ...newQuatY].slice(-MAX_HISTORY_POINTS),
              z: [...prev.quaternion.z, ...newQuatZ].slice(-MAX_HISTORY_POINTS)
            }
          };
        });
    }
  }, []);

  const scheduleUIUpdate = useCallback(() => {
    if (animationFrameId.current !== null) return;
    const now = Date.now();
    if (messageBuffer.current.length > 0 && now - lastUIUpdateTime.current >= UI_UPDATE_INTERVAL) {
      lastUIUpdateTime.current = now;
      animationFrameId.current = requestAnimationFrame(() => {
        processMessageBuffer();
        animationFrameId.current = null;
      });
    }
  }, [processMessageBuffer]);

  // This effect handles WebSocket message reception
  useEffect(() => {
    if (lastJsonMessage && selectedRobotId) {
      const message = lastJsonMessage as any; // Keep as any for initial type checking

      // Ensure message is for the selected robot
      if (message.robot_alias !== selectedRobotId) {
        return;
      }

  if (message.type === 'imu_data' && message.data) { // Check for message.data
        const imuPayload = message.data; // This is the object with euler, quaternion, etc.

        // Create a new ImuData object by extracting values from imuPayload
        const eulerArr = Array.isArray(imuPayload.euler) ? imuPayload.euler : [];
        // Prefer [yaw, pitch, roll] ordering (sim + backend yaw index default), fallback gracefully
        const yawVal = typeof eulerArr[0] === 'number' ? eulerArr[0] : (typeof eulerArr[2] === 'number' ? eulerArr[2] : 0);
        const pitchVal = typeof eulerArr[1] === 'number' ? eulerArr[1] : 0;
        const rollVal = typeof eulerArr[2] === 'number' ? eulerArr[2] : (typeof eulerArr[0] === 'number' ? eulerArr[0] : 0);
        const quatArr = Array.isArray(imuPayload.quaternion) ? imuPayload.quaternion : [];
        const newImuEntry: ImuData = {
          roll: rollVal,
          pitch: pitchVal,
          yaw: yawVal,
          quat_w: typeof quatArr[0] === 'number' ? quatArr[0] : 1.0,
          quat_x: typeof quatArr[1] === 'number' ? quatArr[1] : 0.0,
          quat_y: typeof quatArr[2] === 'number' ? quatArr[2] : 0.0,
          quat_z: typeof quatArr[3] === 'number' ? quatArr[3] : 0.0,
          timestamp: message.timestamp || Date.now() / 1000, // Timestamp from the outer message
          robot_ip: message.robot_ip, 
          robot_alias: message.robot_alias,
          calibrated: currentImuData?.calibrated ?? false 
        };

  // Always buffer and schedule UI updates
  messageBuffer.current.push(newImuEntry);
  scheduleUIUpdate();
        // Update currentImuData to show the latest values even if paused, 
        // so when unpaused, it doesn't jump from very old data.
        setCurrentImuData(newImuEntry);
        setWidgetError(null); // Clear previous errors on new data

      } else if (message.type === 'bno_event' && message.robot_alias === selectedRobotId) {
        // Handle bno_event for calibration status if it's separate
        if (message.event_type === 'calibration_complete' && message.status) {
            const calStatus = message.status as any; 
            const sysCal = calStatus.sys ?? 0;
            const gyroCal = calStatus.gyro ?? 0;
            const accelCal = calStatus.accel ?? 0;
            const magCal = calStatus.mag ?? 0;
            const isCalibrated = sysCal >= 2 && gyroCal >= 2 && accelCal >= 1 && magCal >= 1;
            setCurrentImuData(prev => ({ ...(prev || {} as ImuData), calibrated: isCalibrated }));
        }
      } else if (message.type === 'error' && message.robot_alias === selectedRobotId) {
        setWidgetError(message.message || 'Unknown error from IMU data stream');
      }
    }
  }, [lastJsonMessage, selectedRobotId, currentImuData, scheduleUIUpdate]);

  // Prefill from buffered IMU messages when selecting a robot
  useEffect(() => {
    try {
      if (selectedRobotId && getIncomingForRobot) {
        const stored = getIncomingForRobot(selectedRobotId);
        if (stored && Array.isArray(stored.imu) && stored.imu.length > 0) {
          const recent = stored.imu.slice(-Math.min(100, stored.imu.length));
          recent.forEach((m: any) => {
            const imuPayload = m.data || {};
            const eulerArr = Array.isArray(imuPayload.euler) ? imuPayload.euler : [];
            const yawVal = typeof eulerArr[0] === 'number' ? eulerArr[0] : (typeof eulerArr[2] === 'number' ? eulerArr[2] : 0);
            const pitchVal = typeof eulerArr[1] === 'number' ? eulerArr[1] : 0;
            const rollVal = typeof eulerArr[2] === 'number' ? eulerArr[2] : (typeof eulerArr[0] === 'number' ? eulerArr[0] : 0);
            const quatArr = Array.isArray(imuPayload.quaternion) ? imuPayload.quaternion : [];
            messageBuffer.current.push({
              roll: rollVal,
              pitch: pitchVal,
              yaw: yawVal,
              quat_w: typeof quatArr[0] === 'number' ? quatArr[0] : 1.0,
              quat_x: typeof quatArr[1] === 'number' ? quatArr[1] : 0.0,
              quat_y: typeof quatArr[2] === 'number' ? quatArr[2] : 0.0,
              quat_z: typeof quatArr[3] === 'number' ? quatArr[3] : 0.0,
              timestamp: m.timestamp || Date.now() / 1000,
              robot_ip: m.robot_ip || selectedRobotId,
              robot_alias: selectedRobotId,
              calibrated: false,
            } as ImuData);
          });
          scheduleUIUpdate();
        }
      }
    } catch {}
  }, [selectedRobotId, getIncomingForRobot, scheduleUIUpdate]);

  // Effect for subscribing and unsubscribing to IMU data (always live when connected)
  useEffect(() => {
    if (!selectedRobotId || readyState !== ReadyState.OPEN) {
      if (subscribedToRobotRef.current) {
        sendJsonMessage({ command: "unsubscribe", type: "imu_data", robot_alias: subscribedToRobotRef.current });
        subscribedToRobotRef.current = null;
        clearHistoryAndData();
      }
      return;
    }

    if (subscribedToRobotRef.current && subscribedToRobotRef.current !== selectedRobotId) {
      // Robot changed: unsubscribe previous
      sendJsonMessage({ command: "unsubscribe", type: "imu_data", robot_alias: subscribedToRobotRef.current });
      clearHistoryAndData();
      subscribedToRobotRef.current = null;
    }
    if (subscribedToRobotRef.current !== selectedRobotId) {
      sendJsonMessage({ command: "subscribe", type: "imu_data", robot_alias: selectedRobotId });
      subscribedToRobotRef.current = selectedRobotId;
      clearHistoryAndData();
    }

    return () => {
      if (subscribedToRobotRef.current && readyState === ReadyState.OPEN) {
        sendJsonMessage({ command: "unsubscribe", type: "imu_data", robot_alias: subscribedToRobotRef.current });
        subscribedToRobotRef.current = null;
      }
    };
  }, [selectedRobotId, readyState, sendJsonMessage]);

  // Removed toggleLiveUpdate; always live

  useEffect(() => {
    const intervalId = setInterval(scheduleUIUpdate, UI_UPDATE_INTERVAL);
    return () => clearInterval(intervalId);
  }, [scheduleUIUpdate]);

  useEffect(() => {
    return () => {
      if (animationFrameId.current) cancelAnimationFrame(animationFrameId.current);
    };
  }, []);

  const downloadData = () => {
    if (history.timestamps.length === 0) {
        setWidgetError("Không có dữ liệu lịch sử IMU để tải xuống.");
        return;
    }
    setWidgetError(null);
    let csvHeader = 'Timestamp,Roll,Pitch,Yaw,QuatW,QuatX,QuatY,QuatZ\n';
    const csvRows = history.timestamps.map((ts, idx) => 
        `${ts},${history.orientation.roll[idx]},${history.orientation.pitch[idx]},${history.orientation.yaw[idx]},${history.quaternion.w[idx]},${history.quaternion.x[idx]},${history.quaternion.y[idx]},${history.quaternion.z[idx]}`
    ).join("\n");
    const csvContent = csvHeader + csvRows;
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `imu_data_${selectedRobotId || 'unknown'}_${new Date().toISOString().slice(0,19).replace(/:/g,'-')}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  const clearHistoryAndData = () => {
    setHistory({
      timestamps: [],
      orientation: { roll: [], pitch: [], yaw: [] },
      quaternion: { w: [], x: [], y: [], z: [] }
    });
    setCurrentImuData(null);
    messageBuffer.current = [];
    if (chartRef.current) chartRef.current.resetZoom();
  };

  const resetZoom = () => chartRef.current?.resetZoom();
  // Removed freeze chart toggle

  // Chart configurations (similar to EncoderDataWidget, adapted for IMU)
  const chartDataConfig = {
    labels: history.timestamps,
    datasets: activeChart === 'orientation' ? [
      { label: 'Roll (rad)', data: history.orientation.roll, borderColor: 'rgb(255, 99, 132)', backgroundColor: 'rgba(255, 99, 132, 0.5)', tension: 0.1, pointRadius: 1 },
      { label: 'Pitch (rad)', data: history.orientation.pitch, borderColor: 'rgb(54, 162, 235)', backgroundColor: 'rgba(54, 162, 235, 0.5)', tension: 0.1, pointRadius: 1 },
      { label: 'Yaw (rad)', data: history.orientation.yaw, borderColor: 'rgb(75, 192, 192)', backgroundColor: 'rgba(75, 192, 192, 0.5)', tension: 0.1, pointRadius: 1 }
    ] : [
      { label: 'Quat W', data: history.quaternion.w, borderColor: 'rgb(255, 99, 132)', backgroundColor: 'rgba(255, 99, 132, 0.5)', tension: 0.1, pointRadius: 1 },
      { label: 'Quat X', data: history.quaternion.x, borderColor: 'rgb(54, 162, 235)', backgroundColor: 'rgba(54, 162, 235, 0.5)', tension: 0.1, pointRadius: 1 },
      { label: 'Quat Y', data: history.quaternion.y, borderColor: 'rgb(75, 192, 192)', backgroundColor: 'rgba(75, 192, 192, 0.5)', tension: 0.1, pointRadius: 1 },
      { label: 'Quat Z', data: history.quaternion.z, borderColor: 'rgb(153, 102, 255)', backgroundColor: 'rgba(153, 102, 255, 0.5)', tension: 0.1, pointRadius: 1 }
    ]
  };

  const chartOptionsConfig: any = {
    responsive: true,
    maintainAspectRatio: false,
    animation: false as const,
    scales: { x: { ticks: { maxTicksLimit: 8, color: '#AAA'}, grid: {color: 'rgba(255,255,255,0.1)'} }, y: { beginAtZero: false, ticks:{color: '#AAA'}, grid: {color: 'rgba(255,255,255,0.1)'} } },
    plugins: {
      legend: { position: 'top' as const, labels:{color: '#CCC'} },
      title: { display: false },
      zoom: { pan: { enabled: true, mode: 'x' as const }, zoom: { wheel: { enabled: true }, pinch: { enabled: true }, mode: 'x' as const } }
    },
  };

  const formatAngleDeg = (rad: number = 0) => {
    let deg = rad * 180 / Math.PI;
    deg = ((deg + 180) % 360 + 360) % 360 - 180; // normalize to [-180, 180]
    return `${deg.toFixed(1)}°`;
  };

  // Determine connection status text for WidgetConnectionHeader
  let derivedStatusText: string;
  if (readyState === ReadyState.CONNECTING) {
    derivedStatusText = "WS: Connecting...";
  } else if (readyState !== ReadyState.OPEN) {
    derivedStatusText = "WS: Disconnected";
  } else if (!selectedRobotId) {
    derivedStatusText = "No robot selected";
  } else {
    derivedStatusText = "Connected";
  }

  return (
    <div className="flex flex-col h-full p-4 bg-gray-800 text-gray-200 rounded-lg shadow-xl">
      <WidgetConnectionHeader
        title={`IMU Data (${selectedRobotId || 'Chưa chọn Robot'})`}
        statusTextOverride={derivedStatusText}
        isConnected={readyState === ReadyState.OPEN && !!selectedRobotId}
        error={widgetError}
        // showConnectButton and connectButtonText props removed temporarily
      />

  <div className="flex gap-2 mb-4 items-center flex-wrap">
        <button
          onClick={clearHistoryAndData}
          className="px-3 py-1.5 bg-gray-600 text-white rounded-md flex items-center gap-1 hover:bg-gray-700 disabled:opacity-50"
        >
          <RotateCcw size={14} />
          <span>Clear</span>
        </button>
        <button
          onClick={downloadData}
          disabled={history.timestamps.length === 0}
          className="px-3 py-1.5 bg-indigo-600 text-white rounded-md flex items-center gap-1 hover:bg-indigo-700 disabled:opacity-50"
        >
          <Download size={14}/>
          <span>Download CSV</span>
        </button>
      </div>
      
      {widgetError && (
        <div className="bg-red-100 border border-red-400 text-red-700 px-4 py-3 rounded relative mb-4" role="alert">
          <strong className="font-bold">Lỗi: </strong>
          <span className="block sm:inline">{widgetError}</span>
        </div>
      )}

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4 mb-4">
        <div className="bg-gray-700 p-3 rounded-lg shadow-md">
          <h3 className="font-semibold text-gray-100 mb-2 text-center">3D Orientation</h3>
          <div className="aspect-square w-full max-w-xs mx-auto bg-gray-100 rounded">
            <SimpleYPRVisualizer 
              roll={currentImuData?.roll || 0}
              pitch={currentImuData?.pitch || 0}
              yaw={currentImuData?.yaw || 0}
            />
          </div>
          <div className="grid grid-cols-3 gap-2 text-center mt-2 text-xs">
            <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">Roll</div><div className="font-semibold">{formatAngleDeg(currentImuData?.roll)}</div></div>
            <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">Pitch</div><div className="font-semibold">{formatAngleDeg(currentImuData?.pitch)}</div></div>
            <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">Yaw</div><div className="font-semibold">{formatAngleDeg(currentImuData?.yaw)}</div></div>
          </div>
        </div>

        <div className="bg-gray-700 p-3 rounded-lg shadow-md">
          <h3 className="font-semibold text-gray-100 mb-2 text-center">Current Values</h3>
           <div className="space-y-2 text-sm">
            <div>
              <h4 className="font-medium text-gray-300 mb-1">Orientation (Euler)</h4>
              <div className="grid grid-cols-3 gap-2 text-center mb-2">
                <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">Roll</div><div>{formatAngleDeg(currentImuData?.roll)}</div></div>
                <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">Pitch</div><div>{formatAngleDeg(currentImuData?.pitch)}</div></div>
                <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">Yaw</div><div>{formatAngleDeg(currentImuData?.yaw)}</div></div>
                </div>
              <h4 className="font-medium text-gray-300 mb-1">Quaternion</h4>
              <div className="grid grid-cols-2 sm:grid-cols-4 gap-2 text-center">
                <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">W</div><div>{currentImuData?.quat_w?.toFixed(3) || '1.000'}</div></div>
                <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">X</div><div>{currentImuData?.quat_x?.toFixed(3) || '0.000'}</div></div>
                <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">Y</div><div>{currentImuData?.quat_y?.toFixed(3) || '0.000'}</div></div>
                <div className="p-1.5 bg-gray-600 rounded"><div className="opacity-70">Z</div><div>{currentImuData?.quat_z?.toFixed(3) || '0.000'}</div></div>
                </div>
                </div>
            <div className="text-xs text-gray-400 text-center pt-1">
              Timestamp: {currentImuData?.timestamp ? formatTimestampForChart(currentImuData.timestamp) : 'N/A'}
            </div>
          </div>
        </div>
      </div>

      <div className="flex-grow bg-gray-700 p-3 rounded-lg shadow-md">
        <div className="flex justify-between items-center mb-2">
          <h3 className="font-semibold text-gray-100">IMU Data History</h3>
          <div className="flex items-center gap-2">
            <div className="inline-flex bg-gray-600 rounded-lg p-0.5">
              <button className={`px-2.5 py-1 rounded text-xs ${activeChart === 'orientation' ? 'bg-blue-500 text-white shadow' : 'text-gray-300 hover:bg-gray-500'}`} onClick={() => setActiveChart('orientation')}>Orientation</button>
              <button className={`px-2.5 py-1 rounded text-xs ${activeChart === 'quaternion' ? 'bg-blue-500 text-white shadow' : 'text-gray-300 hover:bg-gray-500'}`} onClick={() => setActiveChart('quaternion')}>Quaternion</button>
            </div>
            <button onClick={resetZoom} className="p-1.5 bg-gray-600 hover:bg-gray-500 rounded text-gray-300" title="Reset Zoom"><RotateCcw size={14} /></button>
            </div>
          </div>
        <div className="relative h-64 md:h-72"> {/* Fixed height for chart area */}
      {history.timestamps.length > 0 ? (
            <Line data={chartDataConfig} options={chartOptionsConfig} ref={chartRef} />
          ) : (
            <div className="h-full flex items-center justify-center text-gray-500 border border-dashed border-gray-600 rounded-md">
        {readyState !== ReadyState.OPEN ? "WebSocket chưa kết nối." : !selectedRobotId ? "Vui lòng chọn một robot." : "Đang chờ dữ liệu IMU..."}
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

export default IMUWidget;