import React, { useState, useEffect, useRef, useCallback, useContext } from 'react';
import { 
  RefreshCw, Download, Calendar, ChevronDown, List, MapPin, Clock, 
  ZoomIn, ZoomOut, Move, Play, Pause, RotateCcw, Filter, Upload,
  XCircle, AlertCircle
} from 'lucide-react';
import { Line } from 'react-chartjs-2';
import { 
  Chart, LinearScale, CategoryScale, PointElement, 
  LineElement, Title, Tooltip, Legend,
  ChartOptions
} from 'chart.js';
import type { DeepPartial } from 'utility-types';
import { ChartData, TooltipItem, Point } from 'chart.js';
import zoomPlugin from 'chartjs-plugin-zoom';
import { GlobalAppContext } from '../contexts/GlobalAppContext';
import { useRobotContext, ReadyState } from './RobotContext';

// Register zoom plugin
Chart.register(
  LinearScale,
  CategoryScale,
  PointElement,
  LineElement, 
  Title,
  Tooltip,
  Legend,
  zoomPlugin
);

interface TrajectoryRecord {
  id: number;
  timestamp: string;
  currentPosition: {
    x: number;
    y: number;
    theta: number;
  };
  points: {
    x: number[];
    y: number[];
    theta: number[];
    timestamps?: number[]; // Thêm mốc thời gian cho mỗi điểm
  };
  status: string;
  metadata?: {
    speed?: number;
    duration?: number;
    distance?: number;
    command?: string;
  };
}

interface TrajectoryFilterOptions {
  time: string;
  minDistance?: number;
  maxDistance?: number;
  searchText?: string;
}

// Thêm interface cho timeFilterOption
interface TimeFilterOption {
  value: string;
  label: string;
}

// Định nghĩa kiểu cho WebSocket payloads mà widget này xử lý
interface BaseTrajectoryPayload {
  type: string;
  robot_id?: string; 
  message?: string; // Cho error payload
}

interface TrajectoryHistoryPayload extends BaseTrajectoryPayload {
  type: 'trajectory_history';
  trajectories: TrajectoryRecord[];
  robot_id: string;
}

interface PositionUpdatePayload extends BaseTrajectoryPayload {
  type: 'position_update';
  position: { x: number; y: number; theta: number };
  robot_id: string;
}

interface TrajectoryUpdateNotificationPayload extends BaseTrajectoryPayload {
  type: 'trajectory_update';
  robot_id: string;
}

interface ErrorPayload extends BaseTrajectoryPayload {
  type: 'error';
  message: string;
  robot_id?: string;
}

type TrajectoryWebSocketPayload = 
  | TrajectoryHistoryPayload 
  | PositionUpdatePayload 
  | TrajectoryUpdateNotificationPayload
  | ErrorPayload;

const TrajectoryWidget: React.FC = () => {
  const {
    selectedRobotId,
    sendJsonMessage,
    lastJsonMessage,
    readyState,
  } = useRobotContext();
  const { firmwareUpdateMode } = useContext(GlobalAppContext);

  // Refs
  const chartRef = useRef<Chart<'line', (number | Point | null)[], unknown> | null>(null);
  const animationRef = useRef<number | null>(null);
  const subscribedToRealtimeRef = useRef<string | null>(null);
  
  // State for trajectory data
  const [trajectoryHistory, setTrajectoryHistory] = useState<TrajectoryRecord[]>([]);
  const [selectedTrajectory, setSelectedTrajectory] = useState<TrajectoryRecord | null>(null);
  const [comparisonTrajectory, setComparisonTrajectory] = useState<TrajectoryRecord | null>(null);
  const [isHistoryOpen, setIsHistoryOpen] = useState(false);
  const [isFilterOpen, setIsFilterOpen] = useState(false);
  const [filterOptions, setFilterOptions] = useState<TrajectoryFilterOptions>({
    time: '24h',
    minDistance: undefined,
    maxDistance: undefined,
    searchText: ''
  });
  const [loading, setLoading] = useState(false);
  const [widgetError, setWidgetError] = useState<string | null>(null);
  const [liveUpdateHistory, setLiveUpdateHistory] = useState(false);
  const [liveUpdatePosition, setLiveUpdatePosition] = useState(true);
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackProgress, setPlaybackProgress] = useState(0);
  const [realTimeTrajectory, setRealTimeTrajectory] = useState<{
    x: number[],
    y: number[],
    theta: number[]
  }>({
    x: [],
    y: [],
    theta: []
  });
  const [showRealTime, setShowRealTime] = useState(true);
  const [isPaused, setIsPaused] = useState(false);
  const [viewMode, setViewMode] = useState<'2d' | '3d'>('2d');
  const [showControls, setShowControls] = useState(false);
  
  const [isSmallScreen, setIsSmallScreen] = useState(window.innerWidth < 768);

  useEffect(() => {
    const handleResize = () => setIsSmallScreen(window.innerWidth < 768);
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, []);

  const timeFilterOptions: TimeFilterOption[] = [
    { value: '1h', label: '1 giờ qua' },
    { value: '6h', label: '6 giờ qua' },
    { value: '24h', label: '24 giờ qua' },
    { value: '7d', label: '7 ngày qua' },
    { value: '30d', label: '30 ngày qua' },
    { value: 'all', label: 'Tất cả' }
  ];

  // Request trajectory history - Sửa lại useCallback
  const requestTrajectoryHistory = useCallback(() => {
    if (readyState !== ReadyState.OPEN || !selectedRobotId) {
      if (readyState !== ReadyState.OPEN) console.warn("TrajectoryWidget: WS not open for history request.");
      if (!selectedRobotId) console.warn("TrajectoryWidget: No robot selected for history request.");
      return; 
    }
    
    setLoading(true);
    setWidgetError(null); 
    sendJsonMessage({
      command: 'request_trajectory',
      robot_alias: selectedRobotId,
      time_filter: filterOptions.time,
    });
  }, [readyState, selectedRobotId, filterOptions.time, sendJsonMessage]);

  // Request trajectory history on selectedRobotId change or filter change
  useEffect(() => {
    if (selectedRobotId && readyState === ReadyState.OPEN) {
      requestTrajectoryHistory();
    }
  }, [selectedRobotId, filterOptions.time, readyState, requestTrajectoryHistory]);

  const handleTrajectoryHistoryResponse = useCallback((data: TrajectoryHistoryPayload) => {
    if (data.robot_id && selectedRobotId && data.robot_id !== selectedRobotId) return;
    
      if (data.trajectories && Array.isArray(data.trajectories)) {
        let filteredTrajectories = data.trajectories;
        
        if (filterOptions.searchText) {
          const searchLower = filterOptions.searchText.toLowerCase();
          filteredTrajectories = filteredTrajectories.filter((traj: TrajectoryRecord) => 
            traj.status.toLowerCase().includes(searchLower) || 
            (traj.metadata?.command && traj.metadata.command.toLowerCase().includes(searchLower))
          );
        }
        
        if (filterOptions.minDistance !== undefined) {
          filteredTrajectories = filteredTrajectories.filter((traj: TrajectoryRecord) => {
            const distance = calculateDistance(traj.points.x, traj.points.y);
            return distance >= (filterOptions.minDistance || 0);
          });
        }
        
        if (filterOptions.maxDistance !== undefined) {
          filteredTrajectories = filteredTrajectories.filter((traj: TrajectoryRecord) => {
            const distance = calculateDistance(traj.points.x, traj.points.y);
            return distance <= (filterOptions.maxDistance || Infinity);
          });
        }
        
        setTrajectoryHistory(filteredTrajectories);
        
        if (!selectedTrajectory && filteredTrajectories.length > 0) {
          setSelectedTrajectory(filteredTrajectories[0]);
        }
      }
      setLoading(false);
      setWidgetError(null);
  }, [selectedRobotId, filterOptions, selectedTrajectory]);

  const handlePositionOrStatusUpdate = useCallback((data: PositionUpdatePayload) => {
    if (data.robot_id && selectedRobotId && data.robot_id !== selectedRobotId) return;

    const positionData = data.position;
    if (positionData && liveUpdatePosition && showRealTime && !isPaused) {
          setRealTimeTrajectory(prev => {
            const maxPoints = 500;
            const newX = [...prev.x, positionData.x].slice(-maxPoints);
            const newY = [...prev.y, positionData.y].slice(-maxPoints);
            const newTheta = [...prev.theta, positionData.theta].slice(-maxPoints);
            return {
              x: newX,
              y: newY,
              theta: newTheta
            };
          });
          setWidgetError(null);
      }
  }, [selectedRobotId, liveUpdatePosition, showRealTime, isPaused]);

  const handleTrajectoryUpdateNotification = useCallback((data: TrajectoryUpdateNotificationPayload) => {
    if (data.robot_id && selectedRobotId && data.robot_id !== selectedRobotId) return;
      if (liveUpdateHistory) {
        requestTrajectoryHistory();
      }
  }, [selectedRobotId, liveUpdateHistory, requestTrajectoryHistory]);

  const handleErrorResponse = useCallback((data: ErrorPayload) => {
    if (data.robot_id && selectedRobotId && data.robot_id !== selectedRobotId && data.type === "error") {
        return;
    }
    setWidgetError(data.message || 'Unknown error occurred from WebSocket');
    setLoading(false);
  }, [selectedRobotId]);
  
  // useEffect for Real-time Position Subscription
  useEffect(() => {
    if (!selectedRobotId || readyState !== ReadyState.OPEN) {
      if (subscribedToRealtimeRef.current) {
        sendJsonMessage({
          command: "direct_unsubscribe",
          type: "position_update",
          robot_alias: subscribedToRealtimeRef.current
        });
        subscribedToRealtimeRef.current = null;
        setRealTimeTrajectory({ x: [], y: [], theta: [] });
      }
      return;
    }

    if (liveUpdatePosition) {
      if (subscribedToRealtimeRef.current && subscribedToRealtimeRef.current !== selectedRobotId) {
        sendJsonMessage({
          command: "direct_unsubscribe",
          type: "position_update",
          robot_alias: subscribedToRealtimeRef.current
        });
        subscribedToRealtimeRef.current = null;
      }
      if (subscribedToRealtimeRef.current !== selectedRobotId) {
        console.log(`TrajectoryWidget: Subscribing to position_update for ${selectedRobotId}`);
        sendJsonMessage({
          command: "direct_subscribe",
          type: "position_update",
          robot_alias: selectedRobotId
        });
        subscribedToRealtimeRef.current = selectedRobotId;
        setRealTimeTrajectory({ x: [], y: [], theta: [] });
      }
    } else {
      if (subscribedToRealtimeRef.current === selectedRobotId) {
        console.log(`TrajectoryWidget: Unsubscribing from position_update for ${selectedRobotId}`);
        sendJsonMessage({
          command: "direct_unsubscribe",
          type: "position_update",
          robot_alias: selectedRobotId
        });
        subscribedToRealtimeRef.current = null;
      }
    }

    return () => {
      if (subscribedToRealtimeRef.current && readyState === ReadyState.OPEN) {
         console.log(`TrajectoryWidget: Cleanup - Unsubscribing from position_update for ${subscribedToRealtimeRef.current}`);
        sendJsonMessage({
          command: "direct_unsubscribe",
          type: "position_update",
          robot_alias: subscribedToRealtimeRef.current
        });
        subscribedToRealtimeRef.current = null;
      }
    };
  }, [selectedRobotId, liveUpdatePosition, readyState, sendJsonMessage]);

  // useEffect for Handling Incoming Messages from RobotContext
  useEffect(() => {
    if (!lastJsonMessage || !selectedRobotId) return;

    const message = lastJsonMessage as any;

    if (message.robot_id !== selectedRobotId) return;

    switch (message.type) {
      case 'position_update':
        handlePositionOrStatusUpdate(message as PositionUpdatePayload);
        break;
      case 'trajectory_history':
        handleTrajectoryHistoryResponse(message as TrajectoryHistoryPayload);
        break;
      case 'error':
        handleErrorResponse(message as ErrorPayload);
        break;
      default:
        break;
    }
  }, [lastJsonMessage, selectedRobotId, handlePositionOrStatusUpdate, handleTrajectoryHistoryResponse, handleErrorResponse]);

  const downloadTrajectory = () => {
    if (!selectedTrajectory) return;
    let csvContent = "X,Y,Theta,Timestamp\n";
    const { points } = selectedTrajectory;
    for (let i = 0; i < points.x.length; i++) {
      const timestamp = points.timestamps ? points.timestamps[i] : '';
      csvContent += `${points.x[i]},${points.y[i]},${points.theta[i]},${timestamp}\n`;
    }
    const blob = new Blob([csvContent], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    const timestampStr = new Date(selectedTrajectory.timestamp).toISOString().replace(/[:.]/g, '-');
    a.download = `trajectory_${selectedRobotId || 'unknown'}_${timestampStr}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  const formatDate = (dateStr: string) => {
    const date = new Date(dateStr);
    return date.toLocaleString();
  };

  const calculateDistance = (x: number[], y: number[]): number => {
    let distance = 0;
    for (let i = 1; i < x.length; i++) {
      const dx = x[i] - x[i-1];
      const dy = y[i] - y[i-1];
      distance += Math.sqrt(dx*dx + dy*dy);
    }
    return distance;
  };
  const MAX_REALTIME_POINTS = 500;

  const trajectoryChartData: ChartData<'line', (number | Point | null)[]> = {
    datasets: [
      ...(selectedTrajectory ? [
        {
          label: 'Quỹ đạo đã chọn',
          data: selectedTrajectory.points.x.map((xVal, i) => ({ x: xVal, y: selectedTrajectory.points.y[i] })),
          borderColor: 'rgba(54, 162, 235, 1)',
          backgroundColor: 'rgba(54, 162, 235, 0.2)',
          pointRadius: 1,
          showLine: true,
          borderWidth: 2,
        },
        {
          label: 'Điểm bắt đầu',
          data: [{ x: selectedTrajectory.points.x[0] || 0, y: selectedTrajectory.points.y[0] || 0 }],
          borderColor: 'rgba(0, 200, 0, 1)',
          backgroundColor: 'rgba(0, 200, 0, 1)',
          pointRadius: 6,
          pointStyle: 'circle' as const,
        },
        {
          label: 'Điểm cuối',
          data: [{ x: selectedTrajectory.currentPosition.x, y: selectedTrajectory.currentPosition.y }],
          borderColor: 'rgba(255, 99, 132, 1)',
          backgroundColor: 'rgba(255, 99, 132, 1)',
          pointRadius: 6,
          pointStyle: 'triangle' as const,
          // @ts-ignore Chart.js typings might not include rotation for pointStyle triangle directly in ChartDataset
          rotation: selectedTrajectory.currentPosition.theta * 180 / Math.PI,
        }
      ] : []),
      ...(comparisonTrajectory ? [
        {
          label: 'Quỹ đạo so sánh',
          data: comparisonTrajectory.points.x.map((xVal, i) => ({ x: xVal, y: comparisonTrajectory.points.y[i] })),
          borderColor: 'rgba(153, 102, 255, 1)',
          backgroundColor: 'rgba(153, 102, 255, 0.2)',
          pointRadius: 1,
          showLine: true,
          borderWidth: 2,
          borderDash: [5, 5]
        }
      ] : []),
      ...(showRealTime && realTimeTrajectory.x.length > 0 ? [
        {
          label: 'Quỹ đạo hiện tại',
          data: realTimeTrajectory.x.map((xVal, i) => ({ x: xVal, y: realTimeTrajectory.y[i] })),
          borderColor: 'rgba(255, 159, 64, 1)',
          backgroundColor: 'rgba(255, 159, 64, 0.2)',
          pointRadius: 1,
          showLine: true,
          borderWidth: 2,
        },
        {
          label: 'Vị trí hiện tại',
          data: realTimeTrajectory.x.length > 0 ? [{ x: realTimeTrajectory.x[realTimeTrajectory.x.length - 1], y: realTimeTrajectory.y[realTimeTrajectory.y.length - 1] }] : [],
          borderColor: 'rgba(255, 159, 64, 1)',
          backgroundColor: 'rgba(255, 159, 64, 1)',
          pointRadius: 7,
          pointStyle: 'triangle' as const,
          // @ts-ignore
          rotation: realTimeTrajectory.theta.length > 0 ? realTimeTrajectory.theta[realTimeTrajectory.theta.length - 1] * 180 / Math.PI : 0,
        }
      ] : []),
      ...(isPlaying && selectedTrajectory ? [
        {
          label: 'Playback',
          data: [{ 
            x: selectedTrajectory.points.x[Math.floor(playbackProgress * selectedTrajectory.points.x.length)] || 0, 
            y: selectedTrajectory.points.y[Math.floor(playbackProgress * selectedTrajectory.points.y.length)] || 0 
          }],
          borderColor: 'rgba(255, 0, 0, 1)',
          backgroundColor: 'rgba(255, 0, 0, 1)',
          pointRadius: 8,
          pointStyle: 'circle' as const,
          borderWidth: 2
        }
      ] : [])
    ]
  };

  type LineChartOptions = ChartOptions<'line'>;
  const trajectoryChartOptions: DeepPartial<LineChartOptions> = {
    scales: {
      x: {
        type: 'linear' as const,
        position: 'bottom' as const,
        title: { display: true, text: 'X (m)' },
        grid: { display: true, color: 'rgba(0, 0, 0, 0.1)' }
      },
      y: {
        type: 'linear' as const,
        title: { display: true, text: 'Y (m)' },
        grid: { display: true, color: 'rgba(0, 0, 0, 0.1)' }
      }
    },
    responsive: true,
    maintainAspectRatio: false,
    plugins: {
      tooltip: {
        callbacks: {
          label: function(context: TooltipItem<'line'>) {
            const point = context.raw as {x: number, y: number};
            return `(X: ${point.x.toFixed(2)}, Y: ${point.y.toFixed(2)})`;
          }
        }
      },
      legend: {
        position: 'top' as const,
        labels: { usePointStyle: true }
      },
      zoom: {
        limits: { x: {minRange: 1}, y: {minRange: 1} },
        pan: { enabled: true, mode: 'xy' as const, modifierKey: undefined },
        zoom: { wheel: { enabled: true }, pinch: { enabled: true }, mode: 'xy' as const }
      }
    },
    animation: false as const
  };

  const getTrajectoryStats = () => {
    if (!selectedTrajectory) return { distance: '0.00', points: 0, avgSpeed: '0.00', duration: '0' };
    let distance = selectedTrajectory.metadata?.distance || 0;
    if (!distance) {
      const { x, y } = selectedTrajectory.points;
      for (let i = 1; i < x.length; i++) {
        const dx = x[i] - x[i-1];
        const dy = y[i] - y[i-1];
        distance += Math.sqrt(dx*dx + dy*dy);
      }
    }
    const duration = selectedTrajectory.metadata?.duration || 
      ((selectedTrajectory.points.timestamps && selectedTrajectory.points.timestamps.length > 1) ? 
      (selectedTrajectory.points.timestamps[selectedTrajectory.points.timestamps.length-1] - selectedTrajectory.points.timestamps[0]).toFixed(1) : '0');
    const avgSpeed = selectedTrajectory.metadata?.speed || (parseFloat(String(duration)) > 0 ? (distance / parseFloat(String(duration))).toFixed(2) : '0.00');
    return { 
      distance: distance.toFixed(2),
      points: selectedTrajectory.points.x.length,
      avgSpeed,
      duration
    };
  };

  const resetZoom = () => { if (chartRef.current) chartRef.current.resetZoom(); };
  const clearRealTimeTrajectory = () => setRealTimeTrajectory({ x: [], y: [], theta: [] });

  const toggleLiveUpdatePosition = () => setLiveUpdatePosition(prev => !prev);
  const toggleLiveUpdateHistory = () => setLiveUpdateHistory(prev => !prev);

  const toggleRealTimeDisplay = () => setShowRealTime(prev => !prev);
  const togglePause = () => setIsPaused(prev => !prev);

  const startPlayback = () => {
    if (!selectedTrajectory || selectedTrajectory.points.x.length === 0) return;
    setIsPlaying(true);
    setPlaybackProgress(0);
    const duration = 5000; 
    const startTime = Date.now();
    const endTime = startTime + duration;
    const animate = () => {
      const now = Date.now();
      const progress = Math.min(1, (now - startTime) / duration);
      setPlaybackProgress(progress);
      if (now < endTime) {
        animationRef.current = requestAnimationFrame(animate);
      } else {
        setIsPlaying(false);
      }
    };
    animationRef.current = requestAnimationFrame(animate);
  };

  const stopPlayback = () => {
    if (animationRef.current) cancelAnimationFrame(animationRef.current);
      animationRef.current = null;
    setIsPlaying(false);
  };

  const toggleViewMode = () => setViewMode(prev => prev === '2d' ? '3d' : '2d');

  const handleFilterChange = (key: keyof TrajectoryFilterOptions, value: any) => {
    setFilterOptions(prev => ({ ...prev, [key]: value }));
  };

  const applyFilters = () => {
    requestTrajectoryHistory();
    setIsFilterOpen(false);
  };

  const resetFilters = () => {
    setFilterOptions({ time: '24h', minDistance: undefined, maxDistance: undefined, searchText: '' });
    requestTrajectoryHistory();
    setIsFilterOpen(false);
  };

  const uploadTrajectory = (e: React.ChangeEvent<HTMLInputElement>) => {
    if (!e.target.files || e.target.files.length === 0) return;
    const file = e.target.files[0];
    const reader = new FileReader();
    reader.onload = async (event) => {
      try {
        if (!event.target || !event.target.result) return;
        const csvText = event.target.result as string;
        const lines = csvText.split('\n');
        if (lines.length < 2) { setWidgetError('Invalid CSV: empty or only headers'); return; }
        const headers = lines[0].toLowerCase().split(',').map(h => h.trim());
        const xIndex = headers.indexOf('x');
        const yIndex = headers.indexOf('y');
        const thetaIndex = headers.indexOf('theta');
        if (xIndex === -1 || yIndex === -1) { setWidgetError('Invalid CSV: missing X or Y columns'); return; }
        const points = { x: [] as number[], y: [] as number[], theta: [] as number[] };
        for (let i = 1; i < lines.length; i++) {
          if (!lines[i].trim()) continue;
          const values = lines[i].split(',');
          points.x.push(parseFloat(values[xIndex]));
          points.y.push(parseFloat(values[yIndex]));
          points.theta.push(thetaIndex >=0 ? parseFloat(values[thetaIndex]) : 0);
          }
        if (points.x.length === 0) { setWidgetError('No valid data in CSV'); return; }
        const uploadedTraj: TrajectoryRecord = {
          id: Date.now(), timestamp: new Date().toISOString(),
          currentPosition: { x: points.x[points.x.length - 1], y: points.y[points.y.length - 1], theta: points.theta[points.theta.length - 1] || 0 },
          points, status: 'uploaded',
          metadata: { distance: calculateDistance(points.x, points.y) }
        };
        setComparisonTrajectory(uploadedTraj);
        setWidgetError(null);
      } catch (err: any) { setWidgetError(`Error processing CSV: ${err.message || err}`); }
    };
    reader.onerror = () => { setWidgetError('Error reading file'); };
    reader.readAsText(file);
  };

  const stats = getTrajectoryStats();

  useEffect(() => {
    return () => {
      if (animationRef.current) cancelAnimationFrame(animationRef.current);
        animationRef.current = null;
    };
  }, []);

  if (firmwareUpdateMode) {
    return (
      <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-4 flex flex-col h-full">
        <div className="flex justify-between items-center mb-4">
          <div className="flex items-center gap-2">
            <h3 className="text-lg font-medium">Quỹ Đạo Robot</h3>
          </div>
        </div>
        <div className="flex-grow flex items-center justify-center bg-gray-50 rounded-lg border border-dashed border-gray-300">
          <div className="text-center p-6">
            <AlertCircle size={32} className="text-yellow-500 mx-auto mb-2" />
            <h3 className="text-lg font-medium text-gray-700">Cập Nhật Firmware Đang Diễn Ra</h3>
            <p className="text-gray-500 mt-1">Dữ liệu quỹ đạo tạm thời không khả dụng.</p>
          </div>
        </div>
      </div>
    );
  }
  
  const webSocketIsConnected = readyState === ReadyState.OPEN;
  const connectionStatusDotColor = webSocketIsConnected && selectedRobotId ? 'bg-green-500' : 'bg-gray-400';
  let connectionStatusText = "Chưa chọn robot";
  if (selectedRobotId) {
      if (readyState === ReadyState.CONNECTING) connectionStatusText = "WS: Đang kết nối...";
      else if (readyState === ReadyState.OPEN) {
        if (liveUpdatePosition && subscribedToRealtimeRef.current === selectedRobotId) {
            connectionStatusText = "Live Position Active";
        } else if (liveUpdatePosition && subscribedToRealtimeRef.current !== selectedRobotId) {
            connectionStatusText = "Live Position Pending...";
        } else {
            connectionStatusText = "WS: Đã kết nối (Idle)";
        }
      } 
      else if (readyState === ReadyState.CLOSING) connectionStatusText = "WS: Đang đóng...";
      else if (readyState === ReadyState.CLOSED) connectionStatusText = "WS: Đã đóng";
      else connectionStatusText = "WS: Chưa kết nối";
  }

  return (
    <div className="bg-white rounded-lg shadow-sm border border-gray-200 p-4 flex flex-col h-full">
      <div className="flex justify-between items-center mb-4">
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${connectionStatusDotColor}`}></div>
          <h3 className="text-lg font-medium">Quỹ Đạo Robot {selectedRobotId && `(${selectedRobotId})`}</h3>
          <span className="text-xs text-gray-500">({connectionStatusText})</span>
        </div>
      </div>

      {isSmallScreen ? (
        <div className="flex justify-between">
          <button>...</button>
          <div className="dropdown">
            <button>More</button>
            <div className="dropdown-content">
            </div>
          </div>
        </div>
      ) : (
        <div className="flex flex-wrap gap-2 mb-4">
          <div className="relative">
            <button 
              onClick={() => setIsHistoryOpen(!isHistoryOpen)}
              className="px-3 py-1.5 bg-blue-50 text-blue-600 rounded-md hover:bg-blue-100 flex items-center gap-1"
              disabled={!selectedRobotId || !webSocketIsConnected}
            >
              <List size={16} />
              <span>Danh sách quỹ đạo</span>
              <ChevronDown size={16} className={isHistoryOpen ? "transform rotate-180" : ""} />
            </button>
            {isHistoryOpen && (
              <div className="absolute top-full left-0 mt-1 w-80 max-h-80 overflow-y-auto z-10 bg-white shadow-lg border rounded-md">
                {trajectoryHistory.length > 0 ? (
                  <div className="divide-y divide-gray-100">
                    {trajectoryHistory.map((trajectory) => (
                      <div 
                        key={trajectory.id} 
                        className={`p-3 hover:bg-gray-50 cursor-pointer flex justify-between ${
                          selectedTrajectory?.id === trajectory.id ? 'bg-blue-50' : ''
                        }`}
                        onClick={() => {
                          setSelectedTrajectory(trajectory);
                          setIsHistoryOpen(false);
                        }}
                      >
                        <div className="flex items-center gap-2">
                          <Clock size={16} className="text-gray-500" />
                          <span>{formatDate(trajectory.timestamp)}</span>
                        </div>
                        <div className="text-xs text-gray-500">
                          {trajectory.points.x.length} điểm
                        </div>
                      </div>
                    ))}
                  </div>
                ) : (
                  <div className="p-4 text-center text-gray-500">Không có dữ liệu quỹ đạo</div>
                )}
              </div>
            )}
          </div>

          <div className="relative">
            <button 
              onClick={() => setIsFilterOpen(!isFilterOpen)}
              className="px-3 py-1.5 bg-gray-50 text-gray-600 rounded-md hover:bg-gray-100 flex items-center gap-1"
              disabled={!selectedRobotId || !webSocketIsConnected}
            >
              <Filter size={16} />
              <span>Bộ lọc</span>
              <ChevronDown size={16} className={isFilterOpen ? "transform rotate-180" : ""} />
            </button>
            {isFilterOpen && (
              <div className="absolute top-full right-0 mt-1 w-64 z-10 bg-white shadow-lg border rounded-md p-3">
                <h4 className="font-medium mb-2">Bộ lọc quỹ đạo</h4>
                <div className="space-y-3">
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Thời gian</label>
                    <select value={filterOptions.time} onChange={(e) => handleFilterChange('time', e.target.value)} className="w-full px-2 py-1 border rounded-md text-sm">
                      {timeFilterOptions.map((option: TimeFilterOption) => (<option key={option.value} value={option.value}>{option.label}</option>))}
                    </select>
                  </div>
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Khoảng cách tối thiểu (m)</label>
                    <input type="number" min="0" step="0.1" value={filterOptions.minDistance || ''} onChange={(e) => handleFilterChange('minDistance', e.target.value ? parseFloat(e.target.value) : undefined)} className="w-full px-2 py-1 border rounded-md text-sm" placeholder="Không giới hạn" />
                  </div>
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Khoảng cách tối đa (m)</label>
                    <input type="number" min="0" step="0.1" value={filterOptions.maxDistance || ''} onChange={(e) => handleFilterChange('maxDistance', e.target.value ? parseFloat(e.target.value) : undefined)} className="w-full px-2 py-1 border rounded-md text-sm" placeholder="Không giới hạn" />
                  </div>
                  <div>
                    <label className="block text-sm font-medium text-gray-700 mb-1">Tìm kiếm</label>
                    <input type="text" value={filterOptions.searchText || ''} onChange={(e) => handleFilterChange('searchText', e.target.value)} className="w-full px-2 py-1 border rounded-md text-sm" placeholder="Tìm theo trạng thái, lệnh..." />
                  </div>
                  <div className="flex justify-end gap-2 pt-2">
                    <button onClick={resetFilters} className="px-2 py-1 bg-gray-100 text-gray-600 rounded-md text-sm hover:bg-gray-200">Đặt lại</button>
                    <button onClick={applyFilters} className="px-2 py-1 bg-blue-600 text-white rounded-md text-sm hover:bg-blue-700">Áp dụng</button>
                  </div>
                </div>
              </div>
            )}
          </div>

          <div className="relative">
            <select
              value={filterOptions.time}
              onChange={(e) => {
                handleFilterChange('time', e.target.value);
              }}
              className="px-2 py-1.5 bg-gray-50 text-gray-800 rounded-md border border-gray-200 appearance-none pr-8"
              disabled={!selectedRobotId || !webSocketIsConnected}
            >
              {timeFilterOptions.map((option: TimeFilterOption) => (
                <option key={option.value} value={option.value}>
                  {option.label}
                </option>
              ))}
            </select>
            <Calendar size={14} className="absolute right-2 top-1/2 transform -translate-y-1/2 text-gray-500 pointer-events-none" />
          </div>

          <button
            onClick={requestTrajectoryHistory}
            disabled={!webSocketIsConnected || !selectedRobotId || loading}
            className="px-3 py-1.5 bg-blue-100 text-blue-600 rounded-md hover:bg-blue-200 disabled:opacity-50 flex items-center gap-1"
          >
            <RefreshCw size={16} className={loading ? "animate-spin" : ""} />
            <span>Làm mới Lịch sử</span>
          </button>

          <button
            onClick={toggleLiveUpdatePosition}
            disabled={!webSocketIsConnected || !selectedRobotId}
            className={`px-3 py-1.5 rounded-md flex items-center gap-1 ${
              liveUpdatePosition 
                ? 'bg-green-100 text-green-600 hover:bg-green-200' 
                : 'bg-gray-100 text-gray-600 hover:bg-gray-200'
            } disabled:opacity-50`}
          >
            {liveUpdatePosition ? <Pause size={16} /> : <Play size={16} />}
            <span>Live Position</span>
          </button>

          <button
            onClick={toggleRealTimeDisplay}
            disabled={!webSocketIsConnected || !selectedRobotId || !liveUpdatePosition}
            className={`px-3 py-1.5 rounded-md flex items-center gap-1 ${
              showRealTime && liveUpdatePosition 
                ? 'bg-blue-100 text-blue-600 hover:bg-blue-200' 
                : 'bg-gray-100 text-gray-600 hover:bg-gray-200'
            } disabled:opacity-50`}
          >
            {showRealTime && liveUpdatePosition ? <Pause size={16} /> : <Play size={16} />}
            <span>Show Real-time Line</span>
          </button>

          <button
            onClick={clearRealTimeTrajectory}
            disabled={!webSocketIsConnected || !selectedRobotId || realTimeTrajectory.x.length === 0}
            className="px-3 py-1.5 bg-gray-100 text-gray-600 rounded-md hover:bg-gray-200 disabled:opacity-50 flex items-center gap-1"
          >
            <RotateCcw size={16} />
            <span>Xóa quỹ đạo thực</span>
          </button>

          {selectedTrajectory && (
            <button onClick={downloadTrajectory} className="px-3 py-1.5 bg-green-100 text-green-600 rounded-md hover:bg-green-200 ml-auto flex items-center gap-1" disabled={!selectedRobotId}>
              <Download size={16} /> <span>Tải CSV</span>
            </button>
          )}
          <div className="relative ml-auto">
            <input type="file" accept=".csv" onChange={uploadTrajectory} className="hidden" id="csv-upload" />
            <label htmlFor="csv-upload" className="px-3 py-1.5 bg-purple-100 text-purple-600 rounded-md hover:bg-purple-200 cursor-pointer flex items-center gap-1"> <Upload size={16} /> <span>So sánh CSV</span> </label>
          </div>
          {comparisonTrajectory && (
            <button onClick={() => setComparisonTrajectory(null)} className="px-3 py-1.5 bg-gray-100 text-gray-600 rounded-md hover:bg-gray-200 flex items-center gap-1">
              <XCircle size={16} /> <span>Xóa so sánh</span>
            </button>
          )}
        </div>
      )}

      {widgetError && (
        <div className="bg-red-50 text-red-700 p-3 mb-4 rounded-md flex items-center">
          <AlertCircle size={16} className="mr-2 flex-shrink-0" />
          <p>{widgetError}</p>
          <button onClick={() => setWidgetError(null)} className="ml-auto text-red-500 hover:text-red-700"><XCircle size={16} /></button>
        </div>
      )}

      {selectedTrajectory && (
        <div className="mb-4 bg-gray-50 p-3 rounded-md">
          <div className="flex flex-wrap justify-between items-center">
            <div className="space-y-1">
              <div className="flex items-center gap-1 text-gray-600"><Clock size={14} /> <span className="text-sm">Thời gian: {formatDate(selectedTrajectory.timestamp)}</span></div>
              <div className="flex items-center gap-1 text-gray-600"><MapPin size={14} /> <span className="text-sm">Điểm cuối: ({selectedTrajectory.currentPosition.x.toFixed(2)}, {selectedTrajectory.currentPosition.y.toFixed(2)}, {selectedTrajectory.currentPosition.theta.toFixed(2)} rad)</span></div>
              </div>
            <div className="flex gap-4 text-center">
              <div><div className="text-sm text-gray-500">Khoảng cách</div><div className="font-semibold">{stats.distance} m</div></div>
              <div><div className="text-sm text-gray-500">Số điểm</div><div className="font-semibold">{stats.points}</div></div>
              <div><div className="text-sm text-gray-500">Vận tốc TB</div><div className="font-semibold">{stats.avgSpeed} m/s</div></div>
              <div><div className="text-sm text-gray-500">Thời gian</div><div className="font-semibold">{stats.duration} s</div></div>
              </div>
              </div>
          {selectedTrajectory.points.x.length > 1 && (
            <div className="mt-3 flex items-center gap-2">
              {!isPlaying ? (
                <button onClick={startPlayback} className="px-2 py-1 bg-blue-100 text-blue-600 rounded-md hover:bg-blue-200 text-sm flex items-center gap-1"> <Play size={14} /> <span>Phát lại</span> </button>
              ) : (
                <button onClick={stopPlayback} className="px-2 py-1 bg-red-100 text-red-600 rounded-md hover:bg-red-200 text-sm flex items-center gap-1"> <Pause size={14} /> <span>Dừng</span> </button>
              )}
              {isPlaying && (<div className="flex-grow h-2 bg-gray-200 rounded-full overflow-hidden"><div className="h-full bg-blue-500 rounded-full" style={{ width: `${playbackProgress * 100}%` }}></div></div>)}
            </div>
          )}
        </div>
      )}

      <div className="flex justify-end gap-2 mb-2">
        <button onClick={toggleRealTimeDisplay} className={`p-1.5 rounded-md ${showRealTime ? 'bg-blue-100 text-blue-600' : 'bg-gray-100 text-gray-600'}`} title={showRealTime ? "Ẩn quỹ đạo thực" : "Hiện quỹ đạo thực"} disabled={!liveUpdatePosition}> <Play size={16} /> </button>
        <button onClick={resetZoom} className="p-1.5 bg-gray-100 text-gray-600 rounded-md hover:bg-gray-200" title="Reset Zoom"> <RotateCcw size={16} /> </button>
        <button onClick={() => chartRef.current?.zoom(1.2)} className="p-1.5 bg-gray-100 text-gray-600 rounded-md hover:bg-gray-200" title="Zoom In"> <ZoomIn size={16} /> </button>
        <button onClick={() => chartRef.current?.zoom(0.8)} className="p-1.5 bg-gray-100 text-gray-600 rounded-md hover:bg-gray-200" title="Zoom Out"> <ZoomOut size={16} /> </button>
        <button onClick={toggleViewMode} className="p-1.5 bg-gray-100 text-gray-600 rounded-md hover:bg-gray-200" title={viewMode === '2d' ? 'Chuyển sang 3D' : 'Chuyển sang 2D'}> {viewMode === '2d' ? '3D' : '2D'} </button>
      </div>

      <div className="flex-grow relative" style={{ minHeight: '300px' }}>
        {trajectoryChartData.datasets.length > 0 ? (
          <Line data={trajectoryChartData} options={trajectoryChartOptions as any} ref={chartRef as any} />
        ) : (
          <div className="h-full flex flex-col items-center justify-center text-gray-400">
            <MapPin size={32} strokeWidth={1} />
            <p>Chưa có dữ liệu quỹ đạo nào</p>
            <p className="text-sm mt-1">
              { !selectedRobotId ? "Vui lòng chọn một robot." : 
                !webSocketIsConnected ? "WebSocket chưa kết nối." :
                loading ? "Đang tải lịch sử quỹ đạo..." :
                "Hãy thử làm mới hoặc chờ dữ liệu từ robot."
              }
            </p>
          </div>
        )}
        <div className="absolute bottom-4 right-4 flex flex-col gap-1 bg-white/80 rounded-md p-1 shadow-sm">
          <div className="text-xs text-gray-500 text-center">Zoom: Cuộn chuột | Kéo: Di chuyển</div>
        </div>
      </div>
    </div>
  );
};

export default TrajectoryWidget;