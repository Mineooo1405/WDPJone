import React, { useState, useEffect, useRef } from 'react';
import { useRobotContext } from './RobotContext';
import { Download, RotateCcw, Search, XCircle } from 'lucide-react';
import WidgetConnectionHeader from './WidgetConnectionHeader';
import { ReadyState } from 'react-use-websocket';

// Cấu trúc một mục log
interface LogEntry {
  message: string;
  timestamp: number;
  robotAlias: string; // Changed from robotIp to robotAlias
  level: string; 
  component: string; 
}

// Interface cho payload log từ WebSocket
interface WebSocketLogPayload {
  type: 'log'; // Nên khớp với type được gửi từ backend
  robot_alias: string; // Changed from robot_id to robot_alias
  message: string;
  timestamp: number;
  level: string;
  component?: string; // component có thể tùy chọn từ backend
  robot_ip?: string; // robot_ip is also available from the message
}

const LOG_LEVEL_COLORS: Record<string, string> = {
  ERROR: 'text-red-600 bg-red-50',
  WARNING: 'text-yellow-600 bg-yellow-50',
  INFO: 'text-blue-600 bg-blue-50',
  DEBUG: 'text-gray-600 bg-gray-100',
  VERBOSE: 'text-purple-600 bg-purple-50',
  DEFAULT: 'text-gray-700 bg-gray-50'
};

const LogWidget: React.FC = () => {
  const { selectedRobotId, sendJsonMessage, lastJsonMessage, readyState } = useRobotContext();

  const [logs, setLogs] = useState<LogEntry[]>([]);
  const [filterText, setFilterText] = useState('');
  const [widgetError, setWidgetError] = useState<string | null>(null);
  
  const scrollContainerRef = useRef<HTMLDivElement>(null);
  const isAutoScrollEnabled = useRef(true);

  const parseComponentFromMessage = (message: string): string => {
    const componentMatch = message.match(/\(([^)]+)\)/); 
    return componentMatch && componentMatch[1] ? componentMatch[1].trim() : '';
  };

  // Auto-consume incoming logs whenever WS is open and a robot is selected
  useEffect(() => {
    if (readyState === ReadyState.OPEN && lastJsonMessage && selectedRobotId) {
      const message = lastJsonMessage as WebSocketLogPayload;
      if (message.type === 'log' && message.robot_alias === selectedRobotId) {
        const newLogEntry: LogEntry = {
          timestamp: message.timestamp || Date.now(),
          level: message.level?.toUpperCase() || 'INFO',
          message: message.message || 'No message content',
          robotAlias: message.robot_alias,
          component: message.component || parseComponentFromMessage(message.message || '')
        };
        setLogs(prev => [...prev, newLogEntry].slice(-1000));
        setWidgetError(null);
      }
    }
  }, [lastJsonMessage, readyState, selectedRobotId]);

  // Auto-scroll to bottom when new logs arrive if user is at bottom
  useEffect(() => {
    if (isAutoScrollEnabled.current && scrollContainerRef.current) {
      scrollContainerRef.current.scrollTop = scrollContainerRef.current.scrollHeight;
    }
  }, [logs]);

  // Auto-subscribe on mount/alias change/WS open, and cleanup
  useEffect(() => {
    if (selectedRobotId && readyState === ReadyState.OPEN) {
      console.log(`LogWidget: Auto-subscribing to log for alias ${selectedRobotId}`);
      sendJsonMessage({ command: 'subscribe', type: 'log', robot_alias: selectedRobotId });
    }
    return () => {
      if (selectedRobotId && readyState === ReadyState.OPEN) {
        console.log(`LogWidget: Auto-unsubscribe from log for alias ${selectedRobotId}`);
        sendJsonMessage({ command: 'unsubscribe', type: 'log', robot_alias: selectedRobotId });
      }
    };
  }, [selectedRobotId, readyState, sendJsonMessage]);

  const filteredLogs = logs.filter(log => {
    if (filterText && !log.message.toLowerCase().includes(filterText.toLowerCase())) return false;
    return true;
  });

  const clearLogs = () => {
    setLogs([]);
  };

  const downloadLogs = () => {
    const logsToDownload = filteredLogs; 
    if (logsToDownload.length === 0) {
        setWidgetError("Không có log nào để tải xuống (dựa trên bộ lọc hiện tại).");
        return;
    }
    setWidgetError(null);
    
    let csvContent = 'Timestamp,Robot Alias,Level,Component,Message\n';
    logsToDownload.forEach(log => {
      const formattedMessage = log.message.replace(/"/g, '""');
      const ts = log.timestamp > 2000000000 ? log.timestamp : log.timestamp * 1000;
      const formattedTime = new Date(ts).toISOString(); 
      csvContent += `"${formattedTime}","${log.robotAlias}","${log.level}","${log.component}","${formattedMessage}"\n`;
    });
    
    const blob = new Blob([csvContent], { type: 'text/csv' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `robot_logs_${selectedRobotId || 'all'}_${new Date().toISOString().slice(0,19).replace(/:/g,'-')}.csv`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
  };

  const formatTimestamp = (timestamp: number) => {
    const ts = timestamp > 2000000000 ? timestamp : timestamp * 1000;
    return new Date(ts).toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit', second: '2-digit', fractionalSecondDigits: 2 });
  };

  const getLogLevelColor = (level: string): string => {
    return LOG_LEVEL_COLORS[level.toUpperCase()] || LOG_LEVEL_COLORS.DEFAULT;
  };

  let derivedStatusText: string;
  if (readyState === ReadyState.CONNECTING) {
    derivedStatusText = "WS: Connecting...";
  } else if (readyState !== ReadyState.OPEN) {
    derivedStatusText = "WS: Disconnected";
  } else if (!selectedRobotId) {
    derivedStatusText = "Chưa chọn robot";
  } else {
    derivedStatusText = "Subscribed - Live Logs";
  }

  return (
  <div className="p-4 bg-white dark:bg-gray-800 text-gray-900 dark:text-gray-200 rounded-lg shadow-xl h-full flex flex-col">
      <WidgetConnectionHeader
        title={`Robot Logs (${selectedRobotId || 'Chưa chọn Robot'})`}
        statusTextOverride={derivedStatusText}
        isConnected={readyState === ReadyState.OPEN && !!selectedRobotId}
        error={widgetError}
      />
      
      <div className="flex flex-wrap gap-2 mb-3 mt-3 items-center">
        <div className="relative flex-grow min-w-[150px]">
          <input
            type="text"
            placeholder="Tìm kiếm logs..."
            value={filterText}
            onChange={e => setFilterText(e.target.value)}
            className="w-full px-3 py-2 pr-10 border border-gray-600 bg-gray-700 text-gray-200 rounded-md text-sm focus:ring-blue-500 focus:border-blue-500"
          />
          <Search size={16} className="absolute right-3 top-1/2 transform -translate-y-1/2 text-gray-400" />
        </div>
        
        <button
          onClick={clearLogs}
          className="px-3 py-2 bg-gray-700 hover:bg-gray-600 text-gray-300 rounded-md flex items-center gap-1 text-sm disabled:opacity-50"
          disabled={logs.length === 0}
        >
          <RotateCcw size={14} />
          <span>Clear</span>
        </button>
        
        <button
          onClick={downloadLogs}
          disabled={filteredLogs.length === 0} 
          className="px-3 py-2 bg-gray-700 hover:bg-gray-600 text-gray-300 rounded-md flex items-center gap-1 text-sm disabled:opacity-50 ml-auto"
        >
          <Download size={14} />
          <span>CSV</span>
        </button>
      </div>
      
      {widgetError && (
        <div className="mb-3 p-3 bg-red-900/50 border border-red-700 text-red-300 rounded-md flex items-center gap-2 text-sm">
          <XCircle size={14} />
          <span>{widgetError}</span>
        </div>
      )}
      
      <div 
        ref={scrollContainerRef}
        className="flex-grow overflow-y-auto font-mono text-xs bg-gray-900 rounded-md p-2 border border-gray-700"
        style={{ minHeight: '200px'}}
        onScroll={(e) => {
          const element = e.currentTarget;
          const isAtBottom = element.scrollHeight - element.scrollTop <= element.clientHeight + 20; 
          isAutoScrollEnabled.current = isAtBottom;
        }}
      >
    {filteredLogs.length === 0 ? (
          <div className="h-full flex items-center justify-center text-gray-500">
      {readyState !== ReadyState.OPEN ? "WebSocket chưa kết nối." :
       !selectedRobotId ? "Vui lòng chọn một robot." :
       "Đang chờ log từ robot..."
            }
          </div>
        ) : (
          <div className="space-y-0.5">
            {filteredLogs.map((log, index) => (
              <div key={index} className={`p-1.5 rounded-sm ${getLogLevelColor(log.level).split(' ')[1]}`}>
                <div className={`flex gap-2 mb-0.5 ${getLogLevelColor(log.level).split(' ')[0]}`}>
                  <span>{formatTimestamp(log.timestamp)}</span>
                  <span>(Alias: {log.robotAlias})</span>
                  {log.component && <span className="text-purple-400">[{log.component}]</span>}
                  <span className="font-semibold">{log.level}</span>
                </div>
                <div className={`whitespace-pre-wrap break-all ${getLogLevelColor(log.level).split(' ')[0]}`}>
                  {log.message}
                </div>
              </div>
            ))}
          </div>
        )}
      </div>
      
      <div className="mt-2 flex justify-between text-xs text-gray-400">
        <span>Hiển thị {filteredLogs.length} / {logs.length} logs</span>
        <span>
      {readyState === ReadyState.OPEN ? 'WS: Connected' : 'WS: Disconnected'} -
      {selectedRobotId ? ' Live' : ' No Robot'}
        </span>
      </div>
    </div>
  );
};

export default LogWidget;