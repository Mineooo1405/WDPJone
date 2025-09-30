import React, { useState, useRef, useEffect, useCallback } from "react"; // Removed useContext
import { RefreshCw, Zap, Terminal, Info, HelpCircle, ChevronDown, ChevronUp } from "lucide-react";
import { useWebSocket } from '../contexts/WebSocketProvider';
import { useRobotContext } from './RobotContext';

// Cập nhật interface cho robot
// Removed unused Robot interface
// interface Robot { 
//   robot_id: string;
//   ip: string;
//   active?: boolean;
//   port?: number;
//   last_seen?: number;
// }

interface FirmwareMessage {
  type: string;
  robot_ip: string;
  version?: string;
  status?: 'success' | 'error';
  message?: string;
}

interface FirmwareHistory {
  timestamp: number;
  version: string;
  status: 'success' | 'failed';
  filesize?: number;
  filename?: string;
  duration?: number;
}

const FirmwareUpdateWidget: React.FC<{ compact?: boolean }> = ({ compact = false }) => {
  const { selectedRobotId, connectedRobots } = useRobotContext(); // Get connectedRobots for IP lookup
  const { sendMessage, subscribeToMessageType, isConnected: webSocketIsConnected, error: webSocketError } = useWebSocket();
  
  const [otaStatus, setOtaStatus] = useState<'idle' | 'robot_selected_for_ota1' | 'uploading_to_bridge' | 'firmware_ready' | 'ota1_upgrade_command_sent' | 'error'>('idle');
  const [errorMessage, setErrorMessage] = useState('');
  const [currentVersion, setCurrentVersion] = useState('');
  const [showLogs, setShowLogs] = useState(false);
  const [updateLogs, setUpdateLogs] = useState<string[]>([]);
  // History/upload state removed in RasPi-only
  // Removed advanced toggle UI per request
  const [firmwareInfo, setFirmwareInfo] = useState<{}>({});
  const [targetRobotForOtaIp, setTargetRobotForOtaIp] = useState<string | null>(null);
  const [otaType, setOtaType] = useState<'OTA1' | null>('OTA1');
  const [upgradeMode, setUpgradeMode] = useState<'raw' | 'newline' | 'both'>('raw');
  const [selectedFile, setSelectedFile] = useState<File | null>(null);
  const [uploadProgress, setUploadProgress] = useState(0);

  // RasPi-only: no OTA0 controls
  
  const fileInputRef = useRef<HTMLInputElement>(null);
  const logContainerRef = useRef<HTMLDivElement>(null);

  // const [ipAddress, setIpAddress] = useState(""); // This will be derived and stored in targetRobotForOtaIp
  
  const addLog = useCallback((message: string) => {
    const timestamp = new Date().toLocaleTimeString();
    const logMessage = `[${timestamp}] ${message}`;
    setUpdateLogs(prev => [...prev, logMessage]);
  }, []);
    
  useEffect(() => {
      if (logContainerRef.current) {
        logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
      }
  }, [updateLogs]);

  // Effect to determine and set the target IP for OTA operations (RasPi-only OTA1)
  useEffect(() => {
    addLog(`IP Determination Effect: otaType=${otaType}, selectedRobotId(alias)=${selectedRobotId}`);
    if (otaType === 'OTA1') {
      if (selectedRobotId) {
        const robot = connectedRobots.find(r => r.alias === selectedRobotId);
        if (robot && robot.ip) {
          setTargetRobotForOtaIp(robot.ip);
          addLog(`OTA1: Using IP from selected robot ${selectedRobotId}: ${robot.ip}`);
        } else {
          setTargetRobotForOtaIp(null);
          addLog(`OTA1: Selected robot ${selectedRobotId} not found or has no IP. Cleared target IP.`);
        }
      } else {
        setTargetRobotForOtaIp(null); // No selected robot for OTA1
        addLog("OTA1: No selected robot. Cleared target IP.");
      }
    } else { // otaType is null
      setTargetRobotForOtaIp(null);
      addLog("No OTA type selected. Cleared target IP.");
    }
  }, [otaType, selectedRobotId, connectedRobots, addLog]);

  // Simplified otaStatus management RasPi-only
  useEffect(() => {
    addLog(`OTA Status Effect: otaType=${otaType}, targetIP=${targetRobotForOtaIp}, currentOtaStatus=${otaStatus}`);

    // Guard: If in an active state (uploading, firmware ready, upgrade sent, or error), don't change status automatically here.
    if (otaStatus === 'error' || otaStatus === 'uploading_to_bridge' || otaStatus === 'firmware_ready' || otaStatus === 'ota1_upgrade_command_sent') {
      addLog(`OTA Status Effect: Guarded, status is ${otaStatus}. No change.`);
      return;
    }

    // Guard: If no OTA type or no target IP, reset to idle (unless already idle).
    if (!otaType || !targetRobotForOtaIp) {
      if (otaStatus !== 'idle') {
        addLog(`OTA Status Effect: No otaType or targetIP. Setting to 'idle'.`);
        setOtaStatus('idle');
      }
      return;
    }

    // When IP is ready and not in a protected state, set to robot_selected_for_ota1
    if (otaStatus === 'idle') {
      setOtaStatus('robot_selected_for_ota1');
    }
  }, [otaType, targetRobotForOtaIp, otaStatus, addLog]); 

  const handleActualFirmwareResponse = useCallback((message: FirmwareMessage) => {
    addLog(`WS MSG: ${JSON.stringify(message)}`);
    // Filter by targetRobotForOtaIp
    if (message.robot_ip !== targetRobotForOtaIp) {
      addLog(`Ignoring message for ${message.robot_ip}, current target is ${targetRobotForOtaIp}`);
      return;
    }

    if (message.type === 'firmware_status' && message.status === 'error') { // General error from bridge for this robot
        setOtaStatus('error');
        setErrorMessage(message.message || `Lỗi từ bridge cho robot ${message.robot_ip}`);
        addLog(`❌ Lỗi từ bridge cho ${message.robot_ip}: ${message.message || 'Không rõ lỗi'}`);
    } else if (message.type === 'firmware_response' && message.status === 'error') {
      setErrorMessage(message.message || "Lỗi không xác định từ firmware_response");
      setOtaStatus('error');
      addLog(`❌ Lỗi firmware_response: ${message.message || "Lỗi không xác định"}`);
    } else if (message.type === 'ota_status') {
        addLog(`OTA: ${message.robot_ip}: ${message.status || ''} ${message.message || ''}`);
    }
  }, [addLog, currentVersion, targetRobotForOtaIp, setOtaStatus, setErrorMessage]);

  const handleActualFirmwareProgress = useCallback((_message: FirmwareMessage) => {
        // No progress in RasPi-only mode
  }, []);

  const handleActualFirmwareVersion = useCallback((message: FirmwareMessage) => {
    const newVersion = message.version || "Unknown";
    setCurrentVersion(newVersion);
    if (message.build_date || message.deviceTarget || message.features || message.description) {
          setFirmwareInfo({
            buildDate: message.build_date,
        deviceTarget: message.deviceTarget,
            features: message.features || [],
            description: message.description
          });
        }
    addLog(`Phiên bản firmware hiện tại: ${newVersion}`);
  }, [addLog, setCurrentVersion, setFirmwareInfo]);
        
  useEffect(() => {
    const uniqueIdPrefix = 'FirmwareUpdateWidget';

    const unsubFirmwareStatusError = subscribeToMessageType('firmware_status', handleActualFirmwareResponse, `${uniqueIdPrefix}-firmware_status_error`);
    const unsubOtaStatus = subscribeToMessageType('ota_status', handleActualFirmwareResponse, `${uniqueIdPrefix}-ota_status`);
    const unsubFirmwareProg = subscribeToMessageType('firmware_progress', handleActualFirmwareProgress, `${uniqueIdPrefix}-firmware_progress`);
    const unsubFirmwareVer = subscribeToMessageType('firmware_version', handleActualFirmwareVersion, `${uniqueIdPrefix}-firmware_version`);
    // Listen for generic acks to surface upgrade status clearly
    const unsubAck = subscribeToMessageType('ack', (msg: any) => {
      if (msg && (msg.command === 'upgrade' || msg.command === 'upgrade_signal')) {
        if (msg.status === 'success') {
          addLog(`✅ Ack: Upgrade command acknowledged for ${msg.robot_alias || msg.robot_ip || targetRobotForOtaIp}.`);
        } else {
          addLog(`❌ Ack: Upgrade command failed for ${msg.robot_alias || msg.robot_ip || targetRobotForOtaIp}.`);
          setOtaStatus(prev => prev === 'ota1_upgrade_command_sent' ? 'firmware_ready' : prev);
        }
      }
    }, `${uniqueIdPrefix}-ack`);
    const unsubFwUploadAck = subscribeToMessageType('firmware_upload_ack', (msg: any) => {
      if (msg.status === 'complete') {
        addLog(`✅ BE đã nhận firmware: ${msg.firmware_path}`);
        setOtaStatus('firmware_ready');
      } else if (msg.status === 'error') {
        addLog(`❌ Lỗi upload firmware: ${msg.message}`);
        setOtaStatus('error');
        setErrorMessage(msg.message);
      }
    }, `${uniqueIdPrefix}-firmware_upload_ack`);

    return () => {
      unsubFirmwareStatusError();
      unsubOtaStatus();
      unsubFirmwareProg();
      unsubFirmwareVer();
      unsubAck();
      unsubFwUploadAck();
  };
  }, [subscribeToMessageType, handleActualFirmwareResponse, handleActualFirmwareProgress, handleActualFirmwareVersion, addLog, targetRobotForOtaIp]);
  
  useEffect(() => {
    if (webSocketError) {
      addLog(`WebSocket Connection Error: ${webSocketError.type}`);
      setErrorMessage(`Lỗi kết nối WebSocket. Kiểm tra console.`);
    }
  }, [webSocketError, addLog, setErrorMessage]);
  
  // RasPi-only: no upload to BE

  const handleFileChange = (event: React.ChangeEvent<HTMLInputElement>): void => {
    const file = event.target.files?.[0];
    if (file) {
      setSelectedFile(file);
      addLog(`Đã chọn file: ${file.name} (${file.size} bytes)`);
    }
  };

  const handleUploadFileToBridge = async () => {
    if (!selectedFile || !selectedRobotId || !webSocketIsConnected) {
      setErrorMessage("Cần chọn file và robot trước khi upload.");
      return;
    }
    setOtaStatus('uploading_to_bridge');
    setUploadProgress(0);
    addLog(`Bắt đầu upload ${selectedFile.name} lên BE...`);
    try {
      // 1) Start upload
      sendMessage({
        command: "upload_firmware_start",
        robot_alias: selectedRobotId,
        filename: selectedFile.name,
        filesize: selectedFile.size,
      });
      // 2) Chunk and send
      const chunkSize = 8192;
      let offset = 0;
      const reader = new FileReader();
      while (offset < selectedFile.size) {
        const blob = selectedFile.slice(offset, offset + chunkSize);
        const b64 = await new Promise<string>((res) => {
          reader.onload = () => res((reader.result as string).split(',')[1]);
          reader.readAsDataURL(blob);
        });
        sendMessage({
          command: "firmware_data_chunk",
          robot_alias: selectedRobotId,
          data: b64,
        });
        offset += chunkSize;
        setUploadProgress(Math.min(100, Math.floor((offset / selectedFile.size) * 100)));
      }
      // 3) End upload
      sendMessage({
        command: "upload_firmware_end",
        robot_alias: selectedRobotId,
      });
      addLog("Upload hoàn tất. Chờ xác nhận từ BE...");
    } catch (e: any) {
      setErrorMessage(e.message || "Lỗi upload file");
      setOtaStatus('error');
      addLog(`❌ Lỗi upload: ${e.message}`);
    }
  };

  const checkCurrentVersion = () => {
    if (!webSocketIsConnected) {
      setErrorMessage("Chưa kết nối tới DirectBridge để kiểm tra phiên bản.");
      return;
    }
    let ipToQuery = targetRobotForOtaIp;
    if (!ipToQuery && selectedRobotId) {
        const robot = connectedRobots.find(r => r.alias === selectedRobotId);
        if (robot && robot.ip) ipToQuery = robot.ip;
    }

    if (!ipToQuery) {
        setErrorMessage("Chưa chọn robot hoặc IP để kiểm tra phiên bản.");
        return;
    }
    addLog(`Đang yêu cầu phiên bản firmware hiện tại cho IP: ${ipToQuery}...`);
    sendMessage({
      type: "get_firmware_version",
      robot_ip: ipToQuery, // Send robot_ip
    });
  };

  const formatFileSize = (bytes: number): string => bytes + ' bytes';

  const formatDate = (timestamp: number): string => {
    return new Date(timestamp).toLocaleString();
  };

  const copyLogs = () => {
    const logsText = updateLogs.join('\n');
    navigator.clipboard.writeText(logsText)
      .then(() => {
        addLog("Logs đã được sao chép vào clipboard");
      })
      .catch(err => {
        console.error("Không thể sao chép logs:", err);
      });
  };

  const handleCommandRobotToUpgradeForOTA1 = () => {
    if (!selectedRobotId || !webSocketIsConnected) {
      setErrorMessage("Lệnh Upgrade: Cần chọn Robot và đảm bảo WebSocket đã kết nối.");
      addLog(`Precondition for Upgrade command failed: alias=${selectedRobotId}, wsConnected=${webSocketIsConnected}`);
      return;
    }
    const targetAlias = selectedRobotId;
    addLog(`Gửi lệnh "Upgrade" tới robot alias=${targetAlias} (mode=${upgradeMode})...`);
    sendMessage({
      command: "upgrade",
      robot_alias: targetAlias,
      mode: upgradeMode,
    });
    setErrorMessage('');
    setOtaStatus('ota1_upgrade_command_sent');
    addLog("Lệnh Upgrade đã gửi. RasPi sẽ xử lý OTA trực tiếp (không upload qua BE).");
  };

  return (
    <div className="bg-white dark:bg-gray-800 text-gray-900 dark:text-gray-100 p-4 rounded-lg shadow border border-gray-200 dark:border-gray-700 h-full flex flex-col overflow-hidden">
  <h3 className="text-lg font-medium mb-4 flex items-center justify-between">
        <span>Cập Nhật Firmware</span>
    <div className="flex items-center gap-2 text-sm text-gray-700 dark:text-gray-200">RasPi mode</div>
      </h3>

      {/* OTA1: Upgrade mode selector */}
      {otaType === 'OTA1' && (
        <div className="mb-3 flex items-center gap-2 text-xs">
          <span className="text-gray-600 dark:text-gray-300">Chế độ lệnh Upgrade:</span>
          <select
            value={upgradeMode}
            onChange={(e) => setUpgradeMode(e.target.value as 'raw' | 'newline' | 'both')}
            className="px-2 py-1 border border-gray-300 dark:border-gray-600 bg-white dark:bg-gray-700 text-gray-900 dark:text-gray-100 rounded"
          >
            <option value="raw">raw ("Upgrade")</option>
            <option value="newline">newline ("Upgrade\\n")</option>
            <option value="both">both (raw + newline)</option>
          </select>
        </div>
      )}

      {/* RasPi-only: no OTA0 input */}


      <div className="flex gap-4 mb-4">
        <div className="flex-1">
          <div className="mb-4 flex items-center bg-blue-50 dark:bg-blue-900/30 p-3 rounded-md text-blue-700 dark:text-blue-200">
            <Info size={20} className="mr-2 flex-shrink-0" />
            <div className="flex-1">
              {(selectedRobotId || targetRobotForOtaIp) && (
                <p className="font-medium">Robot: {selectedRobotId} {targetRobotForOtaIp ? `(IP mục tiêu: ${targetRobotForOtaIp})` : ''}</p>
              )}
              {currentVersion && (
                <p className="text-sm">Phiên bản hiện tại: {currentVersion}</p>
              )}
              
              {firmwareInfo.buildDate && (
                <p className="text-xs mt-1">Build date: {firmwareInfo.buildDate}</p>
              )}
              
              {firmwareInfo.deviceTarget && (
                <p className="text-xs">Target: {firmwareInfo.deviceTarget}</p>
              )}
            </div>
            <button
              onClick={checkCurrentVersion}
        className="ml-auto p-1 hover:bg-blue-100 dark:hover:bg-blue-800/50 rounded-full flex-shrink-0"
              title="Kiểm tra phiên bản"
              disabled={!webSocketIsConnected}
            >
              <RefreshCw size={16} className={!webSocketIsConnected ? "opacity-50" : ""} />
            </button>
          </div>

      <div className="mb-4 flex items-center justify-between bg-gray-50 dark:bg-gray-700 p-3 rounded-md">
            <div className="flex items-center">
              <div className={`w-3 h-3 rounded-full mr-2 ${webSocketIsConnected ? 'bg-green-500' : 'bg-red-500'}`}></div>
              <span>{webSocketIsConnected ? 'Đã kết nối tới DirectBridge' : 'Chưa kết nối'}</span>
            </div>

            {!webSocketIsConnected && (
              <button
                onClick={() => { /*sendMessage({type: 'request_connect'}) or rely on auto-reconnect of WebSocketProvider */ }} 
        className="px-3 py-1 bg-blue-600 text-white text-sm rounded hover:bg-blue-700"
              >
                Kết nối
              </button>
            )}
          </div>

          <div className="mb-4 p-3 bg-gray-50 dark:bg-gray-700 rounded-md text-sm">
            RasPi mode: Chọn file .bin và tải lên BE trước, sau đó nhấn Upgrade để RasPi nhận firmware.
            <div className="mt-2 flex items-center gap-2">
              <input type="file" accept=".bin" onChange={handleFileChange} className="text-xs" ref={fileInputRef} />
              {selectedFile && <span className="text-green-600">✓ {selectedFile.name}</span>}
            </div>
            {otaStatus === 'uploading_to_bridge' && (
              <div className="mt-2">
                <div className="w-full bg-gray-200 rounded-full h-2">
                  <div className="bg-blue-600 h-2 rounded-full" style={{width: `${uploadProgress}%`}}></div>
                </div>
                <p className="text-xs text-center mt-1">{uploadProgress}%</p>
              </div>
            )}
          </div>

          {/* RasPi-only: không hiển thị trạng thái OTA server */}

          {/* RasPi-only: không có OTA0 */}

          {/* OTA1 Path Guidance: Step 1 - Robot Selected, Ready to Command Upgrade */}
          {!compact && otaStatus === 'robot_selected_for_ota1' && otaType === 'OTA1' && targetRobotForOtaIp && (
            <div className="mb-4 bg-orange-50 dark:bg-orange-900/30 border-l-4 border-orange-500 dark:border-orange-700 text-orange-700 dark:text-orange-200 p-3 rounded">
              <p className="font-medium">Chế độ OTA1: Robot mục tiêu IP {targetRobotForOtaIp}.</p>
              <p>Nhấn "Yêu cầu Robot vào Chế Độ Nâng Cấp (OTA1)" để robot chuẩn bị nhận firmware.</p>
            </div>
          )}

          {/* Sau khi gửi Upgrade */}
          {!compact && otaStatus === 'ota1_upgrade_command_sent' && otaType === 'OTA1' && targetRobotForOtaIp && (
            <div className="mb-4 bg-yellow-50 dark:bg-yellow-900/30 border-l-4 border-yellow-500 dark:border-yellow-700 text-yellow-700 dark:text-yellow-200 p-3 rounded">
              <p className="font-medium">Đã gửi lệnh "Upgrade" cho Robot {targetRobotForOtaIp}!</p>
              <p>Robot sẽ vào chế độ OTA và RasPi xử lý trực tiếp. BE/FE không upload file.</p>
            </div>
          )}

          {/* RasPi-only: không có bridge_ready_for_robot */}

          {/* RasPi-only: không có tiến trình upload */}

          {otaStatus === 'error' && (
            <div className="mb-4 bg-red-50 dark:bg-red-900/30 border-l-4 border-red-500 dark:border-red-700 text-red-700 dark:text-red-200 p-3 rounded">
              <p className="font-medium">Lỗi</p>
              <p>{errorMessage}</p>
            </div>
          )}

          <div className="flex justify-end gap-2 mt-2">
            <button
              onClick={handleUploadFileToBridge}
              disabled={!selectedFile || !webSocketIsConnected || otaStatus === 'uploading_to_bridge' || otaStatus === 'firmware_ready'}
              className={`px-4 py-2 rounded-md flex items-center gap-2 bg-blue-500 text-white hover:bg-blue-600
                ${(!selectedFile || !webSocketIsConnected || otaStatus === 'uploading_to_bridge' || otaStatus === 'firmware_ready') ? 'opacity-50 cursor-not-allowed' : ''}
              `}
            >
              <RefreshCw size={16} className={otaStatus === 'uploading_to_bridge' ? 'animate-spin' : ''} />
              <span>{otaStatus === 'uploading_to_bridge' ? 'Đang upload...' : 'Tải lên BE'}</span>
            </button>

            <button
              onClick={handleCommandRobotToUpgradeForOTA1}
              disabled={!(
                otaType === 'OTA1' &&
                targetRobotForOtaIp &&
                webSocketIsConnected &&
                (otaStatus === 'firmware_ready' || otaStatus === 'ota1_upgrade_command_sent')
              )}
              className={`px-4 py-2 rounded-md flex items-center gap-2 bg-orange-500 text-white hover:bg-orange-600
                ${!(
                  otaType === 'OTA1' &&
                  targetRobotForOtaIp &&
                  webSocketIsConnected &&
                  (otaStatus === 'firmware_ready' || otaStatus === 'ota1_upgrade_command_sent')
                ) ? 'opacity-50 cursor-not-allowed' : ''}
              `}
            >
              <Zap size={16} />
              <span>Gửi lệnh Upgrade</span>
            </button>
          </div>
        </div>
        
        {!compact && (
        <div className="flex-1 flex flex-col">
          <div className="flex items-center justify-between mb-2">
            <div 
              className="flex items-center gap-1 cursor-pointer hover:bg-gray-100 dark:hover:bg-gray-700 px-2 py-1 rounded-md"
              onClick={() => setShowLogs(!showLogs)}
            >
              <Terminal size={16} />
              <span className="font-medium text-sm">Logs cập nhật</span>
              {showLogs ? <ChevronUp size={14} /> : <ChevronDown size={14} />}
            </div>
            {showLogs && (
              <div className="flex gap-1">
                <button
                  onClick={copyLogs}
                  className="text-xs px-2 py-1 bg-gray-100 dark:bg-gray-700 rounded hover:bg-gray-200 dark:hover:bg-gray-600"
                >
                  Copy
                </button>
                <button
                  onClick={() => setUpdateLogs([])}
                  className="text-xs px-2 py-1 bg-gray-100 dark:bg-gray-700 rounded hover:bg-gray-200 dark:hover:bg-gray-600"
                >
                  Clear
                </button>
              </div>
            )}
          </div>
          
          {showLogs && (
            <div 
              ref={logContainerRef}
              className="flex-grow h-40 overflow-y-auto bg-gray-900 text-gray-200 p-2 rounded-md mb-3 font-mono text-xs"
            >
              {updateLogs.length > 0 ? (
                updateLogs.map((log, index) => (
                  <div key={index} className="mb-1">
                    {log}
                  </div>
                ))
              ) : (
                <div className="text-gray-500 dark:text-gray-400 italic">
                  Chưa có logs nào. Hãy thực hiện các thao tác để xem logs.
                </div>
              )}
            </div>
          )}
          
          {/* RasPi-only: bỏ toàn bộ lịch sử cập nhật và các icon liên quan */}
          
          <div className="mt-3 bg-blue-50 dark:bg-blue-900/30 p-3 rounded-md text-sm text-blue-800 dark:text-blue-200">
            <div className="flex items-center mb-1">
              <HelpCircle size={16} className="mr-1" />
              <span className="font-medium">Hướng dẫn cập nhật firmware:</span>
            </div>
            <ol className="list-decimal pl-5 space-y-1">
              <li>Chọn chế độ OTA (OTA0 hoặc OTA1).</li>
              <li>Chọn robot từ danh sách.</li>
              <li><b>Nếu OTA1:</b> Nhấn "Yêu cầu Robot vào Chế Độ Nâng Cấp (OTA1)". Chờ robot khởi động lại.</li>
              <li>Chọn file firmware (.bin).</li>
              <li>Nhấn "Tải Firmware lên Server cho Robot [tên robot]".</li>
              <li>Đợi quá trình tải lên bridge hoàn tất và nhận thông báo "Firmware đã sẵn sàng...".</li>
              <li>(Tùy chọn) Kiểm tra phiên bản firmware mới sau khi robot khởi động lại.</li>
            </ol>
          </div>
        </div>
        )}
      </div>
    </div>
  );
};

export default FirmwareUpdateWidget;