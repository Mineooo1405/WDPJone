import React, { useState, useCallback, useRef, useEffect } from "react";
import WidgetConnectionHeader from "./WidgetConnectionHeader";
import { ArrowUp, ArrowDown, ArrowLeft, ArrowRight, RotateCw, RotateCcw as RotateCounter } from 'lucide-react';
import { useRobotContext } from './RobotContext';
import { useWebSocket } from '../contexts/WebSocketProvider';

interface RPMData {
  [key: number]: number; // motor_id -> RPM value
}

// Định nghĩa kiểu dữ liệu cho messages WebSocket
type WebSocketMessage = {
  type: string;
  robot_ip?: string;
  robot_alias?: string;
  robot_id?: string; // Có thể vẫn còn trong một số message cũ
  data?: any; // Cho encoder_data, imu_data
  position?: PositionData; // Cho position_update
  status?: string; // Cho command_response, bno_event
  message?: string; // Cho error, command_response
  event_type?: string; // Cho bno_event
  original_command?: string; // Cho command_response
  payload_type_sent_to_robot?: string; // Cho command_response
  // Các trường cụ thể cho từng loại message (ví dụ: rpm_1, heading) sẽ được truy cập qua message.data
  [key: string]: any; 
};

// Robot info as received from the bridge
// Removed unused RobotInfo interface

// Định nghĩa keys cho điều khiển bàn phím
type KeysPressed = {
  w: boolean; // Tiến
  a: boolean; // Rẽ trái
  s: boolean; // Lùi
  d: boolean; // Rẽ phải
  q: boolean; // Xoay trái
  e: boolean; // Xoay phải
};

// Interface cho Direct Command API (Keep for reference, but primarily using WebSocket commands)
// Removed unused direct command interfaces

// Thêm interface cho giao tiếp qua robot_id
// Removed unused RobotCommandRequest/Response

// Add interface for IMU data state
interface ImuData {
  heading: number;
  pitch: number;
  roll: number;
  calibrated: boolean; // Added for calibration status
  // Add other IMU fields if needed (quat, accel, etc.)
}

// Add interface for Position data state
interface PositionData {
  x: number;
  y: number;
  theta: number;
}

// ĐÂY LÀ INTERFACE QUAN TRỌNG
interface RobotCommandPayload { 
  type: string; 
  [key: string]: any; 
}

const RobotControlWidget: React.FC<{ compact?: boolean }> = ({ compact = false }) => {
  const { selectedRobotId, connectedRobots } = useRobotContext();
  const { 
    sendMessage, 
    isConnected: webSocketIsConnected, 
    error: webSocketError,
    lastJsonMessage
  } = useWebSocket();
  
  const [motorSpeeds, setMotorSpeeds] = useState<number[]>([0, 0, 0]);
  const [rpmValues, setRpmValues] = useState<RPMData>({1: 0, 2: 0, 3: 0});
  const [errorMessage, setErrorMessage] = useState<string>("");
  // Removed unused statusMessage state
  // Tab cố định: chỉ dùng keyboard
  const [activeTab] = useState<'joystick' | 'motors' | 'keyboard'>('keyboard');
  const [velocities, setVelocities] = useState({ x: 0, y: 0, theta: 0 });
  const [maxSpeed] = useState(1.0); 
  const [maxAngular] = useState(2.0); 
  const [keysPressed, setKeysPressed] = useState<KeysPressed>({ w: false, a: false, s: false, d: false, q: false, e: false });
  const [hasFocus, setHasFocus] = useState(false);
  const [commandSending, setCommandSending] = useState(false);
  const [imuData, setImuData] = useState<ImuData>({ heading: 0, pitch: 0, roll: 0, calibrated: false });
  const [robotPosition, setRobotPosition] = useState<PositionData>({ x: 0, y: 0, theta: 0 });
  // Định nghĩa chiều chuẩn: W tiến (x+), D phải (y+)
  const SIGN_X = 1;
  const SIGN_Y = 1;
  // Chế độ chỉ dùng bàn phím
  const KEYBOARD_ONLY = true;
  
  const joystickRef = useRef<HTMLDivElement>(null);
  const knobRef = useRef<HTMLDivElement>(null);
  const rotationKnobRef = useRef<HTMLDivElement>(null);
  const keyboardControlRef = useRef<HTMLDivElement>(null);
  const isDraggingRef = useRef(false);
  const isRotatingRef = useRef(false);
  const prevVelocitiesRef = useRef({ x: 0, y: 0, theta: 0 });
  const commandThrottleRef = useRef<NodeJS.Timeout | null>(null);

  const getSelectedRobotIp = useCallback((): string | null => {
    if (!selectedRobotId) return null;
    const robot = connectedRobots.find(r => r.alias === selectedRobotId);
    return robot ? robot.ip : null;
  }, [selectedRobotId, connectedRobots]);

  const currentSelectedIp = getSelectedRobotIp();

  const sendWsCommand = useCallback(async (payloadForRobot: RobotCommandPayload): Promise<void> => {
    if (!selectedRobotId) {
      setErrorMessage("Lỗi: Chưa chọn robot.");
      console.error("sendWsCommand: Target Robot alias is null or invalid (selectedRobotId).");
      setCommandSending(false);
      return;
    }
    if (!webSocketIsConnected) {
      setErrorMessage("Lỗi: WebSocket không kết nối.");
      setCommandSending(false);
      return;
    }
    
    setCommandSending(true);
    setErrorMessage(""); 
  // status text handled via header state

    const messageToSend = {
      // Map high-level payload types to backend plaintext commands
      ...(payloadForRobot.type === 'motion' ? { command: 'vector_control', robot_alias: selectedRobotId, dot_x: payloadForRobot.x, dot_y: payloadForRobot.y, dot_theta: payloadForRobot.theta, stop_time: (payloadForRobot as any).stop_time ?? 0 } : {}),
      ...(payloadForRobot.type === 'motor_speed' ? { command: 'motor_speed', robot_alias: selectedRobotId, motor_id: payloadForRobot.motor, speed: payloadForRobot.speed } : {}),
      ...(payloadForRobot.type === 'emergency_stop' ? { command: 'emergency_stop', robot_alias: selectedRobotId } : {}),
      ...(payloadForRobot.type === 'pid_values' ? { command: 'set_pid', robot_alias: selectedRobotId, motor_id: payloadForRobot.motor, kp: payloadForRobot.kp, ki: payloadForRobot.ki, kd: payloadForRobot.kd } : {}),
    };
    // console.log(`[RobotControlWidget] Sending command:`, messageToSend); // Bỏ comment nếu cần debug
    sendMessage(messageToSend);
  }, [webSocketIsConnected, sendMessage, selectedRobotId]);
  
  const throttledSendCommand = useCallback((payloadForRobot: RobotCommandPayload) => {
    if (!selectedRobotId || !webSocketIsConnected) return;
    if (commandThrottleRef.current) clearTimeout(commandThrottleRef.current);
    commandThrottleRef.current = setTimeout(() => {
      sendWsCommand(payloadForRobot);
      commandThrottleRef.current = null;
    }, 100); // 100ms throttle
  }, [selectedRobotId, sendWsCommand, webSocketIsConnected]);

  // Gửi motion trực tiếp (phục vụ giữ phím liên tục)
  const sendMotionRaw = useCallback((x: number, y: number, theta: number) => {
    if (!selectedRobotId || !webSocketIsConnected) return;
    const msg = { command: 'vector_control', robot_alias: selectedRobotId, dot_x: x, dot_y: y, dot_theta: theta, stop_time: 0 } as any;
    sendMessage(msg);
  }, [selectedRobotId, webSocketIsConnected, sendMessage]);

  const updateVelocities = useCallback((x: number, y: number, theta: number) => {
    const newVelocities = { x: parseFloat(x.toFixed(2)), y: parseFloat(y.toFixed(2)), theta: parseFloat(theta.toFixed(2)) };
    setVelocities(newVelocities);
    const isStopping = newVelocities.x === 0 && newVelocities.y === 0 && newVelocities.theta === 0;
    const hasSignificantChange = 
      Math.abs(newVelocities.x - prevVelocitiesRef.current.x) > 0.01 ||
      Math.abs(newVelocities.y - prevVelocitiesRef.current.y) > 0.01 ||
      Math.abs(newVelocities.theta - prevVelocitiesRef.current.theta) > 0.01;

    // Gửi lệnh nếu có thay đổi đáng kể HOẶC nếu đang dừng lại (để đảm bảo lệnh dừng được gửi)
    // Và chỉ gửi nếu widget sẵn sàng và không có lệnh nào đang được gửi
    if ((hasSignificantChange || (isStopping && (prevVelocitiesRef.current.x !==0 || prevVelocitiesRef.current.y !==0 || prevVelocitiesRef.current.theta !==0 )) ) 
        && webSocketIsConnected && currentSelectedIp) {
        prevVelocitiesRef.current = newVelocities;
        throttledSendCommand({ type: "motion", x: newVelocities.x, y: newVelocities.y, theta: newVelocities.theta });
    }
  }, [throttledSendCommand, webSocketIsConnected, currentSelectedIp]);

  const setMotorSpeed = useCallback((motorId: number, speed: number): void => {
    if (!selectedRobotId || !webSocketIsConnected) {
      setErrorMessage("Lỗi: Chọn robot và đảm bảo WebSocket kết nối."); return;
    }
    setMotorSpeeds(prevSpeeds => {
      const newSpeeds = [...prevSpeeds]; newSpeeds[motorId - 1] = speed; return newSpeeds;
    });
    throttledSendCommand({ type: "motor_speed", motor: motorId, speed: speed });
  }, [selectedRobotId, webSocketIsConnected, throttledSendCommand]);

  // Emergency stop removed per request

  // --- START: Keyboard Control Logic (continuous, global without focus) ---
  useEffect(() => {
    const KBD_SPEED_LINEAR = 0.3;
    const KBD_ANGULAR_SPEED = 0.5;
    const SEND_INTERVAL_MS = 100;
    const kbRef = keyboardControlRef.current;
    if (!kbRef) return;

    const handleFocus = () => setHasFocus(true);
    const handleBlur = () => {
      setHasFocus(false);
      setKeysPressed({ w: false, a: false, s: false, d: false, q: false, e: false });
      if (webSocketIsConnected && selectedRobotId) sendMotionRaw(0, 0, 0);
    };
    const handleKeyDown = (event: KeyboardEvent) => {
      // Cho phép điều khiển toàn cục; bỏ qua khi đang gõ trong input/textarea/select hoặc contentEditable
      const tgt: any = event.target as any;
      const tag = (tgt && tgt.tagName ? String(tgt.tagName).toLowerCase() : "");
      if (tag === 'input' || tag === 'textarea' || tag === 'select' || (tgt && tgt.isContentEditable)) return;
      if (!webSocketIsConnected || !selectedRobotId) return;
      const key = event.key.toLowerCase();
      if (["w","a","s","d","q","e"].includes(key)) {
        event.preventDefault();
        // Cập nhật state giữ phím
        setKeysPressed(prev => ({ ...prev, [key]: true }));
        // Gửi ngay một khung điều khiển để hỗ trợ nhấn-nhả nhanh (tap)
        const next = { ...keysPressed, [key]: true } as KeysPressed;
        let vx = 0, vy = 0, omg = 0;
        // W/S điều khiển tiến/lùi trên trục dot_y, A/D điều khiển trái/phải trên trục dot_x
        if (next.w) vy += SIGN_Y * (KBD_SPEED_LINEAR * maxSpeed);
        if (next.s) vy -= SIGN_Y * (KBD_SPEED_LINEAR * maxSpeed);
        if (next.a) vx -= SIGN_X * (KBD_SPEED_LINEAR * maxSpeed);
        if (next.d) vx += SIGN_X * (KBD_SPEED_LINEAR * maxSpeed);
        if (next.q) omg += KBD_ANGULAR_SPEED * maxAngular;
        if (next.e) omg -= KBD_ANGULAR_SPEED * maxAngular;
        const isAny = next.w || next.a || next.s || next.d || next.q || next.e;
        if (isAny && webSocketIsConnected && selectedRobotId) {
          prevVelocitiesRef.current = { x: vx, y: vy, theta: omg };
          sendMotionRaw(vx, vy, omg);
        }
      }
    };
    const handleKeyUp = (event: KeyboardEvent) => {
      const tgt: any = event.target as any;
      const tag = (tgt && tgt.tagName ? String(tgt.tagName).toLowerCase() : "");
      if (tag === 'input' || tag === 'textarea' || tag === 'select' || (tgt && tgt.isContentEditable)) return;
      if (!webSocketIsConnected || !selectedRobotId) return;
      const key = event.key.toLowerCase();
      if (["w","a","s","d","q","e"].includes(key)) {
        event.preventDefault();
        setKeysPressed(prev => ({ ...prev, [key]: false }));
        // Gửi ngay để dừng hoặc điều chỉnh vector khi nhả phím
        const next = { ...keysPressed, [key]: false } as KeysPressed;
        let vx = 0, vy = 0, omg = 0;
        if (next.w) vy += SIGN_Y * (KBD_SPEED_LINEAR * maxSpeed);
        if (next.s) vy -= SIGN_Y * (KBD_SPEED_LINEAR * maxSpeed);
        if (next.a) vx -= SIGN_X * (KBD_SPEED_LINEAR * maxSpeed);
        if (next.d) vx += SIGN_X * (KBD_SPEED_LINEAR * maxSpeed);
        if (next.q) omg += KBD_ANGULAR_SPEED * maxAngular;
        if (next.e) omg -= KBD_ANGULAR_SPEED * maxAngular;
        const isAny = next.w || next.a || next.s || next.d || next.q || next.e;
        if (webSocketIsConnected && selectedRobotId) {
          if (isAny) {
            prevVelocitiesRef.current = { x: vx, y: vy, theta: omg };
            sendMotionRaw(vx, vy, omg);
          } else {
            prevVelocitiesRef.current = { x: 0, y: 0, theta: 0 };
            sendMotionRaw(0, 0, 0);
          }
        }
      }
    };

    let intervalId: any = null;
    const startLoop = () => {
      if (intervalId) return;
      intervalId = setInterval(() => {
        if (!webSocketIsConnected || !selectedRobotId) return;
        let vx = 0, vy = 0, omg = 0;
        if (keysPressed.w) vy += SIGN_Y * (KBD_SPEED_LINEAR * maxSpeed);
        if (keysPressed.s) vy -= SIGN_Y * (KBD_SPEED_LINEAR * maxSpeed);
        if (keysPressed.a) vx -= SIGN_X * (KBD_SPEED_LINEAR * maxSpeed);
        if (keysPressed.d) vx += SIGN_X * (KBD_SPEED_LINEAR * maxSpeed);
        if (keysPressed.q) omg += KBD_ANGULAR_SPEED * maxAngular;
        if (keysPressed.e) omg -= KBD_ANGULAR_SPEED * maxAngular;
        const isAny = keysPressed.w || keysPressed.a || keysPressed.s || keysPressed.d || keysPressed.q || keysPressed.e;
        if (isAny) {
          prevVelocitiesRef.current = { x: vx, y: vy, theta: omg };
          sendMotionRaw(vx, vy, omg);
        } else {
          const wasMoving = Math.abs(prevVelocitiesRef.current.x) > 1e-3 || Math.abs(prevVelocitiesRef.current.y) > 1e-3 || Math.abs(prevVelocitiesRef.current.theta) > 1e-3;
          if (wasMoving) {
            prevVelocitiesRef.current = { x: 0, y: 0, theta: 0 };
            sendMotionRaw(0, 0, 0);
          }
        }
      }, SEND_INTERVAL_MS);
    };
    const stopLoop = () => { if (intervalId) { clearInterval(intervalId); intervalId = null; } };

    kbRef.addEventListener('focus', handleFocus);
    kbRef.addEventListener('blur', handleBlur);
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    startLoop();
    return () => {
      stopLoop();
      kbRef.removeEventListener('focus', handleFocus);
      kbRef.removeEventListener('blur', handleBlur);
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  }, [hasFocus, webSocketIsConnected, selectedRobotId, maxSpeed, maxAngular, keysPressed, sendMotionRaw]);
  // --- END: Keyboard Control Logic ---

  // --- START: Joystick Control Logic ---
  useEffect(() => {
    const joyRef = joystickRef.current;
    const kRef = knobRef.current;
    const rotRef = rotationKnobRef.current;

    if (!joyRef || !kRef || !rotRef) return;

    let joyCenterX = 0;
    let joyCenterY = 0;
    let joyRadius = 0;
    let rotKnobWidth = 0;
    let rotTrackWidth = 0;

    const calculateJoyParams = () => {
      const joyRect = joyRef.getBoundingClientRect();
      joyCenterX = joyRect.left + joyRect.width / 2;
      joyCenterY = joyRect.top + joyRect.height / 2;
      joyRadius = joyRect.width / 2 - kRef.offsetWidth / 2; // Bán kính di chuyển của knob
      
      if (rotRef) {
        const rotRect = rotRef.getBoundingClientRect();
        rotTrackWidth = rotRect.width;
        const thumb = rotRef.querySelector('.thumb') as HTMLElement;
        if (thumb) rotKnobWidth = thumb.offsetWidth;
      }
    };

    calculateJoyParams(); // Initial calculation
    window.addEventListener('resize', calculateJoyParams); // Recalculate on resize

    const handleJoyStart = (clientX: number, clientY: number) => {
      isDraggingRef.current = true;
      kRef.style.transition = 'none'; // Disable transition while dragging
      document.body.style.cursor = 'grabbing';
      handleJoyMove(clientX, clientY); // Process initial position
    };

    const handleJoyMove = (clientX: number, clientY: number) => {
      if (!isDraggingRef.current || !joyRef || !kRef) return;

      let dx = clientX - joyCenterX;
      let dy = clientY - joyCenterY;
      let distance = Math.sqrt(dx * dx + dy * dy);

      let normX = 0;
      let normY = 0;

      if (distance > joyRadius) {
        dx = (dx / distance) * joyRadius;
        dy = (dy / distance) * joyRadius;
      }
      
      kRef.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

      if (joyRadius > 0) {
        normX = dx / joyRadius; // -1 .. 1 (right positive)
        normY = -dy / joyRadius; // -1 .. 1 (up positive)
      }
      // Map: vx (forward/back) from vertical; vy (strafe) from horizontal (đảo dấu theo cấu hình)
  const vx = SIGN_X * (normY * maxSpeed);
  const vy = SIGN_Y * (-normX * maxSpeed);
      updateVelocities(vx, vy, velocities.theta);
    };

    const handleJoyEnd = () => {
      if (!isDraggingRef.current || !kRef) return;
      isDraggingRef.current = false;
      kRef.style.transition = 'transform 0.2s ease-out';
      kRef.style.transform = 'translate(-50%, -50%)'; // Return to center
      document.body.style.cursor = 'default';
  updateVelocities(0, 0, velocities.theta); // Stop linear movement
    };

    const handleRotStart = (clientX: number) => {
        if (!rotRef) return;
        isRotatingRef.current = true;
        const thumb = rotRef.querySelector('.thumb') as HTMLElement;
        if (thumb) thumb.style.transition = 'none';
        document.body.style.cursor = 'grabbing';
        handleRotMove(clientX);
    };
    
    const handleRotMove = (clientX: number) => {
        if (!isRotatingRef.current || !rotRef) return;
        const rotRect = rotRef.getBoundingClientRect();
        const thumb = rotRef.querySelector('.thumb') as HTMLElement;
        if (!thumb) return;

        let newLeft = clientX - rotRect.left;
        // Clamp position
        newLeft = Math.max(rotKnobWidth / 2, Math.min(newLeft, rotTrackWidth - rotKnobWidth / 2));
        
        thumb.style.left = `${newLeft}px`;
        
        // Normalize: (current - min) / (max - min) -> maps to 0-1 range
        // Then scale to -1 to 1
  const normalizedTheta = ((newLeft - rotKnobWidth / 2) / (rotTrackWidth - rotKnobWidth)) * 2 - 1;
  // Quy ước: phải = CW = theta âm; trái = CCW = theta dương (khớp với Q/E ở trên)
  updateVelocities(velocities.x, velocities.y, -normalizedTheta * maxAngular);
    };

    const handleRotEnd = () => {
        if (!isRotatingRef.current || !rotRef) return;
        isRotatingRef.current = false;
        const thumb = rotRef.querySelector('.thumb') as HTMLElement;
        if (thumb) {
            thumb.style.transition = 'left 0.2s ease-out';
            thumb.style.left = `calc(50% - ${rotKnobWidth / 2}px)`; // Return to center
        }
        document.body.style.cursor = 'default';
        updateVelocities(velocities.x, velocities.y, 0); // Stop rotation
    };


    // Mouse events for Joystick X/Y
    const onJoyMouseDown = (e: MouseEvent) => handleJoyStart(e.clientX, e.clientY);
    // Mouse events for Rotation Joystick
    const onRotMouseDown = (e: MouseEvent) => handleRotStart(e.clientX);
    
    // Global mouse move and up for both joysticks
    const onGlobalMouseMove = (e: MouseEvent) => {
        if (isDraggingRef.current) handleJoyMove(e.clientX, e.clientY);
        if (isRotatingRef.current) handleRotMove(e.clientX);
    };
    const onGlobalMouseUp = () => {
        if (isDraggingRef.current) handleJoyEnd();
        if (isRotatingRef.current) handleRotEnd();
    };

    // Touch events for Joystick X/Y
    const onJoyTouchStart = (e: TouchEvent) => { e.preventDefault(); if (e.touches.length > 0) handleJoyStart(e.touches[0].clientX, e.touches[0].clientY); };
    // Touch events for Rotation Joystick
    const onRotTouchStart = (e: TouchEvent) => { e.preventDefault(); if (e.touches.length > 0) handleRotStart(e.touches[0].clientX); };

    // Global touch move and end
    const onGlobalTouchMove = (e: TouchEvent) => {
        if (e.touches.length > 0) {
            if (isDraggingRef.current) handleJoyMove(e.touches[0].clientX, e.touches[0].clientY);
            if (isRotatingRef.current) handleRotMove(e.touches[0].clientX);
        }
    };
    const onGlobalTouchEnd = () => {
        if (isDraggingRef.current) handleJoyEnd();
        if (isRotatingRef.current) handleRotEnd();
    };

    joyRef.addEventListener('mousedown', onJoyMouseDown);
    rotRef.addEventListener('mousedown', onRotMouseDown);
    document.addEventListener('mousemove', onGlobalMouseMove);
    document.addEventListener('mouseup', onGlobalMouseUp);

    joyRef.addEventListener('touchstart', onJoyTouchStart, { passive: false });
    rotRef.addEventListener('touchstart', onRotTouchStart, { passive: false });
    document.addEventListener('touchmove', onGlobalTouchMove, { passive: false });
    document.addEventListener('touchend', onGlobalTouchEnd);
    document.addEventListener('touchcancel', onGlobalTouchEnd);


    return () => {
      window.removeEventListener('resize', calculateJoyParams);
      joyRef.removeEventListener('mousedown', onJoyMouseDown);
      rotRef.removeEventListener('mousedown', onRotMouseDown);
      document.removeEventListener('mousemove', onGlobalMouseMove);
      document.removeEventListener('mouseup', onGlobalMouseUp);
      
      joyRef.removeEventListener('touchstart', onJoyTouchStart);
      rotRef.removeEventListener('touchstart', onRotTouchStart);
      document.removeEventListener('touchmove', onGlobalTouchMove);
      document.removeEventListener('touchend', onGlobalTouchEnd);
      document.removeEventListener('touchcancel', onGlobalTouchEnd);
      if (commandThrottleRef.current) clearTimeout(commandThrottleRef.current);
    };
  }, [updateVelocities, velocities.x, velocities.y, velocities.theta, maxSpeed, maxAngular]); // Dependencies
  // --- END: Joystick Control Logic ---

  useEffect(() => {
    // console.log(`[RobotControlWidget] Passive Listener - Selected Alias: ${selectedRobotId}, Resolved IP: ${currentSelectedIp}, LastMsg: `, lastJsonMessage ? lastJsonMessage.type : 'null');

    if (lastJsonMessage) {
      const message = lastJsonMessage as WebSocketMessage;
      
      if (message.robot_ip === currentSelectedIp || message.robot_alias === selectedRobotId) {
        switch (message.type) {
          case 'ack': {
            const cmd = (message as any).command;
            if (cmd === 'vector_control' || cmd === 'motor_speed' || cmd === 'set_pid' || cmd === 'emergency_stop') {
              setCommandSending(false);
              if (message.status === 'error') {
                setErrorMessage(message.message || 'Lệnh thất bại.');
              } else {
                setErrorMessage('');
              }
            }
            break;
          }
          case 'encoder_data':
            if (typeof message.rpm_1 === 'number' &&
                typeof message.rpm_2 === 'number' &&
                typeof message.rpm_3 === 'number') {
                setRpmValues({
                    1: message.rpm_1 ?? (rpmValues[1] || 0),
                    2: message.rpm_2 ?? (rpmValues[2] || 0),
                    3: message.rpm_3 ?? (rpmValues[3] || 0),
                });
            } 
            else if (message.data && Array.isArray(message.data) && message.data.length >=3) {
                const rpmList = message.data as number[];
                setRpmValues({
                    1: rpmList[0] ?? (rpmValues[1] || 0),
                    2: rpmList[1] ?? (rpmValues[2] || 0),
                    3: rpmList[2] ?? (rpmValues[3] || 0),
                });
             }
            break;
          case 'imu_data':
            if (message.data && typeof message.data === 'object') { 
                const imuPayload = message.data as any;
                setImuData(prev => ({ 
                ...prev,
                heading: imuPayload.euler?.[2] ?? prev.heading,
                pitch: imuPayload.euler?.[1] ?? prev.pitch,
                roll: imuPayload.euler?.[0] ?? prev.roll,
                calibrated: prev.calibrated 
                }));
            }
            break;
          case 'position_update':
            if (message.position && typeof message.position === 'object') {
              setRobotPosition({ 
                  x: message.position.x ?? robotPosition.x, 
                  y: message.position.y ?? robotPosition.y, 
                  theta: message.position.theta ?? robotPosition.theta 
                });
            }
            break;
          case 'bno_event': 
            if (message.event_type === 'calibration_complete' && message.status && typeof message.status === 'object') {
                const calStatus = message.status as any; 
                const isCalibrated = (calStatus.sys ?? 0) >= 2 && (calStatus.gyro ?? 0) >= 2 && (calStatus.accel ?? 0) >= 1 && (calStatus.mag ?? 0) >= 1;
                setImuData(prev => ({ ...prev, calibrated: isCalibrated }));
            }
            break;
          case 'command_response': 
            if (message.original_command === "send_to_robot" && 
                (message.robot_ip === currentSelectedIp || message.robot_alias === selectedRobotId)) {
              setCommandSending(false); 
              if (message.status === 'error') {
                  setErrorMessage(message.message || "Lệnh thất bại.");
                  // clear status text
              } else if (message.status === 'success' || message.status === 'sent_to_robot') {
                  setErrorMessage("");
              }
            }
            break;
          default: break;
        }
      }
    }
  }, [lastJsonMessage, selectedRobotId, connectedRobots, rpmValues, getSelectedRobotIp, currentSelectedIp, robotPosition.x, robotPosition.y, robotPosition.theta]);
  
  let statusTextForHeader = "";
  if (!webSocketIsConnected) {
    statusTextForHeader = "WebSocket: Ngắt kết nối";
  } else if (!selectedRobotId) {
    statusTextForHeader = "Vui lòng chọn robot";
  } else if (!currentSelectedIp) { 
      statusTextForHeader = `Đang tìm IP cho ${selectedRobotId}...`;
  } else { 
      const robotInfo = connectedRobots.find(r => r.alias === selectedRobotId);
      const displayName = robotInfo ? robotInfo.alias : selectedRobotId;
      statusTextForHeader = `Sẵn sàng (${displayName} @ ${currentSelectedIp})`;
      if(commandSending && !errorMessage) statusTextForHeader = "Đang gửi lệnh...";
  }
  const finalWidgetReady = webSocketIsConnected && !!currentSelectedIp;

  return (
    <div className="p-4 bg-white dark:bg-gray-800 text-gray-900 dark:text-gray-100 rounded-lg shadow-md h-full flex flex-col">
      <WidgetConnectionHeader
        title="Robot Control"
        isConnected={finalWidgetReady}
        error={errorMessage || webSocketError?.type}
        statusTextOverride={statusTextForHeader}
        hideConnectionControls={true}
      />
      
    {finalWidgetReady && (
      <div className="flex-shrink-0 grid grid-cols-2 gap-2 p-2 mb-3 border rounded-md bg-gray-50 dark:bg-gray-700 border-gray-200 dark:border-gray-600 text-xs">
        <div className="text-center">
          <div className="font-semibold text-gray-800 dark:text-gray-100">Position (X, Y, θ)</div>
          <div className="font-mono text-gray-900 dark:text-gray-100">{robotPosition.x.toFixed(2)}, {robotPosition.y.toFixed(2)}, {(robotPosition.theta * 180 / Math.PI).toFixed(1)}°</div>
        </div>
        <div className="text-center">
          <div className="font-semibold text-gray-800 dark:text-gray-100">IMU (Head, Pitch, Roll)</div>
          <div className="font-mono text-gray-900 dark:text-gray-100">{imuData.heading.toFixed(1)}°, {imuData.pitch.toFixed(1)}°, {imuData.roll.toFixed(1)}°</div>
        </div>
      </div>
    )}

      {!compact && (
        <div className="flex-shrink-0 flex justify-between mb-4 border-b border-gray-200 dark:border-gray-700">
          <div className="flex items-center gap-1 text-sm px-3 py-2">
            Điều khiển: Keyboard Only
          </div>
          <div className="flex items-center px-3 py-2 text-sm text-gray-600 dark:text-gray-300">
            Robot: {selectedRobotId || "Chưa chọn"}
            {currentSelectedIp && ` (${currentSelectedIp})`}
          </div>
        </div>
      )}
      
      {errorMessage && !webSocketError && (
        <div className="flex-shrink-0 bg-red-100 text-red-700 p-2 rounded-md text-sm mb-4">
          {errorMessage}
        </div>
      )}
      
      <div className="flex-grow overflow-y-auto">
      {activeTab === 'keyboard' && (
        <div className="mb-4">
          <div 
            ref={keyboardControlRef} 
              className={`p-4 border-2 rounded-lg mb-4 outline-none focus:border-blue-500 ${hasFocus ? 'border-blue-500' : 'border-gray-300'} ${!finalWidgetReady ? 'opacity-50 cursor-not-allowed' : ''}`}
              tabIndex={finalWidgetReady ? 0 : -1} 
              aria-label="Keyboard control area. Click or tab to activate."
              onClick={() => finalWidgetReady && keyboardControlRef.current?.focus()}
          >
            {!finalWidgetReady && (
                 <div className="text-center text-gray-400 mb-4 italic">
                    Vui lòng chọn robot để kích hoạt điều khiển.
                </div>
            )}
            {finalWidgetReady && !hasFocus && (
                <div className="text-center text-gray-500 mb-4 italic">
                  Click vào đây hoặc Tab để kích hoạt điều khiển bàn phím
              </div>
            )}
            
              <div className="grid grid-cols-3 gap-2 w-48 mx-auto mb-4">
              <div></div>
                 <div className={`p-3 rounded ${keysPressed.w ? 'bg-blue-600 text-white shadow-inner' : 'bg-gray-200 text-gray-700'}`}>
                    <ArrowUp size={20} className="mx-auto"/> <span className="text-xs font-mono">W</span>
                </div>
              <div></div>
                <div className={`p-3 rounded ${keysPressed.a ? 'bg-blue-600 text-white shadow-inner' : 'bg-gray-200 text-gray-700'}`}>
                    <ArrowLeft size={20} className="mx-auto"/> <span className="text-xs font-mono">A</span>
            </div>
                 <div className={`p-3 rounded ${keysPressed.s ? 'bg-blue-600 text-white shadow-inner' : 'bg-gray-200 text-gray-700'}`}>
                    <ArrowDown size={20} className="mx-auto"/> <span className="text-xs font-mono">S</span>
            </div>
                <div className={`p-3 rounded ${keysPressed.d ? 'bg-blue-600 text-white shadow-inner' : 'bg-gray-200 text-gray-700'}`}>
                    <ArrowRight size={20} className="mx-auto"/> <span className="text-xs font-mono">D</span>
              </div>
            </div>
               <div className="flex justify-center gap-4">
                  <div className={`p-3 rounded ${keysPressed.q ? 'bg-blue-600 text-white shadow-inner' : 'bg-gray-200 text-gray-700'}`}>
                      <RotateCounter size={20} className="mx-auto"/> <span className="text-xs font-mono">Q</span>
                  </div>
                  <div className={`p-3 rounded ${keysPressed.e ? 'bg-blue-600 text-white shadow-inner' : 'bg-gray-200 text-gray-700'}`}>
                      <RotateCw size={20} className="mx-auto"/> <span className="text-xs font-mono">E</span>
                  </div>
              </div>
        <div className="text-center mt-4 text-xs text-gray-500">
          W/S: Tiến/Lùi (X±), A/D: Trái/Phải (Y+/Y−), Q/E: Xoay Trái/Phải (θ+/θ−)
        </div>
          </div>
          {/* Velocity readouts removed per request */}
        </div>
      )}
      
      {!compact && !KEYBOARD_ONLY && activeTab === 'joystick' && (
        <div className={`mb-4 ${!finalWidgetReady ? 'opacity-50 cursor-not-allowed pointer-events-none' : ''}`}>
          <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
               <div className="p-4 border rounded-lg text-center">
                 <h4 className="font-medium mb-2">Điều Khiển Vị Trí (X/Y)</h4>
              <div 
                ref={joystickRef}
                   className="w-48 h-48 mx-auto bg-gray-200 rounded-full relative cursor-pointer touch-none border border-gray-300"
                 >
                    <div className="absolute inset-0 rounded-full flex items-center justify-center pointer-events-none">
                        <div className="w-px h-full bg-gray-400 opacity-50"></div>
                        <div className="h-px w-full bg-gray-400 opacity-50 absolute"></div>
                        <div className="w-24 h-24 rounded-full border border-gray-400 opacity-30"></div>
                </div>
                <div 
                  ref={knobRef} 
                     className="absolute w-10 h-10 bg-blue-500 rounded-full left-1/2 top-1/2 transform -translate-x-1/2 -translate-y-1/2 cursor-grab shadow-md border-2 border-white pointer-events-none"
                   />
                  </div>
                 <div className="mt-2 text-sm text-gray-600">
                   Kéo để điều khiển X/Y
                </div>
              </div>
                <div className="p-4 border rounded-lg text-center">
                    <h4 className="font-medium mb-2">Điều Khiển Xoay (Theta)</h4>
                    <div className="mt-8 mb-8">
              <div 
                ref={rotationKnobRef}
                            className="w-full h-4 bg-gray-200 rounded-full relative cursor-pointer touch-none border border-gray-300"
                        >
                            <div className="absolute top-1/2 left-1/2 w-px h-full bg-gray-400 -translate-y-1/2"></div> {/* Center line */}
                            <div 
                                className="thumb absolute top-1/2 transform -translate-y-1/2 -translate-x-1/2 w-6 h-6 bg-red-500 rounded-full shadow-md border-2 border-white cursor-grab pointer-events-none"
                                style={{ left: '50%' }} // Initial position at center
                            />
                </div>
                </div>
                    <div className="mt-2 text-sm text-gray-600">
                        Kéo ngang để xoay: {(velocities.theta * 180 / Math.PI).toFixed(1)}°/s
              </div>
              </div>
            </div>
          {/* Velocity readouts removed per request */}
        </div>
      )}
      
      {!compact && !KEYBOARD_ONLY && activeTab === 'motors' && (
        <div className="mb-4">
            <h3 className="text-lg font-semibold mb-2">Điều Khiển Từng Động Cơ</h3>
             <div className="space-y-2">
          {[1, 2, 3].map((motorId) => (
                     <div key={motorId} className="grid grid-cols-4 gap-4 items-center p-2 border rounded">
                         <div className="font-medium">Motor {motorId}</div>
                         <div className="flex items-center gap-2">
                             <input 
                                type="range" 
                                min="-255" 
                                max="255"
                                step="1"
                                value={motorSpeeds[motorId - 1]}
                                onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
                                  const speed = parseInt(e.target.value);
                                  // Cập nhật state ngay lập tức để slider di chuyển mượt
                                  const newSpeeds = [...motorSpeeds];
                                  newSpeeds[motorId - 1] = speed;
                                  setMotorSpeeds(newSpeeds);
                                  // Gửi lệnh (throttled)
                                  setMotorSpeed(motorId, speed);
                                }}
                                className="w-full h-2 bg-gray-200 rounded-lg appearance-none cursor-pointer"
                                disabled={!finalWidgetReady || commandSending}
                              />
              <input 
                type="number" 
                                value={motorSpeeds[motorId - 1]}
                onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
                  const newSpeeds = [...motorSpeeds];
                                  newSpeeds[motorId - 1] = parseInt(e.target.value) || 0;
                                  setMotorSpeeds(newSpeeds);
                                }}
                                onBlur={(e: React.FocusEvent<HTMLInputElement>) => {
                                     const speed = parseInt(e.target.value) || 0;
                                     setMotorSpeed(motorId, speed); // Gửi lệnh khi mất focus
                                }}
                                className="border rounded px-2 py-1 w-20 text-center"
                                disabled={!finalWidgetReady || commandSending}
                             />
                         </div>
              <button
                             onClick={() => setMotorSpeed(motorId, motorSpeeds[motorId - 1])} // Gửi lại giá trị hiện tại
                             disabled={!finalWidgetReady || commandSending}
                             className="bg-blue-500 hover:bg-blue-600 text-white px-3 py-1 rounded text-sm disabled:bg-gray-400 disabled:cursor-not-allowed">
                Set
              </button>
              <div>
                            <span className="font-mono text-sm">
                                {rpmValues[motorId]?.toFixed(1) ?? "N/A"} RPM 
                </span>
              </div>
            </div>
          ))}
            </div>
        </div>
      )}
          </div>
          
  {/* Emergency Stop button removed per request */}
    </div>
  );
};

export default RobotControlWidget;