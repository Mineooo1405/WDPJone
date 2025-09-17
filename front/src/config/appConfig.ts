// Cấu hình trung tâm cho ứng dụng Frontend
// - Dễ dàng điều chỉnh IP/Port WebSocket, tốc độ cập nhật biểu đồ, số điểm lịch sử, v.v.
// - Có thể ghi đè bằng biến môi trường (.env) khi build/run (REACT_APP_*)
// - Cung cấp giá trị mặc định an toàn khi thiếu biến môi trường

const toInt = (val: any, def: number) => {
  const n = parseInt(String(val), 10);
  return Number.isFinite(n) ? n : def;
};

// WebSocket URL (host/IP + port)
// Ưu tiên REACT_APP_WS_BRIDGE_URL (ví dụ: ws://192.168.1.10:9003)
// Nếu không có, mặc định ws://localhost:9003
const WS_URL = (import.meta as any).env?.REACT_APP_WS_BRIDGE_URL
  || (process as any).env?.REACT_APP_WS_BRIDGE_URL
  || 'ws://localhost:9003';

// Tách host và port từ WS_URL để hiển thị/ghi log thuận tiện
let wsHost = 'localhost';
let wsPort = 9003;
try {
  const u = new URL(WS_URL);
  wsHost = u.hostname || wsHost;
  wsPort = toInt(u.port || wsPort, wsPort);
} catch {}

export const appConfig = {
  // Kết nối WebSocket tới bridge (backend DirectBridge)
  wsUrl: WS_URL,
  wsHost,
  wsPort,

  // Cấu hình cho Encoder widget
  encoder: {
    // Khoảng thời gian đẩy dữ liệu từ buffer lên UI (ms)
    uiUpdateIntervalMs: toInt((import.meta as any).env?.REACT_APP_ENCODER_UI_INTERVAL_MS || (process as any).env?.REACT_APP_ENCODER_UI_INTERVAL_MS || 50, 50),
    // Số điểm lịch sử tối đa để vẽ biểu đồ
    maxHistoryPoints: toInt((import.meta as any).env?.REACT_APP_ENCODER_MAX_HISTORY || (process as any).env?.REACT_APP_ENCODER_MAX_HISTORY || 100, 100),
    // Cấu hình adaptive rate: tự điều chỉnh tốc độ cập nhật UI dựa trên tốc độ gói đến
    adaptive: {
      enabled: String((import.meta as any).env?.REACT_APP_ENCODER_ADAPTIVE_ENABLED || (process as any).env?.REACT_APP_ENCODER_ADAPTIVE_ENABLED || '1') !== '0',
      // Khoảng cách cập nhật nhỏ nhất và lớn nhất (ms)
      minIntervalMs: toInt((import.meta as any).env?.REACT_APP_ENCODER_ADAPTIVE_MIN_INTERVAL_MS || (process as any).env?.REACT_APP_ENCODER_ADAPTIVE_MIN_INTERVAL_MS || 30, 30),
      maxIntervalMs: toInt((import.meta as any).env?.REACT_APP_ENCODER_ADAPTIVE_MAX_INTERVAL_MS || (process as any).env?.REACT_APP_ENCODER_ADAPTIVE_MAX_INTERVAL_MS || 250, 250),
      // Mục tiêu số điểm flush mỗi lần (batch size mong muốn)
      targetBatch: toInt((import.meta as any).env?.REACT_APP_ENCODER_ADAPTIVE_TARGET_BATCH || (process as any).env?.REACT_APP_ENCODER_ADAPTIVE_TARGET_BATCH || 40, 40),
      // Ngưỡng decimation: nếu batch > threshold sẽ lấy mẫu thưa lại
      decimationThreshold: toInt((import.meta as any).env?.REACT_APP_ENCODER_DECIMATION_THRESHOLD || (process as any).env?.REACT_APP_ENCODER_DECIMATION_THRESHOLD || 400, 400),
      // Số phần tử tối đa giữ trong messageBuffer (hard cap)
      maxBuffer: toInt((import.meta as any).env?.REACT_APP_ENCODER_MAX_BUFFER || (process as any).env?.REACT_APP_ENCODER_MAX_BUFFER || 5000, 5000),
    },
  },

  // Cấu hình cho IMU widget
  imu: {
    // Khoảng thời gian cập nhật UI (ms)
    uiUpdateIntervalMs: toInt((import.meta as any).env?.REACT_APP_IMU_UI_INTERVAL_MS || (process as any).env?.REACT_APP_IMU_UI_INTERVAL_MS || 50, 50),
    // Số điểm lịch sử tối đa để vẽ biểu đồ
    maxHistoryPoints: toInt((import.meta as any).env?.REACT_APP_IMU_MAX_HISTORY || (process as any).env?.REACT_APP_IMU_MAX_HISTORY || 100, 100),
    adaptive: {
      enabled: String((import.meta as any).env?.REACT_APP_IMU_ADAPTIVE_ENABLED || (process as any).env?.REACT_APP_IMU_ADAPTIVE_ENABLED || '1') !== '0',
      minIntervalMs: toInt((import.meta as any).env?.REACT_APP_IMU_ADAPTIVE_MIN_INTERVAL_MS || (process as any).env?.REACT_APP_IMU_ADAPTIVE_MIN_INTERVAL_MS || 30, 30),
      maxIntervalMs: toInt((import.meta as any).env?.REACT_APP_IMU_ADAPTIVE_MAX_INTERVAL_MS || (process as any).env?.REACT_APP_IMU_ADAPTIVE_MAX_INTERVAL_MS || 250, 250),
      targetBatch: toInt((import.meta as any).env?.REACT_APP_IMU_ADAPTIVE_TARGET_BATCH || (process as any).env?.REACT_APP_IMU_ADAPTIVE_TARGET_BATCH || 40, 40),
      decimationThreshold: toInt((import.meta as any).env?.REACT_APP_IMU_DECIMATION_THRESHOLD || (process as any).env?.REACT_APP_IMU_DECIMATION_THRESHOLD || 400, 400),
      maxBuffer: toInt((import.meta as any).env?.REACT_APP_IMU_MAX_BUFFER || (process as any).env?.REACT_APP_IMU_MAX_BUFFER || 5000, 5000),
    },
  },

  // Cấu hình cho Trajectory widget
  trajectory: {
    // Số điểm đường đi tối đa hiển thị (giới hạn để đảm bảo hiệu năng)
    maxPathPointsDisplay: toInt((import.meta as any).env?.REACT_APP_TRAJ_MAX_POINTS || (process as any).env?.REACT_APP_TRAJ_MAX_POINTS || 500, 500),
  },

  // Công tắc tính năng
  features: {
    // Nếu bật (true), TrajectoryWidget sẽ đăng ký encoder_data để tự động nhận diện robot mecanum (4 bánh)
    // Chỉ ảnh hưởng tới hiển thị icon; đường đi vẫn vẽ theo position_update
    encoderTypeDetection: String((import.meta as any).env?.REACT_APP_ENCODER_TYPE_DETECT || (process as any).env?.REACT_APP_ENCODER_TYPE_DETECT || '1') !== '0',
  },
} as const;

export type AppConfig = typeof appConfig;
