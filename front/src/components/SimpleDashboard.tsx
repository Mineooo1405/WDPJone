import React from 'react';
import RobotControlWidget from './RobotControlWidget';
import TrajectoryWidget from './TrajectoryWidget';
import IMUWidget from './IMUWidget';
import EncoderDataWidget from './EncoderDataWidget';
import PIDControlWidget from './PIDControlWidget';
import FirmwareUpdateWidget from './FirmwareUpdateWidget';
import LogWidget from './LogWidget';

interface SimpleDashboardProps {
  className?: string;
  visibleWidgetTypes?: string[]; // controls which widgets are rendered
}

// Bố cục lưới đơn giản, tập trung thao tác chính
// Hàng 1: Trạng thái (trái lớn) | Điều khiển (phải)
// Hàng 2: Quỹ đạo (trái) | IMU + Encoder (phải, xếp dọc)
// Hàng 3: PID (trái) | OTA + Log (phải, xếp dọc)
const SimpleDashboard: React.FC<SimpleDashboardProps> = ({ className, visibleWidgetTypes }) => {
  const show = (id: string) => !visibleWidgetTypes || visibleWidgetTypes.includes(id);
  return (
    <div className={`w-full h-full p-4 overflow-auto ${className || ''}`}>
      <div className="grid grid-cols-12 gap-4 auto-rows-[minmax(220px,auto)]">
        {/* Hàng 1 */}
        {show('robot-control') && (
          <div className="col-span-4 row-span-2 bg-white dark:bg-gray-800 rounded-lg shadow p-3">
            <div className="h-full"><RobotControlWidget compact /></div>
          </div>
        )}
        {/* Hàng 2 */}
        {show('trajectory') && (
          <div className="col-span-8 row-span-5 bg-white dark:bg-gray-800 rounded-lg shadow p-3">
            <div className="h-full"><TrajectoryWidget compact /></div>
          </div>
        )}
        {(show('imu') || show('encoder-data')) && (
          <div className="col-span-4 row-span-5 flex flex-col gap-4">
            {show('imu') && (
              <div className="flex-1 bg-white dark:bg-gray-800 rounded-lg shadow p-3 overflow-hidden">
                <IMUWidget />
              </div>
            )}
            {show('encoder-data') && (
              <div className="flex-1 bg-white dark:bg-gray-800 rounded-lg shadow p-3 overflow-hidden">
                <EncoderDataWidget compact />
              </div>
            )}
          </div>
        )}

        {/* Hàng 3 */}
        {show('pid-control') && (
          <div className="col-span-8 row-span-2 bg-white dark:bg-gray-800 rounded-lg shadow p-3">
            <PIDControlWidget />
          </div>
        )}
        {/* Logs and Firmware as independent tiles */}
        {(() => {
          const showLogs = show('logs');
          const showFw = show('firmware-update');
          const both = showLogs && showFw;
          return (
            <>
              {showLogs && (
                <div className={`${both ? 'col-span-6' : 'col-span-4'} row-span-2`}>
                  <div className="min-w-0 w-full bg-white dark:bg-gray-800 rounded-lg shadow p-3 h-full">
                    <LogWidget />
                  </div>
                </div>
              )}
              {showFw && (
                <div className={`${both ? 'col-span-6' : 'col-span-4'} row-span-2`}>
                  <div className="min-w-0 w-full bg-white dark:bg-gray-800 rounded-lg shadow p-3 h-full">
                    <FirmwareUpdateWidget compact />
                  </div>
                </div>
              )}
            </>
          );
        })()}
      </div>
    </div>
  );
};

export default SimpleDashboard;


