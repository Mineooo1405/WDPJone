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
}

// Bố cục lưới đơn giản, tập trung thao tác chính
// Hàng 1: Trạng thái (trái lớn) | Điều khiển (phải)
// Hàng 2: Quỹ đạo (trái) | IMU + Encoder (phải, xếp dọc)
// Hàng 3: PID (trái) | OTA + Log (phải, xếp dọc)
const SimpleDashboard: React.FC<SimpleDashboardProps> = ({ className }) => {
  return (
    <div className={`w-full h-full p-4 overflow-auto ${className || ''}`}>
      <div className="grid grid-cols-12 gap-4 auto-rows-[minmax(220px,auto)]">
        {/* Hàng 1 */}
        <div className="col-span-4 row-span-2 bg-white rounded-lg shadow p-3">
          <div className="h-full"><RobotControlWidget compact /></div>
        </div>
        {/* Hàng 2 */}
        <div className="col-span-8 row-span-5 bg-white rounded-lg shadow p-3">
          <div className="h-full"><TrajectoryWidget compact /></div>
        </div>
        <div className="col-span-4 row-span-5 flex flex-col gap-4">
          <div className="flex-1 bg-white rounded-lg shadow p-3 overflow-hidden">
            <IMUWidget compact />
          </div>
          <div className="flex-1 bg-white rounded-lg shadow p-3 overflow-hidden">
            <EncoderDataWidget compact />
          </div>
        </div>

        {/* Hàng 3 */}
        <div className="col-span-8 row-span-2 bg-white rounded-lg shadow p-3">
          <PIDControlWidget />
        </div>
        <div className="col-span-4 row-span-2 flex flex-col gap-4">
          <div className="bg-white rounded-lg shadow p-3">
            <FirmwareUpdateWidget compact />
          </div>
          <div className="bg-white rounded-lg shadow p-3">
            <LogWidget />
          </div>
        </div>
      </div>
    </div>
  );
};

export default SimpleDashboard;


