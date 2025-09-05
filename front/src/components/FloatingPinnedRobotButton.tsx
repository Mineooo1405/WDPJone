import React from 'react';
import { Bot, PinOff } from 'lucide-react';
import { useRobotContext } from './RobotContext';

// A small floating chip that shows the pinned robot alias.
// - Click: select the pinned robot.
// - Unpin button: clear the pinned alias.
const FloatingPinnedRobotButton: React.FC = () => {
  const { pinnedRobotAliases, connectedRobots, selectedRobotId, setSelectedRobotId, removePinnedRobot } = useRobotContext();

  const pinsOnline = pinnedRobotAliases.filter(alias => connectedRobots.some(r => r.alias === alias));
  if (pinsOnline.length === 0) return null;

  return (
    <div className="fixed left-4 bottom-24 z-[60] flex flex-col items-start gap-2 select-none">
      {pinsOnline.map(alias => {
        const isSelected = selectedRobotId === alias;
        return (
          <div key={alias} className="flex items-center gap-2">
            <button
              onClick={() => setSelectedRobotId(alias)}
              title={isSelected ? 'Đang chọn robot đã ghim' : 'Chọn robot đã ghim'}
              className={`max-w-[60vw] pl-3 pr-2 py-2 rounded-full shadow-lg border flex items-center gap-2 transition-colors
                          ${isSelected ? 'bg-emerald-600 text-white border-emerald-500' : 'bg-white dark:bg-gray-800 text-gray-900 dark:text-gray-100 border-gray-200 dark:border-gray-700'}`}
              aria-label={`Pinned Robot ${alias}`}
            >
              <Bot size={18} className={isSelected ? 'opacity-100' : 'text-blue-600 dark:text-blue-300'} />
              <span className="font-medium truncate">{alias}</span>
            </button>
            <button
              onClick={() => removePinnedRobot(alias)}
              title="Bỏ ghim"
              className="h-9 w-9 rounded-full shadow-lg bg-amber-100 text-amber-800 border border-amber-300 flex items-center justify-center hover:bg-amber-200"
              aria-label={`Unpin ${alias}`}
            >
              <PinOff size={18} />
            </button>
          </div>
        );
      })}
    </div>
  );
};

export default FloatingPinnedRobotButton;
