import React from 'react';
import { Users } from 'lucide-react';

const FloatingRobotsButton: React.FC<{ onClick: () => void }> = ({ onClick }) => {
  return (
    <button
      onClick={onClick}
      title="Chọn Robot"
      className="fixed left-4 bottom-6 z-[60] h-12 w-12 rounded-full shadow-lg
                 bg-blue-600 hover:bg-blue-700 text-white
                 flex items-center justify-center
                 border border-blue-500/60
                 transition-colors"
      aria-label="Robots"
    >
      <Users size={22} />
    </button>
  );
};

export default FloatingRobotsButton;
