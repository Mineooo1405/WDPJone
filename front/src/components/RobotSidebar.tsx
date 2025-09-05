import React, { useMemo, useState } from 'react';
import { useRobotContext, ReadyState } from './RobotContext';
import { X, Bot, RefreshCw, Search, LayoutGrid, Check, Pin, PinOff } from 'lucide-react';

interface RobotSidebarProps {
  open: boolean;
  onClose: () => void;
  widgets: { id: string; name: string; icon?: React.ReactNode }[];
  activeWidgetTypes: string[];
  onToggleWidget: (id: string) => void;
}

const RobotSidebar: React.FC<RobotSidebarProps> = ({ open, onClose, widgets, activeWidgetTypes, onToggleWidget }) => {
  const { connectedRobots, selectedRobotId, setSelectedRobotId, requestRobotListUpdate, readyState, pinnedRobotAliases, addPinnedRobot, removePinnedRobot } = useRobotContext();
  const [query, setQuery] = useState('');
  const [activeTab, setActiveTab] = useState<'robots' | 'widgets'>('robots');
  const [showOnlyPinned, setShowOnlyPinned] = useState(false);

  const filtered = useMemo(() => {
    if (!query) return connectedRobots;
    const q = query.toLowerCase();
    const base = connectedRobots.filter(r => r.alias.toLowerCase().includes(q) || (r.ip || '').toLowerCase().includes(q));
    return showOnlyPinned ? base.filter(r => pinnedRobotAliases.includes(r.alias)) : base;
  }, [connectedRobots, query, showOnlyPinned, pinnedRobotAliases]);

  if (!open) return null;

  return (
    <div className="fixed inset-0 z-50">
      {/* Backdrop */}
      <div className="absolute inset-0 bg-black/40" onClick={onClose} />

      {/* Panel */}
      <div className="absolute left-0 top-0 h-full w-80 max-w-[85vw] bg-white dark:bg-gray-800 border-r border-gray-200 dark:border-gray-700 shadow-2xl flex flex-col animate-[slideIn_.2s_ease-out]">
        {/* Header */}
        <div className="p-3 border-b border-gray-200 dark:border-gray-700 flex items-center gap-2">
          <Bot size={18} className="text-blue-600 dark:text-blue-300" />
          <div className="font-semibold text-gray-900 dark:text-gray-100">Bảng điều khiển</div>
          <button
            onClick={() => readyState === ReadyState.OPEN && requestRobotListUpdate()}
            title="Làm mới danh sách"
            className="ml-auto p-1.5 rounded-md text-gray-600 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-700 disabled:opacity-50"
            disabled={readyState !== ReadyState.OPEN}
          >
            <RefreshCw size={16} />
          </button>
          <button
            onClick={onClose}
            className="p-1.5 rounded-md text-gray-600 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-700"
            title="Đóng"
          >
            <X size={16} />
          </button>
        </div>

        {/* Tabs */}
        <div className="px-3 pt-2 flex gap-2">
          <button
            className={`flex-1 px-3 py-2 rounded-md text-sm font-medium border ${activeTab === 'robots' ? 'bg-blue-600 text-white border-blue-600' : 'bg-gray-100 dark:bg-gray-700 text-gray-800 dark:text-gray-200 border-gray-300 dark:border-gray-600'}`}
            onClick={() => setActiveTab('robots')}
          >
            Robots
          </button>
          <button
            className={`flex-1 px-3 py-2 rounded-md text-sm font-medium border ${activeTab === 'widgets' ? 'bg-blue-600 text-white border-blue-600' : 'bg-gray-100 dark:bg-gray-700 text-gray-800 dark:text-gray-200 border-gray-300 dark:border-gray-600'}`}
            onClick={() => setActiveTab('widgets')}
          >
            Widgets
          </button>
        </div>

        {/* Search + filters (robots tab only) */}
        {activeTab === 'robots' && (
          <div className="px-3 py-2 space-y-2">
            <div className="relative">
              <input
                value={query}
                onChange={(e) => setQuery(e.target.value)}
                placeholder="Tìm alias hoặc IP..."
                className="w-full px-3 py-2 pr-8 border border-gray-300 dark:border-gray-600 rounded-md bg-white dark:bg-gray-700 text-gray-900 dark:text-gray-100 text-sm"
              />
              <Search size={14} className="absolute right-2 top-1/2 -translate-y-1/2 text-gray-400" />
            </div>
            <div className="flex items-center gap-2">
              <button
                onClick={() => setShowOnlyPinned(p => !p)}
                className={`px-2 py-1 rounded border text-sm ${showOnlyPinned ? 'bg-amber-100 text-amber-700 border-amber-300' : 'bg-gray-100 dark:bg-gray-700 text-gray-700 dark:text-gray-200 border-gray-300 dark:border-gray-600'}`}
                title="Chỉ hiện robot đã ghim"
              >
                {showOnlyPinned ? 'Chỉ robot đã ghim' : 'Tất cả robot'}
              </button>
              {pinnedRobotAliases.length > 0 && (
                <div className="text-xs text-gray-500 dark:text-gray-400">Đã ghim: {pinnedRobotAliases.join(', ')}</div>
              )}
            </div>
          </div>
        )}

        {/* List */}
        <div className="flex-1 overflow-y-auto">
          {activeTab === 'robots' ? (
            filtered.length === 0 ? (
              <div className="h-full flex items-center justify-center text-sm text-gray-500 dark:text-gray-400 px-3 text-center">
                {connectedRobots.length === 0 ? 'Chưa phát hiện robot nào.' : 'Không tìm thấy robot phù hợp.'}
              </div>
            ) : (
              <ul className="divide-y divide-gray-200 dark:divide-gray-700">
                {filtered.map((r) => {
                  const isActive = r.alias === selectedRobotId;
                  return (
                    <li key={r.key}>
                      <div className={`w-full px-3 py-2 hover:bg-gray-100 dark:hover:bg-gray-700 ${isActive ? 'bg-blue-50 dark:bg-blue-900/40' : ''}`}>
                        <div className="flex items-center gap-2">
                          <button
                            className="flex-1 text-left"
                            onClick={() => { setSelectedRobotId(r.alias); onClose(); }}
                          >
                            <div className="flex items-center gap-2">
                              <span className={`inline-block w-2 h-2 rounded-full ${isActive ? 'bg-emerald-500' : 'bg-gray-400'}`} />
                              <span className={`font-medium ${isActive ? 'text-blue-700 dark:text-blue-200' : 'text-gray-800 dark:text-gray-100'}`}>{r.alias}</span>
                            </div>
                            <div className="text-xs text-gray-500 dark:text-gray-400 ml-4">{r.ip}</div>
                          </button>
                          <button
                            onClick={() => (pinnedRobotAliases.includes(r.alias) ? removePinnedRobot(r.alias) : addPinnedRobot(r.alias))}
                            className={`p-1.5 rounded-md border ${pinnedRobotAliases.includes(r.alias) ? 'bg-amber-100 text-amber-700 border-amber-300' : 'text-gray-600 dark:text-gray-300 border-gray-300 dark:border-gray-600 hover:bg-gray-100 dark:hover:bg-gray-600'}`}
                            title={pinnedRobotAliases.includes(r.alias) ? 'Bỏ ghim robot' : 'Ghim robot'}
                          >
                            {pinnedRobotAliases.includes(r.alias) ? <PinOff size={16} /> : <Pin size={16} />}
                          </button>
                        </div>
                      </div>
                    </li>
                  );
                })}
              </ul>
            )
          ) : (
            widgets.length === 0 ? (
              <div className="h-full flex items-center justify-center text-sm text-gray-500 dark:text-gray-400 px-3 text-center">
                Không có widget khả dụng.
              </div>
            ) : (
              <ul className="divide-y divide-gray-200 dark:divide-gray-700">
                {widgets.map((w) => {
                  const selected = activeWidgetTypes.includes(w.id);
                  return (
                    <li key={w.id}>
                      <button
                        className={`w-full px-3 py-2 flex items-center gap-3 hover:bg-gray-100 dark:hover:bg-gray-700 ${selected ? 'bg-emerald-50 dark:bg-emerald-900/30' : ''}`}
                        onClick={() => onToggleWidget(w.id)}
                        title={selected ? 'Bấm để ẩn widget' : 'Bấm để thêm widget'}
                      >
                        <div className={`w-7 h-7 rounded-md flex items-center justify-center ${selected ? 'bg-emerald-600 text-white' : 'bg-gray-200 dark:bg-gray-600 text-gray-700 dark:text-gray-200'}`}>
                          {selected ? <Check size={16} /> : (w.icon || <LayoutGrid size={16} />)}
                        </div>
                        <div className="flex-1 text-left">
                          <div className="font-medium text-gray-800 dark:text-gray-100">{w.name}</div>
                          <div className="text-xs text-gray-500 dark:text-gray-400">{selected ? 'Đang hiển thị' : 'Không hiển thị'}</div>
                        </div>
                      </button>
                    </li>
                  );
                })}
              </ul>
            )
          )}
        </div>

        {/* Footer */}
        <div className="p-3 text-xs text-gray-500 dark:text-gray-400 border-t border-gray-200 dark:border-gray-700">
          Robots: chọn robot để thao tác. Widgets: bấm để thêm/xóa widget trên màn hình.
        </div>
      </div>
    </div>
  );
};

export default RobotSidebar;

/*
@keyframes slideIn {
  from { transform: translateX(-100%); }
  to { transform: translateX(0); }
}
*/
