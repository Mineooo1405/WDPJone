import React, { createContext, useContext, useState, ReactNode, useEffect, useCallback } from "react";
import useWebSocket, { ReadyState } from 'react-use-websocket';

// Define the WebSocket URL (align with start.py writes REACT_APP_WS_BRIDGE_URL)
const WEBSOCKET_URL = process.env.REACT_APP_WS_BRIDGE_URL || 'ws://localhost:9003';

// Define the structure for a robot entry
export interface ConnectedRobot {
  alias: string;
  ip: string;
  key: string; // unique_key (ip:port) from backend
  status?: string; // Optional status like "connected"
}

interface RobotContextType {
  selectedRobotId: string | null; // Can be null if no robot is selected
  setSelectedRobotId: (alias: string | null) => void;
  connectedRobots: ConnectedRobot[]; // List of available robots
  sendJsonMessage: (jsonMessage: any, keep?: boolean | undefined) => void; 
  lastJsonMessage: any; 
  readyState: ReadyState;
  requestRobotListUpdate: () => void; 
  pinnedRobotAliases: string[];
  addPinnedRobot: (alias: string) => void;
  removePinnedRobot: (alias: string) => void;
  clearPinnedRobots: () => void;
}

const RobotContext = createContext<RobotContextType>({
  selectedRobotId: null,
  setSelectedRobotId: () => {},
  connectedRobots: [],
  sendJsonMessage: () => console.warn('sendJsonMessage called outside of RobotProvider'),
  lastJsonMessage: null,
  readyState: ReadyState.UNINSTANTIATED,
  requestRobotListUpdate: () => {},
  pinnedRobotAliases: [],
  addPinnedRobot: () => {},
  removePinnedRobot: () => {},
  clearPinnedRobots: () => {},
});

export const useRobotContext = () => {
  const context = useContext(RobotContext);
  if (!context) {
    throw new Error("useRobotContext must be used within a RobotProvider");
  }
  return context;
};

export const RobotProvider: React.FC<{children: ReactNode}> = ({ children }) => {
  const [selectedRobotId, setSelectedRobotIdInternal] = useState<string | null>(null);
  const [connectedRobots, setConnectedRobotsState] = useState<ConnectedRobot[]>([]);
  const [pinnedRobotAliases, setPinnedRobotAliases] = useState<string[]>(() => {
    try {
      const multi = localStorage.getItem('pinnedRobotAliases');
      if (multi) {
        const parsed = JSON.parse(multi);
        return Array.isArray(parsed) ? parsed.filter((v: any) => typeof v === 'string') : [];
      }
      const single = localStorage.getItem('pinnedRobotAlias');
      if (single) return [single];
    } catch {}
    return [];
  });

  const {
    sendJsonMessage,
    lastJsonMessage,
    readyState,
  } = useWebSocket(WEBSOCKET_URL, {
    share: true,
    shouldReconnect: () => true,
    reconnectInterval: 3000,
    onOpen: () => {
      console.log('RobotContext: WebSocket Opened, requesting robot list.');
      sendJsonMessage({ command: 'get_available_robots' });
    },
    onClose: () => {
      console.log('RobotContext: WebSocket Closed.');
      setConnectedRobotsState([]);
      setSelectedRobotIdInternal(null);
    },
    onError: (event: Event) => {
      console.error('RobotContext: WebSocket Error:', event);
      setConnectedRobotsState([]);
      setSelectedRobotIdInternal(null);
    }
  });

  useEffect(() => {
    if (lastJsonMessage) {
      const message = lastJsonMessage as any;

      if ((message.type === 'available_robots_initial_list' || message.type === 'connected_robots_list' || message.type === 'initial_robot_list') && message.robots) {
        console.log(`RobotContext: Received ${message.type} from backend`, message.robots);
        const newRobots: ConnectedRobot[] = Array.isArray(message.robots) ? message.robots.map((r: any) => ({
          alias: r.alias || r.ip, // backend now uses IP as alias
          ip: r.ip || r.alias,
          key: r.key || r.unique_key || r.robot_id,
          status: r.status
        })) : [];

        setConnectedRobotsState(newRobots);

        if (newRobots.length > 0) {
          const currentSelectedStillValid = newRobots.some(robot => robot.alias === selectedRobotId);
          if (selectedRobotId && !currentSelectedStillValid) {
            console.log(`[RobotContext] Selected robot ${selectedRobotId} is no longer valid. Clearing.`);
            setSelectedRobotIdInternal(null);
          } else if (!selectedRobotId) {
            console.log(`[RobotContext] No robot selected. Auto-selecting: ${newRobots[0].alias}`);
            setSelectedRobotIdInternal(newRobots[0].alias);
          }
          // Cleanup any pins for robots that are no longer present
          setPinnedRobotAliases(prev => prev.filter(alias => newRobots.some(r => r.alias === alias)));
        } else {
          if (selectedRobotId !== null) {
            console.log("[RobotContext] Robot list is empty. Clearing selection.");
            setSelectedRobotIdInternal(null);
          }
        }
      } else if (message.type === 'available_robot_update' && message.robot) {
        console.log('RobotContext: Received available_robot_update from backend', message);
        const updatedRobotInfo: ConnectedRobot = {
            alias: message.robot.alias,
            ip: message.robot.ip,
            key: message.robot.unique_key,
            status: message.robot.status
        };

        if (message.action === 'add') {
          setConnectedRobotsState(prevRobots => {
            if (!prevRobots.find(r => r.key === updatedRobotInfo.key)) {
              return [...prevRobots, updatedRobotInfo];
            }
            return prevRobots.map(r => r.key === updatedRobotInfo.key ? updatedRobotInfo : r);
          });
          if (!selectedRobotId) {
             setSelectedRobotIdInternal(updatedRobotInfo.alias);
          }
          // If pinned alias was empty, optionally auto-pin first arrival? We won't auto-pin.
        } else if (message.action === 'remove') {
          setConnectedRobotsState(prevRobots => {
            const newRobots = prevRobots.filter(r => r.key !== updatedRobotInfo.key);
            if (selectedRobotId === updatedRobotInfo.alias) {
              console.log(`[RobotContext] Selected robot ${selectedRobotId} was removed via available_robot_update. Clearing selection.`);
              setSelectedRobotIdInternal(null);
            }
            setPinnedRobotAliases(prev => prev.filter(a => a !== updatedRobotInfo.alias));
            return newRobots;
          });
        }
      }
    }
  }, [lastJsonMessage, selectedRobotId]);

  const setSelectedRobotId = useCallback((alias: string | null) => {
    console.log("[RobotContext] setSelectedRobotId called with:", alias, "| Current:", selectedRobotId);
    if (selectedRobotId !== alias) {
      setSelectedRobotIdInternal(alias);
    }
  }, [selectedRobotId]);

  useEffect(() => {
    console.log("[RobotContext] Current selectedRobotId state is now:", selectedRobotId);
  }, [selectedRobotId]);

  const requestRobotListUpdate = useCallback(() => {
    if (readyState === ReadyState.OPEN) {
      console.log('RobotContext: Manually requesting robot list update.');
      sendJsonMessage({ command: 'get_available_robots' });
    } else {
      console.warn("Cannot request robot list: WebSocket not open.");
    }
  }, [readyState, sendJsonMessage]);

  // Pin management for multiple robots
  const addPinnedRobot = useCallback((alias: string) => {
    setPinnedRobotAliases(prev => prev.includes(alias) ? prev : [...prev, alias]);
  }, []);

  const removePinnedRobot = useCallback((alias: string) => {
    setPinnedRobotAliases(prev => prev.filter(a => a !== alias));
  }, []);

  const clearPinnedRobots = useCallback(() => {
    setPinnedRobotAliases([]);
  }, []);

  // Persist pins
  useEffect(() => {
    try { localStorage.setItem('pinnedRobotAliases', JSON.stringify(pinnedRobotAliases)); } catch {}
  }, [pinnedRobotAliases]);

  return (
    <RobotContext.Provider 
      value={{ 
        selectedRobotId, 
        setSelectedRobotId,
        connectedRobots, 
        sendJsonMessage,
        lastJsonMessage, 
        readyState,
        requestRobotListUpdate,
        pinnedRobotAliases,
        addPinnedRobot,
        removePinnedRobot,
        clearPinnedRobots,
      }}
    >
      {children}
    </RobotContext.Provider>
  );
};

// --- Utility functions for subscription (example) ---
// You might have these elsewhere
// let messageListeners: { [type: string]: { [id: string]: (message: any) => void } } = {};

// export const subscribeToMessageType = (type: string, callback: (message: any) => void, id: string): (() => void) => {
//     if (!messageListeners[type]) {
//         messageListeners[type] = {};
//     }
//     messageListeners[type][id] = callback;
//     return () => unsubscribeFromMessageType(type, id); // Return an unsubscribe function
// };

// export const unsubscribeFromMessageType = (type: string, id: string): void => {
//     if (messageListeners[type] && messageListeners[type][id]) {
//         delete messageListeners[type][id];
//         if (Object.keys(messageListeners[type]).length === 0) {
//             delete messageListeners[type];
//         }
//     }
// };

// // Need to call this function somewhere central when a message arrives from useWebSocket
// export const distributeMessage = (message: any) => {
//     if (message && message.type && messageListeners[message.type]) {
//         Object.values(messageListeners[message.type]).forEach(callback => {
//             try {
//                 callback(message);
//             } catch (error) {
//                 console.error(`Error in message listener for type ${message.type}:`, error);
//             }
//         });
//     }
// };

export { ReadyState };