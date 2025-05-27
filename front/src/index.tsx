import React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';
import './index.css';
import 'tailwindcss/tailwind.css';
import { WebSocketProvider } from './contexts/WebSocketProvider';
import { DndProvider } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';

const rootElement = document.getElementById('root');

if (!rootElement) {
  throw new Error('Root element not found in HTML');
}

ReactDOM.createRoot(rootElement).render(
  <React.StrictMode>
    <WebSocketProvider>
      <DndProvider backend={HTML5Backend}>
        <App />
      </DndProvider>
    </WebSocketProvider>
  </React.StrictMode>
);