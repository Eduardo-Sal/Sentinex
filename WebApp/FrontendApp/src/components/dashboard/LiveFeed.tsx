// src/components/dashboard/LiveFeed.tsx
import React, { useState, useEffect, useRef, useCallback } from "react";
import { io, Socket } from "socket.io-client";
import { motion, AnimatePresence } from "framer-motion";
import {
  Camera as CameraIcon,
  AlertOctagon,
  Maximize2,
  TvMinimal as Monitor,
} from "lucide-react";
import { PubSub } from "@aws-amplify/pubsub";
import Controller from "../DPAD/Controller";
import ControllerCamera from "../DPAD/ControllerCamera";
import { getCachedAwsMetadata } from "../../utils/awsMetadata";

const WS_URL = "http://129.113.224.96:5001";  // <-- use your Nano’s IP, MUST be http://

const LiveFeed: React.FC = () => {
  const [robotId] = useState(() => localStorage.getItem("robotId"));
  const [showNameModal, setShowNameModal] = useState(false);
  const [newName, setNewName] = useState("");
  const [toastMessage, setToastMessage] = useState("");
  const [toastType, setToastType] = useState<"success" | "error">("success");
  const [isControlMode, setIsControlMode] = useState(false);
  const [showControl, setShowControl] = useState(false);

  const imgRef = useRef<HTMLImageElement>(null);
  const socketRef = useRef<Socket>();
  const containerRef = useRef<HTMLDivElement>(null);
  const awsMetadata = getCachedAwsMetadata();

  // — connect to MJPEG stream over plain ws://
  useEffect(() => {
    const sock = io(WS_URL, {
      transports: ["websocket"],
      secure: false,             // force HTTP / ws://
      reconnectionAttempts: 5,
      timeout: 2000,
    });
    socketRef.current = sock;

    sock.on("connect", () => console.log("🔌 Socket.IO connected", sock.id));
    sock.on("frame", (jpg: string) => {
      if (imgRef.current) imgRef.current.src = "data:image/jpeg;base64," + jpg;
    });
    sock.on("disconnect", (r) => console.warn("🔌 disconnected", r));
    sock.on("error", (e) => console.error("❌ socket error", e));

    return () => { sock.disconnect(); };
  }, []);

  if (!robotId) {
    return (
      <div className="h-full p-6 flex items-center justify-center">
        <div className="text-center">
          <Monitor className="h-16 w-16 text-gray-400 mx-auto mb-4" />
          <h3 className="text-lg font-medium text-gray-900 mb-2">No Robot Connected</h3>
          <p className="text-gray-500">Connect a robot using the sidebar to view Live Feed.</p>
        </div>
      </div>
    );
  }

  const debounce = <T extends (...args: any[]) => void>(fn: T, ms: number) => {
    let id: ReturnType<typeof setTimeout>;
    return ((...args: any[]) => {
      clearTimeout(id);
      id = setTimeout(() => fn(...args), ms);
    }) as T;
  };
  const handleAuto = useCallback(
    debounce(() => {
      PubSub.publish(awsMetadata.mode, { number: "1", enabled: true }).catch(console.error);
    }, 300),
    []
  );

  const handleControl = () => {
    const next = isControlMode ? "1" : "2";
    PubSub.publish(awsMetadata.mode, { number: next, enabled: true }).catch(console.error);
    setIsControlMode(!isControlMode);
    setShowControl(!isControlMode);
  };
  const resetCamera = () => PubSub.publish(awsMetadata.camera, { mode: "reset" }).catch(console.error);
  const snap = () => PubSub.publish(awsMetadata.camera, { mode: "snapshot" }).catch(console.error);
  const regFace = () => setShowNameModal(true);

  const saveName = async () => {
    if (!newName.trim()) return;
    setShowNameModal(false);
    try {
      await PubSub.publish(awsMetadata.camera, { mode: "captureFace", name: newName.trim() });
      setToastType("success"); setToastMessage("Face registration command sent!");
    } catch {
      setToastType("error"); setToastMessage("Registration failed");
    }
    setTimeout(() => setToastMessage(""), 3000);
  };

  useEffect(() => {
    const handler = (e: KeyboardEvent) => {
      switch (e.key) {
        case "ArrowUp": PubSub.publish(awsMetadata.direction, { command: "up" }); break;
        case "ArrowDown": PubSub.publish(awsMetadata.direction, { command: "down" }); break;
        case "ArrowLeft": PubSub.publish(awsMetadata.direction, { command: "left" }); break;
        case "ArrowRight": PubSub.publish(awsMetadata.direction, { command: "right" }); break;
        case "a": case "A": PubSub.publish(awsMetadata.camera, { mode: "left" }); break;
        case "d": case "D": PubSub.publish(awsMetadata.camera, { mode: "right" }); break;
        case "r": case "R": resetCamera(); break;
      }
    };
    window.addEventListener("keydown", handler);
    return () => window.removeEventListener("keydown", handler);
  }, []);

  // — speed up/down handlers —
  const incSpeed = useCallback(
    () => PubSub.publish("robot-4/speed", { command: "increment" }).catch(console.error)
  );
  const decSpeed = useCallback(
    () => PubSub.publish("robot-4/speed", { command: "decrement" }).catch(console.error)
  );

  return (
    <>
      {/* Face modal */}
      {showNameModal && (
        <div className="fixed inset-0 bg-black bg-opacity-60 flex items-center justify-center z-50">
          <div className="bg-white p-6 rounded-lg shadow-lg w-full max-w-xs">
            <h2 className="text-lg font-semibold mb-4 text-black">Register Face</h2>
            <input
              type="text"
              value={newName}
              onChange={e => setNewName(e.target.value)}
              placeholder="Name"
              className="w-full p-2 border rounded mb-4 text-black"
            />
            <div className="flex justify-end space-x-2">
              <button onClick={() => setShowNameModal(false)} className="px-4 py-2 bg-black text-white rounded">Cancel</button>
              <button onClick={saveName} className="px-4 py-2 bg-black text-white rounded">Save</button>
            </div>
          </div>
        </div>
      )}

      <div className="h-full flex flex-col p-6 space-y-6">
        {/* Header */}
        <div className="flex justify-between items-center">
          <h1 className="text-2xl font-semibold">Live Feed</h1>
          <motion.button
            onClick={resetCamera}
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
            className="flex items-center space-x-2 bg-black text-white px-4 py-2 rounded"
          >
            <AlertOctagon className="h-5 w-5" /><span>Reset Camera</span>
          </motion.button>
        </div>

        {/* MJPEG Stream */}
        <div ref={containerRef} className="relative w-full aspect-video bg-black rounded-lg overflow-hidden">
          <img ref={imgRef} alt="Live MJPEG" className="absolute inset-0 w-full h-full object-cover" />

          <motion.button
            onClick={() => {
              if (!containerRef.current) return;
              document.fullscreenElement
                ? document.exitFullscreen()
                : containerRef.current.requestFullscreen();
            }}
            whileHover={{ scale: 1.05 }}
            whileTap={{ scale: 0.95 }}
            className="absolute bottom-4 right-4 p-3 rounded bg-white shadow z-20"
          >
            <Maximize2 className="h-5 w-5 text-black" />
          </motion.button>

          {showControl && (
            <div className="absolute inset-0 flex items-center justify-between px-4 z-10">
              <ControllerCamera />
              <Controller />
            </div>
          )}
        </div>

        {/* Five flat buttons */}
        <motion.div className="grid grid-cols-5 gap-4" initial={{ opacity: 0, y: 20 }} animate={{ opacity: 1, y: 0 }} transition={{ duration: 0.5, delay: 0.3 }}>
          <button
            onClick={handleControl}
            className={`py-3 rounded-lg text-white font-medium ${
              isControlMode ? "bg-red-600 hover:bg-red-700" : "bg-green-600 hover:bg-green-700"
            }`}
          >
            {isControlMode ? "Stop Control" : "Control Robot"}
          </button>

          <button
            onClick={handleAuto}
            disabled={isControlMode}
            className="py-3 rounded-lg bg-blue-600 text-white font-medium hover:bg-blue-700 disabled:opacity-50"
          >
            Autonomous
          </button>

          <button
            onClick={regFace}
            className="py-3 rounded-lg bg-black text-white font-medium hover:bg-gray-700"
          >
            Register Face
          </button>

          <button onClick={incSpeed} className="py-3 rounded-lg bg-black text-white">+ Speed</button>
          <button onClick={decSpeed} className="py-3 rounded-lg bg-black text-white">– Speed</button>
        </motion.div>
      </div>

      {/* Toast */}
      <AnimatePresence>
        {toastMessage && (
          <motion.div initial={{ opacity: 0, y: 20 }} animate={{ opacity: 1, y: 0 }} exit={{ opacity: 0, y: 20 }} transition={{ duration: 0.3 }}
            className={`fixed bottom-4 right-4 px-4 py-2 rounded shadow text-white ${
              toastType === "success" ? "bg-green-500" : "bg-red-500"
            }`}
          >
            {toastMessage}
          </motion.div>
        )}
      </AnimatePresence>
    </>
  );
};

export default LiveFeed;