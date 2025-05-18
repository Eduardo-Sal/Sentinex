// src/pages/Dashboard.tsx

import React, {
  createContext,
  useContext,
  useEffect,
  useState
} from 'react';
import {
  Routes,
  Route,
  Link,
  useNavigate,
  useLocation
} from 'react-router-dom';
import {
  Home,
  Video,
  Bell,
  Settings as SettingsIcon,
  LogOut,
  Menu,
  X,
  User,
  BarChart3,
  Notebook as RobotIcon
} from 'lucide-react';
import { motion, AnimatePresence } from 'framer-motion';
import { useAuth } from '../context/AuthContext';
import LiveFeed from '../components/dashboard/LiveFeed';
import Alerts from '../components/dashboard/Alerts';
import Analytics from '../components/dashboard/Analytics';
import DashboardSettings from '../components/dashboard/DashboardSettings';
import DashboardOverview from '@/components/dashboard/DashboardOverview';
import { PubSub } from '@aws-amplify/pubsub';
import { useRobot } from '../hooks/useRobot';
import { AWSService } from '../services/aws-service';



export const MqttContext = createContext<any>(null);
export const useMqtt = () => useContext(MqttContext);

import { getCachedAwsMetadata } from '../utils/awsMetadata';

const Dashboard: React.FC = () => {
  const [isSidebarOpen, setIsSidebarOpen] = useState(true);
  const { logout, isAuthenticated, user } = useAuth();
  const navigate = useNavigate();
  const location = useLocation();

  const {
    robotId,
    isLoading: robotLoading,
    error: robotError,
    pairRobot,
    unpairRobot
  } = useRobot(user?.sub || '');

  const [robotInput, setRobotInput] = useState('');

  // MQTT caching logic
  const STORAGE_KEY = 'sensorData';
  const saved = localStorage.getItem(STORAGE_KEY);
  const [sensorData, setSensorData] = useState(
    saved
      ? JSON.parse(saved)
      : { temperature: null, battery: null, speed: null, location: null }
  );
  useEffect(() => {
    //const awsMetadata = await AWSService.getAwsInfo(localStorage.getItem())

  
    const sub = PubSub.subscribe("robot/data").subscribe({
      next: ({ value }) => {
        try {
          const payload = typeof value === 'string' ? JSON.parse(value) : value;
          const next = {
            temperature: payload.temperature ?? null,
            battery: payload.battery ?? null,
            speed: payload.speed ?? null,
            location: payload.location ?? null
          };
          setSensorData(next);
          localStorage.setItem(STORAGE_KEY, JSON.stringify(next));
        } catch (err) {
          console.error('MQTT parse error:', value, err);
        }
      },
      error: (err) => console.error('MQTT error:', err)
    });
  
    return () => sub.unsubscribe();
  }, [robotId]);

  // Auth redirect
  useEffect(() => {
    if (!isAuthenticated) navigate('/signin');
  }, [isAuthenticated, navigate]);

  // Persist sidebar
  useEffect(() => {
    const savedState = localStorage.getItem('sidebarOpen');
    if (savedState !== null) setIsSidebarOpen(JSON.parse(savedState));
  }, []);
  useEffect(() => {
    localStorage.setItem('sidebarOpen', JSON.stringify(isSidebarOpen));
  }, [isSidebarOpen]);

  const handleLogout = async () => {
    await logout();
    navigate('/signin');
  };

  const handleRobotConnect = async (e: React.FormEvent) => {
    e.preventDefault();
    try {
      const id = parseInt(robotInput, 10);
      if (isNaN(id) || id <= 0) throw new Error('Please enter a valid robot ID');
      await pairRobot(id);
      setRobotInput('');
    } catch {
      // robotError will show
    }
  };

  const navigation = [
    { name: 'Overview', icon: Home, path: '/dashboard' },
    { name: 'Live Feed', icon: Video, path: '/dashboard/live-feed' },
    { name: 'Alerts', icon: Bell, path: '/dashboard/alerts' },
    { name: 'Analytics', icon: BarChart3, path: '/dashboard/analytics' },
   // { name: 'Settings', icon: SettingsIcon, path: '/dashboard/settings' }
  ];
  const isActive = (path: string) => location.pathname === path;

  return (
    <MqttContext.Provider value={sensorData}>
      <div className="min-h-screen bg-gray-50">
        {/* Header */}
        <header className="fixed top-0 inset-x-0 h-16 bg-white z-30 border-b border-gray-200">
          <div className="h-full flex items-center justify-between px-4">
            <motion.button
              whileHover={{ scale: 1.1 }}
              whileTap={{ scale: 0.9 }}
              className="md:hidden text-gray-500 hover:text-gray-700"
              onClick={() => setIsSidebarOpen(!isSidebarOpen)}
            >
              {isSidebarOpen ? <X /> : <Menu />}
            </motion.button>
            <span className="text-lg font-semibold text-gray-900">
              Sentinex Console
            </span>
            <div className="flex items-center space-x-4">
              <div className="hidden md:flex items-center space-x-2 text-sm text-gray-600">
                <User /> <span>{user?.email}</span>
              </div>
              <motion.button whileHover={{ scale: 1.1 }} whileTap={{ scale: 0.9 }}>
                <Bell className="h-6 w-6 text-gray-500 hover:text-gray-700" />
              </motion.button>
            </div>
          </div>
        </header>

        {/* Sidebar */}
        <aside
          className={`
            fixed top-16 bottom-0 left-0 w-64 bg-white border-r border-gray-200
            transform transition-transform duration-300 ease-in-out z-20
            ${isSidebarOpen ? 'translate-x-0' : '-translate-x-full'} md:translate-x-0
          `}
        >
          <nav className="h-full flex flex-col">
            <div className="flex-1 overflow-y-auto py-4">
              {!robotId ? (
                <div className="px-4 py-4 border-b border-gray-200">
                  <h3 className="text-sm font-medium text-gray-700 mb-3">
                    Robot Connection
                  </h3>
                  <form onSubmit={handleRobotConnect} className="space-y-2">
                    <input
                      type="text"
                      value={robotInput}
                      onChange={(e) => setRobotInput(e.target.value)}
                      placeholder="Enter Robot ID"
                      className="w-full rounded-md border-gray-300 focus:border-black focus:ring-black text-black"
                    />
                    <button
                      type="submit"
                      disabled={robotLoading}
                      className="w-full bg-black text-white px-3 py-1 rounded-md disabled:opacity-50"
                    >
                      {robotLoading ? 'Connecting…' : 'Connect'}
                    </button>
                    {robotError && (
                      <p className="text-xs text-red-600">{robotError}</p>
                    )}
                  </form>
                </div>
              ) : (
                <div className="px-4 py-4 border-b border-gray-200 space-y-2">
                  <div className="flex items-center text-green-600 text-sm">
                    <RobotIcon className="mr-1" />
                    Connected to Robot #{robotId}
                  </div>
                  <button
                    onClick={unpairRobot}
                    className="w-full bg-gray-200 text-gray-700 px-3 py-1 rounded-md hover:bg-gray-300"
                  >
                    Unpair Robot
                  </button>
                </div>
              )}

              {navigation.map((item) => (
                <Link
                  key={item.name}
                  to={item.path}
                  className={`
                    flex items-center px-4 py-2 mx-2 text-sm font-medium rounded-md
                    ${isActive(item.path)
                      ? 'bg-gray-100 text-gray-900'
                      : 'text-gray-600 hover:bg-gray-50 hover:text-gray-900'}
                  `}
                >
                  <item.icon className="mr-3 h-5 w-5" />
                  {item.name}
                </Link>
              ))}
            </div>
            <div className="p-4 border-t border-gray-200">
              <motion.button
                whileHover={{ scale: 1.02 }}
                whileTap={{ scale: 0.98 }}
                onClick={handleLogout}
                className="flex items-center w-full text-sm text-gray-600 hover:bg-gray-50 px-3 py-2 rounded-md"
              >
                <LogOut className="mr-2" /> Sign out
              </motion.button>
            </div>
          </nav>
        </aside>

        {/* Main */}
        <main
          className={`pt-16 transition-all duration-300 ${
            isSidebarOpen ? 'md:pl-64' : ''
          }`}
        >
          <div className="p-6">
      {/* inside Dashboard.tsx, replace the entire AnimatePresence <>…</> with: */}
      <AnimatePresence mode="wait">
        {!robotId ? (
          <motion.div
        key="no-robot"
        className="text-center py-20"
        initial={{ opacity: 0 }}
        animate={{ opacity: 1 }}
        exit={{ opacity: 0 }}
      >
        <img 
          src="../../src/assets/blackroboticon.png" 
          alt="No robot paired" 
          className="h-60 w-60 mx-auto mb-4 object-contain" 
        />
        <h2 className="text-xl font-medium text-gray-900 mb-2">
          No robot paired
        </h2>
        <p className="text-gray-600 max-w-md mx-auto">
          Connect your robot via the sidebar to start monitoring your
          security system.
        </p>
      </motion.div>
  ) : (
    <Routes key="with-robot">
      {/* default “dashboard” = overview */}
      <Route
        index
        element={<DashboardOverview robotId={robotId} />}
      />

      {/* nested, **relative** paths */}
      <Route
        path="live-feed"
        element={<LiveFeed />}
      />
      <Route
        path="alerts"
        element={<Alerts />}
      />
      <Route
        path="analytics"
        element={<Analytics />}
      />
      {/*<Route
        path="settings"
        element={<DashboardSettings />}
      />

      {/* fallback to overview */}
      <Route
        path="*"
        element={<DashboardOverview robotId={robotId} />}
      />
    </Routes>
  )}
</AnimatePresence>
          </div>
        </main>
      </div>
    </MqttContext.Provider>
  );
};

export default Dashboard;