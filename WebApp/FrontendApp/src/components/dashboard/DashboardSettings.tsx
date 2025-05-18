// src/components/dashboard/DashboardSettings.tsx
import React, { useState, useEffect } from 'react';
import { 
  Settings as SettingsIcon,
  Bell,
  Wifi,
  Battery,
  Download,
  Save,
  User,
  Lock
} from 'lucide-react';
import { useAuth } from '../../context/AuthContext';
import { useRobot } from '../../hooks/useRobot';

const DashboardSettings = () => {
  const { user } = useAuth();
  const userUuid = user?.sub || '';

  // Robot hook
  const { robotId, unpairRobot } = useRobot(userUuid);

  const [isLoading, setIsLoading] = useState(true);
  const [isUnpairing, setIsUnpairing] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const [robotSettings, setRobotSettings] = useState({
    patrolSpeed: 1.5,
    batteryThreshold: 20,
    recordingQuality: 'high',
    motionSensitivity: 7
  });

  const [networkSettings, setNetworkSettings] = useState({
    wifiEnabled: true,
    autoConnect: true,
    dataSync: '15min'
  });

  const [notificationSettings, setNotificationSettings] = useState({
    emailAlerts: true,
    pushNotifications: true,
    soundAlerts: false,
    lowBatteryWarning: true,
    securityAlerts: true,
    maintenanceReminders: true
  });

  useEffect(() => {
    const timer = setTimeout(() => setIsLoading(false), 800);
    return () => clearTimeout(timer);
  }, []);

  const handleRobotSettingChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value, type } = e.target;
    setRobotSettings(prev => ({
      ...prev,
      [name]: type === 'number' ? parseFloat(value) : value
    }));
  };

  const handleNetworkSettingChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value, type } = e.target;
    setNetworkSettings(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? (e.target as HTMLInputElement).checked : value
    }));
  };

  const handleNotificationSettingChange = (e: React.ChangeEvent<HTMLInputElement>) => {
    const { name, checked } = e.target;
    setNotificationSettings(prev => ({
      ...prev,
      [name]: checked
    }));
  };

  const handleUnpair = async () => {
    setError(null);
    setIsUnpairing(true);
    try {
      await unpairRobot();
      // Optionally show toast or message
    } catch (err) {
      setError((err as Error).message);
    } finally {
      setIsUnpairing(false);
    }
  };

  if (isLoading) {
    return (
      <div className="h-full p-6 flex items-center justify-center">
        <div className="text-center">
          <div className="animate-spin rounded-full h-12 w-12 border-b-2 border-black mx-auto"></div>
          <p className="mt-4 text-gray-500">Loading settings...</p>
        </div>
      </div>
    );
  }

  return (
    <div className="h-full p-6 space-y-6">
      {/* Header */}
      <div className="flex justify-between items-center">
        <h1 className="text-2xl font-semibold text-gray-900">Settings</h1>
        <button className="bg-black text-white px-4 py-2 rounded-md flex items-center space-x-2">
          <Save className="h-4 w-4" />
          <span>Save Changes</span>
        </button>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Robot Configuration */}
        <div className="bg-white rounded-lg p-6 shadow">
          <div className="flex items-center space-x-2 mb-6">
            <SettingsIcon className="h-5 w-5 text-gray-400" />
            <h2 className="text-lg font-medium text-gray-900">Robot Configuration</h2>
          </div>

          {/* Inputs */}
          <div className="space-y-4">
            <div>
              <label className="block text-sm font-medium text-gray-700">Patrol Speed (m/s)</label>
              <input
                type="number"
                name="patrolSpeed"
                value={robotSettings.patrolSpeed}
                onChange={handleRobotSettingChange}
                step="0.1"
                min="0.5"
                max="3.0"
                className="mt-1 block w-full rounded-md border-gray-300 shadow-sm focus:border-black focus:ring-black sm:text-sm"
              />
            </div>
            <div>
              <label className="block text-sm font-medium text-gray-700">Low Battery Threshold (%)</label>
              <input
                type="number"
                name="batteryThreshold"
                value={robotSettings.batteryThreshold}
                onChange={handleRobotSettingChange}
                min="10"
                max="50"
                className="mt-1 block w-full rounded-md border-gray-300 shadow-sm focus:border-black focus:ring-black sm:text-sm"
              />
            </div>
            <div>
              <label className="block text-sm font-medium text-gray-700">Recording Quality</label>
              <select
                name="recordingQuality"
                value={robotSettings.recordingQuality}
                onChange={handleRobotSettingChange}
                className="mt-1 block w-full rounded-md border-gray-300 shadow-sm focus:border-black focus:ring-black sm:text-sm"
              >
                <option value="low">Low (720p)</option>
                <option value="medium">Medium (1080p)</option>
                <option value="high">High (4K)</option>
              </select>
            </div>
            <div>
              <label className="block text-sm font-medium text-gray-700">Motion Sensitivity (1-10)</label>
              <input
                type="range"
                name="motionSensitivity"
                value={robotSettings.motionSensitivity}
                onChange={handleRobotSettingChange}
                min="1"
                max="10"
                className="mt-1 block w-full"
              />
              <div className="flex justify-between text-xs text-gray-500">
                <span>Low</span>
                <span>High</span>
              </div>
            </div>
          </div>

        {/* Unpair Button */}
         { /*
          <div className="mt-6 flex items-center justify-end space-x-4">
            <span className="text-sm text-gray-600">
              {robotId ? `Connected: #${robotId}` : 'No robot paired'}
            </span>
            <button
              onClick={handleUnpair}
              disabled={!robotId || isUnpairing}
              className="bg-black text-white px-4 py-2 rounded-md flex items-center space-x-2 disabled:opacity-50"
            >
              <Lock className="h-4 w-4" />
              <span>{isUnpairing ? 'Unpairing…' : 'Unpair Robot'}</span>
            </button>
          </div>*/}
          {error && <p className="mt-2 text-sm text-red-600">{error}</p>}
        </div>

        {/* Network Settings */}
        <div className="bg-white rounded-lg p-6 shadow">
          <div className="flex items-center space-x-2 mb-6">
            <Wifi className="h-5 w-5 text-gray-400" />
            <h2 className="text-lg font-medium text-gray-900">Network Settings</h2>
          </div>
          <div className="space-y-4">
            <div className="flex items-center justify-between">
              <span className="text-sm font-medium text-gray-700">Enable Wi-Fi</span>
              <label className="relative inline-flex items-center cursor-pointer">
                <input
                  type="checkbox"
                  name="wifiEnabled"
                  checked={networkSettings.wifiEnabled}
                  onChange={handleNetworkSettingChange}
                  className="sr-only peer"
                />
                <div className="w-11 h-6 bg-gray-200 rounded-full peer-checked:bg-black after:absolute after:top-1 after:left-1 after:bg-white after:border-gray-300 after:border after:rounded-full after:h-4 after:w-4 after:transition-all peer-checked:after:translate-x-5"></div>
              </label>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-sm font-medium text-gray-700">Auto-Connect</span>
              <label className="relative inline-flex items-center cursor-pointer">
                <input
                  type="checkbox"
                  name="autoConnect"
                  checked={networkSettings.autoConnect}
                  onChange={handleNetworkSettingChange}
                  className="sr-only peer"
                />
                <div className="w-11 h-6 bg-gray-200 rounded-full peer-checked:bg-black after:absolute after:top-1 after:left-1 after:bg-white after:border-gray-300 after:border after:rounded-full after:h-4 after:w-4 after:transition-all peer-checked:after:translate-x-5"></div>
              </label>
            </div>
            <div>
              <label className="block text-sm font-medium text-gray-700">Data Sync Interval</label>
              <select
                name="dataSync"
                value={networkSettings.dataSync}
                onChange={handleNetworkSettingChange}
                className="mt-1 block w-full rounded-md border-gray-300 shadow-sm focus:border-black focus:ring-black sm:text-sm"
              >
                <option value="5min">Every 5 minutes</option>
                <option value="15min">Every 15 minutes</option>
                <option value="30min">Every 30 minutes</option>
                <option value="1hour">Every hour</option>
              </select>
            </div>
          </div>
        </div>

        {/* Notification Preferences */}
        <div className="bg-white rounded-lg p-6 shadow">
          <div className="flex items-center space-x-2 mb-6">
            <Bell className="h-5 w-5 text-gray-400" />
            <h2 className="text-lg font-medium text-gray-900">Notification Preferences</h2>
          </div>
          <div className="space-y-4">
            {Object.entries(notificationSettings).map(([key, value]) => (
              <div key={key} className="flex items-center justify-between">
                <span className="text-sm font-medium text-gray-700">
                  {key.split(/(?=[A-Z])/).join(' ')}
                </span>
                <label className="relative inline-flex items-center cursor-pointer">
                  <input
                    type="checkbox"
                    name={key}
                    checked={value}
                    onChange={handleNotificationSettingChange}
                    className="sr-only peer"
                  />
                  <div className="w-11 h-6 bg-gray-200 rounded-full peer-checked:bg-black after:absolute after:top-1 after:left-1 after:bg-white after:border-gray-300 after:border after:rounded-full after:h-4 after:w-4 after:transition-all peer-checked:after:translate-x-5"></div>
                </label>
              </div>
            ))}
          </div>
        </div>

        {/* System Maintenance */}
        <div className="bg-white rounded-lg p-6 shadow">
          <div className="flex items-center space-x-2 mb-6">
            <SettingsIcon className="h-5 w-5 text-gray-400" />
            <h2 className="text-lg font-medium text-gray-900">System Maintenance</h2>
          </div>
          <div className="space-y-4">
            <button className="w-full flex items-center justify-between p-3 text-left text-sm font-medium text-gray-700 bg-gray-50 rounded-md hover:bg-gray-100">
              <div className="flex items-center">
                <Download className="h-5 w-5 mr-2" />
                <span>Check for Updates</span>
              </div>
              <span className="text-xs text-gray-500">v2.1.0</span>
            </button>
            <button className="w-full flex items-center justify-between p-3 text-left text-sm font-medium text-gray-700 bg-gray-50 rounded-md hover:bg-gray-100">
              <div className="flex items-center">
                <Battery className="h-5 w-5 mr-2" />
                <span>Battery Calibration</span>
              </div>
              <span className="text-xs text-gray-500">30 days ago</span>
            </button>
            <button className="w-full flex items-center justify-between p-3 text-left text-sm font-medium text-gray-700 bg-gray-50 rounded-md hover:bg-gray-100">
              <div className="flex items-center">
                <Lock className="h-5 w-5 mr-2" />
                <span>Security Scan</span>
              </div>
              <span className="text-xs text-gray-500">2 days ago</span>
            </button>
            <button className="w-full flex items-center justify-between p-3 text-left text-sm font-medium text-gray-700 bg-gray-50 rounded-md hover:bg-gray-100">
              <div className="flex items-center">
                <User className="h-5 w-5 mr-2" />
                <span>User Access Control</span>
              </div>
              <span className="text-xs text-gray-500">3 active</span>
            </button>
          </div>
        </div>
      </div>
    </div>
  );
};

export default DashboardSettings;