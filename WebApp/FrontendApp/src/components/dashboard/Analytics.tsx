// src/components/dashboard/Analytics.tsx

import React, { useState, useEffect } from 'react';
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  Filler
} from 'chart.js';
import { Line } from 'react-chartjs-2';
import {
  Thermometer,
  Volume2,
  Notebook as Robot
} from 'lucide-react';
import { motion } from 'framer-motion';
import { AnalyticsService } from '../../services/analytics';
import { AnalyticsData } from '../../types/analytics';

ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
  Filler
);

const Analytics: React.FC = () => {
  const [robotId] = useState(() => localStorage.getItem('robotId') || '');
  const [analytics, setAnalytics] = useState<AnalyticsData | null>(() => {
    const saved = localStorage.getItem('analyticsData');
    return saved ? (JSON.parse(saved) as AnalyticsData) : null;
  });
  const [useFahrenheit, setUseFahrenheit] = useState(false);

  // Fetch + cache analytics
  const fetchAnalytics = async () => {
    if (!robotId) return;
    const data = await AnalyticsService.getAnalytics(Number(robotId));
    setAnalytics(data);
    localStorage.setItem('analyticsData', JSON.stringify(data));
  };
  useEffect(() => {
    fetchAnalytics();
  }, [robotId]);

  if (!robotId) {
    return (
      <div className="h-full p-6 flex items-center justify-center">
        <div className="text-center">
          <Robot className="h-16 w-16 text-gray-400 mx-auto mb-4" />
          <h3 className="text-lg font-medium text-gray-900 mb-2">
            No Robot Connected
          </h3>
          <p className="text-gray-500">
            Connect a robot to view analytics data.
          </p>
        </div>
      </div>
    );
  }

  // Convert C↔F
  const convertTemp = (c: number) =>
    useFahrenheit ? Math.round((c * 9) / 5 + 32) : Math.round(c);
  const unitLabel = useFahrenheit ? '°F' : '°C';

  // Summary values
  const avgTemp = convertTemp(analytics?.temperature.average ?? 0);
  const maxTemp = convertTemp(analytics?.temperature.max ?? 0);
  const minTemp = convertTemp(analytics?.temperature.min ?? 0);
  const soundCount = analytics?.sound.event_count ?? 0;

  // Temp chart data
  const tempReadings = analytics?.temperature.readings ?? [];
  const tempLabels = tempReadings.map(([ts]) =>
    new Date(ts).toLocaleTimeString()
  );
  const tempData = tempReadings.map(([, v]) => convertTemp(v));

  // Sound chart data + fake zeros
  const soundEvents = analytics?.sound.events ?? [];
  const rawLabels = soundEvents.map((e) =>
    new Date(e.timestamp).toLocaleTimeString()
  );
  const rawValues = soundEvents.map((e) => e.db_level);

  // add zero at start/end for a baseline
  const soundLabels = [' ', ...rawLabels, ' '];
  const soundValues = [0, ...rawValues, 0];

  const chartOptions = {
    responsive: true,
    plugins: { legend: { position: 'top' as const } },
    scales: { y: { beginAtZero: true } }
  };

  return (
    <motion.div
      className="h-full p-6 space-y-6"
      initial={{ opacity: 0 }}
      animate={{ opacity: 1 }}
    >
      <h1 className="text-2xl font-semibold text-gray-900">Analytics</h1>

      {/* black & white toggle */}
      <div className="flex justify-end">
        <button
          onClick={() => setUseFahrenheit(!useFahrenheit)}
          className="px-3 py-1 bg-black text-white rounded hover:opacity-90 text-sm"
        >
          Show in {useFahrenheit ? '°C' : '°F'}
        </button>
      </div>

      {/* Summary Cards */}
      <motion.div
        className="grid grid-cols-1 md:grid-cols-4 gap-4"
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ delay: 0.2 }}
      >
        {[
          {
            icon: Thermometer,
            label: 'Avg. Temperature',
            value: `${avgTemp}${unitLabel}`
          },
          {
            icon: Thermometer,
            label: 'Max Temperature',
            value: `${maxTemp}${unitLabel}`
          },
          {
            icon: Thermometer,
            label: 'Min Temperature',
            value: `${minTemp}${unitLabel}`
          },
          {
            icon: Volume2,
            label: 'Sound Events',
            value: soundCount.toString()
          }
        ].map(({ icon: Icon, label, value }) => (
          <motion.div
            key={label}
            className="bg-white rounded-lg p-6 shadow"
            whileHover={{ y: -2 }}
          >
            <div className="flex items-center justify-between">
              <div className="flex items-center">
                <Icon className="h-5 w-5 text-gray-400" />
                <span className="ml-2 text-sm text-gray-500">{label}</span>
              </div>
            </div>
            <p className="mt-2 text-2xl font-semibold text-gray-900">{value}</p>
          </motion.div>
        ))}
      </motion.div>

      {/* Charts */}
      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        <motion.div
          className="bg-white rounded-lg p-6 shadow"
          initial={{ opacity: 0, x: 20 }}
          animate={{ opacity: 1, x: 0 }}
          transition={{ delay: 0.4 }}
        >
          <h2 className="text-lg font-medium text-gray-900 mb-4">
            Temperature Readings
          </h2>
          <Line
            data={{
              labels: tempLabels,
              datasets: [
                {
                  label: `Temperature (${unitLabel})`,
                  data: tempData,
                  borderColor: 'rgb(239, 68, 68)',
                  backgroundColor: 'rgba(239, 68, 68, 0.1)',
                  fill: true,
                  tension: 0.4
                }
              ]
            }}
            options={chartOptions}
          />
        </motion.div>

        <motion.div
          className="bg-white rounded-lg p-6 shadow lg:col-span-2"
          initial={{ opacity: 0, y: 20 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.6 }}
        >
          <h2 className="text-lg font-medium text-gray-900 mb-4">
            Sound Anomaly Detection
          </h2>
          <Line
            data={{
              labels: soundLabels,
              datasets: [
                {
                  label: 'Sound Anomalies (dB)',
                  data: soundValues,
                  borderColor: 'rgb(59, 130, 246)',
                  backgroundColor: 'rgba(59, 130, 246, 0.1)',
                  fill: true,
                  tension: 0.4
                }
              ]
            }}
            options={chartOptions}
          />
        </motion.div>
      </div>
    </motion.div>
  );
};

export default Analytics;