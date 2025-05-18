// src/components/dashboard/DashboardOverview.tsx

import React, { useState, useEffect } from 'react';
import {
  Bell,
  Camera,
  Volume2,
  User as UserIcon
} from 'lucide-react';
import { motion, Variants } from 'framer-motion';
import { useAuth } from '../../context/AuthContext';
import { useAwsInfo } from '../../hooks/useAWSINFO';

const listVariants: Variants = {
  hidden: {},
  visible: {
    transition: {
      staggerChildren: 0.12
    }
  }
};

const itemVariants: Variants = {
  hidden: { opacity: 0, x: -20 },
  visible: { opacity: 1, x: 0 }
};

const DashboardOverview: React.FC = () => {
  const [isRobotConnected] = useState(() => !!localStorage.getItem('robotId'));
  const [greeting, setGreeting] = useState('');
  const { user } = useAuth();
  const { awsInfo, loading: awsLoading, error: awsError } = useAwsInfo(user?.sub || '');

  // Turn raw events into UI-friendly list
  const recentEvents = (awsInfo?.recent_events || []).map((ev, idx) => {
    const time = new Date(ev.timestamp).toLocaleTimeString(undefined, {
      hour: 'numeric',
      minute: '2-digit',
      hour12: true
    });

    let description = 'New image alert';
    let Icon = Camera;

    if (ev.event_type === 'sound_anomaly') {
      description = `Sound anomaly detected at ${ev.db_level ?? '?'} dB`;
      Icon = Volume2;

    } else if (ev.event_type === 'human_detected') {
      description = 'Human detected';
      Icon = UserIcon;

    } else if (ev.event_type === 'face') {
      description = 'Unknown face detected';
      Icon = UserIcon;

    }
    else {
      description = `Known face detected: ${ev.event_type}`;
      Icon = UserIcon;
    }

    return { id: idx, time, description, Icon };
  });

  useEffect(() => {
    const h = new Date().getHours();
    if (h < 12) setGreeting('Good morning');
    else if (h < 18) setGreeting('Good afternoon');
    else setGreeting('Good evening');
  }, []);

  return (
    <motion.div
      className="space-y-6"
      initial={{ opacity: 0, y: 20 }}
      animate={{ opacity: 1, y: 0 }}
      exit={{ opacity: 0, y: -20 }}
      transition={{ duration: 0.3 }}
    >
      <h1 className="text-2xl font-semibold text-gray-900">Dashboard Overview</h1>

      {isRobotConnected ? (
        <>
          {/* Greeting */}
          <div className="bg-white shadow rounded-lg p-5 flex items-center justify-between">
            <div>
              <h2 className="text-xl font-medium text-gray-900">
                {greeting}, {user?.username || 'User'}!
              </h2>
              <p className="text-sm text-gray-600 mt-1">
                Here’s a quick look at your Sentinex system.
              </p>
            </div>
          </div>

          {/* Recent Events */}
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
            <div className="bg-white shadow rounded-lg p-5">
              <h3 className="text-lg font-medium text-gray-900 mb-3">
                Recent Events
              </h3>

              {awsLoading ? (
                <p className="text-gray-500">Loading events…</p>
              ) : awsError ? (
                <p className="text-red-500">Error loading events</p>
              ) : recentEvents.length > 0 ? (
                <motion.ul
                  variants={listVariants}
                  initial="hidden"
                  animate="visible"
                  className="divide-y divide-gray-200"
                >
                  {recentEvents.map(({ id, time, description, Icon }) => (
                    <motion.li
                      key={id}
                      variants={itemVariants}
                      className="py-3 flex items-center justify-between"
                    >
                      <div className="flex items-center space-x-2">
                        <div className="p-2 rounded-full bg-gray-100">
                          <Icon className="h-5 w-5 text-gray-600" />
                        </div>
                        <p className="text-sm text-gray-700">{description}</p>
                      </div>
                      <span className="text-xs text-gray-500">{time}</span>
                    </motion.li>
                  ))}
                </motion.ul>
              ) : (
                <p className="text-gray-500">No recent events.</p>
              )}
            </div>
          </div>
        </>
      ) : (
        <div className="text-center py-12">
          <img
            src="../../src/assets/blackroboticon.png"
            alt="Welcome"
            className="h-24 w-24 mx-auto mb-4 object-contain"
          />
          <h2 className="text-xl font-medium text-gray-900 mb-2">
            Welcome to Sentinex
          </h2>
          <p className="text-gray-600 max-w-md mx-auto">
            Connect your robot using the sidebar to start monitoring your security system.
          </p>
        </div>
      )}
    </motion.div>
  );
};

export default DashboardOverview;