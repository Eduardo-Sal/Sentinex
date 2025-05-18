// src/components/dashboard/Alerts.tsx

import React, { useState, useEffect } from 'react';
import {
  Bell,
  Camera,
  Eye,
  X,
  Clock,
  AlertCircle,
  Loader,
  RefreshCw,
  Volume2,
  Video,
  User,
} from 'lucide-react';
import { useAuth } from '../../context/AuthContext';
import { useNotifications } from '../../hooks/useNotifications';
import { useAwsInfo } from '@/hooks/useAWSINFO';
import NotificationMedia from './NotificationMedia';
import { motion, AnimatePresence } from 'framer-motion';

const Alerts: React.FC = () => {
  const [robotId] = useState(() => localStorage.getItem('robotId'));
  const { user } = useAuth();
  const [userId, setUserId] = useState<string>('');
  const { notifications, isLoading, error, refetch, deleteNotifications } =
    useNotifications(userId);
  const [modalImage, setModalImage] = useState<{
    url: string;
    id: number;
  } | null>(null);
  const { awsInfo, loading: awsLoading, error: awsError, refetchAwsInfo } =
    useAwsInfo(userId);

  useEffect(() => {
    if (user?.sub) {
      setUserId(user.sub);
    }
  }, [user]);

  if (!robotId) {
    return (
      <div className="h-full p-6 flex items-center justify-center">
        <div className="text-center">
          <Bell className="h-16 w-16 text-gray-400 mx-auto mb-4" />
          <h3 className="text-lg font-medium text-gray-900 mb-2">
            No Robot Connected
          </h3>
          <p className="text-gray-500">
            Connect a robot using the sidebar to view alerts.
          </p>
        </div>
      </div>
    );
  }

  const formatTimestamp = (timestamp: string) => {
    const date = new Date(timestamp);
    return new Intl.DateTimeFormat(undefined, {
      year: 'numeric',
      month: 'short',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit',
      hour12: true,
      timeZoneName: 'short',
    }).format(date);
  };

  return (
    <div className="h-full p-6 space-y-6">
      {/* Header */}
      <div className="flex justify-between items-center">
        <h1 className="text-2xl font-semibold text-gray-900">Alerts</h1>
        <div className="flex space-x-4">
          <button
            onClick={async () => {
              await deleteNotifications();
              await refetchAwsInfo();
            }}
            className="flex items-center space-x-2 px-4 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-black"
          >
            <span>Delete Notifications</span>
          </button>
          <button
            onClick={async () => {
              await refetch();
              await refetchAwsInfo();
            }}
            disabled={isLoading}
            className="flex items-center space-x-2 px-4 py-2 text-sm font-medium text-gray-700 bg-white border border-gray-300 rounded-md hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-offset-2 focus:ring-black"
          >
            {isLoading ? (
              <Loader className="h-4 w-4 animate-spin" />
            ) : (
              <RefreshCw className="h-4 w-4" />
            )}
            <span>Refresh</span>
          </button>
        </div>
      </div>

      {/* Notification List */}
      <div className="bg-white rounded-lg shadow">
        <div className="divide-y divide-gray-200">
          <AnimatePresence>
            {isLoading && notifications.length === 0 && (
              <motion.div
                key="loading"
                initial={{ opacity: 0 }}
                animate={{ opacity: 1 }}
                exit={{ opacity: 0 }}
                className="p-8 text-center"
              >
                <Loader className="h-12 w-12 animate-spin text-gray-400 mx-auto mb-4" />
                <p className="text-gray-500">Loading notifications...</p>
              </motion.div>
            )}

            {!isLoading && error && (
              <motion.div
                key="error"
                initial={{ opacity: 0 }}
                animate={{ opacity: 1 }}
                className="p-8 text-center"
              >
                <AlertCircle className="h-12 w-12 text-red-500 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-red-900 mb-2">
                  Error loading alerts
                </h3>
                <p className="text-red-500 mb-4">{error}</p>
                <button
                  onClick={() => refetch()}
                  className="text-sm font-medium text-black hover:text-gray-700"
                >
                  Try again
                </button>
              </motion.div>
            )}

            {!isLoading && !error && notifications.length === 0 && (
              <motion.div
                key="empty"
                initial={{ opacity: 0 }}
                animate={{ opacity: 1 }}
                className="p-8 text-center"
              >
                <Bell className="h-12 w-12 text-gray-400 mx-auto mb-4" />
                <h3 className="text-lg font-medium text-gray-900 mb-2">
                  No alerts found
                </h3>
                <p className="text-gray-500">
                  Alerts will appear here when detected.
                </p>
              </motion.div>
            )}

            {/* Each Notification */}
            {notifications
              .slice()
              .map((notification) => {
                let alertText: string;
                let IconComponent: React.FC<{ className?: string }>;

                switch (notification.media_type) {
                  case 'sound':
                    alertText = `Sound anomaly detected${
                      notification.db_level
                        ? ` at ${notification.db_level} dB`
                        : ''
                    }`;
                    IconComponent = Volume2;
                    break;
                  case 'image':
                    alertText = 'New image alert';
                    IconComponent = Camera;
                    break;
                  case 'clip':
                    alertText = 'New video alert';
                    IconComponent = Video;
                    break;
                  case 'face':
                    alertText = 'Unknown face detected';
                    IconComponent = User;
                    break;
                  case 'known_face':
                    alertText = `Known face detected: ${notification.event_type}`;
                    IconComponent = User;
                    break;
                  default:
                    alertText = 'New alert';
                    IconComponent = AlertCircle;
                }

                return (
                  <motion.div
                    key={notification.id}
                    initial={{ opacity: 0, y: -10 }}
                    animate={{ opacity: 1, y: 0 }}
                    exit={{ opacity: 0, y: 10 }}
                    transition={{ duration: 0.2 }}
                    className="p-4 hover:bg-gray-50 transition-colors"
                  >
                    <div className="flex items-center justify-between">
                      <div className="flex items-center space-x-4">
                        <div className="p-2 rounded-full bg-gray-100">
                          <IconComponent className="h-5 w-5 text-gray-600" />
                        </div>
                        <div>
                          <p className="text-sm font-medium text-gray-900">
                            {alertText}
                          </p>
                          <div className="flex items-center space-x-2 mt-1">
                            <span className="text-xs text-gray-500 flex items-center">
                              <Clock className="h-3 w-3 mr-1" />
                              {formatTimestamp(notification.timestamp)}
                            </span>
                            {notification.s3_filename.trim() !== '' && (
                              <span className="text-xs text-gray-500">
                                {notification.s3_filename
                                  .split('/')
                                  .pop()}
                              </span>
                            )}
                          </div>
                        </div>
                      </div>

                      {notification.file_url && (
                        <button
                          onClick={() =>
                            setModalImage(
                              modalImage?.id === notification.id
                                ? null
                                : {
                                    url: notification.file_url!,
                                    id: notification.id,
                                  }
                            )
                          }
                          className="p-1 text-gray-400 hover:text-gray-500 transition-colors"
                        >
                          {modalImage?.id === notification.id ? (
                            <X className="h-5 w-5" />
                          ) : (
                            <Eye className="h-5 w-5" />
                          )}
                        </button>
                      )}
                    </div>

                    {/* Thumbnail */}
                    {notification.file_url && (
                      <NotificationMedia
                        fileUrl={notification.file_url}
                        mediaType={notification.media_type}
                        className="mt-4 w-24 h-24 object-cover rounded-md"
                      />
                    )}
                  </motion.div>
                );
              })}
          </AnimatePresence>
        </div>
      </div>

      {/* Modal Lightbox */}
      <AnimatePresence>
        {modalImage && (
          <motion.div
            key="lightbox"
            className="fixed inset-0 z-50 flex items-center justify-center bg-black bg-opacity-75 p-4"
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
          >
            <motion.div
              className="relative max-w-full max-h-full"
              initial={{ scale: 0.8 }}
              animate={{ scale: 1 }}
              exit={{ scale: 0.8 }}
            >
              <button
                onClick={() => setModalImage(null)}
                className="absolute top-2 right-2 text-white p-1 rounded-full bg-black bg-opacity-50"
              >
                <X className="h-6 w-6" />
              </button>
              <img
                src={modalImage.url}
                className="max-w-full max-h-screen rounded-lg shadow-lg"
                alt="Expanded notification"
              />
            </motion.div>
          </motion.div>
        )}
      </AnimatePresence>
    </div>
  );
};

export default Alerts;