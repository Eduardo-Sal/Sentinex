import { useState, useEffect, useCallback } from 'react';
import { Notification, NotificationResponse } from '../types/notification';
import { NotificationService } from '../services/notification-service';


// http://127.0.0.1:8000/api/notifications/users/(user_id)/notifications
// deleteUserNotifications


export const useNotifications = (userId: string) => {
  const [notifications, setNotifications] = useState<Notification[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isEnabled, setIsEnabled] = useState(false);

  // Only enable fetching if we're on a dashboard route
  useEffect(() => {
    const isDashboardRoute = window.location.pathname.startsWith('/dashboard');
    setIsEnabled(isDashboardRoute);
  }, []);

  const fetchNotifications = useCallback(async () => {
    if (!userId || !isEnabled) {
      return;
    }
    
    setIsLoading(true);
    setError(null);

    try {
      console.log('Fetching notifications for user:', userId);
      const response: NotificationResponse = await NotificationService.fetchNotifications(userId);
      const sortedNotifications = response.sort(
        (a, b) => new Date(b.timestamp).getTime() - new Date(a.timestamp).getTime()
      );
      setNotifications(sortedNotifications);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to fetch notifications';
      console.error('Notification fetch error:', errorMessage);
      setError(errorMessage);
      setNotifications([]);
    } finally {
      setIsLoading(false);
    }
  }, [userId, isEnabled]);

  const deleteNotifications = useCallback(async () =>{
    if(!userId){
      console.error(`User ID is required to delete notifications.`);
      return;
    }
    try {
      console.log();
      await NotificationService.deleteNotifications(userId);
      setNotifications([]);
    } catch(error){
      console.log();
    }
  }, [userId]);

  // Initial fetch
  useEffect(() => {
    if (isEnabled) {
      fetchNotifications();
    }
  }, [fetchNotifications, isEnabled]);

  // Refresh every 30 seconds if enabled
  useEffect(() => {
    if (!isEnabled) return;

    const interval = setInterval(fetchNotifications, 30000);
    return () => clearInterval(interval);
  }, [fetchNotifications, isEnabled]);

  return { notifications, isLoading, error, refetch: fetchNotifications, deleteNotifications};
};