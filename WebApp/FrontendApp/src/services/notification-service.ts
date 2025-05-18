import { API_CONFIG } from '../config/api-config';
import { NotificationResponse } from '../types/notification';

export const NotificationService = {
  async fetchNotifications(userId: string): Promise<NotificationResponse> {
    if (!userId) {
      throw new Error('User ID is required');
    }
  
    try {
      const url = `${API_CONFIG.BASE_URL}${API_CONFIG.ENDPOINTS.NOTIFICATIONS}/${userId}`;
      //console.log('Fetching notifications from:', url);
  
      const response = await fetch(url, {
        method: 'GET',
        headers: {
          ...API_CONFIG.HEADERS,
          'Accept': 'application/json'
        },
        mode: 'cors',
        credentials: 'include'
      });
  
      console.log('Response status:', response.status);
  
      if (!response.ok) {
        if (response.status === 404) {
          return []; // Return empty array for no notifications
        }
        throw new Error(`HTTP error! status: ${response.status}`);
      }
  
      const data = await response.json();
      console.log('Response data:', data);
  
      // Validate response format
      if (!Array.isArray(data.notifications)) {
        throw new Error('Invalid response format: expected notifications array');
      }
  
      const isValidNotification = (notification: any): boolean => {
        return (
          typeof notification.id === 'number' &&
          typeof notification.s3_filename === 'string' &&
          (typeof notification.file_url === 'string' || notification.file_url === null) &&
          typeof notification.timestamp === 'string' &&
          ['image', 'clip', 'sound', 'face', 'known_face'].includes(notification.media_type) &&
          (typeof notification.event_type === 'string' || notification.event_type === null)
        );
      };
  
      if (!data.notifications.every(isValidNotification)) {
        throw new Error('Invalid notification format in response');
      }
  
      return data.notifications;
    } catch (error) {
      console.error('Error fetching notifications:', error);
      throw error;
    }
  },
  async deleteNotifications(userId: string): Promise<void>{
    if (!userId){
    throw new Error("User ID is required");
    }
    try{
      const url = `${API_CONFIG.BASE_URL}/notifications/users/${userId}/notifications`;
      console.log('Deleting notifications for user:', userId)

      const response = await fetch(url,{
        method: 'DELETE',
        headers:{
          ... API_CONFIG.HEADERS,
          'Accept': 'applications/json'
        },
        mode: 'cors',
        credentials: 'include'
      });

      console.log('Delete response status:', response.status);

      if(!response.ok){
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      console.log(`Notifications for user ${userId} deleted successfuly.`);
    } catch (error){
      console.error(`Error deleting notifications:`,error);
      throw error
    }
  },
  async deleteUserNotification(notificationId: number): Promise<void>{
    if (!notificationId){
      throw new Error("Notification doesn't exist!");
    }
    try{
      const url =  `${API_CONFIG.BASE_URL}/notifications/notification/${notificationId}`; //input url later
      console.log(`Deleting notification ${notificationId}`)

      const response = await fetch(url,{
        method: 'DELETE',
        headers:{
          ... API_CONFIG.HEADERS,
          'Accept': 'applications/json'
        },
        mode: 'cors',
        credentials: 'include'
      });
      
      console.log("Delete Response Status:", response.status)
      if(!response.ok){
        throw new Error(`${response.status}`)
      }
      console.log(`Deleted successfully`)
    }catch (error){
      console.error(error)
      throw error
    }
  }
};