export interface Notification {
  id: number;
  s3_filename: string;
  file_url: string | null;
  timestamp: string;
  media_type: 'image' | 'clip' | 'sound' | 'face' | 'known_face';
  event_type: string | null;
  db_level?: number | null;
}

export type NotificationResponse = {
  notifications: Notification[];
};