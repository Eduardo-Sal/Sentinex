import React from 'react';
import { useImageUrl } from '../../hooks/useImageUrl';

interface NotificationMediaProps {
  fileUrl?: string;
  mediaType?: 'image' | 'clip' | 'sound' | 'face' | 'known_face';
  className?: string;
}

const NotificationMedia: React.FC<NotificationMediaProps> = ({ fileUrl, mediaType, className = '' }) => {
  const { url, isLoading, error } = useImageUrl(fileUrl);

  if (!fileUrl || error) return null;
  if (isLoading) return <div className={`animate-pulse bg-gray-200 ${className}`} />;

  switch (mediaType) {
    case 'image':
      return <img src={url || ''} alt="Notification" className={`object-cover ${className}`} loading="lazy" />;
    case 'clip':
      return <video src={url || ''} controls className={`object-cover ${className}`} />;
    case 'sound':
      return <p className="text-gray-500">Sound event — no media to display</p>;
    case 'face':
      return <img src={url || ''} alt="Unknown face" className={`object-cover ${className}`} loading="lazy" />;
    case 'known_face':
      return <img src={url || ''} alt="Known face" className={`object-cover ${className}`} loading="lazy" />;
    default:
      return null;
  }
};

export default NotificationMedia;