import React, { useState } from 'react';
import { Camera, X, Eye, AlertTriangle } from 'lucide-react';

interface AlertNotificationProps {
  timestamp: string;
  confidence: number;
  imageUrl: string;
  onDismiss: () => void;
  onView: () => void;
}

const AlertNotification: React.FC<AlertNotificationProps> = ({
  timestamp,
  confidence,
  imageUrl,
  onDismiss,
  onView
}) => {
  const [imageError, setImageError] = useState(false);

  return (
    <div className="bg-white rounded-lg shadow-lg overflow-hidden animate-fade-in">
      {/* Header */}
      <div className="bg-red-600 px-4 py-3 flex items-center justify-between">
        <div className="flex items-center space-x-2">
          <Camera className="h-5 w-5 text-white" />
          <div>
            <h3 className="text-white font-medium">Human Detected</h3>
            <p className="text-red-100 text-sm">{new Date(timestamp).toLocaleString()}</p>
          </div>
        </div>
        <div className="flex items-center">
          <span className="text-red-100 text-sm mr-4">
            Confidence: {confidence}%
          </span>
          <button
            onClick={onDismiss}
            className="text-white hover:text-red-100 transition-colors"
          >
            <X className="h-5 w-5" />
          </button>
        </div>
      </div>

      {/* Image */}
      <div className="relative">
        {imageError ? (
          <div className="h-60 bg-gray-100 flex items-center justify-center">
            <div className="text-center">
              <AlertTriangle className="h-8 w-8 text-gray-400 mx-auto mb-2" />
              <p className="text-gray-500">Failed to load image</p>
            </div>
          </div>
        ) : (
          <img
            src={imageUrl}
            alt="Security Alert"
            className="w-full h-60 object-cover"
            onError={() => setImageError(true)}
            loading="eager"
          />
        )}
      </div>

      {/* Footer */}
      <div className="p-4 bg-gray-50 flex space-x-4">
        <button
          onClick={onDismiss}
          className="flex-1 bg-gray-200 hover:bg-gray-300 text-gray-700 py-2 px-4 rounded-md transition-colors"
        >
          Dismiss
        </button>
        <button
          onClick={onView}
          className="flex-1 bg-black hover:bg-gray-800 text-white py-2 px-4 rounded-md transition-colors flex items-center justify-center"
        >
          <Eye className="h-4 w-4 mr-2" />
          View
        </button>
      </div>
    </div>
  );
};

export default AlertNotification;