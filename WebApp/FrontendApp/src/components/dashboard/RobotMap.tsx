import React from 'react';
import GoogleMapReact from 'google-map-react';
import { Circle, MapPin, Battery, Navigation } from 'lucide-react';

interface MarkerProps {
  lat: number;
  lng: number;
  status: 'active' | 'idle' | 'charging';
  onClick?: () => void;
}

const RobotMarker: React.FC<MarkerProps> = ({ status, onClick }) => {
  const getStatusColor = () => {
    switch (status) {
      case 'active': return 'bg-green-500';
      case 'idle': return 'bg-yellow-500';
      case 'charging': return 'bg-blue-500';
      default: return 'bg-gray-500';
    }
  };

  return (
    <div className="relative -translate-x-1/2 -translate-y-1/2" onClick={onClick}>
      <div className={`p-2 rounded-full ${getStatusColor()} shadow-lg cursor-pointer transform hover:scale-110 transition-transform`}>
        <MapPin className="h-6 w-6 text-white" />
      </div>
      <div className="absolute top-full mt-2 left-1/2 transform -translate-x-1/2 bg-black text-white text-xs py-1 px-2 rounded whitespace-nowrap">
        Robot #1 ({status})
      </div>
    </div>
  );
};

const RobotMap = () => {
  const defaultProps = {
    center: {
      lat: 40.7128,
      lng: -74.0060
    },
    zoom: 14
  };

  const robotStatus = {
    lat: 40.7128,
    lng: -74.0060,
    status: 'active' as const
  };

  return (
    <div className="h-full p-6 space-y-6">
      <div className="flex justify-between items-center">
        <h1 className="text-2xl font-semibold text-gray-900">Robot Map</h1>
        <div className="flex space-x-4">
          <div className="flex items-center space-x-2">
            <Circle className="h-4 w-4 text-green-500 fill-current" />
            <span className="text-sm text-gray-600">Active</span>
          </div>
          <div className="flex items-center space-x-2">
            <Circle className="h-4 w-4 text-yellow-500 fill-current" />
            <span className="text-sm text-gray-600">Idle</span>
          </div>
          <div className="flex items-center space-x-2">
            <Circle className="h-4 w-4 text-blue-500 fill-current" />
            <span className="text-sm text-gray-600">Charging</span>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-4 gap-6">
        <div className="lg:col-span-3">
          <div className="bg-white rounded-lg shadow overflow-hidden" style={{ height: '600px' }}>
            <GoogleMapReact
              bootstrapURLKeys={{ key: '' }} // Add your Google Maps API key here
              defaultCenter={defaultProps.center}
              defaultZoom={defaultProps.zoom}
              options={{
                styles: [
                  {
                    featureType: 'all',
                    elementType: 'all',
                    stylers: [{ saturation: -100 }]
                  }
                ]
              }}
            >
              <RobotMarker
                lat={robotStatus.lat}
                lng={robotStatus.lng}
                status={robotStatus.status}
              />
            </GoogleMapReact>
          </div>
        </div>

        <div className="space-y-4">
          <div className="bg-white rounded-lg p-6 shadow">
            <h3 className="text-sm font-medium text-gray-700 mb-4">Robot Status</h3>
            <div className="space-y-6">
              <div className="flex items-center justify-between">
                <div className="flex items-center">
                  <Battery className="h-5 w-5 text-gray-400 mr-3" />
                  <span className="text-sm text-gray-600">Battery</span>
                </div>
                <span className="text-sm font-medium">87%</span>
              </div>
              
              <div className="flex items-center justify-between">
                <div className="flex items-center">
                  <Navigation className="h-5 w-5 text-gray-400 mr-3" />
                  <span className="text-sm text-gray-600">Speed</span>
                </div>
                <span className="text-sm font-medium">1.2 m/s</span>
              </div>
              
              <div className="flex items-center justify-between">
                <div className="flex items-center">
                  <MapPin className="h-5 w-5 text-gray-400 mr-3" />
                  <span className="text-sm text-gray-600">Location</span>
                </div>
                <span className="text-sm font-medium">
                  {robotStatus.lat.toFixed(4)}, {robotStatus.lng.toFixed(4)}
                </span>
              </div>
            </div>
          </div>

          <div className="bg-white rounded-lg p-6 shadow">
            <h3 className="text-sm font-medium text-gray-700 mb-4">Coverage Area</h3>
            <div className="space-y-4">
              <div>
                <div className="flex justify-between text-sm text-gray-600 mb-1">
                  <span>Area Covered</span>
                  <span>85%</span>
                </div>
                <div className="h-2 bg-gray-200 rounded-full">
                  <div className="h-full bg-green-500 rounded-full" style={{ width: '85%' }}></div>
                </div>
              </div>
              <div>
                <div className="flex justify-between text-sm text-gray-600 mb-1">
                  <span>Route Completion</span>
                  <span>92%</span>
                </div>
                <div className="h-2 bg-gray-200 rounded-full">
                  <div className="h-full bg-blue-500 rounded-full" style={{ width: '92%' }}></div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default RobotMap;