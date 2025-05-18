import React, { useState, useEffect, useRef } from 'react';
import { MapPin, Eye, Bell, Smartphone, Home, Settings, Menu, Search, User, Calendar, Lock } from 'lucide-react';

const InteractiveDemo = () => {
  const [activeTab, setActiveTab] = useState('app');
  const [isVisible, setIsVisible] = useState(false);
  const demoRef = useRef<HTMLDivElement>(null);
  const [showAlert, setShowAlert] = useState(false);

  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        if (entries[0].isIntersecting) {
          setIsVisible(true);
        }
      },
      { threshold: 0.1 }
    );

    if (demoRef.current) {
      observer.observe(demoRef.current);
    }

    return () => {
      if (demoRef.current) {
        observer.unobserve(demoRef.current);
      }
    };
  }, []);

  useEffect(() => {
    if (activeTab === 'app') {
      const alertTimer = setTimeout(() => {
        setShowAlert(true);
        setTimeout(() => setShowAlert(false), 3000);
      }, 5000);
      return () => clearTimeout(alertTimer);
    }
  }, [activeTab]);

  return (
    <section id="demo" className="py-32 bg-black" ref={demoRef}>
      <div className="container mx-auto px-4 md:px-6">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-4xl font-bold mb-4 text-white">Mobile Experience</h2>
          <p className="text-xl text-gray-400 max-w-3xl mx-auto">
            Control your security system from anywhere with our intuitive mobile application.
          </p>
        </div>

        <div className={`transition-all duration-1000 ${isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-20'}`}>
          {/* Tabs */}
          <div className="flex flex-wrap justify-center mb-8 border-b border-white/10">
            <button 
              className={`px-6 py-3 font-medium text-lg ${activeTab === 'app' ? 'text-white border-b-2 border-white' : 'text-gray-400 hover:text-gray-300'}`}
              onClick={() => setActiveTab('app')}
            >
              Mobile Interface
            </button>
            <button 
              className={`px-6 py-3 font-medium text-lg ${activeTab === 'alerts' ? 'text-white border-b-2 border-white' : 'text-gray-400 hover:text-gray-300'}`}
              onClick={() => setActiveTab('alerts')}
            >
              Alert System
            </button>
            <button 
              className={`px-6 py-3 font-medium text-lg ${activeTab === 'settings' ? 'text-white border-b-2 border-white' : 'text-gray-400 hover:text-gray-300'}`}
              onClick={() => setActiveTab('settings')}
            >
              Settings
            </button>
          </div>

          {/* Tab Content */}
          <div className="bg-white/5 backdrop-blur-lg rounded-lg overflow-hidden shadow-xl max-w-4xl mx-auto border border-white/10">
            {activeTab === 'app' && (
              <div className="flex flex-col md:flex-row">
                <div className="w-full md:w-1/3 bg-black/50 p-6 flex flex-col border-r border-white/10">
                  <h3 className="text-xl font-bold mb-6 text-white">Sentinex Mobile</h3>
                  <div className="space-y-4 flex-grow">
                    <div className="p-3 bg-white/10 rounded-md text-white cursor-pointer transition-colors flex items-center">
                      <Home className="h-5 w-5 mr-3" />
                      <div className="font-medium">Dashboard</div>
                    </div>
                    <div className="p-3 bg-white/5 hover:bg-white/10 rounded-md cursor-pointer transition-colors flex items-center">
                      <Eye className="h-5 w-5 mr-3 text-gray-400" />
                      <div className="font-medium text-gray-300">Live Feeds</div>
                    </div>
                    <div className="p-3 bg-white/5 hover:bg-white/10 rounded-md cursor-pointer transition-colors flex items-center">
                      <Bell className="h-5 w-5 mr-3 text-gray-400" />
                      <div className="font-medium text-gray-300">Alerts</div>
                    </div>
                    <div className="p-3 bg-white/5 hover:bg-white/10 rounded-md cursor-pointer transition-colors flex items-center">
                      <MapPin className="h-5 w-5 mr-3 text-gray-400" />
                      <div className="font-medium text-gray-300">Locations</div>
                    </div>
                    <div className="p-3 bg-white/5 hover:bg-white/10 rounded-md cursor-pointer transition-colors flex items-center">
                      <Settings className="h-5 w-5 mr-3 text-gray-400" />
                      <div className="font-medium text-gray-300">Settings</div>
                    </div>
                  </div>
                  <div className="mt-6 pt-6 border-t border-white/10">
                    <div className="flex items-center">
                      <div className="w-3 h-3 bg-green-500 rounded-full mr-2"></div>
                      <span className="text-sm text-gray-400">All systems operational</span>
                    </div>
                  </div>
                </div>
                <div className="w-full md:w-2/3 bg-black/30">
                  <div className="p-6 border-b border-white/10 flex justify-between items-center">
                    <div>
                      <h3 className="text-xl font-bold text-white">Dashboard</h3>
                      <p className="text-gray-400 text-sm">System Overview</p>
                    </div>
                    <div className="flex space-x-2">
                      <button className="p-2 rounded-full bg-white/5 hover:bg-white/10">
                        <Search className="h-5 w-5 text-gray-400" />
                      </button>
                      <button className="p-2 rounded-full bg-white/5 hover:bg-white/10">
                        <User className="h-5 w-5 text-gray-400" />
                      </button>
                    </div>
                  </div>
                  <div className="p-6">
                    <div className="grid grid-cols-2 gap-4 mb-6">
                      <div className="bg-white/5 p-4 rounded-lg border border-white/10">
                        <div className="flex justify-between items-center mb-2">
                          <h4 className="font-medium text-gray-300">Active Units</h4>
                          <span className="text-white font-bold">3/3</span>
                        </div>
                        <div className="h-2 bg-white/5 rounded-full overflow-hidden">
                          <div className="h-full bg-green-500 rounded-full" style={{ width: '100%' }}></div>
                        </div>
                      </div>
                      <div className="bg-white/5 p-4 rounded-lg border border-white/10">
                        <div className="flex justify-between items-center mb-2">
                          <h4 className="font-medium text-gray-300">Battery Status</h4>
                          <span className="text-white font-bold">87%</span>
                        </div>
                        <div className="h-2 bg-white/5 rounded-full overflow-hidden">
                          <div className="h-full bg-green-500 rounded-full" style={{ width: '87%' }}></div>
                        </div>
                      </div>
                    </div>
                    
                    <h4 className="font-medium text-white mb-3">Recent Activity</h4>
                    <div className="space-y-3">
                      <div className="p-3 bg-white/5 rounded-lg border border-white/10 flex items-center">
                        <div className="bg-white/10 p-2 rounded-full mr-3">
                          <Eye className="h-4 w-4 text-white" />
                        </div>
                        <div>
                          <div className="font-medium text-white">Motion detected</div>
                          <div className="text-sm text-gray-400">Front entrance • 5 minutes ago</div>
                        </div>
                      </div>
                      <div className="p-3 bg-white/5 rounded-lg border border-white/10 flex items-center">
                        <div className="bg-white/10 p-2 rounded-full mr-3">
                          <MapPin className="h-4 w-4 text-white" />
                        </div>
                        <div>
                          <div className="font-medium text-white">Patrol completed</div>
                          <div className="text-sm text-gray-400">Warehouse area • 27 minutes ago</div>
                        </div>
                      </div>
                    </div>
                    
                    {showAlert && (
                      <div className="fixed bottom-4 right-4 bg-white/10 backdrop-blur-lg text-white p-4 rounded-lg shadow-lg flex items-center animate-fade-in z-50 border border-white/20">
                        <Bell className="h-5 w-5 mr-3 text-white" />
                        <div>
                          <div className="font-medium">New Alert</div>
                          <div className="text-sm text-gray-300">Motion detected in restricted area</div>
                        </div>
                      </div>
                    )}
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'alerts' && (
              <div className="p-6 bg-black/30">
                <div className="flex justify-between items-center mb-6">
                  <h3 className="text-xl font-bold text-white">Alert System</h3>
                  <button className="bg-white/10 hover:bg-white/20 text-white px-4 py-2 rounded-md text-sm border border-white/10">
                    Configure Alerts
                  </button>
                </div>
                
                <div className="space-y-4">
                  <div className="p-4 border border-white/10 rounded-lg bg-white/5">
                    <div className="flex justify-between items-center mb-2">
                      <div className="flex items-center">
                        <div className="bg-red-500/20 p-2 rounded-full mr-3">
                          <Bell className="h-5 w-5 text-red-500" />
                        </div>
                        <h4 className="font-medium text-white">Motion Detection</h4>
                      </div>
                      <div className="relative inline-block w-10 mr-2 align-middle select-none">
                        <input type="checkbox" id="motion" className="sr-only" defaultChecked />
                        <div className="block h-6 bg-white/10 rounded-full w-10"></div>
                        <div className="dot absolute left-1 top-1 bg-white w-4 h-4 rounded-full transition"></div>
                      </div>
                    </div>
                    <p className="text-gray-400 text-sm">Alert when motion is detected in restricted areas</p>
                  </div>
                  
                  <div className="p-4 border border-white/10 rounded-lg bg-white/5">
                    <div className="flex justify-between items-center mb-2">
                      <div className="flex items-center">
                        <div className="bg-yellow-500/20 p-2 rounded-full mr-3">
                          <User className="h-5 w-5 text-yellow-500" />
                        </div>
                        <h4 className="font-medium text-white">Unauthorized Access</h4>
                      </div>
                      <div className="relative inline-block w-10 mr-2 align-middle select-none">
                        <input type="checkbox" id="access" className="sr-only" defaultChecked />
                        <div className="block h-6 bg-white/10 rounded-full w-10"></div>
                        <div className="dot absolute left-1 top-1 bg-white w-4 h-4 rounded-full transition"></div>
                      </div>
                    </div>
                    <p className="text-gray-400 text-sm">Alert when unauthorized personnel enter secure zones</p>
                  </div>
                </div>
              </div>
            )}

            {activeTab === 'settings' && (
              <div className="p-6 bg-black/30">
                <div className="flex justify-between items-center mb-6">
                  <h3 className="text-xl font-bold text-white">App Settings</h3>
                  <button className="bg-white/10 hover:bg-white/20 text-white px-4 py-2 rounded-md text-sm border border-white/10">
                    Save Changes
                  </button>
                </div>
                
                <div className="space-y-6">
                  <div>
                    <h4 className="font-medium text-white mb-3">Notification Preferences</h4>
                    <div className="space-y-2">
                      <div className="flex items-center justify-between">
                        <label className="text-gray-300">Push Notifications</label>
                        <div className="relative inline-block w-10 mr-2 align-middle select-none">
                          <input type="checkbox" id="push" className="sr-only" defaultChecked />
                          <div className="block h-6 bg-white/10 rounded-full w-10"></div>
                          <div className="dot absolute left-1 top-1 bg-white w-4 h-4 rounded-full transition"></div>
                        </div>
                      </div>
                    </div>
                  </div>
                  
                  <div>
                    <h4 className="font-medium text-white mb-3">Display Settings</h4>
                    <div className="space-y-4">
                      <div>
                        <label className="block text-gray-300 mb-2">Theme</label>
                        <select className="w-full p-2 bg-white/5 border border-white/10 rounded-md text-white">
                          <option>Dark Mode</option>
                          <option>Light Mode</option>
                          <option>System Default</option>
                        </select>
                      </div>
                    </div>
                  </div>
                </div>
              </div>
            )}
          </div>
        </div>
      </div>
    </section>
  );
};

export default InteractiveDemo;