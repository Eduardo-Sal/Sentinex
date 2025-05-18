import React, { useState, useEffect, useRef } from 'react';
import { 
  Eye, 
  Brain, 
  Smartphone, 
  Lock, 
  Wifi, 
  BarChart3,
  Zap
} from 'lucide-react';
import { motion } from 'framer-motion';

const TechnicalBreakdown = () => {
  const [activeCategory, setActiveCategory] = useState('sensors');
  const sectionRef = useRef<HTMLDivElement>(null);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        if (entries[0].isIntersecting) {
          setIsVisible(true);
        }
      },
      { threshold: 0.1 }
    );

    if (sectionRef.current) {
      observer.observe(sectionRef.current);
    }

    return () => {
      if (sectionRef.current) {
        observer.unobserve(sectionRef.current);
      }
    };
  }, []);

  const categories = [
    {
      id: 'sensors',
      icon: <Eye className="h-6 w-6" />,
      title: 'Sensor Technology',
      content: (
        <div>
          <h4 className="text-xl font-semibold mb-4 text-white">Advanced Multi-Spectrum Sensing</h4>
          <p className="text-gray-400 mb-6">
            Sentinex utilizes a sophisticated array of sensors to perceive its environment with unprecedented accuracy.
          </p>
          <div className="space-y-4">
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">4K Ultra HD Cameras</h5>
              <p className="text-sm text-gray-400">360° coverage with night vision capabilities and 30x optical zoom</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Thermal Imaging</h5>
              <p className="text-sm text-gray-400">Detects heat signatures through darkness, smoke, and light fog</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">LiDAR System</h5>
              <p className="text-sm text-gray-400">Precise 3D mapping of surroundings with 30-meter range</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Audio Detection</h5>
              <p className="text-sm text-gray-400">Identifies unusual sounds like breaking glass or raised voices</p>
            </div>
          </div>
        </div>
      )
    },
    {
      id: 'ai',
      icon: <Brain className="h-6 w-6" />,
      title: 'AI Capabilities',
      content: (
        <div>
          <h4 className="text-xl font-semibold mb-4 text-white">Intelligent Threat Assessment</h4>
          <p className="text-gray-400 mb-6">
            Our proprietary AI algorithms process sensor data in real-time to identify potential security threats.
          </p>
          <div className="space-y-4">
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Computer Vision</h5>
              <p className="text-sm text-gray-400">Recognizes people, vehicles, and objects with 99.7% accuracy</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Behavioral Analysis</h5>
              <p className="text-sm text-gray-400">Identifies suspicious activities like loitering or forced entry attempts</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Autonomous Navigation</h5>
              <p className="text-sm text-gray-400">Self-guided patrol with obstacle avoidance and dynamic route planning</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Continuous Learning</h5>
              <p className="text-sm text-gray-400">Improves detection accuracy over time through machine learning</p>
            </div>
          </div>
        </div>
      )
    },
    {
      id: 'app',
      icon: <Smartphone className="h-6 w-6" />,
      title: 'Mobile App',
      content: (
        <div>
          <h4 className="text-xl font-semibold mb-4 text-white">Complete Control at Your Fingertips</h4>
          <p className="text-gray-400 mb-6">
            The Sentinex mobile application provides comprehensive control and monitoring capabilities.
          </p>
          <div className="space-y-4">
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Live Video Feeds</h5>
              <p className="text-sm text-gray-400">Stream real-time footage from any Sentinex unit to your device</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Alert Management</h5>
              <p className="text-sm text-gray-400">Receive, review, and respond to security notifications</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Patrol Configuration</h5>
              <p className="text-sm text-gray-400">Define custom patrol routes and schedules for each unit</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Analytics Dashboard</h5>
              <p className="text-sm text-gray-400">Review historical data and security insights</p>
            </div>
          </div>
        </div>
      )
    },
    {
      id: 'security',
      icon: <Lock className="h-6 w-6" />,
      title: 'Security Protocols',
      content: (
        <div>
          <h4 className="text-xl font-semibold mb-4 text-white">Enterprise-Grade Security</h4>
          <p className="text-gray-400 mb-6">
            Sentinex implements multiple layers of security to protect both physical premises and digital data.
          </p>
          <div className="space-y-4">
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">End-to-End Encryption</h5>
              <p className="text-sm text-gray-400">All video feeds and communications are encrypted using AES-256</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Tamper Detection</h5>
              <p className="text-sm text-gray-400">Immediate alerts if physical tampering is attempted</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Role-Based Access</h5>
              <p className="text-sm text-gray-400">Granular control over who can access which features and data</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Secure Cloud Storage</h5>
              <p className="text-sm text-gray-400">Footage is securely stored with automatic backups</p>
            </div>
          </div>
        </div>
      )
    },
    {
      id: 'integration',
      icon: <Wifi className="h-6 w-6" />,
      title: 'Integration Options',
      content: (
        <div>
          <h4 className="text-xl font-semibold mb-4 text-white">Seamless Ecosystem Integration</h4>
          <p className="text-gray-400 mb-6">
            Sentinex is designed to work with your existing security infrastructure and business systems.
          </p>
          <div className="space-y-4">
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Security Systems</h5>
              <p className="text-sm text-gray-400">Integrates with existing alarm systems, access control, and CCTV</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Building Management</h5>
              <p className="text-sm text-gray-400">Works with smart building systems for comprehensive facility management</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">API Access</h5>
              <p className="text-sm text-gray-400">RESTful API for custom integrations with your business software</p>
            </div>
            <div className="bg-white/5 p-4 rounded-lg border-l-4 border-white">
              <h5 className="font-medium mb-2 text-white">Emergency Services</h5>
              <p className="text-sm text-gray-400">Optional direct connection to police and fire department systems</p>
            </div>
          </div>
        </div>
      )
    }
  ];

  return (
    <section id="technology" className="py-32 bg-black" ref={sectionRef}>
      <div className="container mx-auto px-4 md:px-6">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-4xl font-bold mb-4 text-white">Technical Breakdown</h2>
          <p className="text-xl text-gray-400 max-w-3xl mx-auto">
            Discover the advanced technology that powers Sentinex's autonomous security capabilities.
          </p>
        </div>

        <div className={`flex flex-col lg:flex-row gap-8 transition-all duration-1000 ${isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-20'}`}>
          {/* Category Selection */}
          <div className="lg:w-1/4">
            <div className="bg-white/5 backdrop-blur-lg rounded-lg p-4 sticky top-24 border border-white/10">
              <h3 className="text-xl font-semibold mb-6 text-white">Technology Categories</h3>
              <div className="space-y-2">
                {categories.map((category) => (
                  <button
                    key={category.id}
                    className={`w-full text-left px-4 py-3 rounded-md flex items-center transition-colors ${
                      activeCategory === category.id 
                        ? 'bg-white text-black' 
                        : 'bg-white/5 text-gray-300 hover:bg-white/10'
                    }`}
                    onClick={() => setActiveCategory(category.id)}
                  >
                    <span className="mr-3">{category.icon}</span>
                    <span>{category.title}</span>
                  </button>
                ))}
              </div>
            </div>
          </div>

          {/* Content Area */}
          <div className="lg:w-3/4">
            <div className="bg-white/5 backdrop-blur-lg rounded-lg p-8 border border-white/10">
              {categories.find(c => c.id === activeCategory)?.content}
            </div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default TechnicalBreakdown;