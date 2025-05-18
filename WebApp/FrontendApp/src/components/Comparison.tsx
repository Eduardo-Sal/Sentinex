import React, { useState, useEffect, useRef } from 'react';
import { CheckCircle2, X } from 'lucide-react';
import { motion } from 'framer-motion';

const Comparison = () => {
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

  const comparisonData = [
    {
      feature: '24/7 Coverage',
      traditional: false,
      sentinex: true,
      description: 'Continuous monitoring without breaks or shift changes'
    },
    {
      feature: 'Real-time AI Analysis',
      traditional: false,
      sentinex: true,
      description: 'Instant threat detection and classification'
    },
    {
      feature: 'Mobile Surveillance',
      traditional: false,
      sentinex: true,
      description: 'Active patrolling of the entire premises'
    },
    {
      feature: 'Predictive Analytics',
      traditional: false,
      sentinex: true,
      description: 'Identifies patterns to prevent security incidents'
    },
    {
      feature: 'Multi-sensor Detection',
      traditional: false,
      sentinex: true,
      description: 'Combines visual, thermal, and audio detection'
    },
    {
      feature: 'Remote Monitoring',
      traditional: true,
      sentinex: true,
      description: 'View security feeds from anywhere'
    },
    {
      feature: 'Automatic Alerts',
      traditional: true,
      sentinex: true,
      description: 'Notifications when suspicious activity is detected'
    },
    {
      feature: 'Human Intervention',
      traditional: true,
      sentinex: false,
      description: 'Requires human guards for physical intervention'
    }
  ];

  return (
    <section className="py-32 bg-black" ref={sectionRef}>
      <div className="container mx-auto px-4 md:px-6">
        <div className="text-center mb-16">
          <h2 className="text-3xl md:text-4xl font-bold mb-4 text-white">Traditional vs. Sentinex Security</h2>
          <p className="text-xl text-gray-400 max-w-3xl mx-auto">
            See how Sentinex revolutionizes security compared to traditional surveillance systems.
          </p>
        </div>

        <div className={`transition-all duration-1000 ${isVisible ? 'opacity-100 translate-y-0' : 'opacity-0 translate-y-20'}`}>
          <div className="overflow-x-auto">
            <table className="w-full border-collapse bg-white/5 backdrop-blur-lg rounded-lg shadow-xl border border-white/10">
              <thead>
                <tr>
                  <th className="text-left py-4 px-6 bg-white/10 text-white rounded-tl-lg">Feature</th>
                  <th className="text-center py-4 px-6 bg-white/10 text-white">Traditional Security</th>
                  <th className="text-center py-4 px-6 bg-white/10 text-white rounded-tr-lg">Sentinex</th>
                </tr>
              </thead>
              <tbody>
                {comparisonData.map((item, index) => (
                  <motion.tr 
                    key={index} 
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ delay: index * 0.1 }}
                    className={index % 2 === 0 ? 'bg-white/5' : 'bg-transparent'}
                  >
                    <td className="py-4 px-6 border-t border-white/10">
                      <div className="font-medium text-white">{item.feature}</div>
                      <div className="text-sm text-gray-400">{item.description}</div>
                    </td>
                    <td className="py-4 px-6 text-center border-t border-white/10">
                      {item.traditional ? (
                        <CheckCircle2 className="h-6 w-6 text-green-400 mx-auto" />
                      ) : (
                        <X className="h-6 w-6 text-gray-500 mx-auto" />
                      )}
                    </td>
                    <td className="py-4 px-6 text-center border-t border-white/10">
                      {item.sentinex ? (
                        <CheckCircle2 className="h-6 w-6 text-green-400 mx-auto" />
                      ) : (
                        <X className="h-6 w-6 text-gray-500 mx-auto" />
                      )}
                    </td>
                  </motion.tr>
                ))}
              </tbody>
            </table>
          </div>

          <div className="mt-16 grid grid-cols-1 md:grid-cols-2 gap-8">
            <motion.div 
              className="bg-white/5 backdrop-blur-lg p-8 rounded-lg border border-white/10"
              initial={{ opacity: 0, x: -20 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ delay: 0.2 }}
            >
              <h3 className="text-xl font-bold mb-4 text-white">Traditional Security Challenges</h3>
              <ul className="space-y-3">
                <li className="flex items-start">
                  <X className="h-5 w-5 text-red-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">Human guards require breaks, shifts, and can experience fatigue</span>
                </li>
                <li className="flex items-start">
                  <X className="h-5 w-5 text-red-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">Fixed cameras leave blind spots and coverage gaps</span>
                </li>
                <li className="flex items-start">
                  <X className="h-5 w-5 text-red-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">Manual monitoring is prone to human error and distraction</span>
                </li>
                <li className="flex items-start">
                  <X className="h-5 w-5 text-red-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">High ongoing labor costs with limited scalability</span>
                </li>
                <li className="flex items-start">
                  <X className="h-5 w-5 text-red-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">Reactive rather than proactive threat detection</span>
                </li>
              </ul>
            </motion.div>

            <motion.div 
              className="bg-white/5 backdrop-blur-lg p-8 rounded-lg border border-white/10"
              initial={{ opacity: 0, x: 20 }}
              animate={{ opacity: 1, x: 0 }}
              transition={{ delay: 0.4 }}
            >
              <h3 className="text-xl font-bold mb-4 text-white">Sentinex Advantages</h3>
              <ul className="space-y-3">
                <li className="flex items-start">
                  <CheckCircle2 className="h-5 w-5 text-green-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">Autonomous operation 24/7 without fatigue or breaks</span>
                </li>
                <li className="flex items-start">
                  <CheckCircle2 className="h-5 w-5 text-green-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">Mobile platform actively patrols to eliminate blind spots</span>
                </li>
                <li className="flex items-start">
                  <CheckCircle2 className="h-5 w-5 text-green-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">AI-powered detection with consistent performance</span>
                </li>
                <li className="flex items-start">
                  <CheckCircle2 className="h-5 w-5 text-green-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">Lower long-term costs with greater coverage area</span>
                </li>
                <li className="flex items-start">
                  <CheckCircle2 className="h-5 w-5 text-green-400 mr-2 mt-0.5 flex-shrink-0" />
                  <span className="text-gray-300">Proactive threat prevention through predictive analytics</span>
                </li>
              </ul>
            </motion.div>
          </div>
        </div>
      </div>
    </section>
  );
};

export default Comparison;