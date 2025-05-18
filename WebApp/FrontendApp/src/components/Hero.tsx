import React from 'react';
import { useSpring, animated, config } from '@react-spring/web';
import { motion } from 'framer-motion';
import { ChevronDown } from 'lucide-react';
import { useInView } from 'react-intersection-observer';
//import RobotImage from '../assets/asmrchatgpt.png'; // Robot PNG with transparent bg
import RoomBg from '../assets/bgimage.png'; // New room background image

const Hero = () => {
  const [ref, inView] = useInView({ threshold: 0.1, triggerOnce: true });

  const springProps = useSpring({
    from: { opacity: 0, transform: 'translateY(50px)' },
    to: { opacity: inView ? 1 : 0, transform: inView ? 'translateY(0)' : 'translateY(50px)' },
    config: config.gentle,
  });

  return (
    <section
      ref={ref}
      className="relative min-h-screen flex items-center justify-center pt-32 pb-20 overflow-hidden"
    >
      {/* Room background with dark overlay */}
      <div
        className="absolute inset-0 bg-cover bg-center"
        style={{ backgroundImage: `url('${RoomBg}')` }}
      />
      <div className="absolute inset-0 bg-black/60" />

      <div className="container mx-auto px-4 md:px-6 relative z-10">
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-12 items-center">
          {/* Text column */}
          <animated.div style={springProps} className="text-center lg:text-left">
            <motion.div
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.8 }}
              className="mb-8"
            >
              <h1 className="text-5xl md:text-7xl font-bold text-white mb-6">
                Autonomous
                <br />
                Security Mobile Robot
              </h1>
              <p className="text-xl md:text-2xl text-gray-200">
                Autonomous surveillance powered by advanced AI and robotics
              </p>
            </motion.div>
          </animated.div>

          {/* Robot image column */}
          <animated.div style={springProps} className="relative flex items-end justify-center h-[600px]">
            {/* Ground plane shadow for robot */}

          </animated.div>
        </div>
      </div>

      {/* Scroll indicator */}
      <motion.div
        className="absolute bottom-8 left-1/2 transform -translate-x-1/2 z-10"
        animate={{ y: [0, 10, 0] }}
        transition={{ duration: 2, repeat: Infinity }}
      >
        <ChevronDown className="h-8 w-8 text-white/50" />
      </motion.div>
    </section>
  );
};

export default Hero;
