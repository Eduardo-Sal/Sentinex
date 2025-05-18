import React, { useEffect, useCallback } from 'react';
import { motion } from 'framer-motion';
import { useInView } from 'react-intersection-observer';

interface Particle {
  x: number;
  y: number;
  size: number;
  speedX: number;
  speedY: number;
}

const Background: React.FC = () => {
  const [ref, inView] = useInView({
    threshold: 0,
    triggerOnce: true,
  });

  const createParticle = useCallback((): Particle => ({
    x: Math.random() * window.innerWidth,
    y: Math.random() * window.innerHeight,
    size: Math.random() * 2 + 1,
    speedX: (Math.random() - 0.5) * 0.5,
    speedY: (Math.random() - 0.5) * 0.5,
  }), []);

  const particles = Array.from({ length: 50 }, createParticle);

  return (
    <motion.div
      ref={ref}
      className="fixed inset-0 z-0 pointer-events-none overflow-hidden"
      initial={{ opacity: 0 }}
      animate={{ opacity: inView ? 1 : 0 }}
      transition={{ duration: 1 }}
    >
      {particles.map((particle, index) => (
        <motion.div
          key={index}
          className="absolute rounded-full bg-white/10"
          style={{
            width: particle.size,
            height: particle.size,
          }}
          animate={{
            x: [particle.x, particle.x + (Math.random() - 0.5) * 100],
            y: [particle.y, particle.y + (Math.random() - 0.5) * 100],
          }}
          transition={{
            duration: Math.random() * 3 + 2,
            repeat: Infinity,
            repeatType: "reverse",
            ease: "linear"
          }}
        />
      ))}
      <div className="absolute inset-0 bg-gradient-to-b from-black/50 to-transparent" />
    </motion.div>
  );
}

export default Background;