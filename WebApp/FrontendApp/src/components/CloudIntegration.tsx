import React, { useEffect } from 'react'
import { Shield, ArrowRight, Database, Server, Lock } from 'lucide-react'
import { motion, useAnimation } from 'framer-motion'
import { useInView } from 'react-intersection-observer'

const steps = [
  {
    icon: <Shield className="h-8 w-8 text-white" />,
    title: "Security Data Collection",
    desc: "Sentinex robots gather real‑time data from multi‑modal sensors.",
  },
  {
    icon: <ArrowRight className="h-6 w-6 text-gray-400" />,
    title: "Secure Transmission",
    desc: "Encrypted streaming over AWS network.",
  },
  {
    icon: <Database className="h-8 w-8 text-white" />,
    title: "AWS S3 Storage",
    desc: "Durable video & log storage in S3 buckets.",
  },
  {
    icon: <Server className="h-8 w-8 text-white" />,
    title: "AWS Lambda Processing",
    desc: "Serverless compute analyzes data instantly.",
  },
  {
    icon: <Lock className="h-8 w-8 text-white" />,
    title: "AWS Cognito Auth",
    desc: "Secure user access with Cognito.",
  },
]

export default function CloudIntegration() {
  const controls = useAnimation()
  const [ref, inView] = useInView({ threshold: 0.3 })

  useEffect(() => {
    if (inView) controls.start('visible')
  }, [controls, inView])

  const variants = {
    hidden: { opacity: 0, y: 20 },
    visible: (i: number) => ({
      opacity: 1,
      y: 0,
      transition: { delay: i * 0.2, duration: 0.6 },
    }),
  }

  return (
    <section id="cloud" ref={ref} className="py-32 bg-black">
      <div className="container mx-auto px-4 md:px-6 text-center">
        <h2 className="text-4xl font-bold text-white mb-4">
          Cloud‑Powered Security
        </h2>
        <p className="text-xl text-gray-400 max-w-3xl mx-auto mb-12">
          Sentinex leverages AWS for rock‑solid, scalable security data processing.
        </p>

        <div className="flex flex-wrap justify-center items-center gap-6">
          {steps.map((step, idx) => (
            <motion.div
              key={idx}
              custom={idx}
              initial="hidden"
              animate={controls}
              variants={variants}
              className={`flex flex-col items-center p-6 rounded-lg ${
                idx % 2 === 0 ? 'bg-white/5' : 'bg-white/10'
              } w-44`}
            >
              <div className="mb-3">{step.icon}</div>
              <h4 className="text-white font-semibold mb-2 text-center">
                {step.title}
              </h4>
              <p className="text-gray-400 text-sm">{step.desc}</p>
            </motion.div>
          ))}
        </div>
      </div>
    </section>
  )
}