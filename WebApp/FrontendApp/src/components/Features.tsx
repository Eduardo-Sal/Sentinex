import React from "react"
import { Card, CardContent } from "@/components/ui/card"
import {
  Clock,
  Eye,
  Bell,
  Shield,
  Cpu,
} from "lucide-react"

export default function FeaturesSection() {
  const panels = [
    {
      icon: <Clock className="h-8 w-8 text-white" />,
      title: "24/7 Autonomous Monitoring",
      desc: "Continuous, unattended patrols keep every corner of your facility under watch.",
    },
    {
      icon: <Eye className="h-8 w-8 text-white" />,
      title: "Intruder Detection & Alerts",
      desc: "Instant notifications with live snapshots whenever movement is detected.",
    },
    {
      icon: <Bell className="h-8 w-8 text-white" />,
      title: "Custom Alert Rules",
      desc: "Define zones, schedules, and sensitivity levels so you only get the alerts that matter.",
    },
    {
      icon: <Shield className="h-8 w-8 text-white" />,
      title: "Secure Data Encryption",
      desc: "All video feeds and logs are protected with encryption.",
    },
    {
      icon: <Cpu className="h-8 w-8 text-white" />,
      title: "On‑Board Processing",
      desc: "Runs AI models locally on Jetson for instant inference.",
    },
  ]

  return (
    <section
      id="features"
      className="bg-black dark:bg-transparent py-16 md:py-32"
    >
      <div className="mx-auto max-w-5xl px-6">
        <h2 className="mb-12 text-center text-4xl font-bold text-white">
          Key Robot Features
        </h2>
        <div className="grid grid-cols-6 gap-4">
          {/* Hero card */}
          <Card className="col-span-full lg:col-span-2 flex items-center justify-center p-8 bg-white/5 border border-white/10">
            <CardContent className="text-center">
              <div className="text-6xl font-extrabold text-white">100%</div>
              <p className="mt-2 text-xl font-semibold text-white">
                Autonomous
              </p>
            </CardContent>
          </Card>

          {/* First 3 feature panels */}
          {panels.slice(0, 3).map((p, i) => (
            <Card
              key={i}
              className="col-span-full sm:col-span-3 lg:col-span-2 overflow-hidden bg-white/5 border border-white/10"
            >
              <CardContent className="flex flex-col items-center p-6 text-center">
                <div className="mb-4 rounded-full bg-white/10 p-4">
                  {p.icon}
                </div>
                <h3 className="mb-2 text-lg font-semibold text-white">
                  {p.title}
                </h3>
                <p className="text-sm text-gray-400">{p.desc}</p>
              </CardContent>
            </Card>
          ))}

          {/* Last 2 feature panels */}
          {panels.slice(3).map((p, i) => (
            <Card
              key={i + 3}
              className="col-span-full sm:col-span-3 lg:col-span-2 overflow-hidden bg-white/5 border border-white/10"
            >
              <CardContent className="flex flex-col items-center p-6 text-center">
                <div className="mb-4 rounded-full bg-white/10 p-4">
                  {p.icon}
                </div>
                <h3 className="mb-2 text-lg font-semibold text-white">
                  {p.title}
                </h3>
                <p className="text-sm text-gray-400">{p.desc}</p>
              </CardContent>
            </Card>
          ))}
        </div>
      </div>
    </section>
  )
}