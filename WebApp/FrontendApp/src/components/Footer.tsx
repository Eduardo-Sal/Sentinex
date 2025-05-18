import React from 'react'
import { Link } from 'react-router-dom'

export default function FooterSection() {
  return (
    <footer className="bg-black py-16 md:py-32">
      <div className="mx-auto max-w-5xl px-6 text-center space-y-8">
        {/* Logo (same as in Navbar) */}
        <Link to="/" className="mx-auto block text-xl font-bold tracking-tight">
          <span className="text-white">SENTINEX</span>
        </Link>

        {/* Copyright */}
        <span className="text-gray-400 block text-sm">
          © {new Date().getFullYear()} Sentinex. All rights reserved
        </span>
      </div>
    </footer>
  )
}