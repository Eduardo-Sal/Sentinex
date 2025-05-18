import React from 'react';
import Navbar from '../components/Navbar';
import Hero from '../components/Hero';
import Features from '../components/Features';
import InteractiveDemo from '../components/InteractiveDemo';
import TechnicalBreakdown from '../components/TechnicalBreakdown';
import Comparison from '../components/Comparison';
import CloudIntegration from '../components/CloudIntegration';
import Footer from '../components/Footer';
import TeamSection from '../components/team'


const LandingPage = () => {
  return (
    <div className="bg-white text-gray-900 min-h-screen">
      <Navbar />
      <Hero />
      <Features />
      {/*<InteractiveDemo />*/}
      <CloudIntegration />
      {/*<TechnicalBreakdown />*/}
      {/*<Comparison />*/}
      <TeamSection/>
      <Footer />
    </div>
  );
};

export default LandingPage;