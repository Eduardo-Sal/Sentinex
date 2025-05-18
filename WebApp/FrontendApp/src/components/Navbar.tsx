import React, { useState, useEffect } from 'react';
import { Link, useNavigate } from 'react-router-dom';
import { Menu, X, User } from 'lucide-react';
import { useAuth } from '../context/AuthContext';

const Navbar = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [isScrolled, setIsScrolled] = useState(false);
  const { isAuthenticated, logout } = useAuth();
  const navigate = useNavigate();

  useEffect(() => {
    const handleScroll = () => setIsScrolled(window.scrollY > 10);
    window.addEventListener('scroll', handleScroll);
    return () => window.removeEventListener('scroll', handleScroll);
  }, []);

  const textColor = isScrolled
    ? 'text-gray-900 hover:text-black'
    : 'text-white hover:text-gray-200';
  const bgClasses = isScrolled
    ? 'bg-white/90 backdrop-blur-md shadow-sm py-3'
    : 'bg-transparent py-5';

  const logoColor = isScrolled ? 'text-gray-900' : 'text-white';
  const iconColor = isScrolled ? 'text-gray-900' : 'text-white';

  const handleSignOut = () => {
    logout();
    navigate('/signin');
  };

  return (
    <nav className={`fixed w-full z-50 transition-all duration-300 ${bgClasses}`}>
      <div className="container mx-auto px-4 md:px-6">
        <div className="flex justify-between items-center">
          {/* Logo */}
          <Link to="/" className="text-xl font-bold tracking-tight">
            <span className={logoColor}>SENTINEX</span>
          </Link>

          {/* Desktop Nav */}
          <div className="hidden md:flex items-center space-x-8">
            <a href="#features" className={`${textColor} transition-colors`}>
              Features
            </a>
            <a href="#cloud" className={`${textColor} transition-colors`}>
              Cloud Technology
            </a>

            {isAuthenticated ? (
              <div className="flex items-center space-x-4">
                <Link
                  to="/dashboard"
                  className={`${textColor} transition-colors flex items-center`}
                >
                  <User className="h-5 w-5 mr-1" />
                  Dashboard
                </Link>
                <button
                  onClick={handleSignOut}
                  className="bg-black text-white px-5 py-2 rounded-md font-medium hover:bg-gray-800 transition-colors"
                >
                  Sign Out
                </button>
              </div>
            ) : (
              <Link
                to="/signin"
                className="bg-black text-white px-5 py-2 rounded-md font-medium hover:bg-gray-800 transition-colors"
              >
                Sign In
              </Link>
            )}
          </div>

          {/* Mobile menu button */}
          <button
            className={`md:hidden ${iconColor}`}
            onClick={() => setIsOpen(!isOpen)}
          >
            {isOpen ? <X className="h-6 w-6" /> : <Menu className="h-6 w-6" />}
          </button>
        </div>
      </div>

      {/* Mobile Nav */}
      {isOpen && (
        <div className="md:hidden bg-white/95 backdrop-blur-md absolute top-full left-0 w-full py-4 px-4 shadow-md">
          <div className="flex flex-col space-y-4">
            <a
              href="#features"
              className="text-gray-700 hover:text-black transition-colors py-2"
              onClick={() => setIsOpen(false)}
            >
              Features
            </a>
            <a
              href="#cloud"
              className="text-gray-700 hover:text-black transition-colors py-2"
              onClick={() => setIsOpen(false)}
            >
              Cloud Technology
            </a>

            {isAuthenticated ? (
              <>
                <Link
                  to="/dashboard"
                  className="text-gray-700 hover:text-black transition-colors py-2 flex items-center"
                  onClick={() => setIsOpen(false)}
                >
                  <User className="h-5 w-5 mr-2" />
                  Dashboard
                </Link>
                <button
                  onClick={() => {
                    handleSignOut();
                    setIsOpen(false);
                  }}
                  className="bg-black text-white px-5 py-2 rounded-md font-medium hover:bg-gray-800 transition-colors w-full"
                >
                  Sign Out
                </button>
              </>
            ) : (
              <Link
                to="/signin"
                className="bg-black text-white px-5 py-2 rounded-md font-medium hover:bg-gray-800 transition-colors w-full text-center"
                onClick={() => setIsOpen(false)}
              >
                Sign In
              </Link>
            )}
          </div>
        </div>
      )}
    </nav>
  );
};

export default Navbar;