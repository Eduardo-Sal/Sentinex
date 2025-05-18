import React, { useEffect } from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import LandingPage from './pages/LandingPage';
import SignIn from './pages/SignIn';
import SignUp from './pages/SignUp';
import ForgotPassword from './pages/ForgotPassword';
import Dashboard from './pages/Dashboard';
import { AuthProvider } from './context/AuthContext';
import { SessionProvider } from './context/SessionContext';
import PrivateRoute from './components/PrivateRoute';
import ErrorBoundary from './components/ErrorBoundary';
import { configureAmplify } from './config/aws-config';
import { PubSub } from '@aws-amplify/pubsub';
import { useAwsInfo} from "./hooks/useAWSINFO";
import { useAuth } from './context/AuthContext';

configureAmplify();


const subscription = PubSub.subscribe('i-will-never-publish-on-this-topic').subscribe({
  next: (data) => console.log('Received message:', data),
  error: (error) => console.error('Subscription error:', error),
});

function App() {

  return (
    <ErrorBoundary>
      <Router>
        <AuthProvider>
          <SessionProvider>
            <Routes>
              <Route path="/" element={<LandingPage />} />
              <Route path="/signin" element={<SignIn />} />
              <Route path="/signup" element={<SignUp />} />
              <Route path="/forgot-password" element={<ForgotPassword />} />
              <Route
                path="/dashboard/*"
                element={
                  <PrivateRoute>
                    <Dashboard />
                  </PrivateRoute>
                }
              />
              <Route path="*" element={<Navigate to="/" replace />} />
            </Routes>
          </SessionProvider>
        </AuthProvider>
      </Router>
    </ErrorBoundary>
  );
}

export default App;