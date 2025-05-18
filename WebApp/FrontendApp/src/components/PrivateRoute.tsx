import React, { useEffect } from 'react';
import { Navigate, useLocation } from 'react-router-dom';
import { useAuth } from '../context/AuthContext';
import { useSession } from '../context/SessionContext';

interface PrivateRouteProps {
  children: React.ReactNode;
}

const PrivateRoute: React.FC<PrivateRouteProps> = ({ children }) => {
  const { isAuthenticated } = useAuth();
  const { updateLastVisitedRoute } = useSession();
  const location = useLocation();

  useEffect(() => {
    if (isAuthenticated) {
      updateLastVisitedRoute(location.pathname);
    }
  }, [isAuthenticated, location.pathname, updateLastVisitedRoute]);

  if (!isAuthenticated) {
    return <Navigate to="/signin" state={{ from: location }} replace />;
  }

  return <>{children}</>;
}

export default PrivateRoute;