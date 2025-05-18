import React, { createContext, useContext, useState, useEffect } from 'react';

interface SessionContextType {
  lastVisitedRoute: string;
  updateLastVisitedRoute: (route: string) => void;
  clearSession: () => void;
}

const SessionContext = createContext<SessionContextType | undefined>(undefined);

export function SessionProvider({ children }: { children: React.ReactNode }) {
  const [lastVisitedRoute, setLastVisitedRoute] = useState<string>(() => {
    const saved = sessionStorage.getItem('lastVisitedRoute');
    return saved || '/dashboard';
  });

  useEffect(() => {
    sessionStorage.setItem('lastVisitedRoute', lastVisitedRoute);
  }, [lastVisitedRoute]);

  const updateLastVisitedRoute = (route: string) => {
    setLastVisitedRoute(route);
  };

  const clearSession = () => {
    sessionStorage.clear();
    setLastVisitedRoute('/dashboard');
  };

  return (
    <SessionContext.Provider value={{ lastVisitedRoute, updateLastVisitedRoute, clearSession }}>
      {children}
    </SessionContext.Provider>
  );
}

export function useSession() {
  const context = useContext(SessionContext);
  if (context === undefined) {
    throw new Error('useSession must be used within a SessionProvider');
  }
  return context;
}