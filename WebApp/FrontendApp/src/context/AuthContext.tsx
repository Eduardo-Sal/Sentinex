import React, { createContext, useContext, useState, useEffect } from 'react';
import { Auth } from 'aws-amplify';

interface AuthContextType {
  isAuthenticated: boolean;
  user: User | null;
  login: (email: string, password: string) => Promise<void>;
  logout: () => Promise<void>;
  error: string | null;
}

interface User {
  email: string;
  name?: string;
  sub: string;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export function AuthProvider({ children }: { children: React.ReactNode }) {
  const [isAuthenticated, setIsAuthenticated] = useState(false);
  const [user, setUser] = useState<User | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    checkAuthState();
  }, []);

  async function checkAuthState() {
    try {
      const cognitoUser = await Auth.currentAuthenticatedUser();
      console.log('Current authenticated user:', cognitoUser); // Debug log
      setUser({
        email: cognitoUser.attributes.email,
        name: cognitoUser.attributes.name,
        sub: cognitoUser.attributes.sub
      });
      localStorage.setItem('user_sub', cognitoUser.attributes.sub);
      setIsAuthenticated(true);
    } catch (error) {
      console.error('Auth state check error:', error); // Debug log
      setUser(null);
      setIsAuthenticated(false);
    }
  }

  const login = async (email: string, password: string) => {
    try {
      const cognitoUser = await Auth.signIn(email, password);
      console.log('Login successful:', cognitoUser); // Debug log
      
      // Check if the user is confirmed
      if (!cognitoUser.attributes?.email_verified) {
        throw new Error('UserNotConfirmedException');
      }

      setUser({
        email: cognitoUser.attributes.email,
        name: cognitoUser.attributes.name,
        sub: cognitoUser.attributes.sub
      });
      setIsAuthenticated(true);
      setError(null);
    } catch (err) {
      console.error('Login error:', err); // Debug log
      if (err instanceof Error) {
        setError(err.message);
      } else {
        setError('An unexpected error occurred');
      }
      throw err;
    }
  };

  const logout = async () => {
    try {
      await Auth.signOut();
      setUser(null);
      setIsAuthenticated(false);
    } catch (err) {
      console.error('Error signing out:', err);
    }
  };

  return (
    <AuthContext.Provider value={{ isAuthenticated, user, login, logout, error }}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth() {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}