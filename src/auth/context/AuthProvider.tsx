// src/auth/context/AuthProvider.tsx
import React, { createContext, useContext, useState, useEffect, ReactNode } from 'react';
import { authClient } from '../../lib/better-auth/client';
import { UserProfile, RegistrationData, SignInData } from '../../models/user';

interface AuthContextType {
  user: UserProfile | null;
  session: any | null;
  signIn: (data: SignInData) => Promise<any>;
  signOut: () => Promise<void>;
  signUp: (data: RegistrationData) => Promise<any>;
  isLoading: boolean;
  isAuthenticated: boolean;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

interface AuthProviderProps {
  children: ReactNode;
}

export const AuthProvider: React.FC<AuthProviderProps> = ({ children }) => {
  const [session, setSession] = useState<any>(null);
  const [isLoading, setIsLoading] = useState<boolean>(true);

  // Initialize session
  useEffect(() => {
    const fetchSession = async () => {
      setIsLoading(true);
      try {
        console.log('[AuthProvider] Fetching session from auth server...');
        const currentSession = await authClient.getSession();
        console.log('[AuthProvider] Session response:', currentSession);
        if (currentSession?.data) {
          console.log('[AuthProvider] Session found:', currentSession.data);
          setSession(currentSession.data);
        } else {
          console.log('[AuthProvider] No session data found');
          setSession(null);
        }
      } catch (error) {
        console.error('[AuthProvider] Failed to get session:', error);
        setSession(null);
      } finally {
        setIsLoading(false);
      }
    };

    fetchSession();
  }, []);

  const signIn = async (data: SignInData) => {
    try {
      console.log('[AuthProvider] Signing in...');
      const result = await authClient.signIn.email({
        email: data.email,
        password: data.password,
      });
      console.log('[AuthProvider] Sign in result:', result);

      if (!result.error && result.data) {
        console.log('[AuthProvider] Sign in successful, fetching fresh session...');
        const freshSession = await authClient.getSession();
        console.log('[AuthProvider] Fresh session:', freshSession);
        if (freshSession?.data) {
          setSession(freshSession.data);
        }
      }

      return result;
    } catch (error) {
      console.error('[AuthProvider] Sign in failed:', error);
      throw error;
    }
  };

  const signOut = async () => {
    try {
      await authClient.signOut();
      setSession(null);
    } catch (error) {
      console.error('Sign out failed:', error);
      throw error;
    }
  };

  const signUp = async (data: RegistrationData) => {
    try {
      const result = await authClient.signUp.email({
        email: data.email,
        password: data.password,
        name: data.email.split('@')[0],
        softwareBackground: data.softwareBackground,
        hardwareBackground: data.hardwareBackground,
        backgroundComplete: !!(data.softwareBackground || data.hardwareBackground),
      });

      if (!result.error && result.data) {
        const freshSession = await authClient.getSession();
        if (freshSession?.data) {
          setSession(freshSession.data);
        }
      }

      return result;
    } catch (error) {
      console.error('Sign up failed:', error);
      throw error;
    }
  };

  const value: AuthContextType = {
    user: session?.user || null,
    session: session || null,
    signIn,
    signOut,
    signUp,
    isLoading,
    isAuthenticated: !!(session?.user),
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export const useAuthContext = () => {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuthContext must be used within an AuthProvider');
  }
  return context;
};

export default AuthProvider;