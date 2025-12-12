// src/auth/services/auth-service.ts
import { auth } from '@/lib/better-auth/auth';
import { UserProfile, RegistrationData, SignInData, SessionData } from '@/models/user';

class AuthService {
  // Sign up with background data
  async signUp(data: RegistrationData): Promise<{ user: UserProfile; session: SessionData } | { error: string }> {
    try {
      // In a real implementation, we would call the Better-Auth API here
      // For now, we'll return a placeholder implementation
      // The actual API endpoint will handle the registration
      console.log('Sign up called with:', data);

      // This is a client-side service that would typically make API calls
      // to the backend endpoints we'll create later
      const response = await fetch('http://localhost:3001/api/auth/sign-up', { // Better-Auth default endpoint
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.message || 'Sign up failed');
      }

      const result = await response.json();
      return result;
    } catch (error) {
      console.error('Sign up error:', error);
      return { error: (error as Error).message || 'Sign up failed' };
    }
  }

  // Sign in
  async signIn(data: SignInData): Promise<{ user: UserProfile; session: SessionData } | { error: string }> {
    try {
      console.log('Sign in called with:', data);

      const response = await fetch('http://localhost:3001/api/auth/sign-in', { // Better-Auth default endpoint
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(data),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.message || 'Sign in failed');
      }

      const result = await response.json();
      return result;
    } catch (error) {
      console.error('Sign in error:', error);
      return { error: (error as Error).message || 'Sign in failed' };
    }
  }

  // Sign out
  async signOut(): Promise<{ success: boolean } | { error: string }> {
    try {
      const response = await fetch('http://localhost:3001/api/auth/sign-out', { // Better-Auth default endpoint
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.message || 'Sign out failed');
      }

      const result = await response.json();
      return result;
    } catch (error) {
      console.error('Sign out error:', error);
      return { error: (error as Error).message || 'Sign out failed' };
    }
  }

  // Get current user profile
  async getProfile(): Promise<UserProfile | null> {
    try {
      const response = await fetch('http://localhost:3001/api/auth/get-session', { // Better-Auth default endpoint
        method: 'GET',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (!response.ok) {
        return null;
      }

      const result = await response.json();
      return result.user;
    } catch (error) {
      console.error('Get profile error:', error);
      return null;
    }
  }

  // Update user profile with background data
  async updateProfile(userId: string, profileData: Partial<UserProfile>): Promise<UserProfile | { error: string }> {
    try {
      // Note: Better-Auth doesn't have a direct update profile endpoint
      // We would need to implement custom logic or use hooks
      // For now, this is a placeholder
      console.warn('Profile update not directly supported by Better-Auth default endpoints');
      return { error: 'Profile update endpoint not implemented' };
    } catch (error) {
      console.error('Update profile error:', error);
      return { error: (error as Error).message || 'Update profile failed' };
    }
  }
}

export const authService = new AuthService();