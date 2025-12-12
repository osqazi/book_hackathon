// src/auth/utils/session-utils.ts
// Session management utilities for Better-Auth integration

import { SessionData, UserProfile } from '@/models/user';

/**
 * Checks if a session is expired based on the expiration time
 * @param session - The session object containing expiration time
 * @returns boolean indicating if the session is expired
 */
export const isSessionExpired = (session: SessionData | null): boolean => {
  if (!session) {
    return true;
  }

  const now = new Date();
  const expiresAt = new Date(session.expiresAt);

  return now > expiresAt;
};

/**
 * Gets the remaining time for a session in milliseconds
 * @param session - The session object containing expiration time
 * @returns number of milliseconds until session expires, or 0 if expired
 */
export const getSessionRemainingTime = (session: SessionData | null): number => {
  if (!session) {
    return 0;
  }

  const now = new Date();
  const expiresAt = new Date(session.expiresAt);

  const remainingTime = expiresAt.getTime() - now.getTime();
  return Math.max(0, remainingTime);
};

/**
 * Formats session expiration time to a human-readable string
 * @param session - The session object containing expiration time
 * @returns formatted string indicating when the session expires
 */
export const formatSessionExpiry = (session: SessionData | null): string => {
  if (!session) {
    return 'No active session';
  }

  const expiresAt = new Date(session.expiresAt);
  return expiresAt.toLocaleString();
};

/**
 * Checks if the user has specific background information
 * @param user - The user profile object
 * @param category - The background category to check ('software' or 'hardware')
 * @returns boolean indicating if the user has background information in the specified category
 */
export const hasBackgroundInfo = (user: UserProfile | null, category: 'software' | 'hardware'): boolean => {
  if (!user) {
    return false;
  }

  if (category === 'software') {
    return !!user.softwareBackground && Object.keys(user.softwareBackground).length > 0;
  } else if (category === 'hardware') {
    return !!user.hardwareBackground && Object.keys(user.hardwareBackground).length > 0;
  }

  return false;
};

/**
 * Gets the user's experience level from their background information
 * @param user - The user profile object
 * @param category - The background category ('software' or 'hardware')
 * @returns the experience level string or null if not found
 */
export const getUserExperienceLevel = (user: UserProfile | null, category: 'software' | 'hardware'): string | null => {
  if (!user) {
    return null;
  }

  if (category === 'software' && user.softwareBackground) {
    return user.softwareBackground.experienceLevel || null;
  } else if (category === 'hardware' && user.hardwareBackground) {
    return user.hardwareBackground.experienceLevel || null;
  }

  return null;
};