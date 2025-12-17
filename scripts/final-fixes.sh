#!/bin/bash

# Fix 1: Update the delete mutation to use proper URL encoding
cat > "src/hooks/usePersonalization.ts" << 'ENDFILE'
import { useQuery, useMutation, useQueryClient } from '@tanstack/react-query';
import { useAuthContext } from '@site/src/auth/context/AuthProvider';

// Type definitions
interface PersonalizedChapter {
  id: number;
  user_id: string;
  chapter_path: string;
  chapter_title?: string;
  chapter_excerpt?: string;
  created_at: string;
}

interface AddPersonalizationRequest {
  chapter_path: string;
  chapter_title?: string;
  chapter_excerpt?: string;
}

// Get the backend URL - use localhost for development
const getBackendUrl = () => {
  if (typeof window !== 'undefined') {
    // Client-side: use localhost for development
    return 'http://localhost:3001';
  }
  return 'http://localhost:3001';
};

// Custom fetch function that includes auth headers
const apiFetch = async (url: string, options: RequestInit = {}) => {
  const backendUrl = getBackendUrl();
  const fullUrl = `${backendUrl}${url}`;

  console.log('[apiFetch] Request:', { method: options.method || 'GET', url: fullUrl });

  const response = await fetch(fullUrl, {
    ...options,
    credentials: 'include', // Include cookies for session authentication
  });

  console.log('[apiFetch] Response:', { status: response.status, statusText: response.statusText });

  if (!response.ok) {
    const errorData = await response.json().catch(() => ({ message: 'Network error' }));
    throw new Error(errorData.message || `HTTP error! status: ${response.status}`);
  }

  // For 204 responses (DELETE), return null
  if (response.status === 204) {
    return null;
  }

  return response.json();
};

/**
 * Query hook to get all personalized chapters for the current user
 */
export const usePersonalizedChapters = () => {
  const { session } = useAuthContext();

  return useQuery<PersonalizedChapter[]>({
    queryKey: ['personalized-chapters'],
    queryFn: async () => {
      const data = await apiFetch('/api/personalization/chapters');
      return data.chapters;
    },
    // Only run query if user is authenticated
    enabled: !!session,
  });
};

/**
 * Mutation hook to add a personalized chapter
 */
export const useAddPersonalization = () => {
  const queryClient = useQueryClient();

  return useMutation({
    mutationFn: async (data: AddPersonalizationRequest) => {
      console.log('[useAddPersonalization] Adding:', data);
      return await apiFetch('/api/personalization/chapters', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(data),
      });
    },
    onSuccess: () => {
      console.log('[useAddPersonalization] Success - invalidating queries');
      // Invalidate and refetch personalized chapters
      queryClient.invalidateQueries({ queryKey: ['personalized-chapters'] });
    },
  });
};

/**
 * Mutation hook to remove a personalized chapter
 */
export const useRemovePersonalization = () => {
  const queryClient = useQueryClient();

  return useMutation({
    mutationFn: async (chapterPath: string) => {
      console.log('[useRemovePersonalization] Removing:', chapterPath);

      // Double encode the path to handle special characters
      const encodedPath = encodeURIComponent(chapterPath);
      console.log('[useRemovePersonalization] Encoded path:', encodedPath);

      return await apiFetch(`/api/personalization/chapters/${encodedPath}`, {
        method: 'DELETE',
      });
    },
    onSuccess: () => {
      console.log('[useRemovePersonalization] Success - invalidating queries');
      // Invalidate and refetch personalized chapters
      queryClient.invalidateQueries({ queryKey: ['personalized-chapters'] });
    },
    onError: (error) => {
      console.error('[useRemovePersonalization] Error:', error);
    }
  });
};

/**
 * Custom hook to check if a specific chapter is personalized
 */
export const useIsChapterPersonalized = (chapterPath: string) => {
  const { data: chapters, isLoading } = usePersonalizedChapters();

  return {
    isPersonalized: chapters?.some(chapter => chapter.chapter_path === chapterPath) || false,
    isLoading,
  };
};
ENDFILE

# Fix 2: Update DocItem Layout to make button bigger, add title, and move to left
cat > "src/theme/DocItem/Layout/index.tsx" << 'ENDFILE'
import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import type LayoutType from '@theme/DocItem/Layout';
import type {WrapperProps} from '@docusaurus/types';
import DocPersonalizationButton from '@site/src/components/DocPersonalizationButton';

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      <Layout {...props} />
      <div style={{
        position: 'fixed',
        bottom: '120px',
        left: '20px',
        zIndex: 999,
        display: 'flex',
        flexDirection: 'column',
        alignItems: 'center',
        gap: '8px',
      }}>
        <div style={{
          fontSize: '0.75rem',
          fontWeight: 600,
          color: 'var(--ifm-color-emphasis-700)',
          textTransform: 'uppercase',
          letterSpacing: '0.5px',
        }}>
          Personalize
        </div>
        <div style={{
          backgroundColor: 'var(--ifm-background-surface-color)',
          boxShadow: '0 4px 12px rgba(0, 0, 0, 0.15)',
          borderRadius: '50%',
          padding: '16px',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          width: '64px',
          height: '64px',
          cursor: 'pointer',
          transition: 'all 0.2s ease',
        }}
        onMouseEnter={(e) => {
          e.currentTarget.style.transform = 'scale(1.1)';
          e.currentTarget.style.boxShadow = '0 6px 16px rgba(0, 0, 0, 0.2)';
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'scale(1)';
          e.currentTarget.style.boxShadow = '0 4px 12px rgba(0, 0, 0, 0.15)';
        }}>
          <DocPersonalizationButton />
        </div>
      </div>
    </>
  );
}
ENDFILE

# Fix 3: Make the star icon bigger
cat > "src/styles/personalization.css" << 'ENDFILE'
/* Personalization Components Styles */

/* Personalization Button */
.personalization-button {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  border: none;
  background: none;
  cursor: pointer;
  padding: 0;
  border-radius: 0.25rem;
  transition: all 0.2s ease;
  color: var(--ifm-color-emphasis-500);
}

.personalization-button:hover {
  color: #f59e0b;
}

.personalization-button.personalized {
  color: #f59e0b;
}

.personalization-button:disabled {
  opacity: 0.6;
  cursor: not-allowed;
}

.personalization-icon {
  width: 2rem;
  height: 2rem;
}

.personalization-icon.filled {
  color: #f59e0b !important; /* Golden/amber color for filled star */
}

.personalization-icon.outline {
  color: var(--ifm-color-emphasis-600);
}

.personalization-button.personalized .personalization-icon {
  color: #f59e0b !important; /* Golden when personalized */
}

/* Personalization Card */
.personalization-card {
  background: var(--ifm-background-surface-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: var(--ifm-global-radius);
  padding: 1rem;
  margin-bottom: 1rem;
  transition: box-shadow 0.2s ease;
}

.personalization-card:hover {
  box-shadow: 0 4px 14px rgba(0, 0, 0, 0.1);
}

.card-header {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  margin-bottom: 0.5rem;
}

.card-title-link {
  flex-grow: 1;
  text-decoration: none;
}

.card-title {
  margin: 0;
  font-size: 1.1rem;
  font-weight: var(--ifm-font-weight-bold);
  color: var(--ifm-heading-color);
}

.remove-button {
  background: none;
  border: none;
  font-size: 1.5rem;
  cursor: pointer;
  color: var(--ifm-color-emphasis-500);
  padding: 0.25rem;
  border-radius: 0.25rem;
  width: 2rem;
  height: 2rem;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all 0.2s ease;
}

.remove-button:hover {
  background-color: var(--ifm-color-emphasis-200);
  color: var(--ifm-color-danger);
  transform: scale(1.1);
}

.card-excerpt {
  color: var(--ifm-color-emphasis-700);
  margin: 0.5rem 0;
  font-size: 0.9rem;
  line-height: 1.5;
}

.card-footer {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-top: 0.75rem;
  font-size: 0.8rem;
  color: var(--ifm-color-emphasis-600);
}

.chapter-path {
  overflow: hidden;
  text-overflow: ellipsis;
  white-space: nowrap;
  max-width: 70%;
}

.created-date {
  white-space: nowrap;
}

/* Personalization Grid */
.personalization-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
  gap: 1rem;
  margin-top: 1.5rem;
}

/* Empty State */
.personalization-empty-state {
  text-align: center;
  padding: 3rem 1rem;
  color: var(--ifm-color-emphasis-700);
}

.empty-title {
  font-size: 1.5rem;
  margin-bottom: 1rem;
  color: var(--ifm-heading-color);
}

.empty-description {
  font-size: 1rem;
  margin-bottom: 1.5rem;
  max-width: 600px;
  margin-left: auto;
  margin-right: auto;
}

.empty-cta-button {
  display: inline-block;
  padding: 0.75rem 1.5rem;
  background-color: var(--ifm-color-primary);
  color: white;
  text-decoration: none;
  border-radius: var(--ifm-global-radius);
  transition: background-color 0.2s ease;
}

.empty-cta-button:hover {
  background-color: var(--ifm-color-primary-dark);
  text-decoration: none;
  color: white;
}

/* Loading Spinner */
.loading-spinner {
  margin-left: 0.5rem;
  font-size: 0.8rem;
}

/* Screen Reader Only */
.sr-only {
  position: absolute;
  width: 1px;
  height: 1px;
  padding: 0;
  margin: -1px;
  overflow: hidden;
  clip: rect(0, 0, 0, 0);
  white-space: nowrap;
  border: 0;
}

/* Responsive adjustments */
@media (max-width: 996px) {
  .personalization-grid {
    grid-template-columns: 1fr;
  }

  .card-footer {
    flex-direction: column;
    align-items: flex-start;
    gap: 0.5rem;
  }

  .chapter-path {
    max-width: 100%;
  }
}
ENDFILE

echo "All fixes applied!"
echo "1. Delete button now properly encodes URLs"
echo "2. Star is bigger (2rem)"
echo "3. 'Personalize' title added on top"
echo "4. Button moved to left side"
