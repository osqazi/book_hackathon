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

// Get the backend URL based on environment
const getBackendUrl = () => {
  if (typeof window !== 'undefined') {
    const hostname = window.location.hostname;
    console.log('[Personalization] Current hostname:', hostname);

    // Production: GitHub Pages
    if (hostname === 'osqazi.github.io') {
      const backendUrl = 'https://book-hackathon-alpha.vercel.app';
      console.log('[Personalization] Using production backend:', backendUrl);
      return backendUrl;
    }
  }

  // Development: localhost
  const backendUrl = 'http://localhost:3001';
  console.log('[Personalization] Using development backend:', backendUrl);
  return backendUrl;
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
