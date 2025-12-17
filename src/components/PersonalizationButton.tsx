import React from 'react';
import { useAuthContext } from '@site/src/auth/context/AuthProvider';
import {
  useIsChapterPersonalized,
  useAddPersonalization,
  useRemovePersonalization
} from '@site/src/hooks/usePersonalization';
import PersonalizationIcon from './PersonalizationIcon';

interface PersonalizationButtonProps {
  chapterPath: string;
  chapterTitle?: string;
  chapterExcerpt?: string;
}

const PersonalizationButton: React.FC<PersonalizationButtonProps> = ({
  chapterPath,
  chapterTitle = '',
  chapterExcerpt = ''
}) => {
  const { session } = useAuthContext();
  const { isPersonalized, isLoading: isCheckLoading } = useIsChapterPersonalized(chapterPath);

  const addMutation = useAddPersonalization();
  const removeMutation = useRemovePersonalization();

  const isLoading = addMutation.isPending || removeMutation.isPending;

  // Don't show the button if user is not authenticated
  if (!session) {
    return null;
  }

  const handleToggle = async () => {
    console.log('[PersonalizationButton] Toggle clicked', {
      isPersonalized,
      chapterPath,
      action: isPersonalized ? 'REMOVE' : 'ADD'
    });

    try {
      if (isPersonalized) {
        // Remove personalization - clicking filled star removes it
        console.log('[PersonalizationButton] Removing personalization');
        await removeMutation.mutateAsync(chapterPath);
        console.log('[PersonalizationButton] Successfully removed');
      } else {
        // Add personalization - clicking empty star adds it
        console.log('[PersonalizationButton] Adding personalization');
        await addMutation.mutateAsync({
          chapter_path: chapterPath,
          chapter_title: chapterTitle,
          chapter_excerpt: chapterExcerpt
        });
        console.log('[PersonalizationButton] Successfully added');
      }
    } catch (error) {
      console.error('[PersonalizationButton] Toggle error:', error);
    }
  };

  const handleClick = async (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    await handleToggle();
  };

  // Determine button state and tooltip
  const tooltipText = isLoading
    ? 'Updating...'
    : (isPersonalized ? 'Click to remove from bookmarks' : 'Click to bookmark this chapter');

  return (
    <button
      onClick={handleClick}
      className={`personalization-button ${isPersonalized ? 'personalized' : 'not-personalized'} ${isLoading ? 'loading' : ''}`}
      title={tooltipText}
      disabled={isLoading}
      aria-label={isPersonalized ? 'Remove bookmark' : 'Add bookmark'}
      style={{
        color: isPersonalized ? '#f59e0b' : 'inherit'
      }}
    >
      <PersonalizationIcon filled={isPersonalized} />
      {isLoading && (
        <span className="loading-spinner" aria-hidden="true">...</span>
      )}
      <span className="sr-only">
        {isPersonalized ? 'Remove bookmark' : 'Add bookmark'}
      </span>
    </button>
  );
};

export default PersonalizationButton;
