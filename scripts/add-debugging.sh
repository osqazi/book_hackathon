#!/bin/bash

# Add debugging to PersonalizationButton
cat > "src/components/PersonalizationButton.tsx" << 'ENDFILE'
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

  // Debug logging
  console.log('[PersonalizationButton] State:', {
    chapterPath,
    chapterTitle,
    chapterExcerpt,
    isPersonalized,
    isLoading
  });

  // Don't show the button if user is not authenticated
  if (!session) {
    return null;
  }

  const handleToggle = async () => {
    console.log('[PersonalizationButton] Toggle clicked', {
      isPersonalized,
      chapterPath,
      chapterTitle,
      chapterExcerpt
    });

    if (isPersonalized) {
      // Remove personalization
      console.log('[PersonalizationButton] Removing personalization');
      await removeMutation.mutateAsync(chapterPath);
    } else {
      // Add personalization
      console.log('[PersonalizationButton] Adding personalization');
      await addMutation.mutateAsync({
        chapter_path: chapterPath,
        chapter_title: chapterTitle,
        chapter_excerpt: chapterExcerpt
      });
    }
  };

  const handleClick = async (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    await handleToggle();
  };

  // Determine button state and tooltip
  const isActionPending = addMutation.isPending || removeMutation.isPending;
  const tooltipText = isActionPending
    ? 'Updating...'
    : (isPersonalized ? 'Remove from bookmarks' : 'Bookmark this chapter');

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
ENDFILE

# Add debugging to PersonalizationCard
cat > "src/components/PersonalizationCard.tsx" << 'ENDFILE'
import React from 'react';
import Link from '@docusaurus/Link';
import { useRemovePersonalization } from '@site/src/hooks/usePersonalization';

interface PersonalizationCardProps {
  id: number;
  chapterPath: string;
  chapterTitle: string;
  chapterExcerpt?: string;
  createdAt: string;
}

const PersonalizationCard: React.FC<PersonalizationCardProps> = ({
  chapterPath,
  chapterTitle,
  chapterExcerpt,
  createdAt
}) => {
  const removeMutation = useRemovePersonalization();

  console.log('[PersonalizationCard] Props:', {
    chapterPath,
    chapterTitle,
    chapterExcerpt,
    createdAt
  });

  const handleRemove = async (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();

    console.log('[PersonalizationCard] Removing:', chapterPath);

    try {
      await removeMutation.mutateAsync(chapterPath);
      console.log('[PersonalizationCard] Remove successful');
    } catch (error) {
      console.error('[PersonalizationCard] Remove failed:', error);
    }
  };

  // Format the date for display - handle various date formats
  const formatDate = (dateString: string) => {
    console.log('[PersonalizationCard] Formatting date:', dateString);
    try {
      const date = new Date(dateString);
      if (isNaN(date.getTime())) {
        console.warn('[PersonalizationCard] Invalid date:', dateString);
        return 'Recently added';
      }
      return date.toLocaleDateString('en-US', {
        year: 'numeric',
        month: 'short',
        day: 'numeric'
      });
    } catch (error) {
      console.error('[PersonalizationCard] Date format error:', error);
      return 'Recently added';
    }
  };

  const formattedDate = formatDate(createdAt);

  return (
    <div className="personalization-card">
      <div className="card-header">
        <Link to={chapterPath} className="card-title-link">
          <h3 className="card-title">{chapterTitle || 'Untitled Chapter'}</h3>
        </Link>
        <button
          onClick={handleRemove}
          className="remove-button"
          title="Remove from bookmarks"
          aria-label="Remove bookmark"
        >
          ×
        </button>
      </div>
      {chapterExcerpt && (
        <p className="card-excerpt">
          {chapterExcerpt.length > 50
            ? chapterExcerpt.substring(0, 50) + '...'
            : chapterExcerpt}
        </p>
      )}
      <div className="card-footer">
        <span className="chapter-path">{chapterPath}</span>
        <span className="created-date">{formattedDate}</span>
      </div>
    </div>
  );
};

export default PersonalizationCard;
ENDFILE

echo "Debugging added to components!"
