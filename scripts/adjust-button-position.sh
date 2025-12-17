#!/bin/bash

# Fix 1: Move the button down to avoid menu overlap
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
        bottom: '40px',
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

# Fix 2: Update PersonalizationButton to ensure toggle works properly
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
ENDFILE

echo "Position adjusted and toggle confirmed!"
echo "1. Button moved to bottom: 40px (lower position)"
echo "2. Clicking filled star will remove personalization"
echo "3. Clicking empty star will add personalization"
