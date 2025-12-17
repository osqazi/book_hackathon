import React, { useEffect, useState } from 'react';
import { useLocation } from '@docusaurus/router';
import { useAuthContext } from '@site/src/auth/context/AuthProvider';
import { useIsChapterPersonalized, useAddPersonalization, useRemovePersonalization } from '@site/src/hooks/usePersonalization';
import PersonalizationButton from './PersonalizationButton';

const DocPersonalizationButton: React.FC = () => {
  const location = useLocation();
  const { session } = useAuthContext();

  // Get title and excerpt immediately (synchronously) from document
  const getDocTitle = () => {
    if (typeof document !== 'undefined') {
      const title = document.title.replace(' | Humanoid Robotics Book', '').trim();
      return title || 'Documentation';
    }
    return 'Documentation';
  };

  const getDocExcerpt = () => {
    if (typeof document !== 'undefined') {
      const descriptionMeta = document.querySelector('meta[name="description"]');
      return descriptionMeta?.getAttribute('content') || '';
    }
    return '';
  };

  // Use the current pathname as the chapter path
  const chapterPath = location.pathname;

  // Check if the current chapter is personalized
  const { isPersonalized, isLoading: isCheckLoading } = useIsChapterPersonalized(chapterPath);

  // Don't show the button if user is not authenticated
  if (!session) {
    return null;
  }

  // Get title and excerpt at render time
  const docTitle = getDocTitle();
  const docExcerpt = getDocExcerpt();

  return (
    <PersonalizationButton
      chapterPath={chapterPath}
      chapterTitle={docTitle}
      chapterExcerpt={docExcerpt}
    />
  );
};

export default DocPersonalizationButton;
