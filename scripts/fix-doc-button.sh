#!/bin/bash

cat > "src/components/DocPersonalizationButton.tsx" << 'ENDFILE'
import React, { useEffect, useState } from 'react';
import { useLocation } from '@docusaurus/router';
import { useAuthContext } from '@site/src/auth/context/AuthProvider';
import { useIsChapterPersonalized, useAddPersonalization, useRemovePersonalization } from '@site/src/hooks/usePersonalization';
import PersonalizationButton from './PersonalizationButton';

const DocPersonalizationButton: React.FC = () => {
  const location = useLocation();
  const { session } = useAuthContext();
  const [docTitle, setDocTitle] = useState<string>('');
  const [docExcerpt, setDocExcerpt] = useState<string>('');

  // Extract title and excerpt from the page
  useEffect(() => {
    if (typeof document !== 'undefined') {
      // Get title from document
      const title = document.title.replace(' | Humanoid Robotics Book', '').trim();
      setDocTitle(title || 'Documentation');

      // Try to get excerpt from the first paragraph or description meta tag
      const descriptionMeta = document.querySelector('meta[name="description"]');
      const excerpt = descriptionMeta?.getAttribute('content') || '';
      setDocExcerpt(excerpt);
    }
  }, [location.pathname]);

  // Use the current pathname as the chapter path
  const chapterPath = location.pathname;

  // Check if the current chapter is personalized
  const { isPersonalized, isLoading: isCheckLoading } = useIsChapterPersonalized(chapterPath);

  // Don't show the button if user is not authenticated
  if (!session) {
    return null;
  }

  return (
    <PersonalizationButton
      chapterPath={chapterPath}
      chapterTitle={docTitle}
      chapterExcerpt={docExcerpt}
    />
  );
};

export default DocPersonalizationButton;
ENDFILE

echo "DocPersonalizationButton fixed!"
