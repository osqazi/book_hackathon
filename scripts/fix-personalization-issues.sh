#!/bin/bash

# Fix 1: Update CSS to make filled star golden
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
  padding: 0.25rem;
  border-radius: 0.25rem;
  transition: all 0.2s ease;
  color: var(--ifm-color-emphasis-500);
}

.personalization-button:hover {
  background-color: var(--ifm-color-emphasis-200);
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
  width: 1.2rem;
  height: 1.2rem;
}

.personalization-icon.filled {
  color: #f59e0b !important; /* Golden/amber color for filled star */
}

.personalization-icon.outline {
  color: var(--ifm-color-emphasis-500);
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
}

.remove-button:hover {
  background-color: var(--ifm-color-emphasis-200);
  color: var(--ifm-color-danger);
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

# Fix 2: Update DocPersonalizationButton to get title immediately
cat > "src/components/DocPersonalizationButton.tsx" << 'ENDFILE'
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
ENDFILE

echo "Personalization issues fixed!"
echo "1. Star icon will now turn golden when clicked"
echo "2. Chapter titles and dates will be saved correctly"
