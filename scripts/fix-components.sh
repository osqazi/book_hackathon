#!/bin/bash

# Update PersonalizationCard.tsx
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

  const handleRemove = async (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    await removeMutation.mutateAsync(chapterPath);
  };

  // Format the date for display - handle various date formats
  const formatDate = (dateString: string) => {
    try {
      const date = new Date(dateString);
      if (isNaN(date.getTime())) {
        return 'Recently added';
      }
      return date.toLocaleDateString('en-US', {
        year: 'numeric',
        month: 'short',
        day: 'numeric'
      });
    } catch (error) {
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

echo "Files updated successfully!"
