// src/components/personalization/ExampleAdapter.tsx
// Example component that adapts content based on user expertise level

import React from 'react';
import { useAuth } from '@/auth/hooks/useAuth';
import ContentAdapter from './ContentAdapter';

interface ExampleAdapterProps {
  beginnerContent: React.ReactNode;
  intermediateContent: React.ReactNode;
  advancedContent: React.ReactNode;
  expertContent: React.ReactNode;
}

const ExampleAdapter: React.FC<ExampleAdapterProps> = ({
  beginnerContent,
  intermediateContent,
  advancedContent,
  expertContent,
}) => {
  const { user } = useAuth();

  // Determine which content to show based on user's experience level
  const getAdaptedContent = () => {
    if (!user || (!user.softwareBackground && !user.hardwareBackground)) {
      // If user has no background data, default to beginner
      return beginnerContent;
    }

    // Check software experience level
    const softwareLevel = user.softwareBackground?.experienceLevel as string;
    if (softwareLevel) {
      switch (softwareLevel.toLowerCase()) {
        case 'beginner':
          return beginnerContent;
        case 'intermediate':
          return intermediateContent;
        case 'advanced':
          return advancedContent;
        case 'expert':
          return expertContent;
        default:
          return beginnerContent;
      }
    }

    // Check hardware experience level
    const hardwareLevel = user.hardwareBackground?.experienceLevel as string;
    if (hardwareLevel) {
      switch (hardwareLevel.toLowerCase()) {
        case 'beginner':
          return beginnerContent;
        case 'intermediate':
          return intermediateContent;
        case 'advanced':
          return advancedContent;
        case 'expert':
          return expertContent;
        default:
          return beginnerContent;
      }
    }

    // Default to beginner content
    return beginnerContent;
  };

  return (
    <div className="example-adapter">
      {getAdaptedContent()}
    </div>
  );
};

// Export a version that uses the ContentAdapter for more sophisticated personalization
export const PersonalizedExample: React.FC<ExampleAdapterProps> = (props) => {
  return (
    <ContentAdapter>
      <ExampleAdapter {...props} />
    </ContentAdapter>
  );
};

export default ExampleAdapter;