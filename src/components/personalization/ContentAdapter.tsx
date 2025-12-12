// src/components/personalization/ContentAdapter.tsx
import React, { ReactNode, useEffect, useState } from 'react';
import { useAuth } from '../../auth/hooks/useAuth';
import { UserProfile } from '../../models/user';

interface ContentAdapterProps {
  children: ReactNode;
  fallback?: ReactNode; // Content to show when user is not authenticated
  softwareLevelThreshold?: number; // Threshold for showing advanced software content (0-1 scale)
  hardwareLevelThreshold?: number; // Threshold for showing advanced hardware content (0-1 scale)
}

const ContentAdapter: React.FC<ContentAdapterProps> = ({
  children,
  fallback = null,
  softwareLevelThreshold = 0.5,
  hardwareLevelThreshold = 0.5,
}) => {
  const { user, isAuthenticated, isLoading } = useAuth();
  const [adaptedContent, setAdaptedContent] = useState<ReactNode>(null);
  const [isReady, setIsReady] = useState(false);

  useEffect(() => {
    if (isLoading) {
      // Still loading, show nothing or a loading state
      setAdaptedContent(null);
      setIsReady(false);
      return;
    }

    if (!isAuthenticated || !user) {
      // User is not authenticated, show fallback
      setAdaptedContent(fallback);
      setIsReady(true);
      return;
    }

    // User is authenticated, adapt content based on their background
    const adapted = adaptContentForUser(children, user, {
      softwareLevelThreshold,
      hardwareLevelThreshold,
    });

    setAdaptedContent(adapted);
    setIsReady(true);
  }, [user, isAuthenticated, isLoading, children, fallback, softwareLevelThreshold, hardwareLevelThreshold]);

  // Function to adapt content based on user background
  const adaptContentForUser = (
    content: ReactNode,
    userProfile: UserProfile,
    thresholds: { softwareLevelThreshold: number; hardwareLevelThreshold: number }
  ): ReactNode => {
    // This function would contain more sophisticated logic to adapt content
    // based on the user's background information
    if (!userProfile.softwareBackground && !userProfile.hardwareBackground) {
      // User has no background data, show generic content
      return (
        <div className="content-adapter content-generic">
          <div className="user-background-prompt">
            <p>Complete your profile to get personalized content recommendations.</p>
            {/* Link to profile update page would go here */}
          </div>
          {content}
        </div>
      );
    }

    // Determine user expertise levels
    const softwareLevel = getUserExperienceLevel(userProfile, 'software');
    const hardwareLevel = getUserExperienceLevel(userProfile, 'hardware');

    // Create adapted content based on user's expertise
    return (
      <div
        className={`content-adapter content-adapted user-software-${softwareLevel || 'unknown'} user-hardware-${hardwareLevel || 'unknown'}`}
        data-user-software-level={softwareLevel}
        data-user-hardware-level={hardwareLevel}
      >
        <div className="content-personalization-info">
          <p>Content adapted for your experience level</p>
        </div>
        {content}
      </div>
    );
  };

  // Helper function to get experience level from user background
  const getUserExperienceLevel = (user: UserProfile, category: 'software' | 'hardware'): string | undefined => {
    if (category === 'software' && user.softwareBackground) {
      return user.softwareBackground.experienceLevel as string;
    } else if (category === 'hardware' && user.hardwareBackground) {
      return user.hardwareBackground.experienceLevel as string;
    }
    return undefined;
  };

  if (!isReady) {
    return <div className="loading-content">Loading personalized content...</div>;
  }

  return <>{adaptedContent}</>;
};

export default ContentAdapter;