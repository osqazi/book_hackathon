import React from 'react';
import Link from '@docusaurus/Link';

const PersonalizationEmptyState: React.FC = () => {
  return (
    <div className="personalization-empty-state">
      <h2 className="empty-title">No personalized chapters yet</h2>
      <p className="empty-description">
        You haven't bookmarked any chapters yet. Start exploring the documentation and
        click the bookmark icon on any chapter page to save it here for quick access.
      </p>
      <Link to="/" className="empty-cta-button">
        Browse Documentation
      </Link>
    </div>
  );
};

export default PersonalizationEmptyState;